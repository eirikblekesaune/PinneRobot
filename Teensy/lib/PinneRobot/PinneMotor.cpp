#include <PinneMotor.h>

PinneMotor::PinneMotor(int topStopSensorPin, int slackStopSensorPin,
                       int encoderInterruptPinA, int encoderInterruptPinB,
                       int currentSensePin, VNH5019Driver *driver,
                       address_t address, PinneComm *comm)
    : _topStopSensorPin(topStopSensorPin),
      _slackStopSensorPin(slackStopSensorPin),
      _encoderInterruptPinA(encoderInterruptPinA),
      _encoderInterruptPinB(encoderInterruptPinB),
      _currentSensePin(currentSensePin), _driver(driver), _address(address),
      _comm(comm) {
  _minPosition = POSITION_ALL_UP;
  _maxPosition = POSITION_DEFAULT_MAX;
  _targetPosition = TARGET_NONE;
  _targetSpeedPIDInput = 0.0;
  _currentPosition = POSITION_ALL_UP;
  _prevPosition = _currentPosition;
  _measuredSpeed = 0.0;
  _state = STOPPED;
}

void PinneMotor::init() {
  pinMode(_encoderInterruptPinA, INPUT_PULLUP);
  pinMode(_encoderInterruptPinB, INPUT_PULLUP);
  _encoder = new Encoder(_encoderInterruptPinB, _encoderInterruptPinA);
  pinMode(_topStopSensorPin, INPUT_PULLUP);
  pinMode(_slackStopSensorPin, INPUT_PULLUP);
  pinMode(_currentSensePin, INPUT);
  SetMotorControlMode(CONTROL_MODE_PWM);
  _measuredCurrent = static_cast<float>(analogRead(_currentSensePin));
  _topStopButton = new Bounce(_topStopSensorPin, 5);
  _slackStopButton = new Bounce(_slackStopSensorPin, 200);

  _speedometerInterval = 50;
  _speedometerMetro = new Metro(_speedometerInterval);

  _speedPID =
      new PID_CLASS(&_measuredSpeed, &_targetSpeedPIDOutput,
                    &_targetSpeedPIDInput, 2.0, 50.0, 10.0, P_ON_M, DIRECT);

  _speedPID->SetOutputLimits(static_cast<double>(_driver->SPEED_MIN),
                             static_cast<double>(1000));
  _speedPID->SetSampleTime(_speedometerInterval);

  _driver->init();
  _driver->SetDirection(DIRECTION_UP);
  SetDirection(DIRECTION_DOWN);
  Stop();
  _blocked = false;
  _topStopSensorValue = digitalRead(_topStopSensorPin);
  if (_topStopSensorValue == TOP_SENSOR_IN) {
    _TopStopSensorIn();
  } else if (_topStopSensorValue == TOP_SENSOR_OUT) {
    _TopStopSensorOut();
  }
  _slackStopSensorValue = digitalRead(_slackStopSensorPin);
  if (_slackStopSensorValue == SLACK_SENSOR_OUT) {
    _SlackStopSensorOut();
  } else if (_slackStopSensorValue == SLACK_SENSOR_IN) {
    _SlackStopSensorIn();
  }
}

position_t PinneMotor::GetCurrentPosition() { return _encoder->read(); };

void PinneMotor::Stop() { _driver->SetPWM(VNH5019Driver::SPEED_STOP); }

void PinneMotor::SetStop(int value) {
  _stoppingSpeed = constrain(value, 0, 5000);
  _driver->SetPWM(VNH5019Driver::SPEED_STOP);
}

void PinneMotor::SetPWM(int speed) {
  if (speed <= 0) {
    Stop();
  } else {
    if (!IsBlocked()) {
      _driver->SetPWM(speed);
    }
  }
}

float PinneMotor::GetMeasuredSpeed() { return _measuredSpeed; }

void PinneMotor::SetDirection(direction_t direction) {
  if (GetDirection() != direction) {
    direction_t dir;
    _driver->SetDirection(direction);
    dir = GetDirection();
    if (dir == DIRECTION_DOWN) {
      _GoingDown();
    } else if (dir == DIRECTION_UP) {
      _GoingUp();
    }
  } else {
    _driver->UpdateDirection();
  }
}

bool PinneMotor::IsBlocked() {
  if (_state >= BLOCKED_BY_TOP_SENSOR) {
    direction_t direction = GetDirection();
    switch (_state) {
    case BLOCKED_BY_MIN_POSITION:
    case BLOCKED_BY_TOP_SENSOR:
    case BLOCKED_BY_ABS_MIN_POSITION:
      if (direction == DIRECTION_UP) {
        return true;
      } else {
        return false;
      }
      break;
    case BLOCKED_BY_MAX_POSITION:
    case BLOCKED_BY_SLACK_SENSOR:
      if (direction == DIRECTION_DOWN) {
        return true;
      } else {
        return false;
      }
    default:
      //
      return false;
    }
  } else {
    return false;
  }
}

// Read all sensor values and check positions
// Stop motor if needed
void PinneMotor::UpdateState() {
  ReadTopStopSensor();
  ReadSlackStopSensor();
  _UpdateSpeedometer();
  _UpdateCurrentSense();
  if ((_state == BLOCKED_BY_TOP_SENSOR) ||
      (_state == BLOCKED_BY_SLACK_SENSOR)) { // sensors have priority over
                                             // position lilmits

  } else {
    position_t currPosition = GetCurrentPosition();
    position_t minPosition = GetMinPosition();
    if (currPosition < minPosition) {
      _MinPositionReached();
    } else if (currPosition > GetMaxPosition()) {
      _MaxPositionReached();
    } else {
      switch (_motorControlMode) {
      case CONTROL_MODE_PWM:
        this->_PWMModeUpdate();
        break;
      case CONTROL_MODE_TARGET_POSITION:
        this->_TargetPositionModeUpdate();
        break;
      case CONTROL_MODE_TARGET_SPEED:
        this->_TargetSpeedModeUpdate();
        break;
      };
    }
  }
}

void PinneMotor::_PWMModeUpdate() {
  direction_t direction = GetDirection();
  if (direction == DIRECTION_DOWN) {
    _GoingDown();
  } else {
    _GoingUp();
  }
}

void PinneMotor::_TargetPositionModeUpdate() {
  position_t currPosition = GetCurrentPosition();
  direction_t direction = GetDirection();
  if (direction == DIRECTION_DOWN) {
    if ((_targetPosition != TARGET_NONE) && (currPosition > _targetPosition)) {
      _TargetReached();
    }
  } else {
    if ((_targetPosition != TARGET_NONE) && (currPosition < _targetPosition)) {
      _TargetReached();
    }
  }
}

void PinneMotor::_TargetSpeedModeUpdate() {
  _speedPID->Compute();
  this->SetPWM(static_cast<int>(_targetSpeedPIDOutput));
  /* OSCMessage msg("/PID"); */
  /* msg.add(_targetSpeedPIDOutput); */
  /* _comm->SendOSCMessage(msg); */
}

void PinneMotor::_UpdateSpeedometer() {
  if (_speedometerMetro->check() == 1) {
    float prevSpeed = _measuredSpeed;
    position_t currentPosition = GetCurrentPosition();
    /* _measuredSpeed = (currentPosition - _prevPosition); */
    _measuredSpeed = (currentPosition - _prevPosition) * 0.5;
    _measuredSpeed = _measuredSpeed + (prevSpeed * 0.5);
    _prevPosition = currentPosition;
  }
}

void PinneMotor::_UpdateCurrentSense() {
  _measuredCurrent = static_cast<float>(analogRead(_currentSensePin));
}

void PinneMotor::ReadTopStopSensor() {
  _topStopButton->update();
  if (_topStopButton->fallingEdge()) {
    // pin is pulled LOW when button is pressed
    _topStopSensorValue = TOP_SENSOR_IN;
    _TopStopSensorIn();
  } else if (_topStopButton->risingEdge()) {
    // pin is pulled HIGH when button is released
    _topStopSensorValue = TOP_SENSOR_OUT;
    _TopStopSensorOut();
  }
}

void PinneMotor::ReadSlackStopSensor() {
  _slackStopButton->update();
  if (_slackStopButton->fallingEdge()) {
    // pin is pulled HIGH when button is pressed
    _slackStopSensorValue = SLACK_SENSOR_OUT;
    _SlackStopSensorOut();
  } else if (_slackStopButton->risingEdge()) {
    // pin is pulled LOW when button is released
    _slackStopSensorValue = SLACK_SENSOR_IN;
    _SlackStopSensorIn();
  }
}

void PinneMotor::_TopStopSensorIn() {
  Stop();
  _state = BLOCKED_BY_TOP_SENSOR;
  SetCurrentPosition(POSITION_ALL_UP);
  _comm->NotifyStateChange(BLOCKED_BY_TOP_SENSOR, _address);
}

void PinneMotor::_TopStopSensorOut() {
  direction_t direction;
  direction = GetDirection();
  if (direction == DIRECTION_DOWN) {
    _GoingDown();
  } else if (direction == DIRECTION_UP) {
    _GoingUp();
  }
}

void PinneMotor::_SlackStopSensorOut() {
  Stop();
  _state = BLOCKED_BY_SLACK_SENSOR;
  _comm->NotifyStateChange(BLOCKED_BY_SLACK_SENSOR, _address);
}

void PinneMotor::_SlackStopSensorIn() {
  direction_t direction;
  direction = GetDirection();
  if (direction == DIRECTION_DOWN) {
    _GoingDown();
  } else if (direction == DIRECTION_UP) {
    _GoingUp();
  }
}

void PinneMotor::_GoingUp() {
  if (_state != GOING_UP) {
    _comm->NotifyStateChange(GOING_UP, _address);
    _state = GOING_UP;
  }
}

void PinneMotor::_GoingDown() {
  if (_state != GOING_DOWN) {
    _comm->NotifyStateChange(GOING_DOWN, _address);
    _state = GOING_DOWN;
  }
}

void PinneMotor::_AbsMinPositionReached() {
  if (_state != BLOCKED_BY_ABS_MIN_POSITION) {
    Stop();
    _comm->NotifyStateChange(BLOCKED_BY_ABS_MIN_POSITION, _address);
    _state = BLOCKED_BY_ABS_MIN_POSITION;
  }
}

void PinneMotor::_MinPositionReached() {
  if (_state != BLOCKED_BY_MIN_POSITION) {
    Stop();
    _comm->NotifyStateChange(BLOCKED_BY_MIN_POSITION, _address);
    _state = BLOCKED_BY_MIN_POSITION;
  }
}

void PinneMotor::_MaxPositionReached() {
  if (_state != BLOCKED_BY_MAX_POSITION) {
    Stop();
    _comm->NotifyStateChange(BLOCKED_BY_MAX_POSITION, _address);
    _state = BLOCKED_BY_MAX_POSITION;
  }
}

void PinneMotor::_TargetReached() {
  if (_state != STOPPED_AT_TARGET) {
    Stop();
    _targetPosition = TARGET_NONE;
    _comm->NotifyStateChange(STOPPED_AT_TARGET, _address);
    _state = STOPPED_AT_TARGET;
  }
}

void PinneMotor::_GoingToTarget() {
  _state = GOING_TO_TARGET;
  _comm->NotifyStateChange(GOING_TO_TARGET, _address);
}

void PinneMotor::SetTargetPosition(position_t targetPosition) {
  position_t value;
  if (targetPosition == TARGET_NONE) {
    _targetPosition = TARGET_NONE;
  } else {
    value = constrain(targetPosition, GetMinPosition(), GetMaxPosition());
    _targetPosition = value;
    position_t currPosition = GetCurrentPosition();

    // change the direction if target in the opposite direction
    if (GetDirection() == DIRECTION_DOWN) {
      if (currPosition > _targetPosition) {
        SetDirection(DIRECTION_UP);
      }
    } else {
      if (currPosition < _targetPosition) {
        SetDirection(DIRECTION_DOWN);
      }
    }
  }
}

void PinneMotor::SetTargetSpeed(float value) { _targetSpeedPIDInput = value; }

void PinneMotor::SetCurrentPosition(position_t currentPosition) {
  currentPosition =
      constrain(currentPosition, GetMinPosition(), GetMaxPosition());
  _encoder->write(currentPosition);
}

void PinneMotor::SetBrake(int brake) { _driver->SetBrake(brake); }

void PinneMotor::SetMinPosition(position_t minPosition) {
  _minPosition = max(minPosition, POSITION_ALL_UP);
}

void PinneMotor::SetMaxPosition(position_t maxPosition) {
  /* _maxPosition = min(maxPosition, POSITION_DEFAULT_MAX); */
  _maxPosition = maxPosition;
}

void PinneMotor::GoToParkingPosition(int speed) {
  // Check if already at top and do parking only if the sensor is out
  _topStopSensorValue = digitalRead(_topStopSensorPin);
  if (_topStopSensorValue == TOP_SENSOR_OUT) {
    // fake the currentPosition to default max
    SetCurrentPosition(POSITION_DEFAULT_MAX);
    SetDirection(DIRECTION_UP);
    SetPWM(speed);
  }
}

void PinneMotor::GoToParkingPosition() {
  GoToParkingPosition(VNH5019Driver::SPEED_MAX / 4);
}

void PinneMotor::GoToTargetPosition() {
  if (GetTargetPosition() != TARGET_NONE) {
    _GoingToTarget();
  }
}

void PinneMotor::GoToTargetPosition(position_t value) {
  if (value > 0) {
    if (GetTargetPosition() != TARGET_NONE) {
      _GoingToTarget();
    }
  } else {
    if (GetDirection() ==
        DIRECTION_DOWN) // cancel goto target when speed is set
    {
      _GoingDown();
    } else {
      _GoingUp();
    }
  }
}

void PinneMotor::SetMotorControlMode(controlMode_t mode) {
  if (mode != _motorControlMode) {
    controlMode_t prevMode = _motorControlMode;
    _motorControlMode = mode;

    switch (prevMode) {
    case CONTROL_MODE_PWM:
      break;
    case CONTROL_MODE_TARGET_POSITION:
      break;
    case CONTROL_MODE_TARGET_SPEED:
      _speedPID->SetMode(MANUAL);
      break;
    }

    switch (_motorControlMode) {
    case CONTROL_MODE_PWM:
      break;
    case CONTROL_MODE_TARGET_POSITION:
      break;
    case CONTROL_MODE_TARGET_SPEED:
      _speedPID->SetMode(AUTOMATIC);
      break;
    }
  }
}

bool PinneMotor::routeOSC(OSCMessage &msg, int initialOffset) {
  int offset;
  offset = msg.match("/bipolarPWM", initialOffset);
  if (offset) {
    this->_RouteBipolarPWMMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/stop", initialOffset);
  if (offset) {
    this->_RouteStopMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/targetPosition", initialOffset);
  if (offset) {
    this->_RouteTargetPositionMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/bipolarTargetSpeed", initialOffset);
  if (offset) {
    this->_RouteTargetSpeedMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/currentPosition", initialOffset);
  if (offset) {
    this->_RouteCurrentPositionMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/brake", initialOffset);
  if (offset) {
    this->_RouteCurrentPositionMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/stateChange", initialOffset);
  if (offset) {
    this->_RouteStateChangeMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/minPosition", initialOffset);
  if (offset) {
    this->_RouteMinPositionMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/maxPosition", initialOffset);
  if (offset) {
    this->_RouteMaxPositionMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/goToParkingPosition", initialOffset);
  if (offset) {
    this->_RouteGoToParkingPositionMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/goToTargetPosition", initialOffset);
  if (offset) {
    this->_RouteGoToTargetPositionMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/measuredSpeed", initialOffset);
  if (offset) {
    this->_RouteMeasuredSpeedMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/echoMessages", initialOffset);
  if (offset) {
    this->_RouteEchoMessagesMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/pidParameters", initialOffset);
  if (offset) {
    this->_RoutePIDParametersMsg(msg, offset + initialOffset);
    return true;
  }
  return false;
}

void PinneMotor::_RouteStopMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    this->SetStop(msg.getInt(0));
  } else {
    this->Stop();
  }
}

void PinneMotor::_RouteBipolarPWMMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    int bp = msg.getInt(0);
    if (bp == 0) {
      this->SetPWM(0);
    } else if (bp > 0) {
      this->SetDirection(DIRECTION_UP);
      this->SetPWM(bp);
    } else {
      this->SetDirection(DIRECTION_DOWN);
      this->SetPWM(abs(bp));
    }
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetPWM());
      _comm->ReturnQueryValue(CMD_SPEED, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteDirectionMsg(OSCMessage &msg, int initialOffset) {
  if (msg.size() > 0) {
    direction_t direction;
    if (msg.isInt(0)) {
      direction = static_cast<direction_t>(msg.getInt(0));
      this->SetDirection(direction);
    } else if (msg.isString(0)) {
      char dirStr[8];
      msg.getString(0, dirStr, 8);
      if (strcmp(dirStr, "up") == 0) {
        direction = DIRECTION_UP;
        this->SetDirection(direction);
      } else if (strcmp(dirStr, "down") == 0) {
        direction = DIRECTION_DOWN;
        this->SetDirection(direction);
      }
    }
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      direction_t dir = GetDirection();
      OSCMessage replyMsg("/");
      if (dir == DIRECTION_DOWN) {
        replyMsg.add("down");
      } else if (dir == DIRECTION_UP) {
        replyMsg.add("up");
      }
      _comm->ReturnQueryValue(CMD_DIRECTION, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteTargetPositionMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    int pos = msg.getInt(0);
    this->SetTargetPosition(pos);
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetTargetPosition());
      _comm->ReturnQueryValue(CMD_TARGET_POSITION, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteTargetSpeedMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isFloat(0))) {
    float pos = msg.getFloat(0);
    this->SetTargetSpeed(pos);
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetTargetSpeed());
      _comm->ReturnQueryValue(CMD_TARGET_SPEED, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteCurrentPositionMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    int pos = msg.getInt(0);
    this->SetCurrentPosition(static_cast<position_t>(pos));
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(static_cast<int>(GetCurrentPosition()));
      _comm->ReturnQueryValue(CMD_CURRENT_POSITION, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteBrakeMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    int speed = msg.getInt(0);
    this->SetBrake(speed);
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetBrake());
      _comm->ReturnQueryValue(CMD_BRAKE, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteStateChangeMsg(OSCMessage &msg, int initialOffset) {
  if (_comm->HasQueryAddress(msg, initialOffset)) {
    OSCMessage replyMsg("/");
    String stateStr = String(StateChangeMap.at(_state));
    replyMsg.add(stateStr.c_str());
    _comm->ReturnQueryValue(CMD_STATE_CHANGE, _address, replyMsg);
  }
}

void PinneMotor::_RouteMinPositionMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    int pos = msg.getInt(0);
    this->SetMinPosition(pos);
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetMinPosition());
      _comm->ReturnQueryValue(CMD_MIN_POSITION, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteMaxPositionMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    int pos = msg.getInt(0);
    this->SetMaxPosition(pos);
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetMaxPosition());
      _comm->ReturnQueryValue(CMD_MAX_POSITION, _address, replyMsg);
    }
  }
}

void PinneMotor::_RouteGoToParkingPositionMsg(OSCMessage &msg,
                                              int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    this->GoToParkingPosition(msg.getInt(0));
  } else {
    this->GoToParkingPosition();
  }
}

void PinneMotor::_RouteGoToTargetPositionMsg(OSCMessage &msg,
                                             int initialOffset) {
  if (msg.size() == 0) {
    this->GoToTargetPosition();
  }
}

void PinneMotor::_RouteMeasuredSpeedMsg(OSCMessage &msg, int initialOffset) {
  if (_comm->HasQueryAddress(msg, initialOffset)) {
    OSCMessage replyMsg("/");
    replyMsg.add(GetMeasuredSpeed());
    _comm->ReturnQueryValue(CMD_MEASURED_SPEED, _address, replyMsg);
  }
}

void PinneMotor::_RouteEchoMessagesMsg(OSCMessage &msg, int initialOffset) {}

void PinneMotor::_RoutePIDParametersMsg(OSCMessage &msg, int initialOffset) {
  if (msg.size() == 3) {
    pidvalue_t kp, ki, kd;
    kp = msg.getFloat(0);
    ki = msg.getFloat(1);
    kd = msg.getFloat(2);
    _speedPID->SetTunings(kp, ki, kd);
  } else if (_comm->HasQueryAddress(msg, initialOffset)) {
    OSCMessage replyMsg("/");
    replyMsg.add(static_cast<float>(_speedPID->GetKp()));
    replyMsg.add(static_cast<float>(_speedPID->GetKi()));
    replyMsg.add(static_cast<float>(_speedPID->GetKd()));
    _comm->ReturnQueryValue(CMD_PID_PARAMETERS, _address, replyMsg);
  }
}

