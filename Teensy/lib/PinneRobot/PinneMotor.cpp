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
  _targetSpeed = 0.81;
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
  SetMotorControlMode(CONTROL_MODE_MANUAL);
  _measuredCurrent = static_cast<float>(analogRead(_currentSensePin));
  _topStopButton = new Bounce(_topStopSensorPin, 5);
  _slackStopButton = new Bounce(_slackStopSensorPin, 200);

  _speedometerInterval = 50;
  _speedometerMetro = new Metro(_speedometerInterval);
  _kp = 2.0;
  _ki = 5.0;
  _kd = 1.0;

  _torquePID = new FastFloatPID(&_measuredCurrent, &_torquePID_output,
                                &_targetSpeed, _kp, _ki, _kd, P_ON_M, DIRECT);

  _torquePID->SetOutputLimits(_driver->SPEED_MIN, 1000);
  _torquePID->SetSampleTime(static_cast<float>(_speedometerInterval));
  /* _speedPID->SetSampleTime(static_cast<float>(_speedometerInterval) /
   * 1000.0); */
  /* _speedPID->SetOutputLimits(_driver->SPEED_MIN, _driver->SPEED_MAX); */

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

  _speedRamper = new SpeedRamping(VNH5019Driver::SPEED_MAX * 0.115,
                                  VNH5019Driver::SPEED_MAX);
}

position_t PinneMotor::GetCurrentPosition() { return _encoder->read(); };

void PinneMotor::Stop() { _driver->SetSpeed(VNH5019Driver::SPEED_STOP); }

void PinneMotor::SetStop(int value) {
  _stoppingSpeed = constrain(value, 0, 5000);
  _driver->SetSpeed(VNH5019Driver::SPEED_STOP);
}

void PinneMotor::SetSpeed(int speed) {
  if (speed <= 0) {
    Stop();
  } else {
    if (!IsBlocked()) {
      _driver->SetSpeed(speed);
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
      case CONTROL_MODE_MANUAL:
        this->_ManualModeUpdate();
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

void PinneMotor::_ManualModeUpdate() {
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
  _torquePID->Compute();
  this->SetSpeed(static_cast<int>(_torquePID_output));
  OSCMessage msg("/torquePID");
  msg.add(this->GetSpeed());
  msg.add(this->GetCurrentPosition());
  msg.add(_measuredSpeed);
  msg.add(_measuredCurrent);
  _comm->SendOSCMessage(msg);
}

void PinneMotor::_UpdateSpeedometer() {
  if (_speedometerMetro->check() == 1) {
    double prevSpeed = _measuredSpeed;
    position_t currentPosition = GetCurrentPosition();
    _measuredSpeed = (currentPosition - _prevPosition) * 0.1;
    _measuredSpeed = _measuredSpeed + (prevSpeed * 0.9);
    _prevPosition = currentPosition;
  }
}

void PinneMotor::_UpdateCurrentSense() {
  _measuredCurrent = static_cast<float>(analogRead(_currentSensePin));
}

void PinneMotor::_UpdateSpeedRamp() {
  if (_speedRamper->Calculate(GetCurrentPosition())) {
    int newSpeed;
    newSpeed = _speedRamper->GetCurrentValue();
    SetSpeed(newSpeed);
  }
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

void PinneMotor::SetTargetSpeed(float value) { _targetSpeed = value; }

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
    SetSpeed(speed);
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
      _speedRamper->Start(GetCurrentPosition(), GetTargetPosition(), value);
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

void PinneMotor::SetGoToSpeedRampUp(int value) {
  _speedRamper->SetRampUp(static_cast<float>(value));
}

void PinneMotor::SetGoToSpeedRampDown(int value) {
  _speedRamper->SetRampDown(static_cast<float>(value));
}

void PinneMotor::SetGoToSpeedScaling(int value) {
  _speedRamper->SetSpeedScaling(static_cast<float>(value) / 1000.0);
}

void PinneMotor::SetMotorControlMode(controlMode_t mode) {
  if (mode != _motorControlMode) {
    controlMode_t prevMode = _motorControlMode;
    _motorControlMode = mode;

    switch (prevMode) {
    case CONTROL_MODE_MANUAL:
      break;
    case CONTROL_MODE_TARGET_POSITION:
      break;
    case CONTROL_MODE_TARGET_SPEED:
      _torquePID->SetMode(MANUAL);
      break;
    }

    switch (_motorControlMode) {
    case CONTROL_MODE_MANUAL:
      break;
    case CONTROL_MODE_TARGET_POSITION:
      break;
    case CONTROL_MODE_TARGET_SPEED:
      _torquePID->SetMode(AUTOMATIC);
      break;
    }
  }
}

bool PinneMotor::routeOSC(OSCMessage &msg, int initialOffset) {
  int offset;
  offset = msg.match("/bipolarSpeed", initialOffset);
  if (offset) {
    this->_RouteBipolarSpeedMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/speed", initialOffset);
  if (offset) {
    this->_RouteSpeedMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/direction", initialOffset);
  if (offset) {
    this->_RouteDirectionMsg(msg, offset + initialOffset);
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
  offset = msg.match("/targetSpeed", initialOffset);
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
  offset = msg.match("/goToSpeedRampDown", initialOffset);
  if (offset) {
    this->_RouteGoToSpeedRampDownMsg(msg, offset + initialOffset);
    return true;
  }
  offset = msg.match("/goToSpeedScaling", initialOffset);
  if (offset) {
    this->_RouteGoToSpeedScalingMsg(msg, offset + initialOffset);
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
  offset = msg.match("/motorControlMode", initialOffset);
  if (offset) {
    this->_RouteMotorControlModeMsg(msg, offset + initialOffset);
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

void PinneMotor::_RouteSpeedMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    int speed = msg.getInt(0);
    this->SetSpeed(speed);
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetSpeed());
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

void PinneMotor::_RouteGoToSpeedRampDownMsg(OSCMessage &msg,
                                            int initialOffset) {}

void PinneMotor::_RouteGoToSpeedScalingMsg(OSCMessage &msg, int initialOffset) {
}

void PinneMotor::_RouteEchoMessagesMsg(OSCMessage &msg, int initialOffset) {}

void PinneMotor::_RoutePIDParametersMsg(OSCMessage &msg, int initialOffset) {
  if (msg.size() == 3) {
    _kp = msg.getFloat(0);
    _ki = msg.getFloat(1);
    _kd = msg.getFloat(2);
    _torquePID->SetTunings(_kp, _ki, _kd);
  } else if (_comm->HasQueryAddress(msg, initialOffset)) {
    OSCMessage replyMsg("/");
    replyMsg.add(_kp);
    replyMsg.add(_ki);
    replyMsg.add(_kd);
    _comm->ReturnQueryValue(CMD_PID_PARAMETERS, _address, replyMsg);
  }
}

void PinneMotor::_RouteMotorControlModeMsg(OSCMessage &msg, int initialOffset) {
  if (msg.size() > 0) {
    if (msg.isString(0)) {
      char dirStr[16];
      msg.getString(0, dirStr, 16);
      if (strcmp(dirStr, "manual") == 0) {
        this->SetMotorControlMode(CONTROL_MODE_MANUAL);
      } else if (strcmp(dirStr, "targetPosition") == 0) {
        this->SetMotorControlMode(CONTROL_MODE_TARGET_POSITION);
      } else if (strcmp(dirStr, "targetSpeed") == 0) {
        this->SetMotorControlMode(CONTROL_MODE_TARGET_SPEED);
      }
    }
    } else {
      if (_comm->HasQueryAddress(msg, initialOffset)) {
        controlMode_t mode = GetMotorControlMode();
        OSCMessage replyMsg("/");
        if (mode == CONTROL_MODE_MANUAL) {
          replyMsg.add("manual");
        } else if (mode == CONTROL_MODE_TARGET_POSITION) {
          replyMsg.add("targetPosition");
        } else if (mode == CONTROL_MODE_TARGET_SPEED) {
          replyMsg.add("targetSpeed");
        }
        _comm->ReturnQueryValue(CMD_MOTOR_CONTROL_MODE, _address, replyMsg);
      }
    }
}

void PinneMotor::_RouteBipolarSpeedMsg(OSCMessage &msg, int initialOffset) {
  if (msg.size() > 0) {
    if (msg.isInt(0)) {
      int bp = msg.getInt(0);
      if (bp == 0) {
        this->SetSpeed(0);
      } else if (bp > 0) {
        this->SetDirection(DIRECTION_UP);
        this->SetSpeed(bp);
      } else {
        this->SetDirection(DIRECTION_DOWN);
        this->SetSpeed(abs(bp));
      }
    }
  }
}