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
  _targetSpeedPIDSetpoint = 0.0;
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
  _targetPositionMover = new TargetPositionMover();
  SetMotorControlMode(CONTROL_MODE_PWM);
  _measuredCurrent = static_cast<float>(analogRead(_currentSensePin));
  _topStopButton = new Bounce(_topStopSensorPin, 5);
  _slackStopButton = new Bounce(_slackStopSensorPin, 200);

  _speedometerInterval = 50;
  _speedometerMetro = new Metro(_speedometerInterval);

  _speedPID =
      new PID_CLASS(&_measuredSpeed, &_targetSpeedPIDOutput,
                    &_targetSpeedPIDSetpoint, 2.0, 1000.0, 1.0, P_ON_M, DIRECT);

  _speedPID->SetOutputLimits(-_driver->SPEED_MAX, _driver->SPEED_MAX);
  _speedPID->SetSampleTime(_speedometerInterval);
  _targetSpeedStopThreshold = 0.1;
  _targetSpeedState = TARGET_SPEED_STOPPED;

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

void PinneMotor::Stop() {
  switch (_motorControlMode) {
  case CONTROL_MODE_PWM:
    this->SetPWM(0);
    break;
  case CONTROL_MODE_TARGET_SPEED:
    this->SetBipolarTargetSpeed(0.0);
    break;
  case CONTROL_MODE_TARGET_POSITION:
    this->SetPWM(0);
    break;
  }
}

void PinneMotor::SetPWM(int speed) {
  if (speed <= 0) {
    _driver->SetPWM(0);
  } else {
    if (!IsBlocked()) {
      _driver->SetPWM(speed);
    }
  }
}

void PinneMotor::SetBipolarPWM(int bp) {
  if (bp == 0) {
    this->SetPWM(0);
  } else if (bp > 0) {
    this->SetDirection(DIRECTION_DOWN);
    this->SetPWM(bp);
  } else {
    this->SetDirection(DIRECTION_UP);
    this->SetPWM(abs(bp));
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

void PinneMotor::Update() {
  ReadTopStopSensor();
  ReadSlackStopSensor();
  CheckPositionLimits();
  _UpdateSpeedometer();
  _UpdateCurrentSense();
  this->UpdateState();
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

void PinneMotor::UpdateState() {
  if (_state < BLOCKED_BY_TOP_SENSOR) {
    if (GetPWM() == 0) {
      _Stopped();
    } else {
      direction_t direction = GetDirection();
      if (direction == DIRECTION_DOWN) {
        _GoingDown();
      } else {
        _GoingUp();
      }
    }
  }
}

void PinneMotor::CheckPositionLimits() {
  position_t currPosition = GetCurrentPosition();
  position_t minPosition = GetMinPosition();
  if ((_state == BLOCKED_BY_TOP_SENSOR) ||
      (_state == BLOCKED_BY_SLACK_SENSOR)) {

  } else {
    if (currPosition < minPosition) {
      _MinPositionReached();
    } else if (currPosition > GetMaxPosition()) {
      _MaxPositionReached();
    };
  }
}

void PinneMotor::_PWMModeUpdate() {
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
  if (IsBlocked()) {
    if (_targetSpeedState != TARGET_SPEED_STOPPED) {
      _targetSpeedState = TARGET_SPEED_STOPPED;
      /* _speedPID->SetMode(MANUAL); */
      this->SetBipolarPWM(0);
    }
  }
  if ((_targetSpeedPIDSetpoint < _targetSpeedStopThreshold) &&
      (_targetSpeedPIDSetpoint > -_targetSpeedStopThreshold)) {
    if (_targetSpeedState != TARGET_SPEED_STOPPED) {
      /* _speedPID->SetMode(MANUAL); */
      this->SetBipolarPWM(0);
      _targetSpeedState = TARGET_SPEED_STOPPED;
    }
  } else {
    if (_targetSpeedPIDSetpoint >= _targetSpeedStopThreshold) {
      if (_targetSpeedState != TARGET_SPEED_GOING_DOWN) {
        _targetSpeedState = TARGET_SPEED_GOING_DOWN;
        /* _speedPID->SetMode(AUTOMATIC); */
      }
    } else {
      if (_targetSpeedState != TARGET_SPEED_GOING_UP) {
        _targetSpeedState = TARGET_SPEED_GOING_UP;
        /* _speedPID->SetMode(AUTOMATIC); */
      }
    }
    /* _speedPID->Compute(); */
    this->SetBipolarPWM(static_cast<int>(_targetSpeedPIDOutput));
  }
  /* static Metro metro(100); */
  /* if (metro.check() == 1) { */
  /*   OSCMessage msg("/PID"); */
  /*   msg.add(_targetSpeedPIDOutput); */
  /*   _comm->SendOSCMessage(msg); */
  /* } */
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

void PinneMotor::_Stopped() {
  if (_state != STOPPED) {
    _state = STOPPED;
    _comm->NotifyStateChange(STOPPED, _address);
  }
}

void PinneMotor::_GoingUp() {
  if (_state != GOING_UP) {
    _state = GOING_UP;
    _comm->NotifyStateChange(GOING_UP, _address);
  }
}

void PinneMotor::_GoingDown() {
  if (_state != GOING_DOWN) {
    _state = GOING_DOWN;
    _comm->NotifyStateChange(GOING_DOWN, _address);
  }
}

void PinneMotor::_AbsMinPositionReached() {
  if (_state != BLOCKED_BY_ABS_MIN_POSITION) {
    Stop();
    _state = BLOCKED_BY_ABS_MIN_POSITION;
    _comm->NotifyStateChange(BLOCKED_BY_ABS_MIN_POSITION, _address);
  }
}

void PinneMotor::_MinPositionReached() {
  if (_state != BLOCKED_BY_MIN_POSITION) {
    Stop();
    _state = BLOCKED_BY_MIN_POSITION;
    _comm->NotifyStateChange(BLOCKED_BY_MIN_POSITION, _address);
  }
}

void PinneMotor::_MaxPositionReached() {
  if (_state != BLOCKED_BY_MAX_POSITION) {
    Stop();
    _state = BLOCKED_BY_MAX_POSITION;
    _comm->NotifyStateChange(BLOCKED_BY_MAX_POSITION, _address);
  }
}

void PinneMotor::_TargetReached() {
    Stop();
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

void PinneMotor::SetBipolarTargetSpeed(float value) {
  _targetSpeedPIDSetpoint = value;
}

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

void PinneMotor::GoToTargetPositionByDuration(int targetPosition, int duration,
                                              double minSpeed, double beta,
                                              double skirtRatio) {
  if (!_targetPositionMover->IsMoving()) {
    position_t currentPosition = GetCurrentPosition();
    _targetPositionMover->PlanMoveByDuration(
        currentPosition, targetPosition, duration, minSpeed, beta, skirtRatio);
    _targetPositionMover->StartMove();
  }
}

void PinneMotor::GoToTargetPositionByMaxSpeed(int targetPosition,
                                              double minSpeed, double maxSpeed,
                                              double beta, double skirtRatio) {
  if (!_targetPositionMover->IsMoving()) {
    position_t currentPosition = GetCurrentPosition();
    _targetPositionMover->PlanMoveByMaxSpeed(
        currentPosition, targetPosition, maxSpeed, minSpeed, beta, skirtRatio);
    _targetPositionMover->StartMove();
  }
}

void PinneMotor::GoToParkingPosition() {
  GoToParkingPosition(VNH5019Driver::SPEED_MAX / 4);
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
    this->Stop();
}

void PinneMotor::_RouteBipolarPWMMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    if (_motorControlMode == CONTROL_MODE_PWM) {
      int bp = msg.getInt(0);
      this->SetBipolarPWM(bp);
    }
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      int bpPWM = GetPWM();
      if (GetDirection() == DIRECTION_UP) {
        bpPWM = -bpPWM;
      }
      OSCMessage replyMsg("/");
      replyMsg.add(GetPWM());
      _comm->ReturnQueryValue(CMD_BIPOLAR_PWM, _address, replyMsg);
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
    this->SetBipolarTargetSpeed(pos);
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      replyMsg.add(GetBipolarTargetSpeed());
      _comm->ReturnQueryValue(CMD_BIPOLAR_TARGET_SPEED, _address, replyMsg);
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
  if (GetMotorControlMode() == CONTROL_MODE_TARGET_POSITION) {
    if (msg.size() >= 2 && (msg.isInt(0))) {
      int targetPosition = msg.getInt(0);
      double minSpeed, beta, skirtRatio;
      if (msg.size() >= 3 && (msg.isFloat(2))) {
        minSpeed = msg.getFloat(2);
      } else {
        minSpeed = _targetSpeedStopThreshold;
      }
      if (msg.size() >= 4 && (msg.isFloat(3))) {
        beta = msg.getFloat(3);
      } else {
        beta = 3.0;
      }
      if (msg.size() >= 5) {
        skirtRatio = msg.getFloat(4);
      } else {
        skirtRatio = 0.1;
      }
      int offset = msg.match("/byDuration", initialOffset);
      if (offset) {
        if (msg.isInt(1)) {
          int duration = msg.getInt(1);
          int beta, minSpeed;
          this->GoToTargetPositionByDuration(targetPosition, duration, minSpeed,
                                             beta, skirtRatio);
        }
      }
      offset = msg.match("/byMaxSpeed", initialOffset);
      if (offset) {
        if (msg.isFloat(1)) {
          double maxSpeed = msg.getFloat(1);
          this->GoToTargetPositionByMaxSpeed(targetPosition, maxSpeed, minSpeed,
                                             beta, skirtRatio);
        }
      }
    }
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

