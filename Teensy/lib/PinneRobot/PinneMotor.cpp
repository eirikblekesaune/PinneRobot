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
  _targetSpeedPIDOutput = 0.0;
  _currentPosition = POSITION_ALL_UP;
  _prevPosition = _currentPosition;
  _measuredSpeed = 0.0;
  _blockingMask = NOTHING_BLOCKS;
}

void PinneMotor::init() {
  pinMode(_encoderInterruptPinA, INPUT_PULLUP);
  pinMode(_encoderInterruptPinB, INPUT_PULLUP);
  _encoder = new Encoder(_encoderInterruptPinB, _encoderInterruptPinA);
  pinMode(_topStopSensorPin, INPUT_PULLUP);
  pinMode(_slackStopSensorPin, INPUT_PULLUP);
  pinMode(_currentSensePin, INPUT);
  _targetPositionMover =
      new TargetPositionMover(&_address, _comm, &_targetSpeedStopThreshold);

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
  SetDirection(DIRECTION_DOWN);
  Stop();
  _blocked = true;
}

position_t PinneMotor::GetCurrentPosition() { return _encoder->read(); };

void PinneMotor::Stop() {
  switch (_motorControlMode) {
  case CONTROL_MODE_PWM:
    this->SetBipolarPWM(0);
    break;
  case CONTROL_MODE_TARGET_SPEED:
    this->SetBipolarTargetSpeed(0);
    break;
  case CONTROL_MODE_TARGET_POSITION:
    _targetPositionMover->StopMove();
    this->SetBipolarTargetSpeed(0);
    break;
  }
}

void PinneMotor::SetPWM(int speed) {
  if (!IsBlocked()) {
    _driver->SetPWM(max(0, speed));
  } else if (speed == 0) {
    _driver->SetPWM(0);
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
    _driver->SetDirection(direction);
  } else {
    _driver->UpdateDirection();
  }
}

bool PinneMotor::IsBlocked() {
  if (_blockingMask > NOTHING_BLOCKS) {
    direction_t direction = GetDirection();
    if ((_blockingMask & (MIN_POSITION_BLOCKS | TOP_SENSOR_BLOCKS)) > 0) {
      if (direction == DIRECTION_UP) {
        return true;
      } else {
        return false;
      }
    } else if ((_blockingMask & (MAX_POSITION_BLOCKS | SLACK_SENSOR_BLOCKS)) >
               0) {
      if (direction == DIRECTION_DOWN) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

// Read all sensor values and check positions
// Stop motor if needed

void PinneMotor::Update() {
  ReadTopStopSensor();
  ReadSlackStopSensor();
  CheckPositionLimits();
  _UpdateSpeedometer();
  _UpdateCurrentSense();
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
  this->UpdateState();
}

void PinneMotor::UpdateState() {
  switch (_state) {
  case STOPPED:
    if (GetPWM() != 0) {
      direction_t direction = GetDirection();
      if (direction == DIRECTION_DOWN) {
        _GoingDown();
      } else {
        _GoingUp();
      }
    }
    break;
  case GOING_DOWN:
    if (GetDirection() == DIRECTION_UP) {
      _GoingUp();
    }
    break;
  case GOING_UP:
    if (GetDirection() == DIRECTION_DOWN) {
      _GoingDown();
    }
    break;
  default:
    break;
  }
}

bool PinneMotor::_CheckSensorBlockingState(blockingMask_t sensorMask) {
  return (_blockingMask & sensorMask) > 1;
}

void PinneMotor::CheckPositionLimits() {
  position_t currPosition = GetCurrentPosition();
  position_t minPosition = GetMinPosition();
  if (_CheckSensorBlockingState(MIN_POSITION_BLOCKS)) {
    if (currPosition > minPosition) {
      _MinPositionLeft();
    }
  } else {
    if (currPosition <= minPosition) {
      _MinPositionReached();
    }
  }
  position_t maxPosition = GetMaxPosition();
  if (_CheckSensorBlockingState(MAX_POSITION_BLOCKS)) {
    if (currPosition < maxPosition) {
      _MaxPositionLeft();
    }
  } else {
    if (currPosition >= maxPosition) {
      _MaxPositionReached();
    }
  }
}

void PinneMotor::_PWMModeUpdate() {
}

void PinneMotor::_TargetPositionModeUpdate() {
  if (_targetPositionMover->IsMoving()) {
    double targetSpeed;
    position_t currentPosition = GetCurrentPosition();
    _targetPositionMover->Update(currentPosition);
    targetSpeed = _targetPositionMover->GetCurrentSpeed();
    if (_targetPositionMover->DidReachTarget()) {
      this->Stop();
      OSCMessage msg("/DidReachTarget");
      _comm->SendOSCMessage(msg);
    } else {
      this->SetBipolarTargetSpeed(targetSpeed);
    }
  }
  this->_TargetSpeedModeUpdate();
}

void PinneMotor::_ActivateTargetSpeedPID() {
  _targetSpeedPIDOutput = _measuredSpeed;
  _speedPID->SetMode(AUTOMATIC);
  _speedPID->Compute();
}

void PinneMotor::_DeactivateTargetSpeedPID() { _speedPID->SetMode(MANUAL); }

void PinneMotor::_TargetSpeedModeUpdate() {
  targetSpeedState_t previousState = _targetSpeedState;
  if ((_targetSpeedPIDSetpoint < _targetSpeedStopThreshold) &&
      (_targetSpeedPIDSetpoint > -_targetSpeedStopThreshold)) {
    _targetSpeedState = TARGET_SPEED_STOPPED;
  } else if (_targetSpeedPIDSetpoint >= _targetSpeedStopThreshold) {
    // going down
    _targetSpeedState = TARGET_SPEED_GOING_DOWN;
  } else if (_targetSpeedPIDSetpoint <= _targetSpeedStopThreshold) {
    // going up
    _targetSpeedState = TARGET_SPEED_GOING_UP;
  }
  if (previousState != _targetSpeedState) {
    switch (_prevPosition) {
    case TARGET_SPEED_STOPPED:
      break;
    case TARGET_SPEED_GOING_DOWN:
    case TARGET_SPEED_GOING_UP:
      break;
    }

    switch (_targetSpeedState) {
    case TARGET_SPEED_STOPPED:
      this->_DeactivateTargetSpeedPID();
      break;
    case TARGET_SPEED_GOING_DOWN:
      this->_ActivateTargetSpeedPID();
      break;
    case TARGET_SPEED_GOING_UP:
      this->_ActivateTargetSpeedPID();
      break;
    }
  }

  _speedPID->Compute();
  switch (_targetSpeedState) {
  case TARGET_SPEED_GOING_DOWN:
    this->SetBipolarPWM(static_cast<int>(_targetSpeedPIDOutput));
    break;
  case TARGET_SPEED_GOING_UP:
    this->SetBipolarPWM(static_cast<int>(_targetSpeedPIDOutput));
    break;
  case TARGET_SPEED_STOPPED:
    this->SetBipolarPWM(0);
    break;
  }
}

void PinneMotor::_UpdateSpeedometer() {
  if (_speedometerMetro->check() == 1) {
    float prevSpeed = _measuredSpeed;
    position_t currentPosition = GetCurrentPosition();
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
    _TopStopSensorIn();
  } else if (_topStopButton->risingEdge()) {
    // pin is pulled HIGH when button is released
    _TopStopSensorOut();
  }
}

void PinneMotor::ReadSlackStopSensor() {
  _slackStopButton->update();
  if (_slackStopButton->fallingEdge()) {
    // pin is pulled HIGH when button is pressed
    _SlackStopSensorOut();
  } else if (_slackStopButton->risingEdge()) {
    // pin is pulled LOW when button is released
    _SlackStopSensorIn();
  }
}

void PinneMotor::_ChangeState(motorState_t state) {
  motorState_t previousState = _state;
  _state = state;
  if (previousState != _state) {
    _comm->NotifyMotorStateChange(_state, _address);
  }
}

void PinneMotor::_TopStopSensorIn() {
  Stop();
  SetCurrentPosition(POSITION_ALL_UP);
  _SetBlockingMaskBit(TOP_SENSOR_BLOCKS);
  _ChangeState(BLOCKED_BY_TOP_SENSOR);
}

void PinneMotor::_TopStopSensorOut() {
  _ClearBlockingMaskBit(TOP_SENSOR_BLOCKS);
}

void PinneMotor::_SlackStopSensorOut() {
  Stop();
  _SetBlockingMaskBit(SLACK_SENSOR_BLOCKS);
  _ChangeState(BLOCKED_BY_SLACK_SENSOR);
}

void PinneMotor::_SlackStopSensorIn() {
  _ClearBlockingMaskBit(SLACK_SENSOR_BLOCKS);
}


void PinneMotor::_Stopped() { _ChangeState(STOPPED); }

void PinneMotor::_GoingUp() { _ChangeState(GOING_UP); }

void PinneMotor::_GoingDown() { _ChangeState(GOING_DOWN); }

void PinneMotor::_MinPositionReached() {
  _SetBlockingMaskBit(MIN_POSITION_BLOCKS);
  _ChangeState(BLOCKED_BY_MIN_POSITION);
  Stop();
}

void PinneMotor::_MinPositionLeft() {
  _ClearBlockingMaskBit(MIN_POSITION_BLOCKS);
}

void PinneMotor::_MaxPositionReached() {
  _SetBlockingMaskBit(MAX_POSITION_BLOCKS);
  _ChangeState(BLOCKED_BY_MAX_POSITION);
  Stop();
}

void PinneMotor::_MaxPositionLeft() {
  _ClearBlockingMaskBit(MAX_POSITION_BLOCKS);
}

void PinneMotor::_SetBlockingMaskBit(blockingMask_t sensorMask) {
  _blockingMask |= sensorMask;
}

void PinneMotor::_ClearBlockingMaskBit(blockingMask_t sensorMask) {
  _blockingMask &= ~sensorMask;
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
  // TODO: implement this
}

void PinneMotor::GoToTargetPositionByDuration(int targetPosition, int duration,
                                              double minSpeed, double beta,
                                              double skirtRatio) {
  OSCMessage a("/GoToTargetPositionByDuration");
  a.add(targetPosition);
  a.add(duration);
  a.add(minSpeed);
  a.add(beta);
  a.add(skirtRatio);
  _comm->SendOSCMessage(a);
  if (!_targetPositionMover->IsMoving()) {
    position_t currentPosition = GetCurrentPosition();
    _targetPositionMover->PlanMoveByDuration(
        currentPosition, targetPosition, duration, minSpeed, beta, skirtRatio);
    if (_targetPositionMover->StartMove()) {
      OSCMessage b("/StartedMove");
      _comm->SendOSCMessage(b);
    } else {
      OSCMessage c("/CouldNotStartMove");
      _comm->SendOSCMessage(c);
    }
  }
}

int PinneMotor::GetBipolarPWM() {
  int result;
  if (GetDirection() == DIRECTION_UP) {
    result = -GetPWM();
  } else {
    result = GetPWM();
  }
  return result;
}

void PinneMotor::GoToTargetPositionByMaxSpeed(int targetPosition,
                                              double minSpeed, double maxSpeed,
                                              double beta, double skirtRatio) {
  /* if (!_targetPositionMover->IsMoving()) { */
  /*   position_t currentPosition = GetCurrentPosition(); */
  /*   _targetPositionMover->PlanMoveByMaxSpeed( */
  /*       currentPosition, targetPosition, maxSpeed, minSpeed, beta,
   * skirtRatio); */
  /*   _targetPositionMover->StartMove(); */
  /* } */
}

void PinneMotor::GoToTargetPositionByConstantSpeed(int targetPosition,
                                                   double speed,
                                                   double minSpeed, double beta,
                                                   double skirtRatio) {
  if (!_targetPositionMover->IsMoving()) {
    position_t currentPosition = GetCurrentPosition();
    _targetPositionMover->PlanMoveByConstantSpeed(
        currentPosition, targetPosition, speed, minSpeed, beta, skirtRatio);
    if (_targetPositionMover->StartMove()) {
      OSCMessage b("/StartedMove");
      _comm->SendOSCMessage(b);
    } else {
      OSCMessage c("/CouldNotStartMove");
      _comm->SendOSCMessage(c);
    }
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
      Stop();
      break;
    case CONTROL_MODE_TARGET_POSITION:
      _DeactivateTargetSpeedPID();
      Stop();
      break;
    case CONTROL_MODE_TARGET_SPEED:
      _DeactivateTargetSpeedPID();
      Stop();
      break;
    }

    switch (_motorControlMode) {
    case CONTROL_MODE_PWM:
      break;
    case CONTROL_MODE_TARGET_POSITION:
      _targetSpeedState = TARGET_SPEED_STOPPED;
      break;
    case CONTROL_MODE_TARGET_SPEED:
      _targetSpeedState = TARGET_SPEED_STOPPED;
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
    this->_RouteMotorStateChangeMsg(msg, offset + initialOffset);
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
  _Stopped();
}

void PinneMotor::_RouteBipolarPWMMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isInt(0))) {
    if (_motorControlMode == CONTROL_MODE_PWM) {
      int bp = msg.getInt(0);
      if (bp == 0) {
        this->Stop();
        _Stopped();
      } else {
        this->SetBipolarPWM(bp);
      }
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

void PinneMotor::_RouteTargetSpeedMsg(OSCMessage &msg, int initialOffset) {
  if ((msg.size() > 0) && (msg.isFloat(0))) {
    if (GetMotorControlMode() == CONTROL_MODE_TARGET_SPEED) {
      float targetSpeed = msg.getFloat(0);
      if ((targetSpeed < _targetSpeedStopThreshold) &&
          (targetSpeed > -_targetSpeedStopThreshold)) {
        this->Stop();
        _Stopped();
      } else {
        this->SetBipolarTargetSpeed(targetSpeed);
      }
    }
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

void PinneMotor::_RouteMotorStateChangeMsg(OSCMessage &msg, int initialOffset) {
  if (_comm->HasQueryAddress(msg, initialOffset)) {
    OSCMessage replyMsg("/");
    String stateStr = String(MotorStateChangeMap.at(_state));
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
      if (msg.size() >= 5 && (msg.isFloat(4))) {
        skirtRatio = msg.getFloat(4);
      } else {
        skirtRatio = 0.1;
      }
      int offset = msg.match("/byDuration", initialOffset);
      if (offset) {
        if (msg.isInt(1)) {
          int duration = msg.getInt(1);
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

      offset = msg.match("/byConstantSpeed", initialOffset);
      if (offset) {
        if (msg.isFloat(1)) {
          double speed = msg.getFloat(1);
          this->GoToTargetPositionByConstantSpeed(targetPosition, speed,
                                                  minSpeed, beta, skirtRatio);
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
  } else if (msg.size() == 4) {
    pidvalue_t kp, ki, kd;
    int pOnE;
    kp = msg.getFloat(0);
    ki = msg.getFloat(1);
    kd = msg.getFloat(2);
    pOnE = msg.getInt(3);
    _speedPID->SetTunings(kp, ki, kd, pOnE);
  } else if (_comm->HasQueryAddress(msg, initialOffset)) {
    OSCMessage replyMsg("/");
    replyMsg.add(static_cast<float>(_speedPID->GetKp()));
    replyMsg.add(static_cast<float>(_speedPID->GetKi()));
    replyMsg.add(static_cast<float>(_speedPID->GetKd()));
    _comm->ReturnQueryValue(CMD_PID_PARAMETERS, _address, replyMsg);
  }
}

