#include "PinneMotor.h"


PinneMotor::PinneMotor(int topStopSensorPin, int slackStopSensorPin,
                       int encoderInterruptPinA, int encoderInterruptPinB,
                       VNH5019Driver *driver, address_t address,
                       PinneComm *comm)
    : _topStopSensorPin(topStopSensorPin),
      _slackStopSensorPin(slackStopSensorPin),
      _encoderInterruptPinA(encoderInterruptPinA),
      _encoderInterruptPinB(encoderInterruptPinB), _driver(driver),
      _address(address), _comm(comm) {
  _minPosition = POSITION_ALL_UP;
  _maxPosition = POSITION_DEFAULT_MAX;
  _targetPosition = TARGET_NONE;
  _currentPosition = POSITION_ALL_UP;
}

void PinneMotor::init() {
  _encoder = new Encoder(_encoderInterruptPinA, _encoderInterruptPinB);
  pinMode(_topStopSensorPin, INPUT);
  pinMode(_slackStopSensorPin, INPUT);
  _driver->init();
  _driver->SetDirection(DIRECTION_UP);
  SetDirection(DIRECTION_DOWN);
  Stop();
  _blocked = false;
  _topStopSensorValue = digitalRead(_topStopSensorPin);
  if (_topStopSensorValue == TOP_SENSOR_IN)
    _TopStopSensorIn();
  _slackStopSensorValue = digitalRead(_slackStopSensorPin);
  if (_slackStopSensorValue == SLACK_SENSOR_OUT)
    _SlackStopSensorOut();

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
  Serial.println("SetSpeed");
  if (speed <= 0) {
    Stop();
  } else {
    if (!IsBlocked()) {
      _driver->SetSpeed(speed);
    }
  }
}

int PinneMotor::GetMeasuredSpeed() { return 123; }

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

boolean PinneMotor::IsBlocked() {
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
  // ReadSlackStopSensor();//not using the slack sensor anymore
  if ((_state == BLOCKED_BY_TOP_SENSOR) ||
      (_state == BLOCKED_BY_SLACK_SENSOR)) { // sensors have priority over
                                             // position lilmits

  } else {
    position_t currPosition = _encoder->read();
    position_t minPosition = GetMinPosition();
    if (currPosition < minPosition) {
      _MinPositionReached();
    } else if (currPosition > GetMaxPosition()) {
      _MaxPositionReached();
    } else {
      direction_t direction = GetDirection();
      if (direction == DIRECTION_DOWN) {
        if ((_targetPosition != TARGET_NONE) &&
            (currPosition > _targetPosition)) {
          _TargetReached();
        } else {
          if (_state == GOING_TO_TARGET) {
            _UpdateSpeedRamp();
          } else {
            _GoingDown();
          }
        }
      } else {
        if ((_targetPosition != TARGET_NONE) &&
            (currPosition < _targetPosition)) {
          _TargetReached();
        } else {
          if (_state == GOING_TO_TARGET) {
            _UpdateSpeedRamp();
          } else {
            _GoingUp();
          }
        }
      }
    }
  }
}

void PinneMotor::_UpdateSpeedRamp() {
  if (_speedRamper->Calculate(GetCurrentPosition())) {
    int newSpeed;
    newSpeed = _speedRamper->GetCurrentValue();
    SetSpeed(newSpeed);
  }
}

void PinneMotor::ReadTopStopSensor() {
  int newReading;
  newReading = digitalRead(_topStopSensorPin);
  if (newReading != _lastTopSensorReading) {
    _lastTopSensorReadTime = millis();
  }
  if ((millis() - _lastTopSensorReadTime) > topSensorDebounceDelay) {
    if (newReading != _topStopSensorValue) {
      _topStopSensorValue = newReading;
      if (_topStopSensorValue == TOP_SENSOR_IN) {
        _TopStopSensorIn();
      } else if (_topStopSensorValue == TOP_SENSOR_OUT) {
        _TopStopSensorOut();
      }
    }
  }
  _lastTopSensorReading = newReading;
}

void PinneMotor::ReadSlackStopSensor() {
  int newReading;
  newReading = digitalRead(_slackStopSensorPin);
  if (newReading != _lastSlackSensorReading) {
    _lastSlackSensorReadTime = millis();
  }
  if ((millis() - _lastSlackSensorReadTime) > slackSensorDebounceDelay) {
    if (newReading != _slackStopSensorValue) {
      _slackStopSensorValue = newReading;
      if (_slackStopSensorValue == SLACK_SENSOR_IN) {
        _SlackStopSensorIn();
      } else if (_topStopSensorValue == SLACK_SENSOR_OUT) {
        _SlackStopSensorOut();
      }
    }
  }
  _lastSlackSensorReading = newReading;
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
  _maxPosition = min(maxPosition, POSITION_DEFAULT_MAX);
}

void PinneMotor::GoToParkingPosition() {
  // Check if already at top and do parking only if the sensor is out
  _topStopSensorValue = digitalRead(_topStopSensorPin);
  if (_topStopSensorValue == TOP_SENSOR_OUT) {
    // fake the currentPosition to default max
    SetCurrentPosition(POSITION_DEFAULT_MAX);
    SetDirection(DIRECTION_UP);
    SetSpeed(VNH5019Driver::SPEED_MAX / 4);
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

bool PinneMotor::routeOSC(OSCMessage &msg, int initialOffset) {
  Serial.println("got inner OSC motor msg");
  int offset;
  offset = msg.match("/speed", initialOffset);
  if (offset) {
    Serial.println("/speed routed");
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
  offset = msg.match("/info", initialOffset);
  if (offset) {
    this->_RouteInfoMsg(msg, offset + initialOffset);
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
  Serial.println("Direction routing");
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
  if ((msg.size() > 0) && (msg.isInt(1))) {
    int pos = msg.getInt(1);
    this->SetTargetPosition(pos);
  }
}
void PinneMotor::_RouteCurrentPositionMsg(OSCMessage &msg, int initialOffset) {}
void PinneMotor::_RouteBrakeMsg(OSCMessage &msg, int initialOffset) {}
void PinneMotor::_RouteStateChangeMsg(OSCMessage &msg, int initialOffset) {}
void PinneMotor::_RouteInfoMsg(OSCMessage &msg, int initialOffset) {}
void PinneMotor::_RouteMinPositionMsg(OSCMessage &msg, int initialOffset) {}
void PinneMotor::_RouteMaxPositionMsg(OSCMessage &msg, int initialOffset) {}
void PinneMotor::_RouteGoToParkingPositionMsg(OSCMessage &msg,
                                              int initialOffset) {}
void PinneMotor::_RouteGoToTargetPositionMsg(OSCMessage &msg,
                                             int initialOffset) {}
void PinneMotor::_RouteMeasuredSpeedMsg(OSCMessage &msg, int initialOffset) {}
void PinneMotor::_RouteGoToSpeedRampDownMsg(OSCMessage &msg,
                                            int initialOffset) {}
void PinneMotor::_RouteGoToSpeedScalingMsg(OSCMessage &msg, int initialOffset) {

}
void PinneMotor::_RouteEchoMessagesMsg(OSCMessage &msg, int initialOffset) {}
