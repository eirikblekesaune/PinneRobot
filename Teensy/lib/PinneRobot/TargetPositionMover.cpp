#include <TargetPositionMover.h>

TargetPositionMover::TargetPositionMover(address_t *address, PinneComm *comm,
                                         float *stopSpeedThreshold)
    : _address(address), _comm(comm), _stopSpeedThreshold(stopSpeedThreshold) {
  this->_Reset();
  _metro = new Metro(_tickDuration);
  for (size_t i = 0; i < _fadeSegmentBufferSize; i++) {
    _fadeSegmentBuffer[i] = 0.0;
  }
}

void TargetPositionMover::PlanMoveByDuration(position_t startPosition,
                                             position_t targetPosition,
                                             int duration, double minSpeed,
                                             double beta, double skirtRatio,
                                             int tickDuration) {

  this->_Reset();
  this->_InitMove(startPosition, targetPosition, minSpeed, beta, skirtRatio,
                  tickDuration);
  _duration = max(_tickDuration, duration);
  _numTicks = _duration / _tickDuration;
  _skirtSegmentDuration = _duration * _skirtRatio;
  _numSkirtTicks = _skirtSegmentDuration / _tickDuration;
  _maxSpeedPlanned = (_distance / (_numTicks - _numSkirtTicks)) - _minSpeed;
  _maxSpeedSegmentNumTicks = _numTicks - (_numSkirtTicks * 2);
  _FinalizeMovePlan();
}

void TargetPositionMover::PlanMoveByMaxSpeed(position_t startPosition,
                                             position_t targetPosition,
                                             double maxSpeed, double minSpeed,
                                             double beta, double skirtRatio,
                                             int tickDuration) {
  this->_Reset();
  this->_InitMove(startPosition, targetPosition, minSpeed, beta, skirtRatio,
                  tickDuration);
  _maxSpeedPlanned = max(maxSpeed, _minSpeed);
  _numTicks = _distance / (_maxSpeedPlanned + (_minSpeed * _skirtRatio));
  _numSkirtTicks = _numTicks * _skirtRatio;
  _maxSpeedSegmentNumTicks = _numTicks - _numSkirtTicks;
  _FinalizeMovePlan();
}

void TargetPositionMover::_FinalizeMovePlan() {
  _maxSpeedSegmentStartIndex = _numSkirtTicks;
  _fadeDownSegmentStartIndex = _numSkirtTicks + _maxSpeedSegmentNumTicks;
  _numFadeSegmentSamplesPerTick = static_cast<float>(_fadeSegmentBufferSize) /
                                  static_cast<float>(_numSkirtTicks);
  _CalculateFadeSegmentBuffer();
  _ChangeState(TARGET_POSITION_MOVER_STATE_READY);
}

void TargetPositionMover::_CalculateFadeSegmentBuffer() {
  double inc = 1.0 / (_fadeSegmentBufferSize - 1);
  for (size_t i = 0; i < _fadeSegmentBufferSize; i++) {
    double t = inc * i;
    double val = 1.0 / (1.0 + pow((t / (1.0 - t)), -_beta));
    /* val = val / _fadeSegmentBufferSize * (_maxSpeedPlanned - _minSpeed) + */
    /* _minSpeed; */
    _fadeSegmentBuffer[i] = val;
  }
  /* for (size_t i = 0; i < 4; i++) { */
  /*   OSCMessage msg("/scurve"); */
  /*   size_t k = _fadeSegmentBufferSize / 4; */
  /*   for (size_t j = 0; j < k; j++) { */
  /*     float v = static_cast<float>(_fadeSegmentBuffer[j + (i * k)]); */
  /*     msg.add(v); */
  /*   } */
  /*   _comm->SendOSCMessage(msg); */
  /* } */
}

void TargetPositionMover::_InitMove(position_t startPosition,
                                    position_t targetPosition, double minSpeed,
                                    double beta, double skirtRatio,
                                    int tickDuration) {
  _startPosition = max(0, startPosition);
  _targetPosition = max(0, targetPosition);
  _minSpeed = abs(minSpeed);
  _beta = max(1.0, beta);
  _skirtRatio = constrain(skirtRatio, 0.0, 0.5);
  _tickDuration = max(1, tickDuration);
  _metro->interval(_tickDuration);
  _distance = _targetPosition - _startPosition;
  OSCMessage msg("/direction");
  if (_distance < 0.0) {
    _direction = DIRECTION_UP;
    msg.add("going_up");
  } else {
    _direction = DIRECTION_DOWN;
    msg.add("going_down");
  }
  _distance = abs(_distance);
  msg.add(_distance);
  _comm->SendOSCMessage(msg);
}

bool TargetPositionMover::StartMove() {
  if (_state == TARGET_POSITION_MOVER_STATE_READY) {
    _moveStartTime = millis();
    _previousUpdateTime = _moveStartTime;
    _currentTickIndex = 0;
    _isMoving = true;
    _currentSpeedUnmapped = _minSpeed;
    _maxSpeedAdjusted = _maxSpeedPlanned;
    _metro->reset();
    _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT);
    return true;
  } else {
    return false;
  }
}

void TargetPositionMover::StopMove() { _isMoving = false; }

void TargetPositionMover::_Reset() {
  _isMoving = false;
  _startPosition = 0;
  _targetPosition = 0;
  _numTicks = 0;
  _duration = 0;
  _currentSpeedUnmapped = 0.0;
  _minSpeed = 0.0;
  _maxSpeedPlanned = 0.0;
  _maxSpeedAdjusted = 0.0;
  _beta = 0.0;
  _distance = 0.0;
  _expectedDistanceTravelled = 0.0;
  _travelError = 0.0;
  _maxSpeedSegmentStartIndex = 0;
  _maxSpeedSegmentNumTicksAdjustment = 0.0;
  _maxSpeedSegmentNumTicks = 0;
  _fadeDownSegmentStartIndex = 0;
  _mode = TARGET_POSITION_MODE_UNKNOWN;
  _ChangeState(TARGET_POSITION_MOVER_STATE_NOT_READY);
}

double TargetPositionMover::GetCurrentSpeed() {
  double speed = _MapSpeedValue(_currentSpeedUnmapped, _maxSpeedAdjusted);
  if (_direction == DIRECTION_DOWN) {
    return speed;
  } else {
    return -speed;
  }
}

bool TargetPositionMover::DidReachTarget() {
  return _state == TARGET_POSITION_MOVER_STATE_REACHED_TARGET;
}

void TargetPositionMover::Update(position_t currentPosition) {
  if (IsMoving()) {
    if (_CheckPositionTargetHit(currentPosition)) {
      _ChangeState(TARGET_POSITION_MOVER_STATE_REACHED_TARGET);
      StopMove();
    } else {
      if (_metro->check() == 1) {
        /* OSCMessage msg("/UpdateMetroLoop"); */
        _currentTickIndex += NumTicksSincePreviousUpdate();
        /* msg.add(_currentTickIndex); */
        int idx = static_cast<int>(_currentTickIndex);
        // Update state first in case a transition has happened between updates
        if ((idx < _numSkirtTicks) &&
            (_state == TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT)) {
          // redundant assignment but a bit more readable
          _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT);
        } else if ((idx >= _numSkirtTicks) &&
                   (_state == TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT)) {
          _ChangeState(TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT);
        } else if ((idx >= (_fadeDownSegmentStartIndex +
                            _maxSpeedSegmentNumTicksAdjustment)) &&
                   (_state == TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT)) {
          _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT);
        } else if ((idx >= _numTicks) &&
                   (_state == TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT)) {
          _ChangeState(
              TARGET_POSITION_MOVER_STATE_FINISHED_BUT_TARGET_NOT_REACHED);
        }

        size_t fadeSegmentBufferIndex;
        _distanceTravelled = (currentPosition - _startPosition);
        _expectedDistanceTravelled += GetCurrentSpeed();
        _travelError = _expectedDistanceTravelled - _distanceTravelled;
        switch (_state) {
        case TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT:
          fadeSegmentBufferIndex =
              _numFadeSegmentSamplesPerTick * _currentTickIndex;
          _currentSpeedUnmapped = _fadeSegmentBuffer[fadeSegmentBufferIndex];
          break;
        case TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT:
          _currentSpeedUnmapped = 1.0;
          if (_mode == TARGET_POSITION_MODE_BY_MAX_SPEED) {
            // Will we reach the target with the current numTicks?
            // Take the sum of the remaining maxSpeed ticks and add it
            // to the sum of the the fade out segment values.
            // The estimated travel distance is what we at this moment can
            // foresee the travel distance will be.
            double estimatedNumTicksToTarget = _EstimateRemainingNumTicks(
                abs(numTicks - (_numTicks - _currentTickIndex)));
            double plannedNumTicksToTarget;
            // The planned travel distance may differ from this, that is what
            // we call an error that we will adjust for, by adding ticks to the
            // max speed segment.
          }
          break;
        case TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT:
          fadeSegmentBufferIndex =
              abs((_fadeSegmentBufferSize - 1) -
                  (_numFadeSegmentSamplesPerTick *
                   (_currentTickIndex - _fadeDownSegmentStartIndex)));
          _currentSpeedUnmapped = _fadeSegmentBuffer[fadeSegmentBufferIndex];
          break;
        case TARGET_POSITION_MOVER_STATE_FINISHED_BUT_TARGET_NOT_REACHED:
          _currentSpeedUnmapped =
              this->_PerformLastStretchStrategy(currentPosition);
          break;
        case TARGET_POSITION_MOVER_STATE_REACHED_TARGET:
        case TARGET_POSITION_MOVER_STATE_READY:
        case TARGET_POSITION_MOVER_STATE_NOT_READY:
          break;
        }
        msg.empty();

        /* _comm->SendOSCMessage(msg); */
        _previousUpdateTime = millis();
      }
    }
  }
}

double TargetPositionMover::_UnmapSpeedValue(double speed, double maxSpeed) {
  return (speed - _minSpeed) / (maxSpeed - _minSpeed);
}

double TargetPositionMover::_MapSpeedValue(double speed, double maxSpeed) {
  return (speed * (maxSpeed - _minSpeed)) + _minSpeed;
}

double
TargetPositionMover::_PerformLastStretchStrategy(double currentPosition) {
  double speed = 0.0;
  switch (_mode) {
  case TARGET_POSITION_MODE_BY_DURATION:
    speed = *_stopSpeedThreshold;
    break;
  case TARGET_POSITION_MODE_BY_MAX_SPEED:
    speed = *_stopSpeedThreshold;
    break;
  default:
    break;
  }
  return speed;
}

double TargetPositionMover::_GetFadeSegmentValue(size_t tickIndex) {
  size_t fadeSegmentBufferIndex =
      _numFadeSegmentSamplesPerTick * _fadeSegmentBufferSize;
  return _fadeSegmentBuffer[fadeSegmentBufferIndex];
}

size_t TargetPositionMover::NumTicksSincePreviousUpdate() {
  return TimeSincePreviousUpdate() / GetTickDuration();
}

unsigned long TargetPositionMover::TimeSincePreviousUpdate() {
  unsigned long now = millis();
  return now - _previousUpdateTime;
}

bool TargetPositionMover::_CheckPositionTargetHit(position_t currentPosition) {
  /* OSCMessage msg("/UpdateReachedTarget"); */
  /* msg.add(currentPosition); */
  /* msg.add(_targetPosition); */
  /* _comm->SendOSCMessage(msg); */
  if (_direction == DIRECTION_DOWN) {
    if (currentPosition >= _targetPosition) {
      return true;
    }
  } else if (_direction == DIRECTION_UP) {
    if (currentPosition <= _targetPosition) {
      return true;
    }
  }
  return false;
}

void TargetPositionMover::_ChangeState(targetPositionMoverState_t state) {
  if (_state != state) {
    _state = state;
    _comm->NotifyTargetPositionMoverStateChange(_state, *_address);
  }
}
