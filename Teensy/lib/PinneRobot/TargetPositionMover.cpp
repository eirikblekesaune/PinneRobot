#include <TargetPositionMover.h>

TargetPositionMover::TargetPositionMover(address_t *address, PinneComm *comm,
                                         float *stopSpeedThreshold)
    : _address(address), _comm(comm), _stopSpeedThreshold(stopSpeedThreshold) {
  _metro = new Metro(_tickDuration);
  for (size_t i = 0; i < _fadeSegmentBufferSize; i++) {
    _fadeSegmentBuffer[i] = 0.0;
  }
  /* _maxSpeedPID = */
  /*     new PID(&_estimatedRemainingDistance, &_maxSpeedAdjusted, */
  /*             &_expectedRemainingDistance, 13.0, 10.0, 1.0, P_ON_M, DIRECT);
   */
  _maxSpeedPID =
      new PID(&_distanceTravelled, &_maxSpeedAdjusted,
              &_plannedDistanceTravelled, 2.0, 100.0, 1.0, P_ON_E, DIRECT);
  _maxSpeedPID->SetOutputLimits(
      0.0, 2.0); // twice the max speed is upper adjustment limit
  this->_Reset();
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
  _mode = TARGET_POSITION_MODE_BY_DURATION;
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
  _mode = TARGET_POSITION_MODE_BY_MAX_SPEED;
  _FinalizeMovePlan();
}

void TargetPositionMover::PlanMoveByConstantSpeed(
    position_t startPosition, position_t targetPosition, double speed,
    double minSpeed, double beta, double skirtRatio, int tickDuration) {

  this->_Reset();
  _maxSpeedPlanned = speed;
  _startPosition = max(0, startPosition);
  _targetPosition = max(0, targetPosition);
  _tickDuration = max(1, tickDuration);
  _minSpeed = abs(minSpeed);
  _metro->interval(_tickDuration);
  _skirtRatio = constrain(skirtRatio, 0.0, 0.5);
  _beta = beta;
  _distance = _targetPosition - _startPosition;
  if (_distance < 0.0) {
    _direction = DIRECTION_UP;
  } else {
    _direction = DIRECTION_DOWN;
  }
  _distance = abs(_distance);
  _numFadeInTicks =
      static_cast<int>(static_cast<double>(_distance) * skirtRatio);
  _numFadeOutTicks = _numFadeInTicks;
  _mode = TARGET_POSITION_MODE_BY_CONSTANT_SPEED;
  _ChangeState(TARGET_POSITION_MOVER_STATE_READY);
}

void TargetPositionMover::_FinalizeMovePlan() {
  _maxSpeedSegmentStartIndex = _numSkirtTicks;
  _fadeDownSegmentStartIndex = _numSkirtTicks + _maxSpeedSegmentNumTicks;
  _numFadeSegmentSamplesPerTick = static_cast<float>(_fadeSegmentBufferSize) /
                                  static_cast<float>(_numSkirtTicks);
  _CalculateFadeSegmentBuffer();

  _state = TARGET_POSITION_MOVER_STATE_READY;
  _ChangeState(TARGET_POSITION_MOVER_STATE_READY);
}

double TargetPositionMover::_GetSCurveValue(double t, double beta) {
  return 1.0 / (1.0 + pow((t / (1.0 - t)), -beta));
}

void TargetPositionMover::_CalculateFadeSegmentBuffer() {
  double inc = 1.0 / (_fadeSegmentBufferSize - 1);
  for (size_t i = 0; i < _fadeSegmentBufferSize; i++) {
    double t = inc * i;
    /* double val = 1.0 / (1.0 + pow((t / (1.0 - t)), -_beta)); */
    double val = _GetSCurveValue(t, _beta);
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

void TargetPositionMover::StopMove() { this->_Reset(); }

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
  _plannedDistanceTravelled = 0.0;
  _travelError = 0.0;
  _maxSpeedSegmentStartIndex = 0;
  _maxSpeedSegmentNumTicksAdjustment = 0.0;
  _maxSpeedSegmentNumTicks = 0;
  _fadeDownSegmentStartIndex = 0;
  _maxSpeedPID->SetMode(MANUAL);
  _mode = TARGET_POSITION_MODE_UNKNOWN;
  _ChangeState(TARGET_POSITION_MOVER_STATE_NOT_READY);
}

double TargetPositionMover::GetCurrentSpeed() {
  double speed = _MapSpeedValue(_currentSpeedUnmapped, GetAdjustedMaxSpeed());
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
        _currentTickIndex += NumTicksSincePreviousUpdate();
        int idx = static_cast<int>(_currentTickIndex);
        if (_mode == TARGET_POSITION_MODE_BY_CONSTANT_SPEED) {
          int distanceToTarget = abs(_targetPosition - currentPosition);
          int distanceFromStart = abs(currentPosition - _startPosition);
          if (distanceFromStart < _numFadeInTicks) {
            double increment = 1.0 / static_cast<double>(_numFadeInTicks);
            double v = max(static_cast<double>(_minSpeed),
                           static_cast<double>(distanceFromStart) * increment);
            v = _GetSCurveValue(v, _beta);
            _currentSpeedUnmapped = v;
            /* OSCMessage aaa("/FADE_IN"); */
            /* aaa.add(_currentSpeedUnmapped); */
            /* aaa.add(distanceToTarget); */
            /* aaa.add(distanceFromStart); */
            /* _comm->SendOSCMessage(aaa); */
          } else if (distanceToTarget < _numFadeOutTicks) {
            double decrement = 1.0 / static_cast<double>(_numFadeOutTicks);
            double v = max(static_cast<double>(_minSpeed),
                           static_cast<double>(distanceToTarget) * decrement);
            v = _GetSCurveValue(v, _beta);
            _currentSpeedUnmapped = v;
            /* OSCMessage aaa("/FADE_OUT"); */
            /* aaa.add(_currentSpeedUnmapped); */
            /* aaa.add(distanceToTarget); */
            /* aaa.add(distanceFromStart); */
            /* _comm->SendOSCMessage(aaa); */
          } else {
            _currentSpeedUnmapped = 1.0;
            /* OSCMessage aaa("/MAX_SPEED"); */
            /* aaa.add(_currentSpeedUnmapped); */
            /* aaa.add(distanceToTarget); */
            /* aaa.add(distanceFromStart); */
            /* _comm->SendOSCMessage(aaa); */
          }
        } else {
          // Update state first in case a transition has happened between
          // updates
          if ((idx < _numSkirtTicks) &&
              (_state == TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT)) {
            // redundant assignment but a bit more readable
            _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT);
          } else if ((idx >= _numSkirtTicks) &&
                     (_state == TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT)) {
            _ChangeState(TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT);
            if (_mode == TARGET_POSITION_MODE_BY_DURATION) {
              _maxSpeedPID->SetSampleTime(_tickDuration);
              _maxSpeedPID->SetMode(AUTOMATIC);
              _maxSpeedPID->Compute();
            }
          } else if ((idx >= (_fadeDownSegmentStartIndex +
                              _maxSpeedSegmentNumTicksAdjustment)) &&
                     (_state ==
                      TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT)) {
            _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT);
          } else if ((idx >= _numTicks) &&
                     (_state == TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT)) {
            _ChangeState(
                TARGET_POSITION_MOVER_STATE_FINISHED_BUT_TARGET_NOT_REACHED);
          }

          size_t fadeSegmentBufferIndex;
          // This is how far we actually have travelld at this moment
          _distanceTravelled = (currentPosition - _startPosition);
          // How far we should have travelled
          _plannedDistanceTravelled += GetCurrentSpeed();
          /* _travelError = _plannedDistanceTravelled - _distanceTravelled; */
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
              // foresee that the travel distance will be.
              //
              // The planned travel distance may differ from this, that is what
              // we call an error that we will adjust for, by adding ticks to
              // the max speed segment.
              //
              // This is what the clockwork of ticks has left to churn out.
              double remainingTicksToTarget =
                  _numTicks -
                  (_currentTickIndex + _maxSpeedSegmentNumTicksAdjustment);

              // An estimate of how long distance will be travelled by the
              // current clockwork state.
              double estimatedRemainingDistance =
                  _EstimateRemainingDistance(remainingTicksToTarget);

              // What the mover has planned to have travelled a said distance by
              // now. This is what is expected to be the remaining distance.
              double expectedRemainingDistance =
                  _distance - _plannedDistanceTravelled;

              // The difference between these is the error that needs correction
              _travelError =
                  expectedRemainingDistance - estimatedRemainingDistance;
              double correction = _travelError / GetMaxSpeed();
              if (correction < 0.0) {
                _maxSpeedSegmentNumTicksAdjustment =
                    static_cast<int>(_travelError);
              } else {
                _maxSpeedSegmentNumTicksAdjustment =
                    static_cast<int>(_travelError) + 1;
              }
            } else if (_mode == TARGET_POSITION_MODE_BY_DURATION) {
              _maxSpeedPID->Compute();
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
        }

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

double
TargetPositionMover::_EstimateRemainingDistance(double remainingTicksToTarget) {
  return (remainingTicksToTarget * GetMaxSpeed()) - _GetSkirtSum();
}

double TargetPositionMover::_GetSkirtSum() {
  double result = (_numSkirtTicks / 2.0) * (1.0 + _minSpeed);
  return result;
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
