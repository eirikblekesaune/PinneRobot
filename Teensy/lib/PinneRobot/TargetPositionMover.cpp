#include <TargetPositionMover.h>

TargetPositionMover::TargetPositionMover(address_t *address, PinneComm *comm)
    : _address(address), _comm(comm) {
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
  _maxSpeed = (_distance / (_numTicks - _numSkirtTicks)) - _minSpeed;
  _maxSpeedSegmentNumTicks = _numTicks - (_numSkirtTicks * 2);
  _maxSpeedSegmentStartIndex = _numSkirtTicks;
  _fadeDownSegmentStartIndex = _numSkirtTicks + _maxSpeedSegmentNumTicks;
  _numFadeSegmentSamplesPerTick = static_cast<float>(_numSkirtTicks) /
                                  static_cast<float>(_fadeSegmentBufferSize);
  _CalculateFadeSegmentBuffer();
  _ChangeState(TARGET_POSITION_MOVER_STATE_READY);
}

void TargetPositionMover::PlanMoveByMaxSpeed(position_t startPosition,
                                             position_t targetPosition,
                                             double maxSpeed, double minSpeed,
                                             double beta, double skirtRatio,
                                             int tickDuration) {
  this->_Reset();
  this->_InitMove(startPosition, targetPosition, minSpeed, beta, skirtRatio,
                  tickDuration);
  _maxSpeed = max(maxSpeed, _minSpeed);
  _numTicks = _distance / (_maxSpeed + (_minSpeed * _skirtRatio));
  _numSkirtTicks = _numTicks * _skirtRatio;
  _maxSpeedSegmentNumTicks = _numTicks - _numSkirtTicks;
  _maxSpeedSegmentStartIndex = _numSkirtTicks;
  _fadeDownSegmentStartIndex = _numSkirtTicks + _maxSpeedSegmentNumTicks;
  _numFadeSegmentSamplesPerTick = static_cast<float>(_numSkirtTicks) /
                                  static_cast<float>(_fadeSegmentBufferSize);
  _CalculateFadeSegmentBuffer();
  _ChangeState(TARGET_POSITION_MOVER_STATE_READY);
}

void TargetPositionMover::_CalculateFadeSegmentBuffer() {
  double inc = 1.0 / (_fadeSegmentBufferSize - 1);
  for (size_t i = 0; i < _fadeSegmentBufferSize; i++) {
    double t = inc * i;
    _fadeSegmentBuffer[i] =
        _fadeSegmentBufferSize / (1.0 + pow((t / (1.0 - t)), -_beta));
    _fadeSegmentBuffer[i] =
        _fadeSegmentBuffer[i] / (_maxSpeed - _minSpeed) + _minSpeed;
  }
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
  if (_distance < 0.0) {
    _direction = DIRECTION_UP;
  } else {
    _direction = DIRECTION_DOWN;
  }
  _distance = abs(_distance);
}

// TODO: This method could return a bool upon success or not.
void TargetPositionMover::StartMove() {
  if (_state == TARGET_POSITION_MOVER_STATE_READY) {
    _moveStartTime = millis();
    _previousUpdateTime = _moveStartTime;
    _currentTickIndex = 0;
    _isMoving = true;
    _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT);
    _currentSpeed = _minSpeed;
  }
}

void TargetPositionMover::StopMove() { _isMoving = false; }

void TargetPositionMover::_Reset() {
  _isMoving = false;
  _startPosition = 0;
  _targetPosition = 0;
  _numTicks = 0;
  _duration = 0;
  _currentSpeed = 0.0;
  _minSpeed = 0.0;
  _maxSpeed = 0.0;
  _beta = 0.0;
  _distance = 0.0;
  _maxSpeedSegmentStartIndex = 0;
  _fadeDownSegmentStartIndex = 0;
  _mode = TARGET_POSITION_MODE_UNKNOWN;
  _ChangeState(TARGET_POSITION_MOVER_STATE_NOT_READY);
}

double TargetPositionMover::GetCurrentSpeed() {
  if (_direction == DIRECTION_DOWN) {
    return _currentSpeed;
  } else {
    return -_currentSpeed;
  }
}

bool TargetPositionMover::DidReachTarget() {
  return _state == TARGET_POSITION_MOVER_STATE_REACHED_TARGET;
}

void TargetPositionMover::Update(position_t currentPosition) {
  if (_CheckPositionTargetHit(currentPosition)) {
    _ChangeState(TARGET_POSITION_MOVER_STATE_REACHED_TARGET);
    StopMove();
  } else {
    if (_metro->check() == 1) {
      _currentTickIndex += NumTicksSincePreviousUpdate();
      int idx = static_cast<int>(_currentTickIndex);
      // Update state first in case a transition has happened between updates
      if ((idx < _numSkirtTicks) &&
          (_state == TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT)) {
        // redundant assignment but a bit more readable
        _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT);
      } else if ((idx >= _numSkirtTicks) &&
                 (_state == TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT)) {
        _ChangeState(TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT);
      } else if ((idx >= _fadeDownSegmentStartIndex) &&
                 (_state == TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT)) {
        _ChangeState(TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT);
      } else if ((idx >= _numTicks) &&
                 (_state == TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT)) {
        _ChangeState(
            TARGET_POSITION_MOVER_STATE_FINISHED_BUT_TARGET_NOT_REACHED);
      }

      size_t fadeSegmentBufferIndex;
      switch (_state) {
      case TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT:
        fadeSegmentBufferIndex =
            _numFadeSegmentSamplesPerTick * _currentTickIndex;
        _currentSpeed = _fadeSegmentBuffer[fadeSegmentBufferIndex];
        break;
      case TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT:
        _currentSpeed = _maxSpeed;
        break;
      case TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT:
        fadeSegmentBufferIndex =
            abs(_fadeSegmentBufferSize -
                (_numFadeSegmentSamplesPerTick *
                 (_currentTickIndex - _fadeDownSegmentStartIndex)));
        _currentSpeed = _fadeSegmentBuffer[fadeSegmentBufferIndex];
        break;
      case TARGET_POSITION_MOVER_STATE_FINISHED_BUT_TARGET_NOT_REACHED:
        break;
      case TARGET_POSITION_MOVER_STATE_REACHED_TARGET:
      case TARGET_POSITION_MOVER_STATE_READY:
      case TARGET_POSITION_MOVER_STATE_NOT_READY:
        break;
      }
      _previousUpdateTime = millis();
    }
  }
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
  }
}
