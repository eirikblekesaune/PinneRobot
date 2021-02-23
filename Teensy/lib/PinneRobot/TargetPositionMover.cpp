#include <TargetPositionMover.h>

TargetPositionMover::TargetPositionMover() {
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

void TargetPositionMover::StartMove() {
  _moveStartTime = millis();
  _previousUpdateTime = _moveStartTime;
  _currentTickIndex = 0;
  _isMoving = true;
  _state = TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT;
  _currentSpeedValue = _minSpeed;
}

void TargetPositionMover::StopMove() { _isMoving = false; }

void TargetPositionMover::_Reset() {
  _isMoving = false;
  _startPosition = 0;
  _targetPosition = 0;
  _numTicks = 0;
  _duration = 0;
  _currentSpeedValue = 0.0;
  _minSpeed = 0.0;
  _maxSpeed = 0.0;
  _beta = 0.0;
  _distance = 0.0;
  _maxSpeedSegmentStartIndex = 0;
  _fadeDownSegmentStartIndex = 0;
  _mode = TARGET_POSITION_MODE_UNKNOWN;
  _state = TARGET_POSITION_MOVER_STATE_READY;
}

double TargetPositionMover::GetNextSpeedValue() { return _currentSpeedValue; }

void TargetPositionMover::Update(position_t currentPosition) {
  if (_CheckPositionTargetHit(currentPosition)) {
    _state = TARGET_POSITION_MOVER_STATE_REACHED_TARGET;
    StopMove();
  } else {
    _currentTickIndex += NumTicksSincePreviousUpdate();
    size_t fadeSegmentBufferIndex;
    switch (_state) {
    case TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT:
      fadeSegmentBufferIndex =
          _numFadeSegmentSamplesPerTick * _fadeSegmentBufferSize;
      _currentSpeedValue = _fadeSegmentBuffer[fadeSegmentBufferIndex];
      break;
    case TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT:
      _currentSpeedValue = _maxSpeed;
      break;
    case TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT:
      fadeSegmentBufferIndex =
          abs(_fadeSegmentBufferSize -
              (_numFadeSegmentSamplesPerTick * _fadeSegmentBufferSize));
      _currentSpeedValue = _fadeSegmentBuffer[fadeSegmentBufferIndex];
      break;
    case TARGET_POSITION_MOVER_STATE_REACHED_TARGET:
      this->_Reset();
    case TARGET_POSITION_MOVER_STATE_READY:
      break;
    }
    _previousUpdateTime = millis();
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
