#include <TargetPositionMover.h>

TargetPositionMover::TargetPositionMover() { this->_Reset(); }

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
  _maxSpeedNumTicks = _numTicks - (_numSkirtTicks * 2);
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
  _maxSpeedNumTicks = _numTicks - _numSkirtTicks;
}

void TargetPositionMover::_InitMove(position_t startPosition,
                                    position_t targetPosition, double minSpeed,
                                    double beta, double skirtRatio,
                                    int tickDuration) {
  _startPosition = max(0, startPosition);
  _targetPosition = max(0, targetPosition);
  _minSpeed = minSpeed;
  _beta = max(1.0, beta);
  _skirtRatio = constrain(skirtRatio, 0.0, 1.0);
  _tickDuration = max(1, tickDuration);
  _distance = _targetPosition - _startPosition;
  if (_distance < 0.0) {
    _direction = DIRECTION_UP;
  } else {
    _direction = DIRECTION_DOWN;
  }
  _distance = abs(_distance);
}

void TargetPositionMover::StartMove() { _isMoving = true; }

void TargetPositionMover::StopMove() { _isMoving = false; }

void TargetPositionMover::_Reset() {
  _isMoving = false;
  _startPosition = 0;
  _targetPosition = 0;
  _numTicks = 0;
  _duration = 0;
  _tickDuration = 0;
  _currentSpeedValue = 0.0;
  _minSpeed = 0.0;
  _maxSpeed = 0.0;
  _beta = 0.0;
  _distance = 0.0;
}

bool TargetPositionMover::GetNextSpeedValue(double *val) {
  if (true) {
    *val = 1.0;
    return true;
  } else {
    return false;
  }
}
