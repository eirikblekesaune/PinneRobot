#ifndef TARGET_POSITION_MOVER_H
#define TARGET_POSITION_MOVER_H
#include <Arduino.h>
#include <PinneComm.h>
#include <PinneMotor.h>
#include <PinneRobot.h>

typedef int position_t;
enum direction_t : uint8_t;

enum targetPositionMode_t : uint8_t {
  TARGET_POSITION_MODE_BY_DURATION,
  TARGET_POSITION_MODE_BY_MAX_SPEED,
  TARGET_POSITION_MODE_UNKNOWN
};

class TargetPositionMover {
public:
  TargetPositionMover();
  void PlanMoveByDuration(position_t startPosition, position_t targetPosition,
                          int duration, double minSpeed, double beta,
                          double skirtRatio) {
    this->PlanMoveByDuration(startPosition, targetPosition, duration, minSpeed,
                             beta, skirtRatio, 50);
  };
  void PlanMoveByMaxSpeed(position_t startPosition, position_t targetPosition,
                          double maxSpeed, double minSpeed, double beta,
                          double skirtRatio) {
    this->PlanMoveByMaxSpeed(startPosition, targetPosition, maxSpeed, minSpeed,
                             beta, skirtRatio, 50);
  };
  void PlanMoveByDuration(position_t startPosition, position_t targetPosition,
                          int duration, double minSpeed, double beta,
                          double skirtRatio, int tickDuration);
  void PlanMoveByMaxSpeed(position_t startPosition, position_t targetPosition,
                          double maxSpeed, double minSpeed, double beta,
                          double skirtRatio, int tickDuration);
  void StartMove();
  void StopMove();
  bool GetNextSpeedValue(double *val);
  position_t GetStartPosition() { return _startPosition; };
  position_t GetTargetPosition() { return _targetPosition; };
  int GetNumTicks() { return _numTicks; };
  int GetTickDuration() { return _tickDuration; };
  int GetDuration() { return _duration; };
  int GetDistance() { return abs(GetTargetPosition() - GetStartPosition()); };
  double GetMinSpeed() { return _minSpeed; };
  double GetMaxSpeed() { return _maxSpeed; };
  double GetBeta() { return _beta; };
  direction_t GetDirection() { return _direction; };
  double GetSkirtRatio() { return _skirtRatio; };
  bool IsMoving() { return _isMoving; };

private:
  void _Reset();
  void _InitMove(position_t startPosition, position_t targetPosition,
                 double minSpeed, double beta, double skirtRatio,
                 int tickDuration);
  bool _isMoving;
  int _startPosition;
  int _targetPosition;
  int _distance;
  int _duration;
  int _tickDuration;
  int _numTicks;
  int _maxSpeedNumTicks;
  int _numSkirtTicks;
  int _skirtSegmentDuration;
  direction_t _direction;
  double _currentSpeedValue;
  double _minSpeed;
  double _maxSpeed;
  double _beta;
  double _skirtRatio;
};

#endif
