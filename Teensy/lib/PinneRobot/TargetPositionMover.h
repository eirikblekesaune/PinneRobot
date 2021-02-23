#ifndef TARGET_POSITION_MOVER_H
#define TARGET_POSITION_MOVER_H
#include <Arduino.h>
#include <Metro.h>
#include <PinneComm.h>
#include <PinneMotor.h>
#include <PinneRobot.h>
#include <map>

typedef int position_t;
enum direction_t : uint8_t;
enum address_t : uint8_t;
enum targetPositionMoverState_t : uint8_t;
class PinneComm;

enum targetPositionMode_t : uint8_t {
  TARGET_POSITION_MODE_BY_DURATION,
  TARGET_POSITION_MODE_BY_MAX_SPEED,
  TARGET_POSITION_MODE_UNKNOWN
};


class TargetPositionMover {
public:
  TargetPositionMover(address_t *address, PinneComm *comm);
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
  void Update(position_t currentPosition);
  bool DidReachTarget();
  double GetCurrentSpeed();
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
  unsigned long GetMoveStartTime() { return _moveStartTime; };
  size_t NumTicksSincePreviousUpdate();
  unsigned long TimeSincePreviousUpdate();
  size_t CurrentTickIndex() { return _currentTickIndex; }
  targetPositionMode_t GetMode() { return _mode; };

private:
  void _Reset();
  void _InitMove(position_t startPosition, position_t targetPosition,
                 double minSpeed, double beta, double skirtRatio,
                 int tickDuration);
  bool _CheckPositionTargetHit(position_t currentPosition);
  void _CalculateFadeSegmentBuffer();
  double _GetFadeSegmentValue(size_t tickIndex);
  bool _isMoving;
  address_t *_address;
  PinneComm *_comm;
  void _ChangeState(targetPositionMoverState_t state);

  unsigned long _moveStartTime;
  Metro *_metro;
  static const size_t _fadeSegmentBufferSize = 512;
  double _fadeSegmentBuffer[_fadeSegmentBufferSize];
  float _numFadeSegmentSamplesPerTick;
  int _startPosition;
  int _targetPosition;
  int _distance;
  int _duration;
  int _tickDuration;
  int _numTicks;
  size_t _currentTickIndex;
  int _maxSpeedSegmentNumTicks;
  int _numSkirtTicks;
  int _skirtSegmentDuration;
  unsigned long _previousUpdateTime;
  direction_t _direction;
  double _currentSpeed;
  double _minSpeed;
  double _maxSpeed;
  double _beta;
  double _skirtRatio;
  int _maxSpeedSegmentStartIndex;
  int _fadeDownSegmentStartIndex;
  targetPositionMode_t _mode;
  targetPositionMoverState_t _state;
};

#endif
