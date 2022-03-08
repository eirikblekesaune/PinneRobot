#ifndef TARGET_POSITION_MOVER_H
#define TARGET_POSITION_MOVER_H
#include <Arduino.h>
#include <Metro.h>
#include <PID_v1.h>
#include <PinneComm.h>
#include <PinneMotor.h>
#include <PinneRobot.h>
#include <map>

typedef int position_t;
enum direction_t : uint8_t;
enum address_t : uint8_t;
enum targetPositionMoverState_t : uint8_t;
enum targetPositionMode_t : uint8_t;
class PinneComm;
class PinneMotor;

class TargetPositionMover {
public:
  TargetPositionMover(PinneMotor *motor, PinneComm *comm);
  void PlanMoveByDuration(position_t startPosition, position_t targetPosition,
                          int duration, double minSpeed, double beta,
                          double skirtRatio, int tickDuration, int32_t moveId);
  void PlanMoveByMaxSpeed(position_t startPosition, position_t targetPosition,
                          double maxSpeed, double minSpeed, double beta,
                          double skirtRatio, int tickDuration, int32_t moveId);
  void PlanMoveByConstantSpeed(position_t startPosition,
                               position_t targetPosition, double speed,
                               double minSpeed, double beta, double skirtRatio,
                               int tickDuration, int32_t moveId);

  bool StartMove();
  void StopMove();
  void CancelMove();
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
  double GetMaxSpeed() { return _maxSpeedPlanned; };
  double GetAdjustedMaxSpeed() { return _maxSpeedAdjusted; };
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
                 int tickDuration, int32_t moveId);
  bool _CheckPositionTargetHit(position_t currentPosition);
  void _FinalizeMovePlan();
  void _CalculateFadeSegmentBuffer();
  double _PerformLastStretchStrategy(double currentPosition);
  double _GetFadeSegmentValue(size_t tickIndex);
  double _UnmapSpeedValue(double speed, double maxSpeed);
  double _MapSpeedValue(double speed, double maxSpeed);
  double _EstimateRemainingDistance(double remainingTicksToTarget);
  double _GetSkirtSum();
  int32_t _moveId;
  bool _isMoving;
  PinneMotor *_motor;
  PinneComm *_comm;
  float *_stopSpeedThreshold;
  void _ChangeState(targetPositionMoverState_t state);
  double _GetSCurveValue(double t, double beta);
  void _UpdateProgress(int distanceToTarget);

  unsigned long _moveStartTime;
  PID *_maxSpeedPID;
  Metro *_metro;
  static const size_t _fadeSegmentBufferSize = 512;
  double _fadeSegmentBuffer[_fadeSegmentBufferSize];
  float _numFadeSegmentSamplesPerTick;
  int _startPosition;
  int _targetPosition;
  float _progress;
  int _distance;
  int _duration;
  int _tickDuration;
  int _numTicks;
  int _numFadeInTicks;
  int _numFadeOutTicks;
  size_t _currentTickIndex;
  int _maxSpeedSegmentNumTicks;
  int _maxSpeedSegmentNumTicksAdjustment;
  int _numSkirtTicks;
  int _skirtSegmentDuration;
  unsigned long _previousUpdateTime;
  direction_t _direction;
  double _currentSpeedUnmapped;
  double _minSpeed;
  double _maxSpeedPlanned;
  double _maxSpeedAdjusted;
  double _beta;
  double _skirtRatio;
  double _distanceTravelled;
  double _plannedDistanceTravelled;
  double _estimatedRemainingDistance;
  double _expectedRemainingDistance;
  double _travelError;
  int _maxSpeedSegmentStartIndex;
  int _fadeDownSegmentStartIndex;
  targetPositionMode_t _mode;
  targetPositionMoverState_t _state;
};

#endif
