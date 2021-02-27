#ifndef PINNE_MOTOR_H
#define PINNE_MOTOR_H
#include <Arduino.h>
#include <Bounce.h>
#include <Encoder.h>
#include <Metro.h>
#include <OSCMessage.h>
#include <PID_v1.h>
#include <PinneComm.h>
#include <TargetPositionMover.h>
#include <VNH5019Driver.h>

// todo: implement private inheritance of PID in motor classes
//
enum address_t : uint8_t;
enum direction_t : uint8_t;
enum motorState_t : uint8_t;
enum blockingMask_t : uint8_t;
enum controlMode_t : uint8_t;
typedef int position_t;
#define PID_CLASS PID
typedef double pidvalue_t;

class PinneComm;
class TargetPositionMover;

enum targetSpeedState_t : uint8_t {
  TARGET_SPEED_STOPPED,
  TARGET_SPEED_GOING_UP,
  TARGET_SPEED_GOING_DOWN
};

class PinneMotor
{
	public:
          PinneMotor(int topStopSensorPin, int slackStopSensorPin,
                     int encoderInterruptPinA, int encoderInterruptPinB,
                     int currentSensePin, VNH5019Driver *driver,
                     address_t address, PinneComm *comm);
          const int TOP_SENSOR_IN = 1;
          const int TOP_SENSOR_OUT = 0;
          const int SLACK_SENSOR_IN = 1;
          const int SLACK_SENSOR_OUT = 0;
          const position_t POSITION_ALL_UP = 1;
          const position_t POSITION_DEFAULT_MAX = 4096;
          const position_t TARGET_NONE = 0;
          const unsigned long topSensorDebounceDelay = 50;
          const unsigned long slackSensorDebounceDelay = 350;
          void init();

          void Stop();

          void SetStop(int value);
          void SetPWM(int speed);
          void SetBipolarPWM(int speed);
          void SetDirection(direction_t direction);
          void SetTargetPosition(position_t pos);
          void SetBipolarTargetSpeed(float value);
          void SetCurrentPosition(position_t pos);
          void SetBrake(int brake);
          void SetMaxPosition(position_t maxPos);
          void SetMinPosition(position_t minPos);

          int GetPWM() { return static_cast<int>(_driver->GetPWM()); };
          int GetBipolarPWM();
          direction_t GetDirection() {
            return static_cast<direction_t>(_driver->GetDirection());
          };
          position_t GetTargetPosition() { return _targetPosition; };
          float GetBipolarTargetSpeed() { return _targetSpeedPIDSetpoint; };
          position_t GetCurrentPosition();
          int GetBrake() { return static_cast<int>(_driver->GetBrake()); };
          position_t GetMaxPosition() { return _maxPosition; };
          position_t GetMinPosition() { return _minPosition; };

          controlMode_t GetMotorControlMode() { return _motorControlMode; };
          void SetMotorControlMode(controlMode_t mode);

          float GetMeasuredSpeed();
          int GetCurrentSense() { return static_cast<int>(_measuredCurrent); };


          bool IsBlocked();
          void Update();
          void UpdateState();
          void ReadTopStopSensor();
          void ReadSlackStopSensor();
          void CheckPositionLimits();
          void GoToParkingPosition(int speed);
          void GoToParkingPosition();
          void GoToTargetPositionByDuration(int targetPosition, int duration,
                                            double minSpeed, double beta,
                                            double skirtRatio);
          void GoToTargetPositionByMaxSpeed(int targetPosition, double maxSpeed,
                                            double minSpeed, double beta,
                                            double skirtRatio);
          void GoToTargetPositionByConstantSpeed(int targetPosition,
                                                 double speed, double minSpeed,
                                                 double beta,
                                                 double skirtRatio);

          bool routeOSC(OSCMessage &msg, int initialOffset);

        private:
          int _topStopSensorPin;
          int _slackStopSensorPin;
          Bounce *_topStopButton;
          Bounce *_slackStopButton;
          int _encoderInterruptPinA;
          int _encoderInterruptPinB;
          int _currentSensePin;
          TargetPositionMover *_targetPositionMover;
          VNH5019Driver *_driver;
          address_t _address;
          PinneComm *_comm;
          position_t _minPosition;
          position_t _maxPosition;
          position_t _currentPosition;
          position_t _targetPosition;
          Encoder *_encoder;
          controlMode_t _motorControlMode;

          PID_CLASS *_speedPID;
          int _speedometerInterval;
          Metro *_speedometerMetro;
          position_t _prevPosition;
          pidvalue_t _targetSpeedPIDSetpoint;
          pidvalue_t _targetSpeedPIDOutput;
          pidvalue_t _measuredSpeed;
          targetSpeedState_t _targetSpeedState;
          uint8_t _blockingMask;
          void _UpdateSpeedometer();
          void _UpdateCurrentSense();
          float _measuredCurrent;
          void _ActivateTargetSpeedPID();
          void _DeactivateTargetSpeedPID();

          bool _blocked;
          int _stoppingSpeed;
          float _targetSpeedStopThreshold;
          void _ChangeState(motorState_t state);
          void _SetBlockingMaskBit(blockingMask_t sensorMask);
          void _ClearBlockingMaskBit(blockingMask_t sensorMask);
          void _Stopped();
          void _GoingUp();
          void _GoingDown();
          void _TargetReached();
          void _TopStopSensorIn();
          void _TopStopSensorOut();
          void _SlackStopSensorIn();
          void _SlackStopSensorOut();
          void _AbsMinPositionReached();
          void _MinPositionReached();
          void _MinPositionLeft();
          void _MaxPositionReached();
          void _MaxPositionLeft();
          void _PWMModeUpdate();
          void _TargetPositionModeUpdate();
          void _TargetSpeedModeUpdate();
          bool _CheckSensorBlockingState(blockingMask_t sensorMask);

          motorState_t _state;

          void _RouteStopMsg(OSCMessage &msg, int initialOffset);
          void _RouteBipolarPWMMsg(OSCMessage &msg, int initialOffset);
          void _RouteTargetSpeedMsg(OSCMessage &msg, int initialOffset);
          void _RouteCurrentPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteBrakeMsg(OSCMessage &msg, int initialOffset);
          void _RouteMotorStateChangeMsg(OSCMessage &msg, int initialOffset);
          void _RouteMinPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteMaxPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteGoToParkingPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteGoToTargetPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteMeasuredSpeedMsg(OSCMessage &msg, int initialOffset);
          void _RouteEchoMessagesMsg(OSCMessage &msg, int initialOffset);
          void _RoutePIDParametersMsg(OSCMessage &msg, int initialOffset);
};

#endif
