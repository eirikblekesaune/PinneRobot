#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <Encoder.h>
#include <OSCMessage.h>
#include <PinneComm.h>
#include <SpeedRamping.h>
#include <VNH5019Driver.h>

// todo: implement private inheritance of PID in motor classes
//
enum address_t : uint8_t;
enum direction_t : uint8_t;
enum stateChange_t : uint8_t;
typedef long position_t;
class PinneComm;

class PinneMotor
{
	public:
          PinneMotor(int topStopSensorPin, int slackStopSensorPin,
                     int encoderInterruptPinA, int encoderInterruptPinB,
                     VNH5019Driver *driver, address_t address, PinneComm *comm);
          //		enum DIRECTION { DIRECTION_DOWN, DIRECTION_UP };
          const int TOP_SENSOR_IN = 0;
          const int TOP_SENSOR_OUT = 1;
          const int SLACK_SENSOR_IN = 0;
          const int SLACK_SENSOR_OUT = 1;
          const position_t POSITION_ALL_UP = 1;
          const position_t POSITION_DEFAULT_MAX = 4096;
          const position_t TARGET_NONE = 0;
          const unsigned long topSensorDebounceDelay = 50;
          const unsigned long slackSensorDebounceDelay = 350;
          void init();

          void Stop();

          void SetStop(int value);
          void SetSpeed(int speed);
          void SetDirection(direction_t direction);
          void SetTargetPosition(position_t pos);
          void SetCurrentPosition(position_t pos);
          void SetBrake(int brake);
          void SetMaxPosition(position_t maxPos);
          void SetMinPosition(position_t minPos);
          void SetGoToSpeedScaling(int value);
          void SetGoToSpeedRampUp(int value);
          void SetGoToSpeedRampDown(int value);

          int GetSpeed() { return static_cast<int>(_driver->GetSpeed()); };
          direction_t GetDirection() {
            return static_cast<direction_t>(_driver->GetDirection());
          };
          position_t GetTargetPosition() { return _targetPosition; };
          position_t GetCurrentPosition();
          int GetBrake() { return static_cast<int>(_driver->GetBrake()); };
          position_t GetMaxPosition() { return _maxPosition; };
          position_t GetMinPosition() { return _minPosition; };

          int GetGoToSpeedScaling() {
            return static_cast<int>(_speedRamper->GetSpeedScaling() * 1000);
          };
          int GetMeasuredSpeed();
          int GetGoToSpeedRampDown() {
            return static_cast<int>(_speedRamper->GetRampDown());
          };
          int GetStop() { return _stoppingSpeed; };

          void GoToTargetPosition(position_t value);

          bool IsBlocked();
          void UpdateState();
          void ReadTopStopSensor();
          void ReadSlackStopSensor();
          void GoToParkingPosition(int speed);
          void GoToParkingPosition();

          bool routeOSC(OSCMessage &msg, int initialOffset);

        private:
          int _topStopSensorPin;
          int _slackStopSensorPin;
          int _encoderInterruptPinA;
          int _encoderInterruptPinB;
          VNH5019Driver *_driver;
          address_t _address;
          PinneComm *_comm;
          position_t _minPosition;
          position_t _maxPosition;
          position_t _currentPosition;
          position_t _targetPosition;
          Encoder *_encoder;

          int _topStopSensorValue;
          int _slackStopSensorValue;
          unsigned long _lastTopSensorReadTime;
          int _lastTopSensorReading;
          unsigned long _lastSlackSensorReadTime;
          int _lastSlackSensorReading;
          bool _blocked;
          int _stoppingSpeed;
          void _GoingUp();
          void _GoingDown();
          void _TargetReached();
          void _TopStopSensorIn();
          void _TopStopSensorOut();
          void _SlackStopSensorIn();
          void _SlackStopSensorOut();
          void _AbsMinPositionReached();
          void _MinPositionReached();
          void _MaxPositionReached();
          void _GoingToTarget();

          void _UpdateSpeedRamp();
          void _CalculateAndSetSpeed();
          void _SetBlocked(bool block){};
          stateChange_t _state;

          void _RouteStopMsg(OSCMessage &msg, int initialOffset);
          void _RouteSpeedMsg(OSCMessage &msg, int initialOffset);
          void _RouteDirectionMsg(OSCMessage &msg, int initialOffset);
          void _RouteTargetPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteCurrentPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteBrakeMsg(OSCMessage &msg, int initialOffset);
          void _RouteStateChangeMsg(OSCMessage &msg, int initialOffset);
          void _RouteMinPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteMaxPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteGoToParkingPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteGoToTargetPositionMsg(OSCMessage &msg, int initialOffset);
          void _RouteMeasuredSpeedMsg(OSCMessage &msg, int initialOffset);
          void _RouteGoToSpeedRampDownMsg(OSCMessage &msg, int initialOffset);
          void _RouteGoToSpeedScalingMsg(OSCMessage &msg, int initialOffset);
          void _RouteEchoMessagesMsg(OSCMessage &msg, int initialOffset);

          // SpeedRamp
          SpeedRamping *_speedRamper;
};

#endif
