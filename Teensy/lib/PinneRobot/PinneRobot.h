#ifndef PINNE_ROBOT_H
#define PINNE_ROBOT_H
#include <Arduino.h>
#include <OSCMessage.h>
#include <PinneComm.h>
#include <PinneMotor.h>
#include <VNH5019Driver.h>

class PinneComm;
class PinneMotor;
typedef int position_t;
enum controlMode_t : uint8_t;

class PinneRobot {
public:
  PinneRobot(PinneComm *comm);
  void init();
  void update();
  PinneMotor *motorA;
  PinneMotor *motorB;
  void GoToParkingPosition(float speed);
  void routeOSC(OSCMessage &msg, int initialOffset);
  controlMode_t GetMotorControlMode() { return _motorControlMode; };
  void SetMotorControlMode(controlMode_t mode);
  void Sync(int32_t syncStamp);

private:
  position_t _lastAPositionSent;
  position_t _lastBPositionSent;
  int _lastPwmA;
  int _lastPwmB;
  PinneComm *_comm;
  controlMode_t _motorControlMode;
  void _RouteMotorControlModeMsg(OSCMessage &msg, int initialOffset);
  void _RouteGoToParkingPositionMsg(OSCMessage &msg, int initialOffset);
  void _RouteStopMsg(OSCMessage &msg, int initialOffset);
  void _RouteSyncMsg(OSCMessage &msg, int initialOffset);
  int32_t _syncStamp;
  uint32_t _syncTime;
};

#endif
