#ifndef PINNE_ROBOT_H
#define PINNE_ROBOT_H
#include <Arduino.h>
#include <OSCMessage.h>
#include <PinneComm.h>
#include <PinneMotor.h>
#include <VNH5019Driver.h>

class PinneComm;
class PinneMotor;
typedef long position_t;

class PinneRobot {
public:
  PinneRobot(PinneComm *comm);
  void init();
  void update();
  PinneMotor *motorA;
  PinneMotor *motorB;
  void GoToParkingPosition();
  void routeOSC(OSCMessage &msg, int initialOffset);

private:
  position_t _lastAPositionSent;
  position_t _lastBPositionSent;
  PinneComm *_comm;
};

#endif
