#ifndef MOVER_H
#define MOVER_H
#include <Arduino.h>
#include <Metro.h>
#include <PID_v1.h>
#include <PinneComm.h>
#include <PinneMotor.h>
#include <PinneRobot.h>

enum moverMode_t : uint8_t {
  MOVER_MODE_BY_DURATION,
  MOVER_MODE_BY_MAX_SPEED,
  MOVER_MODE_BY_CONSTANT_SPEED,
  MOVER_MODE_UNKNOWN
};

class Mover {
public:
  Mover(PinneMotor *motor, PinneComm *comm);

private:
  PinneMotor *_motor;
  PinneComm *_comm;
  Metro *_metro;
  float _tickDuration;
};

#endif
