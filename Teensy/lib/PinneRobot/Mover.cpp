#include <Mover.h>

Mover::Mover(PinneMotor *motor, PinneComm *comm)
    : _motor(motor), _comm(comm) {
      _metro = new Metro(_tickDuration);

    }
