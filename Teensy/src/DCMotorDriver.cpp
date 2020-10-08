#include "DCMotorDriver.h"

speed_t DCMotorDriver::GetSpeed() {
	return _speed;
}

int DCMotorDriver::GetDirection() {
	return _direction;
}

speed_t DCMotorDriver::GetBrake() {
	return _brake;
}
