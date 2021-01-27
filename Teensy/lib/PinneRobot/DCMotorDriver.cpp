#include <DCMotorDriver.h>

speed_t DCMotorDriver::GetSpeed() {
	return _speed;
}

uint8_t DCMotorDriver::GetDirection() { return _direction; }

speed_t DCMotorDriver::GetBrake() {
	return _brake;
}
