#include <DCMotorDriver.h>

pwm_t DCMotorDriver::GetPWM() { return _pwm; }

uint8_t DCMotorDriver::GetDirection() { return _direction; }

pwm_t DCMotorDriver::GetBrake() { return _brake; }
