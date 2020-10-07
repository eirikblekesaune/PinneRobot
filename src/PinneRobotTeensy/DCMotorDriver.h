#ifndef DCMotorDriver_h
#define DCMotorDriver_h

#include <Arduino.h>
#include "PinneAPI.h"

typedef int speed_t;

class DCMotorDriver
{
	public:
		DCMotorDriver() {};
		virtual void init() = 0;// Set pin directions etc.

		virtual void SetSpeed(speed_t speed) = 0;
		virtual void SetDirection(int direction) = 0;
		virtual void SetBrake(speed_t brake) = 0;

		virtual speed_t GetSpeed();
		virtual int GetDirection();
		virtual speed_t GetBrake();

		virtual void UpdateDirection();
	protected:
		speed_t _speed;
		int _direction;
		speed_t _brake;
};
#endif
