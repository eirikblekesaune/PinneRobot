#ifndef DCMotorDriver_h
#define DCMotorDriver_h
#include <Arduino.h>

typedef int speed_t;

class DCMotorDriver
{
	public:
          DCMotorDriver() {}
          virtual void init() = 0;
          virtual void SetSpeed(speed_t speed) = 0;
          virtual void SetDirection(uint8_t direction) = 0;
          virtual void SetBrake(speed_t brake) = 0;
          virtual speed_t GetSpeed();
          virtual uint8_t GetDirection();
          virtual speed_t GetBrake();
          virtual void UpdateDirection();

        protected:
          speed_t _speed;
          uint8_t _direction;
          speed_t _brake;
};
#endif
