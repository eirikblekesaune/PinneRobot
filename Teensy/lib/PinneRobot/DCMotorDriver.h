#ifndef DCMotorDriver_h
#define DCMotorDriver_h
#include <Arduino.h>

typedef int pwm_t;

class DCMotorDriver
{
	public:
          DCMotorDriver() {}
          virtual void init() = 0;
          virtual void SetPWM(pwm_t pwm) = 0;
          virtual void SetDirection(uint8_t direction) = 0;
          virtual void SetBrake(pwm_t brake) = 0;
          virtual pwm_t GetPWM();
          virtual uint8_t GetDirection();
          virtual pwm_t GetBrake();
          virtual void UpdateDirection();

        protected:
          pwm_t _pwm;
          uint8_t _direction;
          pwm_t _brake;
};
#endif
