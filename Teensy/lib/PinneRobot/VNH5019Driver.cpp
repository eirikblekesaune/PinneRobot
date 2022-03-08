///This code is based on Pololu's DualVNH5019Motorshield.h code from:
//https://github.com/pololu/Dual-VNH5019-Motor-Shield
//Changed to work with Teensy4.1 in 2020.

#include <VNH5019Driver.h>

const pwm_t VNH5019Driver::SPEED_STOP = 0;
const pwm_t VNH5019Driver::SPEED_MIN = 0;
const pwm_t VNH5019Driver::SPEED_MAX = 4095;
const pwm_t VNH5019Driver::BRAKE_NONE = 0;
const pwm_t VNH5019Driver::BRAKE_FULL = 4095;

VNH5019Driver::VNH5019Driver(unsigned char INA, unsigned char INB,
                             unsigned char ENDIAG, unsigned char PWM)
    : _INA(INA), _INB(INB), _ENDIAG(ENDIAG), _PWM(PWM) {}

void VNH5019Driver::init()
{
	pinMode(_INA,OUTPUT);
	pinMode(_INB,OUTPUT);
        pinMode(_ENDIAG, INPUT);
        pinMode(_PWM, OUTPUT);
        analogWriteFrequency(_PWM, 36621);

        _brake = 0;
        SetPWM(0);
        SetDirection(0);
}

void VNH5019Driver::SetPWM(pwm_t pwm) {
  if (pwm < 0) {
    pwm = 0;
  }
  if (pwm > SPEED_MAX) { // Max PWM dutycycle
    pwm = SPEED_MAX;
  }
  _pwm = pwm;
  analogWrite(_PWM, pwm);
  if (pwm == 0) {
    digitalWrite(_INA, LOW); // Make the motor coast no
    digitalWrite(_INB, LOW); // matter which direction it is spinning.
  } else {
    UpdateDirection();
  }
}

void VNH5019Driver::SetDirection(uint8_t direction) {
  if (direction < 0)
    direction = 0;
  if (direction > 1)
    direction = 1;
  _direction = direction;
  UpdateDirection();
}

void VNH5019Driver::UpdateDirection()
{
	if (!_direction)
	{
		digitalWrite(_INA, LOW);
		digitalWrite(_INB, HIGH);
	}
	else
	{
		digitalWrite(_INA, HIGH);
		digitalWrite(_INB, LOW);
	}
}

void VNH5019Driver::SetBrake(pwm_t brake) {
  if (brake < 0)
    brake = 0;
  if (brake > SPEED_MAX) // Max brake
    brake = SPEED_MAX;
  digitalWrite(_INA, LOW);
  digitalWrite(_INB, LOW);
  _brake = brake;
  analogWrite(_PWM, brake);
}

unsigned char VNH5019Driver::GetFault()
{
	return !digitalRead(_ENDIAG);
}
					 
	 
								 
