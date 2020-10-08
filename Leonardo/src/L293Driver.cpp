#include "MotorDriver.h"

const int L293Driver::SPEED_MIN = 0;
const int L293Driver::SPEED_MAX = 512;

L293Driver::L293Driver(unsigned char INA, unsigned char INB, unsigned char PWM) :
	_INA(INA),
	_INB(INB),
	_PWM(PWM)
{
}
void L293Driver::init()
{
	pinMode(_INA, OUTPUT);
	pinMode(_INB, OUTPUT);
	pinMode(_PWM, OUTPUT);
	TCCR3A = 0b10000000;
	TCCR3B = 0b00010001;
	ICR3 = SPEED_MAX;
	SetDirection(0);
	SetSpeed(0);
}

void L293Driver::SetSpeed(speed_t speed)
{
	if (speed < SPEED_MIN)
		speed = SPEED_MIN;
	if (speed > SPEED_MAX)  // Max PWM dutycycle
		speed = SPEED_MAX;
	_speed = speed;
	OCR3A = _speed;
	UpdateDirection();
}

void L293Driver::SetDirection(int direction)
{
	if(direction < 0)
		direction = 0;
	if(direction > 1)
		direction = 1;
	_direction = direction;
	UpdateDirection();
}

void L293Driver::UpdateDirection()
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

void L293Driver::SetBrake(speed_t brake)
{
	_brake = brake;
}
