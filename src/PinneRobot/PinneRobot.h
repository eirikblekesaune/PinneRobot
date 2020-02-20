#ifndef PINNE_ROBOT_H
#define PINNE_ROBOT_H
#include <Arduino.h>
#include "Motor.h"
#include "PinneAPI.h"


class PinneRobot
{
public:

	PinneRobot();
	void init();
	void update();
	PinneMotor *leftMotor;
	PinneMotor *rightMotor;
	RotationMotor *rotationMotor;
	void GoToParkingPosition();
};



#endif
