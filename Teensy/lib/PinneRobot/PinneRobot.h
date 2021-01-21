#ifndef PINNE_ROBOT_H
#define PINNE_ROBOT_H
#include <Arduino.h>
#include "PinneMotor.h"
#include "PinneAPI.h"

class PinneRobot
{
public:
	PinneRobot();
	void init();
	void update();
	PinneMotor *leftMotor;
	PinneMotor *rightMotor;
	void GoToParkingPosition();
private:
	unsigned long _lastPositionUpdate;
	int _lastLeftPositionSent;
	int _lastRightPositionSent;
};



#endif
