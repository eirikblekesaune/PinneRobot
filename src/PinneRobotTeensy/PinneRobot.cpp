#include "PinneRobot.h"
#include "PinneAPI.h"

//pin connections
//left driver aka motor 1, right driver motor 2
const int leftDriverPWM = 9;
const int leftDriverINA = 11;//Remapped due to interrupt
const int leftDriverINB = 4;
const int leftMotorSlackStopSensor = A5; //todo: cut trace, was current sense pin
const int leftDriverENDIAG = 6;
const int leftMotorEncoderInterruptIndex = 1;// digital pin 3 on Leonardo implicitly
const int leftMotorTopStopSensor = A0;

const int rightDriverPWM = 10;
const int rightDriverINA = 7;
const int rightDriverINB = 8;
const int rightMotorSlackStopSensor = 13;//todo cut trace, was current sense pin
const int rightDriverENDIAG = 12;
const int rightMotorEncoderInterruptIndex = 0;// digital pin 2 on Leonardo implicitly
const int rightMotorTopStopSensor = A1;

PinneRobot::PinneRobot()
{
	VNH5019Driver *leftDriver = new VNH5019Driver(leftDriverINA, leftDriverINB, leftDriverENDIAG, leftDriverPWM);
	VNH5019Driver *rightDriver = new VNH5019Driver(rightDriverINA, rightDriverINB, rightDriverENDIAG, rightDriverPWM);
	leftMotor = new PinneMotor(leftMotorTopStopSensor, leftMotorSlackStopSensor, leftMotorEncoderInterruptIndex, leftDriver, ADDRESS_LEFT);
	rightMotor = new PinneMotor(rightMotorTopStopSensor, rightMotorSlackStopSensor, rightMotorEncoderInterruptIndex, rightDriver, ADDRESS_RIGHT);
}

void PinneRobot::init()
{
	leftMotor->init();
	rightMotor->init();
	_lastPositionUpdate = millis();
	_lastLeftPositionSent = -1; // -1 for forcing init update
	_lastRightPositionSent = -1;
}

void PinneRobot::update()
{
	leftMotor->UpdateState();
	rightMotor->UpdateState();
	if((millis() - _lastPositionUpdate) > 50)
	{
		// DebugPrint("hello");
		int pos = leftMotor->GetCurrentPosition();
		if(_lastLeftPositionSent != pos)
		{
			ReturnGetValue(CMD_CURRENT_POSITION, ADDRESS_LEFT, pos);
			_lastLeftPositionSent = pos;
		}
		pos = rightMotor->GetCurrentPosition();
		if(_lastRightPositionSent != pos)
		{
			ReturnGetValue(CMD_CURRENT_POSITION, ADDRESS_RIGHT, pos);
			_lastRightPositionSent = pos;
		}
		_lastPositionUpdate = millis();
	}
}

void PinneRobot::GoToParkingPosition()
{
	leftMotor->GoToParkingPosition();
	rightMotor->GoToParkingPosition();
}
