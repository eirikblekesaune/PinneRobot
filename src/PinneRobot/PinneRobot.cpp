#include "PinneRobot.h"

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

const int rotationDriver1A = A3;
const int rotationDriver2A = A2;
const int rotationDriverPWM = 5;
const int rotationPotmeterPin = A4;

PinneRobot::PinneRobot()
{
	VNH5019Driver *leftDriver = new VNH5019Driver(leftDriverINA, leftDriverINB, leftDriverENDIAG, leftDriverPWM);
	VNH5019Driver *rightDriver = new VNH5019Driver(rightDriverINA, rightDriverINB, rightDriverENDIAG, rightDriverPWM);
	L293Driver *rotationDriver = new L293Driver(rotationDriver1A, rotationDriver2A, rotationDriverPWM);
	leftMotor = new PinneMotor(leftMotorTopStopSensor, leftMotorSlackStopSensor, leftMotorEncoderInterruptIndex, leftDriver, ADDRESS_LEFT);
	rightMotor = new PinneMotor(rightMotorTopStopSensor, rightMotorSlackStopSensor, rightMotorEncoderInterruptIndex, rightDriver, ADDRESS_RIGHT);
	rotationMotor = new RotationMotor(rotationPotmeterPin, rotationDriver, ADDRESS_ROTATION);
}

void PinneRobot::init()
{
	leftMotor->init();
	rightMotor->init();
	rotationMotor->init();
}

void PinneRobot::update()
{
	leftMotor->UpdateState();
	rightMotor->UpdateState();
	rotationMotor->UpdateState();
}

void PinneRobot::GoToParkingPosition()
{
	leftMotor->GoToParkingPosition();
	rightMotor->GoToParkingPosition();
	rotationMotor->GoToParkingPosition(); 
}





