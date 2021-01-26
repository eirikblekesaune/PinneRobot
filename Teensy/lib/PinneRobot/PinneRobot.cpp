#include <PinneRobot.h>

//pin connections
const int driverA_PWM = 0;
const int driverA_INA = 11;
const int driverA_INB = 6;
const int motorASlackStopSensor = A5;
const int driverA_ENDIAG = 9;
const int motorAEncoderInterruptPinA = 2;
const int motorAEncoderInterruptPinB = 3;
const int motorATopStopSensor = A0;

const int driverB_PWM = 1;
const int driverB_INA = 7;
const int driverB_INB = 8;
const int motorBSlackStopSensor = 13;
const int driverB_ENDIAG = 12;
const int motorBEncoderInterruptPinA = 4;
const int motorBEncoderInterruptPinB = 5;
const int motorBTopStopSensor = A1;

PinneRobot::PinneRobot(PinneComm *comm) : _comm(comm) {
  VNH5019Driver *driverA_ =
      new VNH5019Driver(driverA_INA, driverA_INB, driverA_ENDIAG, driverA_PWM);
  VNH5019Driver *driverB_ =
      new VNH5019Driver(driverB_INA, driverB_INB, driverB_ENDIAG, driverB_PWM);

  motorA = new PinneMotor(
      motorATopStopSensor, motorASlackStopSensor, motorAEncoderInterruptPinA,
      motorAEncoderInterruptPinB, driverA_, ADDRESS_LEFT, _comm);
  motorB = new PinneMotor(
      motorBTopStopSensor, motorBSlackStopSensor, motorBEncoderInterruptPinA,
      motorBEncoderInterruptPinB, driverB_, ADDRESS_RIGHT, _comm);
}

void PinneRobot::init()
{
  /* motorA->init(); */
  /* motorB->init(); */
  _lastPositionUpdate = millis();
  _lastAPositionSent = -1; // -1 for forcing init update
  _lastBPositionSent = -1;
}

void PinneRobot::update()
{
  /* motorA->UpdateState(); */
  /* motorB->UpdateState(); */
  /* if((millis() - _lastPositionUpdate) > 50) */
  /* { */
  /* 	// DebugPrint("hello"); */
  /* 	int pos = motorA->GetCurrentPosition(); */
  /* 	if(_lastAPositionSent != pos) */
  /* 	{ */
  /* 		ReturnGetValue(CMD_CURRENT_POSITION, ADDRESS_LEFT, pos); */
  /* 		_lastAPositionSent = pos; */
  /* 	} */
  /* 	pos = motorB->GetCurrentPosition(); */
  /* 	if(_lastBPositionSent != pos) */
  /* 	{ */
  /* 		ReturnGetValue(CMD_CURRENT_POSITION, ADDRESS_RIGHT, pos); */
  /* 		_lastBPositionSent = pos; */
  /* 	} */
  /* 	_lastPositionUpdate = millis(); */
  /* } */
}

void PinneRobot::GoToParkingPosition()
{
  /* motorA->GoToParkingPosition(); */
  /* motorB->GoToParkingPosition(); */
}

void PinneRobot::routeOSC(OSCMessage &msg, int initialOffset) {
  int offset;
  bool handled = false;
  offset = msg.match("/motorA", initialOffset);
  if (offset) {
    Serial.print("/motorA: ");
    motorA->routeOSC(msg, offset);
  }
  offset = msg.match("/motorB", initialOffset);
  if (offset) {
    motorB->routeOSC(msg, offset);
  }
}
