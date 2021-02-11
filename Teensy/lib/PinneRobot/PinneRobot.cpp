#include <PinneRobot.h>

//pin connections
const int driverA_PWM = 2;
const int driverA_INA = 4;
const int driverA_INB = 5;
const int driverA_ENDIAG = 8;
const int motorATopStopSensor = 24;
const int motorASlackStopSensor = 25;
const int motorAEncoderInterruptPinA = 26;
const int motorAEncoderInterruptPinB = 27;

const int driverB_PWM = 3;
const int driverB_INA = 6;
const int driverB_INB = 7;
const int driverB_ENDIAG = 9;
const int motorBTopStopSensor = 28;
const int motorBSlackStopSensor = 29;
const int motorBEncoderInterruptPinA = 31;
const int motorBEncoderInterruptPinB = 30;

PinneRobot::PinneRobot(PinneComm *comm) : _comm(comm) {
  VNH5019Driver *driverA_ =
      new VNH5019Driver(driverA_INA, driverA_INB, driverA_ENDIAG, driverA_PWM);
  VNH5019Driver *driverB_ =
      new VNH5019Driver(driverB_INA, driverB_INB, driverB_ENDIAG, driverB_PWM);

  motorA = new PinneMotor(
      motorATopStopSensor, motorASlackStopSensor, motorAEncoderInterruptPinA,
      motorAEncoderInterruptPinB, driverA_, ADDRESS_A, _comm);
  motorB = new PinneMotor(
      motorBTopStopSensor, motorBSlackStopSensor, motorBEncoderInterruptPinA,
      motorBEncoderInterruptPinB, driverB_, ADDRESS_B, _comm);
}

void PinneRobot::init()
{
  motorA->init();
  motorB->init();
  _lastPositionUpdate = millis();
  _lastAPositionSent = -1; // -1 for forcing init update
  _lastBPositionSent = -1;
}

void PinneRobot::update()
{
  motorA->UpdateState();
  motorB->UpdateState();
  if ((millis() - _lastPositionUpdate) > 50) {
    // DebugPrint("hello");
    position_t pos = motorA->GetCurrentPosition();
    if (_lastAPositionSent != pos) {
      OSCMessage msg("/pinne/motorA/currentPosition");
      msg.add(pos);
      _comm->SendOSCMessage(msg);
      _lastAPositionSent = pos;
    }
    pos = motorB->GetCurrentPosition();
    if (_lastBPositionSent != pos) {
      OSCMessage msg("/pinne/motorB/currentPosition");
      msg.add(pos);
      _comm->SendOSCMessage(msg);
      _lastBPositionSent = pos;
    }
    _lastPositionUpdate = millis();
  }
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
    Serial.println("/motorA: ");
    motorA->routeOSC(msg, offset + initialOffset);
  }
  offset = msg.match("/motorB", initialOffset);
  if (offset) {
    Serial.println("/motorB: ");
    motorB->routeOSC(msg, offset + initialOffset);
  }
}
