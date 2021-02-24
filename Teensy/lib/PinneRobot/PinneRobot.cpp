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
const int motorACurrentSensePin = A0;

const int driverB_PWM = 3;
const int driverB_INA = 6;
const int driverB_INB = 7;
const int driverB_ENDIAG = 9;
const int motorBTopStopSensor = 28;
const int motorBSlackStopSensor = 29;
const int motorBEncoderInterruptPinA = 31;
const int motorBEncoderInterruptPinB = 30;
const int motorBCurrentSensePin = A1;

PinneRobot::PinneRobot(PinneComm *comm) : _comm(comm) {
  VNH5019Driver *driverA_ =
      new VNH5019Driver(driverA_INA, driverA_INB, driverA_ENDIAG, driverA_PWM);
  VNH5019Driver *driverB_ =
      new VNH5019Driver(driverB_INA, driverB_INB, driverB_ENDIAG, driverB_PWM);

  motorA =
      new PinneMotor(motorATopStopSensor, motorASlackStopSensor,
                     motorAEncoderInterruptPinA, motorAEncoderInterruptPinB,
                     motorACurrentSensePin, driverA_, ADDRESS_A, _comm);
  motorB =
      new PinneMotor(motorBTopStopSensor, motorBSlackStopSensor,
                     motorBEncoderInterruptPinA, motorBEncoderInterruptPinB,
                     motorBCurrentSensePin, driverB_, ADDRESS_B, _comm);
  SetMotorControlMode(CONTROL_MODE_PWM);
}

void PinneRobot::init()
{
  motorA->init();
  motorB->init();
  _lastAPositionSent = -1; // -1 for forcing init update
  _lastBPositionSent = -1;
}

void PinneRobot::update()
{
  motorA->Update();
  motorB->Update();
  position_t posA = motorA->GetCurrentPosition();
  position_t posB = motorB->GetCurrentPosition();
  if (_lastAPositionSent != posA || (_lastBPositionSent != posB)) {
    OSCMessage msg("/pinne/currentPosition");
    msg.add(posA);
    msg.add(posB);
    msg.add(motorA->GetBipolarTargetSpeed());
    msg.add(motorB->GetBipolarTargetSpeed());
    msg.add(motorA->GetMeasuredSpeed());
    msg.add(motorB->GetMeasuredSpeed());
    _comm->SendOSCMessage(msg);
    _lastAPositionSent = posA;
    _lastBPositionSent = posB;
  }
}

void PinneRobot::GoToParkingPosition()
{
  /* motorA->GoToParkingPosition(); */
  /* motorB->GoToParkingPosition(); */
}

void PinneRobot::SetMotorControlMode(controlMode_t mode) {
  _motorControlMode = mode;
  motorA->SetMotorControlMode(_motorControlMode);
  motorB->SetMotorControlMode(_motorControlMode);
}

void PinneRobot::routeOSC(OSCMessage &msg, int initialOffset) {
  int offset;
  offset = msg.match("/motorA", initialOffset);
  if (offset) {
    motorA->routeOSC(msg, offset + initialOffset);
  }
  offset = msg.match("/motorB", initialOffset);
  if (offset) {
    motorB->routeOSC(msg, offset + initialOffset);
  }
  offset = msg.match("/motorControlMode", initialOffset);
  if (offset) {
    this->_RouteMotorControlModeMsg(msg, offset + initialOffset);
  }
}

void PinneRobot::_RouteMotorControlModeMsg(OSCMessage &msg, int initialOffset) {
  if (msg.size() > 0) {
    if (msg.isString(0)) {
      char dirStr[16];
      msg.getString(0, dirStr, 16);
      if (strcmp(dirStr, "pwm") == 0) {
        this->SetMotorControlMode(CONTROL_MODE_PWM);
      } else if (strcmp(dirStr, "targetPosition") == 0) {
        this->SetMotorControlMode(CONTROL_MODE_TARGET_POSITION);
      } else if (strcmp(dirStr, "targetSpeed") == 0) {
        this->SetMotorControlMode(CONTROL_MODE_TARGET_SPEED);
      }
    }
  } else {
    if (_comm->HasQueryAddress(msg, initialOffset)) {
      controlMode_t mode = GetMotorControlMode();
      OSCMessage replyMsg("/");
      if (mode == CONTROL_MODE_PWM) {
        replyMsg.add("pwm");
      } else if (mode == CONTROL_MODE_TARGET_POSITION) {
        replyMsg.add("targetPosition");
      } else if (mode == CONTROL_MODE_TARGET_SPEED) {
        replyMsg.add("targetSpeed");
      }
      _comm->ReturnQueryValue(CMD_MOTOR_CONTROL_MODE, replyMsg);
    }
  }
}
