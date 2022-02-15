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
  motorA->otherMotor = motorB;
  motorB->otherMotor = motorA;
  SetMotorControlMode(CONTROL_MODE_PWM);
}

void PinneRobot::init()
{
  motorA->init();
  motorB->init();
  _lastAPositionSent = -1; // -1 for forcing init update
  _lastBPositionSent = -1;
}

static bool motorDidStop(int currentPwm, int previousPwm) {
  return (currentPwm == 0) && (previousPwm != 0);
}

void PinneRobot::update()
{
  motorA->Update();
  motorB->Update();
  position_t posA = motorA->GetCurrentPosition();
  position_t posB = motorB->GetCurrentPosition();
  int pwmA = motorA->GetBipolarPWM();
  int pwmB = motorB->GetBipolarPWM();
  if (_lastAPositionSent != posA || (_lastBPositionSent != posB) || (motorDidStop(pwmA, _lastPwmA) || (motorDidStop(pwmB, _lastPwmB)))) {
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
  _lastPwmA = pwmA;
  _lastPwmB = pwmB;
}

void PinneRobot::GoToParkingPosition(float speed)
{
  motorA->GoToParkingPosition(speed);
  motorB->GoToParkingPosition(speed);
}

void PinneRobot::SetMotorControlMode(controlMode_t mode) {
  _motorControlMode = mode;
  motorA->SetMotorControlMode(_motorControlMode);
  motorB->SetMotorControlMode(_motorControlMode);
}

void PinneRobot::Sync(int32_t syncStamp) {
  _syncTime = millis();
  _syncStamp = syncStamp;
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
  offset = msg.match("/stop", initialOffset);
  if (offset) {
    this->_RouteStopMsg(msg, offset + initialOffset);
  }
  offset = msg.match("/goToParkingPosition", initialOffset);
  if (offset) {
    this->_RouteGoToParkingPositionMsg(msg, offset + initialOffset);
  }
  offset = msg.match("/sync", initialOffset);
  if (offset) {
    this->_RouteSyncMsg(msg, offset + initialOffset);
  }
}

void PinneRobot::_RouteGoToParkingPositionMsg(OSCMessage &msg, int initialOffset) {
  if (_motorControlMode == CONTROL_MODE_PARKING) {
    if( msg.size() == 0) {
      GoToParkingPosition(1.0);
    } else {
      if(msg.isFloat(0)) {
        int speed = msg.getFloat(0);
        this->GoToParkingPosition(speed);
      }
    }
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
      } else if (strcmp(dirStr, "parking") == 0) {
        this->SetMotorControlMode(CONTROL_MODE_PARKING);
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

void PinneRobot::_RouteStopMsg(OSCMessage &msg, int initialOffset) {
  motorA->Stop();
  motorB->Stop();
}

void PinneRobot::_RouteSyncMsg(OSCMessage &msg, int initialOffset) {
    if( (msg.size() == 1) && (msg.isInt(0)) ) {
      int32_t syncStamp = msg.getInt(0);
      this->Sync(syncStamp);
    } else if(_comm->HasQueryAddress(msg, initialOffset)) {
      OSCMessage replyMsg("/");
      uint32_t timeSinceSync = millis() - _syncTime;
      replyMsg.add(_syncStamp);
      replyMsg.add(static_cast<uint16_t>(timeSinceSync >> 16)); // time since last sync
      replyMsg.add(static_cast<uint16_t>(timeSinceSync & 0x0000FFFF)); // time since last sync
      _comm->ReturnQueryValue(CMD_SYNC, replyMsg);
    }
}
