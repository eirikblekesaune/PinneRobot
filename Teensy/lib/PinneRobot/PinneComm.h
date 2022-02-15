#ifndef PINNE_COMM_H
#define PINNE_COMM_H
#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <PinneRobot.h>
#include <SPI.h>
#include <map>

struct PinneSettings {
  String name;
  String hostname;
  int port;
  String targetHostname;
  int targetPort;
};

struct PinneSettingsRaw {
  char name[16];
  char hostname[16];
  int port;
  char targetHostname[16];
  int targetPort;
};

enum targetSpeedState_t : uint8_t {
  TARGET_SPEED_STOPPED,
  TARGET_SPEED_GOING_UP,
  TARGET_SPEED_GOING_DOWN
};

enum parkingProcedureState_t : uint8_t {
  PARKING_PROCEDURE_STATE_NOT_RUNNING,
  PARKING_PROCEDURE_STATE_AIMING_FOR_COARSE_CALIBRATION,
  PARKING_PROCEDURE_STATE_WAITING_FOR_OTHER_MOTOR,
  PARKING_PROCEDURE_STATE_STABILIZE_AFTER_COARSE_CALIBRATION,
  PARKING_PROCEDURE_STATE_POSITIONING_FOR_FINE_CALIBRATION,
  PARKING_PROCEDURE_STATE_STABILIZE_BEFORE_FINE_CALIBRATION,
  PARKING_PROCEDURE_STATE_AIMING_FOR_FINE_CALIBRATION,
  PARKING_PROCEDURE_STATE_SLIGHT_LOWERING,
  PARKING_PROCEDURE_STATE_PARKED
};

enum command_t : uint8_t {
  CMD_STOP,
  CMD_BIPOLAR_PWM,
  CMD_DIRECTION,
  CMD_TARGET_POSITION,
  CMD_CURRENT_POSITION,
  CMD_BRAKE,
  CMD_STATE_CHANGE,
  CMD_INFO,
  CMD_MIN_POSITION,
  CMD_MAX_POSITION,
  CMD_GOTO_PARKING_POSITION,
  CMD_GOTO_TARGET,
  CMD_MEASURED_SPEED,
  CMD_ECHO_MESSAGES,
  CMD_PID_PARAMETERS,
  CMD_MOTOR_CONTROL_MODE,
  CMD_BIPOLAR_TARGET_SPEED,
  CMD_TARGET_POSITION_MOVER,
  CMD_SYNC,
  CMD_UNKNOWN
};

enum address_t : uint8_t {
  ADDRESS_A,
  ADDRESS_B,
  ADDRESS_ROTATION,
  ADDRESS_GLOBAL,
  ADDRESS_UNKNOWN
};

enum controlMode_t : uint8_t {
  CONTROL_MODE_PWM,
  CONTROL_MODE_TARGET_POSITION,
  CONTROL_MODE_TARGET_SPEED,
  CONTROL_MODE_PARKING
};

enum targetPositionMoverState_t : uint8_t {
  TARGET_POSITION_MOVER_STATE_NOT_READY,
  TARGET_POSITION_MOVER_STATE_READY,
  TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT,
  TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT,
  TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT,
  TARGET_POSITION_MOVER_STATE_FINISHED_BUT_TARGET_NOT_REACHED,
  TARGET_POSITION_MOVER_STATE_REACHED_TARGET
};

const std::map<targetPositionMoverState_t, String>
    TargetPositionMoverStateChangeMap{
        {TARGET_POSITION_MOVER_STATE_NOT_READY, "not_ready"},
        {TARGET_POSITION_MOVER_STATE_READY, "ready"},
        {TARGET_POSITION_MOVER_STATE_FADE_IN_SEGMENT, "fade_in_segment"},
        {TARGET_POSITION_MOVER_STATE_MAX_SPEED_SEGMENT, "max_speed_segment"},
        {TARGET_POSITION_MOVER_STATE_FADE_OUT_SEGMENT, "fade_out_segment"},
        {TARGET_POSITION_MOVER_STATE_FINISHED_BUT_TARGET_NOT_REACHED,
         "finished_but_target_not_reached"},
        {TARGET_POSITION_MOVER_STATE_REACHED_TARGET, "reached_target"}};

enum setGet_t : uint8_t { SET_MESSAGE, GET_MESSAGE, SETGET_UNKNOWN };

enum byteType_t : uint8_t { BYTE_COMMAND, BYTE_DATA, BYTE_UNKNOWN };

enum direction_t : uint8_t { DIRECTION_DOWN = 0x00, DIRECTION_UP = 0x01 };

const std::map<direction_t, String> DirectionMap{{DIRECTION_DOWN, "down"},
                                                 {DIRECTION_UP, "up"}};
enum blockingMask_t : uint8_t {
  NOTHING_BLOCKS = 0x00,      //
  TOP_SENSOR_BLOCKS = 0x01,   // The stop sensor was hit
  SLACK_SENSOR_BLOCKS = 0x02, // T
  MIN_POSITION_BLOCKS = 0x04, // Position counter is below range
  MAX_POSITION_BLOCKS = 0x08, // Position counter is above range
};

enum motorState_t : uint8_t {
  GOING_DOWN,            // direction set to up
  GOING_UP,              // directiom set to down
  STOPPED,               // Stopped manually
  BLOCKED_BY_TOP_SENSOR, // The stop sensor was hit
  BLOCKED_BY_SLACK_SENSOR,
  BLOCKED_BY_MIN_POSITION, // Position counter is below range
  BLOCKED_BY_MAX_POSITION, // Position counter is above range
};

const std::map<controlMode_t, String> ControlModeMap{
    {CONTROL_MODE_PWM, "pwm"},
    {CONTROL_MODE_TARGET_POSITION, "targetPosition"},
    {CONTROL_MODE_TARGET_SPEED, "targetSpeed"},
    {CONTROL_MODE_PARKING, "parking"}};

const std::map<command_t, String> CommandMap{
    {CMD_STOP, "stop"},
    {CMD_BIPOLAR_PWM, "bipolarPWM"},
    {CMD_TARGET_POSITION, "targetPosition"},
    {CMD_BIPOLAR_TARGET_SPEED, "bipolarTargetSpeed"},
    {CMD_CURRENT_POSITION, "currentPosition"},
    {CMD_BRAKE, "brake"},
    {CMD_STATE_CHANGE, "stateChange"},
    {CMD_INFO, "info"},
    {CMD_MIN_POSITION, "minPosition"},
    {CMD_MAX_POSITION, "maxPosition"},
    {CMD_GOTO_PARKING_POSITION, "goToParkingPosition"},
    {CMD_GOTO_TARGET, "goToTarget"},
    {CMD_MEASURED_SPEED, "measuredSpeed"},
    {CMD_ECHO_MESSAGES, "echoMessages"},
    {CMD_PID_PARAMETERS, "pidParameters"},
    {CMD_MOTOR_CONTROL_MODE, "motorControlMode"},
    {CMD_SYNC, "sync"}};

const std::map<address_t, String> AddressMap{{ADDRESS_A, "motorA"},
                                             {ADDRESS_B, "motorB"},
                                             {ADDRESS_ROTATION, "rotation"},
                                             {ADDRESS_GLOBAL, "global"}};

const std::map<motorState_t, String> MotorStateChangeMap{
    {STOPPED, "stopped"},
    {GOING_DOWN, "going_down"},
    {GOING_UP, "going_up"},
    {BLOCKED_BY_TOP_SENSOR, "blocked_by_top_sensor"},
    {BLOCKED_BY_SLACK_SENSOR, "blocked_by_slack_sensor"},
    {BLOCKED_BY_MIN_POSITION, "blocked_by_min_position"},
    {BLOCKED_BY_MAX_POSITION, "blocked_by_max_position"}};

const std::map<blockingMask_t, String> BlockingMaskMap{
    {NOTHING_BLOCKS, "nothing_blocks"},
    {TOP_SENSOR_BLOCKS, "top_sensor_blocks"},
    {SLACK_SENSOR_BLOCKS, "slack_sensor_blocks"},
    {MIN_POSITION_BLOCKS, "min_position_blocks"},
    {MAX_POSITION_BLOCKS, "max_position_blocks"}};

const std::map<parkingProcedureState_t, String> ParkingStateMap{
  {PARKING_PROCEDURE_STATE_NOT_RUNNING, "not_running"},
  {PARKING_PROCEDURE_STATE_AIMING_FOR_COARSE_CALIBRATION, "aiming_for_coarse_calibration"},
  {PARKING_PROCEDURE_STATE_WAITING_FOR_OTHER_MOTOR, "waiting_for_other_motor"},
  {PARKING_PROCEDURE_STATE_STABILIZE_AFTER_COARSE_CALIBRATION, "stabilize_after_coarse_calibration"},
  {PARKING_PROCEDURE_STATE_POSITIONING_FOR_FINE_CALIBRATION, "positioning_for_fine_calibration"},
  {PARKING_PROCEDURE_STATE_AIMING_FOR_FINE_CALIBRATION, "aiming_for_fine_calibration"},
  {PARKING_PROCEDURE_STATE_STABILIZE_BEFORE_FINE_CALIBRATION, "stabilize_before_fine_calibration"},
  {PARKING_PROCEDURE_STATE_SLIGHT_LOWERING, "slight_lowering"},
  {PARKING_PROCEDURE_STATE_PARKED, "parked"}};

extern EthernetUDP Udp;
class PinneRobot;

class PinneComm {
public:
  PinneComm(PinneSettings *settings);
  void RouteMsg(OSCBundle &bundle);
  void SendInfo();

  void Reply(const char *);
  void SendOSCMessage(OSCMessage &msg);
  void ReturnGetValue(command_t command, address_t address, int value);
  void ReturnQueryValue(command_t command, address_t address,
                        OSCMessage &replyMsg);
  void ReturnQueryValue(command_t command, OSCMessage &replyMsg);
  bool HasQueryAddress(OSCMessage &msg, int initialOffset);
  void NotifyMotorStateChange(motorState_t stateChange, address_t address);
  void
  NotifyTargetPositionMoverStateChange(targetPositionMoverState_t stateChange,
                                       address_t address, uint8_t moveId);
  void SendParkingProceduresState(parkingProcedureState_t state, address_t address);

  void SendTargetPositionMoverProgress(float progress, address_t address, uint8_t moveId);
  void DebugUnitPrint(address_t address, const char *);
  void DebugUnitPrint(address_t address, int val);
  void DebugPrint(int val);
  void DebugPrint(float val);
  void DebugPrint(const char *);
  void DebugMessagePrint(command_t command, address_t address, setGet_t setGet,
                         int value);
  void msgReceive();
  void handlePinneMsg(OSCMessage &msg);

  uint8_t initResult = 0;
  enum InitResults : uint8_t {
    validSettings = 0x00,
    invalidHostname = 0x01,
    invalidTargetHostname = 0x02
  };

  void routeOSC(OSCMessage &msg, int initial_offset);
  void setRobot(PinneRobot *robot) { _robot = robot; };

private:
  String _name;
  uint8_t *_mac;
  EthernetUDP _Udp;
  IPAddress *_ip;
  unsigned int _port;
  IPAddress *_targetIp;
  IPAddress *_broadcastIp;
  unsigned int _targetPort;
  PinneRobot *_robot;
};

#endif
