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

enum command_t : uint8_t {
  CMD_STOP = 0x00, // speed fade out time as argument
  CMD_SPEED = 0x01,
  CMD_DIRECTION = 0x02,
  CMD_TARGET_POSITION = 0x03,
  CMD_CURRENT_POSITION = 0x04,
  CMD_BRAKE = 0x05,
  CMD_STATE_CHANGE = 0x06,
  CMD_INFO = 0x07, // used for debugging, arbitrary numboer of asci characters
  CMD_MIN_POSITION = 0x08,
  CMD_MAX_POSITION = 0x09,
  CMD_GOTO_PARKING_POSITION = 0x0A,
  CMD_GOTO_TARGET = 0x0B, // duration after halfway point as argument
  CMD_MEASURED_SPEED = 0x0C,
  CMD_GOTO_SPEED_RAMP_DOWN = 0x0D,
  CMD_GOTO_SPEED_SCALING = 0x0E,
  CMD_ECHO_MESSAGES = 0x0F,
  CMD_UNKNOWN
};

enum address_t : uint8_t {
  ADDRESS_A = 0x00,
  ADDRESS_B = 0x10,
  ADDRESS_ROTATION = 0x20,
  ADDRESS_GLOBAL = 0x30,
  ADDRESS_UNKNOWN
};

enum setGet_t : uint8_t {
  SET_MESSAGE = 0x00,
  GET_MESSAGE = 0x40,
  SETGET_UNKNOWN
};

enum byteType_t : uint8_t {
  BYTE_COMMAND = 0x80,
  BYTE_DATA = 0x00,
  BYTE_UNKNOWN
};

enum parseMask_t : uint8_t {
  PARSE_MASK_MESSAGE_TYPE = 0x80, // command byte of data byte
  PARSE_MASK_SETGET = 0x40,       // setter or getter
  PARSE_MASK_ADDRESS = 0x30,      // which motor is address
  PARSE_MASK_COMMAND = 0x0F,
  PARSE_MASK_UNKNOWN
};

enum direction_t : uint8_t { DIRECTION_DOWN = 0x00, DIRECTION_UP = 0x01 };

const std::map<direction_t, String> DirectionMap{{DIRECTION_DOWN, "down"},
                                                 {DIRECTION_UP, "up"}};

enum stateChange_t : uint8_t {
  STOPPED,           // Stopped manually
  GOING_DOWN,        // direction set to up
  GOING_UP,          // directiom set to down
  STOPPED_AT_TARGET, //
  GOING_TO_TARGET,
  BLOCKED_BY_TOP_SENSOR, // The stop sensor was hit
  BLOCKED_BY_SLACK_SENSOR,
  BLOCKED_BY_MIN_POSITION,     // Position counter is below range
  BLOCKED_BY_MAX_POSITION,     // Position counter is above range
  BLOCKED_BY_ABS_MIN_POSITION, //
  DRIVER_FAULT                 // Something is wrong with the driver itself
};

const std::map<command_t, String> CommandMap{
    {CMD_STOP, "stop"},
    {CMD_SPEED, "speed"},
    {CMD_DIRECTION, "direction"},
    {CMD_TARGET_POSITION, "targetPosition"},
    {CMD_CURRENT_POSITION, "currentPosition"},
    {CMD_BRAKE, "brake"},
    {CMD_STATE_CHANGE, "stateChange"},
    {CMD_INFO, "info"},
    {CMD_MIN_POSITION, "minPosition"},
    {CMD_MAX_POSITION, "maxPosition"},
    {CMD_GOTO_PARKING_POSITION, "goToParkingPosition"},
    {CMD_GOTO_TARGET, "goToTarget"},
    {CMD_MEASURED_SPEED, "measuredSpeed"},
    {CMD_GOTO_SPEED_RAMP_DOWN, "goToSpeedRampDown"},
    {CMD_GOTO_SPEED_SCALING, "goToSpeedScaling"},
    {CMD_ECHO_MESSAGES, "echoMessages"}};

const std::map<address_t, String> AddressMap{{ADDRESS_A, "motorA"},
                                             {ADDRESS_B, "motorB"},
                                             {ADDRESS_ROTATION, "rotation"},
                                             {ADDRESS_GLOBAL, "global"}};

const std::map<stateChange_t, String> StateChangeMap{
    {STOPPED, "stopped"},
    {GOING_DOWN, "going_down"},
    {GOING_UP, "going_up"},
    {STOPPED_AT_TARGET, "stopped_at_target"},
    {GOING_TO_TARGET, "going_to_target"},
    {BLOCKED_BY_TOP_SENSOR, "blocked_by_top_sensor"},
    {BLOCKED_BY_SLACK_SENSOR, "blocked_by_slack_sensor"},
    {BLOCKED_BY_MIN_POSITION, "blocked_by_min_position"},
    {BLOCKED_BY_MAX_POSITION, "blocked_by_max_position"},
    {BLOCKED_BY_ABS_MIN_POSITION, "blocked_by_abs_min_position"},
    {DRIVER_FAULT, "driver_fault"}};

extern EthernetUDP Udp;
class PinneRobot;

class PinneComm {
	public:
          PinneComm(PinneSettings *settings);
          void RouteMsg(OSCBundle &bundle);

          void Reply(const char *);
          void SendOSCMessage(OSCMessage &msg);
          void ReturnGetValue(command_t command, address_t address, int value);
          void ReturnQueryValue(command_t command, address_t address,
                                OSCMessage &replyMsg);
          bool HasQueryAddress(OSCMessage &msg, int initialOffset);
          void NotifyStateChange(stateChange_t stateChange, address_t address);
          void DebugUnitPrint(address_t address, const char *);
          void DebugUnitPrint(address_t address, int val);
          void DebugPrint(int val);
          void DebugPrint(float val);
          void DebugPrint(const char *);
          void DebugMessagePrint(command_t command, address_t address,
                                 setGet_t setGet, int value);
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
