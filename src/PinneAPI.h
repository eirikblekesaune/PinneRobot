#ifndef PINNE_API_H
#define PINNE_API_H
#include <Arduino.h>
//This file contains all global definitions, macros, typedefs, enums etc.

enum command_t
{
	CMD_STOP = 0x00,//speed fade out time as argument
	CMD_SPEED = 0x01,
	CMD_DIRECTION = 0x02,
	CMD_TARGET_POSITION = 0x03,
	CMD_CURRENT_POSITION = 0x04,
	CMD_BRAKE = 0x05,
	CMD_STATE_CHANGE = 0x06,
	CMD_INFO = 0x07,//used for debugging, arbitrary numboer of asci characters
	CMD_MIN_POSITION = 0x08,
	CMD_MAX_POSITION = 0x09,
	CMD_GOTO_PARKING_POSITION = 0x0A,
	CMD_GOTO_TARGET = 0x0B, //duration after halfway point as argument
	CMD_GOTO_SPEED_RAMP_UP = 0x0C,
	CMD_GOTO_SPEED_RAMP_DOWN = 0x0D,
	CMD_GOTO_SPEED_SCALING = 0x0E,
	CMD_UNKNOWN
};

enum address_t {
	ADDRESS_LEFT = 0x00,
	ADDRESS_RIGHT = 0x10,
	ADDRESS_ROTATION = 0x20,
	ADDRESS_GLOBAL = 0x30,
	ADDRESS_UNKNOWN
};

enum setGet_t
{
	SET_MESSAGE = 0x00,
	GET_MESSAGE = 0x40,
	SETGET_UNKNOWN
};

enum byteType_t
{
	BYTE_COMMAND = 0x80,
	BYTE_DATA = 0x00,
	BYTE_UNKNOWN
};

enum parseMask_t {
	PARSE_MASK_MESSAGE_TYPE = 0x80,//command byte of data byte
	PARSE_MASK_SETGET = 0x40,//setter or getter
	PARSE_MASK_ADDRESS = 0x30,//which motor is address
	PARSE_MASK_COMMAND = 0x0F,
	PARSE_MASK_UNKNOWN
};

enum stateChange_t
{
	STOPPED,//Stopped manually
	GOING_DOWN,//direction set to up
	GOING_UP,//directiom set to down
	STOPPED_AT_TARGET,//
	GOING_TO_TARGET,
	BLOCKED_BY_TOP_SENSOR,//The stop sensor was hit
	BLOCKED_BY_SLACK_SENSOR,
	BLOCKED_BY_MIN_POSITION,//Position counter is below range
	BLOCKED_BY_MAX_POSITION,//Position counter is above range
	BLOCKED_BY_ABS_MIN_POSITION,//
	DRIVER_FAULT//Something is wrong with the driver itself
};

static void Reply(const char* str)
{
	Serial1.println(str);
}

static void ReturnGetValue(command_t command, address_t address, int value)
{
	unsigned char data[2];
	data[0] = (value >> 7) & 0x7F;
	data[1] = lowByte(value) & 0x7F;
	Serial1.write(BYTE_COMMAND | SET_MESSAGE | address | command);
	Serial1.write(data[0]);
	Serial1.write(data[1]);
}

static void NotifyStateChange(stateChange_t stateChange, address_t address)
{
	Serial1.write(BYTE_COMMAND | SET_MESSAGE | address | CMD_STATE_CHANGE);
	Serial1.write(stateChange);
}

static void DebugUnitPrint(address_t address, const char* msg)
{
	Serial1.write(BYTE_COMMAND | SET_MESSAGE | address | CMD_INFO );
	Serial1.print(msg);
	Serial1.write(4);
}

static void DebugUnitPrint(address_t address, int msg)
{
	Serial1.write(BYTE_COMMAND | SET_MESSAGE | address | CMD_INFO );
	Serial1.print(msg);
	Serial1.write(4);
}

static void DebugPrint(const char* msg)
{
	Serial1.write(BYTE_COMMAND | SET_MESSAGE | ADDRESS_GLOBAL | CMD_INFO );
	Serial1.print(msg);
	Serial1.write(4);
}

static void DebugPrint(int msg)
{
	Serial1.write(BYTE_COMMAND | SET_MESSAGE | ADDRESS_GLOBAL | CMD_INFO );
	Serial1.print(msg);
	Serial1.write(4);
}

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) DebugPrint(x)
#define DEBUG_NL
#else
#define DEBUG_PRINT(x)
#define DEBUG_NL
#endif


#endif
