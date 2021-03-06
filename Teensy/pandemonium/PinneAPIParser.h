#ifndef PINNE_API_PARSER_H
#define PINNE_API_PARSER_H

#include <Arduino.h>
#include "PinneRobot.h"
#include "PinneAPI.h"

class PinneAPIParser
{
	public:
		PinneAPIParser(PinneRobot* robot);
		void parseIncomingByte(byte inByte);

	private:
		PinneRobot *_robot;
		command_t _currentCommand;
		setGet_t _currentSetGet;
		address_t _currentAddress;
		byte _currentChecksum;
		char _dataByteBuffer[2];
		boolean _echoMessages;

		void _parseCommand(byte inByte);
		boolean _getDataBytes();
		int _parseDataValue();
		boolean _getChecksum();
		boolean _verifyGetCommandChecksum(byte inByte);
		boolean _verifySetCommandChecksum(byte inByte);

		void _processSetStopCommand();
		void _processSetSpeedCommand();
		void _processSetDirectionCommand();
		void _processSetTargetPositionCommand();
		void _processSetCurrentPositionCommand();
		void _processSetBrakeCommand();
		void _processSetMinPositionCommand();
		void _processSetMaxPositionCommand();
		void _processSetGoToTargetCommand();
		void _processSetMeasuredSpeed();
		void _processSetGoToSpeedRampDownCommand();
		void _processSetGoToSpeedScalingCommand();
		void _processSetGotoParkingPosition();
		void _processSetEchoMessages();

		void _processGetStateCommand();
		void _processGetSpeedCommand();
		void _processGetDirectionCommand();
		void _processGetTargetPositionCommand();
		void _processGetCurrentPositionCommand();
		void _processGetBrakeCommand();
		void _processGetMinPositionCommand();
		void _processGetMaxPositionCommand();
		void _processGetMeasuredSpeed();
		void _processGetGoToSpeedRampDownCommand();
		void _processGetGoToSpeedScalingCommand();
		void _processGetStopCommand();
		void _processGetEchoMessages();
};
#endif
