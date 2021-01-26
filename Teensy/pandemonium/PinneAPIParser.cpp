#include "PinneAPIParser.h"

PinneAPIParser::PinneAPIParser(PinneRobot *robot) : _robot(robot),
													_currentCommand(CMD_UNKNOWN),
													_currentSetGet(SETGET_UNKNOWN),
													_currentAddress(ADDRESS_UNKNOWN),
													_echoMessages(false)
{
}

void PinneAPIParser::parseIncomingByte(byte inByte)
{
	_parseCommand(inByte);
}

void PinneAPIParser::_parseCommand(byte inByte)
{
	_currentCommand = (command_t)(inByte & PARSE_MASK_COMMAND);
	_currentAddress = (address_t)(inByte & PARSE_MASK_ADDRESS);
	_currentSetGet = (setGet_t)(inByte & PARSE_MASK_SETGET);

	switch (_currentSetGet)
	{
	case SET_MESSAGE:
		_getDataBytes();
		_getChecksum();
		if (_verifySetCommandChecksum(inByte))
		{
			switch (_currentCommand)
			{
			case CMD_STOP:
				_processSetStopCommand();
				break;
			case CMD_SPEED:
				_processSetSpeedCommand();
				break;
			case CMD_DIRECTION:
				_processSetDirectionCommand();
				break;
			case CMD_TARGET_POSITION:
				_processSetTargetPositionCommand();
				break;
			case CMD_CURRENT_POSITION:
				_processSetCurrentPositionCommand();
				break;
			case CMD_BRAKE:
				_processSetBrakeCommand();
				break;
			case CMD_MIN_POSITION:
				_processSetMinPositionCommand();
				break;
			case CMD_MAX_POSITION:
				_processSetMaxPositionCommand();
				break;
			case CMD_GOTO_PARKING_POSITION:
				_processSetGotoParkingPosition();
				break;
			case CMD_GOTO_TARGET:
				_processSetGoToTargetCommand();
				break;
			case CMD_MEASURED_SPEED:
				_processSetMeasuredSpeed();
				break;
			case CMD_GOTO_SPEED_RAMP_DOWN:
				_processSetGoToSpeedRampDownCommand();
				break;
			case CMD_GOTO_SPEED_SCALING:
				_processSetGoToSpeedScalingCommand();
				break;
			case CMD_ECHO_MESSAGES:
				_processSetEchoMessages();
				break;
			default:
				DebugPrint("Unknown command");
				DebugPrint(_currentCommand);
			}
		}
		else
		{
			DebugPrint("CHECKSUM FAILED");
		}

		break;
	case GET_MESSAGE:
		_getChecksum();
		if(_verifyGetCommandChecksum(inByte))
		{
			switch (_currentCommand)
			{
			case CMD_STOP:
				_processGetStopCommand();
				break;
			case CMD_SPEED:
				_processGetSpeedCommand();
				break;
			case CMD_DIRECTION:
				_processGetDirectionCommand();
				break;
			case CMD_TARGET_POSITION:
				_processGetTargetPositionCommand();
				break;
			case CMD_CURRENT_POSITION:
				_processGetCurrentPositionCommand();
				break;
			case CMD_BRAKE:
				_processGetBrakeCommand();
				break;
			case CMD_STATE_CHANGE:
				_processGetStateCommand();
				break;
			case CMD_MIN_POSITION:
				_processGetMinPositionCommand();
				break;
			case CMD_MAX_POSITION:
				_processGetMaxPositionCommand();
				break;
			case CMD_MEASURED_SPEED:
				_processGetMeasuredSpeed();
				break;
			case CMD_GOTO_SPEED_RAMP_DOWN:
				_processGetGoToSpeedRampDownCommand();
				break;
			case CMD_GOTO_SPEED_SCALING:
				_processGetGoToSpeedScalingCommand();
				break;
			case CMD_ECHO_MESSAGES:
				_processGetEchoMessages();
				break;
			default:
				DEBUG_PRINT("Unknown command");
			}
		}
		else
		{
			DebugPrint("CHECKSUM FAILED");
		}
		break;
	default:
		DEBUG_PRINT("SetGet fault");
	}
	if (_echoMessages)
	{
		DebugMessagePrint(_currentCommand, _currentAddress, _currentSetGet, _parseDataValue());
	}

	_currentCommand = CMD_UNKNOWN;
	_currentAddress = ADDRESS_UNKNOWN;
	_currentSetGet = SETGET_UNKNOWN;
	_currentChecksum = 0;
}

boolean PinneAPIParser::_getDataBytes()
{
	boolean result;
	result = Serial1.readBytes(_dataByteBuffer, 2) == 2;
	return result;
}

boolean PinneAPIParser::_getChecksum()
{
	boolean result;
	result = Serial1.readBytes(&_currentChecksum, 1) == 1;
	return result;
}

boolean PinneAPIParser::_verifySetCommandChecksum(byte inByte)
{
	// DebugPrint("set checksumByte");
	// DebugPrint(_currentChecksum);
	byte checksum = inByte;
	checksum += _dataByteBuffer[0];
	checksum += _dataByteBuffer[1];
	checksum = 0xFF - checksum;
	// DebugPrint("checksum");
	// DebugPrint(checksum);
	return _currentChecksum == checksum;
}

boolean PinneAPIParser::_verifyGetCommandChecksum(byte inByte)
{
	// DebugPrint("get checksumByte");
	// DebugPrint(_currentChecksum);
	byte checksum = inByte;
	checksum = 0xFF - checksum;
	// DebugPrint("checksum");
	// DebugPrint(checksum);
	return _currentChecksum == checksum;
}

int PinneAPIParser::_parseDataValue()
{
	int result;
	result = _dataByteBuffer[1] | (_dataByteBuffer[0] << 7);
	return result;
}

//STOP COMMAND
void PinneAPIParser::_processSetStopCommand()
{
	int value;
	value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetStop(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetStop(value);
		break;
	case ADDRESS_GLOBAL:
		_robot->leftMotor->SetStop(value);
		_robot->rightMotor->SetStop(value);
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
	}
}

void PinneAPIParser::_processGetStopCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetStop();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetStop();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		////DEBUG_PRINT("Something wrong with geting speed");DEBUG_NL;
	}
}

//SPPED COMMAND
void PinneAPIParser::_processSetSpeedCommand()
{
	int value;
	value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetSpeed(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetSpeed(value);
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
	}
}

void PinneAPIParser::_processGetSpeedCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetSpeed();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetSpeed();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		////DEBUG_PRINT("Something wrong with geting speed");DEBUG_NL;
	}
}

//DIRECTION COMMAND
void PinneAPIParser::_processSetDirectionCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetDirection(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetDirection(value);
		break;
	default:
		DEBUG_PRINT("Unknown address");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processGetDirectionCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetDirection();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetDirection();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting direction");DEBUG_NL;
	}
}

////TARGET POSITION COMMAND
void PinneAPIParser::_processSetTargetPositionCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetTargetPosition(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetTargetPosition(value);
		break;
	default:
		DEBUG_PRINT("Unknown address");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processGetTargetPositionCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetTargetPosition();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetTargetPosition();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting target position");DEBUG_NL;
	}
}
////PARKING COMMAND

void PinneAPIParser::_processSetGotoParkingPosition()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->GoToParkingPosition();
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->GoToParkingPosition();
		break;
	case ADDRESS_GLOBAL:
		_robot->leftMotor->GoToParkingPosition();
		_robot->rightMotor->GoToParkingPosition();
		break;
	default:
		DEBUG_PRINT("Unknown address");
		DEBUG_NL;
	}
}

////CURRENT POSITION COMMAND
void PinneAPIParser::_processSetCurrentPositionCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetCurrentPosition(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetCurrentPosition(value);
		break;
	default:
		DEBUG_PRINT("Unknown address");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processGetCurrentPositionCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetCurrentPosition();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetCurrentPosition();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting current brake");DEBUG_NL;
	}
}

////BRAKE COMMAND
void PinneAPIParser::_processSetBrakeCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetBrake(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetBrake(value);
		break;
	default:
		DEBUG_PRINT("Unknown address");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processGetBrakeCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetBrake();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetBrake();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting current brake");DEBUG_NL;
	}
}

////MIN POSITION COMMAND
void PinneAPIParser::_processSetMinPositionCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetMinPosition(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetMinPosition(value);
		break;
	default:
		DEBUG_PRINT("Unknown address");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processGetMinPositionCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetMinPosition();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetMinPosition();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting min position");DEBUG_NL;
	}
}

////MAX POSITION COMMAND
void PinneAPIParser::_processSetMaxPositionCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetMaxPosition(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetMaxPosition(value);
		break;
	default:
		DEBUG_PRINT("Unknown address");
	}
}

void PinneAPIParser::_processGetMaxPositionCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetMaxPosition();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetMaxPosition();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting max position");DEBUG_NL;
	}
}

void PinneAPIParser::_processGetStateCommand()
{
	//DEBUG_PRINT("State command");DEBUG_NL;
}

void PinneAPIParser::_processSetGoToTargetCommand()
{
	int value;
	value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->GoToTargetPosition(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->GoToTargetPosition(value);
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processSetMeasuredSpeed()
{
//	int value = _parseDataValue();
//	switch (_currentAddress)
//	{
//	case ADDRESS_LEFT:
//		_robot->leftMotor->SetGoToSpeedRampUp(value);
//		break;
//	case ADDRESS_RIGHT:
//		_robot->rightMotor->SetGoToSpeedRampUp(value);
//		break;
//	default:
//		DEBUG_PRINT("Unknown address\n");
//		DEBUG_NL;
//	}
}

void PinneAPIParser::_processSetGoToSpeedRampDownCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetGoToSpeedRampDown(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetGoToSpeedRampDown(value);
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processSetGoToSpeedScalingCommand()
{
	int value = _parseDataValue();
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		_robot->leftMotor->SetGoToSpeedScaling(value);
		break;
	case ADDRESS_RIGHT:
		_robot->rightMotor->SetGoToSpeedScaling(value);
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
}

void PinneAPIParser::_processGetMeasuredSpeed()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetMeasuredSpeed();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetMeasuredSpeed();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting max position");DEBUG_NL;
	}
}

void PinneAPIParser::_processGetGoToSpeedRampDownCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetGoToSpeedRampDown();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetGoToSpeedRampDown();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting max position");DEBUG_NL;
	}
}

void PinneAPIParser::_processGetGoToSpeedScalingCommand()
{
	int value;
	value = -1;
	switch (_currentAddress)
	{
	case ADDRESS_LEFT:
		value = _robot->leftMotor->GetGoToSpeedScaling();
		break;
	case ADDRESS_RIGHT:
		value = _robot->rightMotor->GetGoToSpeedScaling();
		break;
	default:
		DEBUG_PRINT("Unknown address\n");
		DEBUG_NL;
	}
	if (value >= 0)
	{
		ReturnGetValue(_currentCommand, _currentAddress, value);
	}
	else
	{
		//DEBUG_PRINT("Something wrong with geting max position");DEBUG_NL;
	}
}

void PinneAPIParser::_processSetEchoMessages()
{
	int value;
	value = _parseDataValue();
	if (value)
	{
		_echoMessages = true;
	}
	else
	{
		_echoMessages = false;
	}
}

void PinneAPIParser::_processGetEchoMessages()
{
	ReturnGetValue(CMD_ECHO_MESSAGES, ADDRESS_GLOBAL, _echoMessages);
}