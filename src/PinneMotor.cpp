#include "Motor.h"

volatile int encoderCounter1 = 0;
volatile int encoderCounter2 = 0;
volatile int encoderIncrement1 = 1;
volatile int encoderIncrement2 = 1;

//init static variables
const int PinneMotor::DIRECTION_DOWN = 0;
const int PinneMotor::DIRECTION_UP = 1;
const int PinneMotor::TOP_SENSOR_IN = 0;
const int PinneMotor::TOP_SENSOR_OUT = 1;
const int PinneMotor::SLACK_SENSOR_IN = 0;
const int PinneMotor::SLACK_SENSOR_OUT = 1;
const int PinneMotor::POSITION_ALL_UP = 1;
const int PinneMotor::POSITION_DEFAULT_MAX = 4096;
const int PinneMotor::TARGET_NONE = 0;
const int PinneMotor::DIRECTION_DOWN_INCREMENT = 1;
const int PinneMotor::DIRECTION_UP_INCREMENT = -1;

//variables for Sensor debouncing
const unsigned long topSensorDebounceDelay = 50;
const unsigned long slackSensorDebounceDelay = 350;

void encoderISR1()
{
	encoderCounter1 = max(encoderCounter1 + encoderIncrement1, 0);
}

void encoderISR2()
{
	encoderCounter2 = max(encoderCounter2 + encoderIncrement2, 0);
}

int PinneMotor::GetCurrentPosition()
{
	int value;
//	uint8_t data[sizeof(int)];
//	noInterrupts();
//	data[0] = (*_encoderCounter >> 7) & 0x7F;
//	data[1] = lowByte(*_encoderCounter) & 0x7F;
//	interrupts();
//	value = (data[0] << 7) | data[1];
	//value = static_cast<int>(*_encoderCounter);
	value = *_encoderCounter;
	return value;
};


PinneMotor::PinneMotor(int topStopSensorPin, int slackStopSensorPin, int encoderInterruptIndex, VNH5019Driver* driver, address_t address) :
	_currentPosition(POSITION_ALL_UP),
	_targetPosition(TARGET_NONE),
	_minPosition(POSITION_ALL_UP),
	_maxPosition(POSITION_DEFAULT_MAX),
	_topStopSensorPin(topStopSensorPin),
	_slackStopSensorPin(slackStopSensorPin),
	_driver(driver),
	_address(address),
	_encoderInterruptIndex(encoderInterruptIndex)
{

}

void PinneMotor::init()
{
	switch(_encoderInterruptIndex)
	{
	case 0:
		attachInterrupt(0, encoderISR1, CHANGE);
		_encoderCounter = &encoderCounter1;
		_encoderIncrement = &encoderIncrement1;
		break;
	case 1:
		attachInterrupt(1, encoderISR2, CHANGE);
		_encoderCounter = &encoderCounter2;
		_encoderIncrement = &encoderIncrement2;
		break;
	}
	pinMode(_topStopSensorPin, INPUT);
	pinMode(_slackStopSensorPin, INPUT);
	_driver->init();
	_driver->SetDirection(DIRECTION_UP);
	SetDirection(DIRECTION_DOWN);
	Stop();
	_blocked = false;
	_topStopSensorValue = digitalRead(_topStopSensorPin);
	if(_topStopSensorValue == TOP_SENSOR_IN)
		_TopStopSensorIn();
	_slackStopSensorValue = digitalRead(_slackStopSensorPin);
	if(_slackStopSensorValue == SLACK_SENSOR_OUT)
		_SlackStopSensorOut();

	_speedRamper = new SpeedRamping(
	VNH5019Driver::SPEED_MAX * 0.115,
	VNH5019Driver::SPEED_MAX
	);
}

void PinneMotor::Stop()
{
	_driver->SetSpeed(VNH5019Driver::SPEED_STOP);
}

void PinneMotor::SetStop(int value)
{
	_stoppingSpeed = constrain(value, 0, 5000);
	_driver->SetSpeed(VNH5019Driver::SPEED_STOP);
}

void PinneMotor::SetSpeed(int speed)
{
	if(speed <= 0)
	{
	Stop();
	}
	else {
	if(!IsBlocked())
		{
			_driver->SetSpeed(speed);
		}
	}
}

void PinneMotor::SetDirection(int direction)
{
	if(GetDirection() != direction)
	{
		int dir;
		_driver->SetDirection(direction);
		dir = GetDirection();
		if(dir == DIRECTION_DOWN)
		{
			*_encoderIncrement = DIRECTION_DOWN_INCREMENT;
			_GoingDown();
		}
		else if(dir == DIRECTION_UP) {
			*_encoderIncrement = DIRECTION_UP_INCREMENT;
			_GoingUp();
		}
	}
	else {
		_driver->UpdateDirection();
	}
}

boolean PinneMotor::IsBlocked()
{
	if(_state >= BLOCKED_BY_TOP_SENSOR)
	{
		int direction = GetDirection();
		switch(_state)
		{
			case BLOCKED_BY_MIN_POSITION: 
			case BLOCKED_BY_TOP_SENSOR:
			case BLOCKED_BY_ABS_MIN_POSITION:
				if(direction == DIRECTION_UP)
				{
					return true;
				} else {
					return false;
				}
				break;
			case BLOCKED_BY_MAX_POSITION:
			case BLOCKED_BY_SLACK_SENSOR:
				if(direction == DIRECTION_DOWN)
				{
					return true;
				} else {
					return false;
				}
			default:
				//
				return false;
		}	 
	} else {
		return false;
	}
}

//Read all sensor values and check positions
//Stop motor if needed
void PinneMotor::UpdateState()
{
	ReadTopStopSensor();
	//ReadSlackStopSensor();//not using the slack sensor anymore
	if((_state == BLOCKED_BY_TOP_SENSOR) || (_state == BLOCKED_BY_SLACK_SENSOR))
	{//sensors have priority over position lilmits
		 
	} else {
		int currPosition = *_encoderCounter;
		int minPosition = GetMinPosition();
		if(currPosition < minPosition)
		{
			_MinPositionReached();
		} else if(currPosition > GetMaxPosition())
		{
			_MaxPositionReached();
		} else {
			int direction = GetDirection();
			if(direction == DIRECTION_DOWN)
			{
				if((_targetPosition != TARGET_NONE) && (currPosition > _targetPosition))
				{
					_TargetReached();
				} else {
					if(_state == GOING_TO_TARGET)
					{
						_UpdateSpeedRamp();
					} else {
						_GoingDown();
					}
				}
			} else {
					if((_targetPosition != TARGET_NONE) && (currPosition < _targetPosition))
					{
						_TargetReached();
					} else {
						if(_state == GOING_TO_TARGET)
						{
							_UpdateSpeedRamp();
						} else {
							_GoingUp();
						}
				 }
			}
		}
	}
}

void PinneMotor::_UpdateSpeedRamp()
{
	if(_speedRamper->Calculate( GetCurrentPosition() ))
	{
		int newSpeed;
		newSpeed = _speedRamper->GetCurrentValue();
		SetSpeed(newSpeed);
	}
}

void PinneMotor::ReadTopStopSensor()
{
	int newReading;
	newReading = digitalRead(_topStopSensorPin);
	if(newReading != _lastTopSensorReading)
	{
		_lastTopSensorReadTime = millis();
	}
	if((millis() - _lastTopSensorReadTime) > topSensorDebounceDelay)
	{
		if(newReading != _topStopSensorValue)
		{
			_topStopSensorValue = newReading;
			if(_topStopSensorValue == TOP_SENSOR_IN)
			{
				_TopStopSensorIn();
			} else if( _topStopSensorValue == TOP_SENSOR_OUT){
				_TopStopSensorOut();
			}
		}
	}
	_lastTopSensorReading = newReading;
}

void PinneMotor::ReadSlackStopSensor()
{
	int newReading;
	newReading = digitalRead(_slackStopSensorPin);
	if(newReading != _lastSlackSensorReading)
	{
		_lastSlackSensorReadTime = millis();
	}
	if((millis() - _lastSlackSensorReadTime) > slackSensorDebounceDelay)
	{
		if(newReading != _slackStopSensorValue)
		{
			_slackStopSensorValue = newReading;
			if(_slackStopSensorValue == SLACK_SENSOR_IN)
			{
				_SlackStopSensorIn();
			} else if( _topStopSensorValue == SLACK_SENSOR_OUT){
				_SlackStopSensorOut();
			}
		}
	}
	_lastSlackSensorReading = newReading;
}

void PinneMotor::_TopStopSensorIn()
{
	Stop();
	_state = BLOCKED_BY_TOP_SENSOR;
	SetCurrentPosition(POSITION_ALL_UP);
	NotifyStateChange(BLOCKED_BY_TOP_SENSOR, _address);
}

void PinneMotor::_TopStopSensorOut()
{
	int direction;
	direction = GetDirection();
	if(direction == DIRECTION_DOWN)
	{
		_GoingDown();
	} else if(direction == DIRECTION_UP) {
		_GoingUp();
	}
}

void PinneMotor::_SlackStopSensorOut()
{
	Stop();
	_state = BLOCKED_BY_SLACK_SENSOR;
	NotifyStateChange(BLOCKED_BY_SLACK_SENSOR, _address);
}

void PinneMotor::_SlackStopSensorIn()
{
 int direction;
	direction = GetDirection();
	if(direction == DIRECTION_DOWN)
	{
		_GoingDown();
	} else if(direction == DIRECTION_UP) {
		_GoingUp();
	}
}


void PinneMotor::_GoingUp()
{
	if(_state != GOING_UP)
	{
		NotifyStateChange(GOING_UP, _address);
		_state = GOING_UP;
	}
}

void PinneMotor::_GoingDown()
{
	if(_state != GOING_DOWN)
	{
		NotifyStateChange(GOING_DOWN, _address);
		_state = GOING_DOWN;
	}
}

void PinneMotor::_AbsMinPositionReached()
{
	if(_state != BLOCKED_BY_ABS_MIN_POSITION)
	{
		Stop();
		NotifyStateChange(BLOCKED_BY_ABS_MIN_POSITION, _address);
		_state = BLOCKED_BY_ABS_MIN_POSITION;
	}
}

void PinneMotor::_MinPositionReached()
{
	if(_state != BLOCKED_BY_MIN_POSITION)
	{
		Stop();
		NotifyStateChange(BLOCKED_BY_MIN_POSITION, _address);
		_state = BLOCKED_BY_MIN_POSITION;
	}
}

void PinneMotor::_MaxPositionReached()
{
	if(_state != BLOCKED_BY_MAX_POSITION)
	{
		Stop();
		NotifyStateChange(BLOCKED_BY_MAX_POSITION, _address);
		_state = BLOCKED_BY_MAX_POSITION;
	}
}

void PinneMotor::_TargetReached()
{
	if(_state != STOPPED_AT_TARGET)
	{
		Stop();
		_targetPosition = TARGET_NONE;
		NotifyStateChange(STOPPED_AT_TARGET, _address);
		_state = STOPPED_AT_TARGET;
	}
}

void PinneMotor::_GoingToTarget()
{
	_state = GOING_TO_TARGET;
	NotifyStateChange(GOING_TO_TARGET, _address);
}

void PinneMotor::SetTargetPosition(int targetPosition)
{
	int value;
	if(targetPosition == TARGET_NONE)
	{
		_targetPosition = TARGET_NONE;
	} else {
		value = constrain(targetPosition, GetMinPosition(), GetMaxPosition());
		_targetPosition = value;
		int currPosition = GetCurrentPosition();
	 
		//change the direction if target in the opposite direction
		if(GetDirection() == DIRECTION_DOWN)
		{
			if(currPosition > _targetPosition)
			{
				SetDirection(DIRECTION_UP);
			}
		} else {
			if(currPosition < _targetPosition)
			{
				SetDirection(DIRECTION_DOWN);
			}
		}
	}
}

void PinneMotor::SetCurrentPosition(int currentPosition)
{
	currentPosition = constrain(currentPosition, GetMinPosition(), GetMaxPosition());
	noInterrupts();
	*_encoderCounter = currentPosition;
	interrupts();
}

void PinneMotor::SetBrake(int brake)
{
	_driver->SetBrake(brake);
}

void PinneMotor::SetMinPosition(int minPosition)
{
	_minPosition = max(minPosition, POSITION_ALL_UP);
}

void PinneMotor::SetMaxPosition(int maxPosition)
{
	_maxPosition = min(maxPosition, POSITION_DEFAULT_MAX);
}

void PinneMotor::GoToParkingPosition()
{
	//Check if already at top and do parking only if the sensor is out
	_topStopSensorValue = digitalRead(_topStopSensorPin);
	if(_topStopSensorValue == TOP_SENSOR_OUT)
	{
		//fake the currentPosition to default max
		SetCurrentPosition(POSITION_DEFAULT_MAX);
		SetDirection(DIRECTION_UP);
		SetSpeed(VNH5019Driver::SPEED_MAX / 4);	 
	}
}

void PinneMotor::GoToTargetPosition(int value)
{
	if(value > 0)
	{
		if(GetTargetPosition() != TARGET_NONE)
		{
			_speedRamper->Start(GetCurrentPosition(), GetTargetPosition(), value);
			_GoingToTarget();
		}
	} else {
		if(GetDirection() == DIRECTION_DOWN)//cancel goto target when speed is set
		{
			_GoingDown();
		} else {
			_GoingUp();
		}
	}
}

void PinneMotor::SetGoToSpeedRampUp(int value)
{
	_speedRamper->SetRampUp(static_cast<float>(value));
}

void PinneMotor::SetGoToSpeedRampDown(int value)
{
	_speedRamper->SetRampDown(static_cast<float>(value));
}

void PinneMotor::SetGoToSpeedScaling(int value)
{
	_speedRamper->SetSpeedScaling(static_cast<float>(value) / 1000.0);
}
