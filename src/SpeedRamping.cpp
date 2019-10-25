
#include "SpeedRamping.h"
#include "kRISE_TABLE.h"
#define DEFAULT_RAMP_TIME 500

const unsigned long SpeedRamping::DEFAULT_CALC_INTERVAL = 50;

SpeedRamping::SpeedRamping(int minSpeed, int maxSpeed) :
	_minSpeed(minSpeed),
	_maxSpeed(maxSpeed),
	_calcInterval(DEFAULT_CALC_INTERVAL),
	_halfwayReached(false),
	_startPosition(0),
	_endPosition(0),
	_direction(RISING_RAMP),
	_rampUp(DEFAULT_RAMP_TIME),
	_rampDown(DEFAULT_RAMP_TIME),
	_speedScaling(1.0)
{
}


void SpeedRamping::Start(int startPosition, int endPosition, int peakDuration)
{
	//calculate halfway point
	_startPosition = startPosition;
	_endPosition = endPosition;
	_halfwayPosition = ((_endPosition - _startPosition) / 2) + _startPosition;
	_speedRange = abs(_maxSpeed - _minSpeed);
	_peakDuration = peakDuration;
	if(_startPosition <= _endPosition)
	{
		_direction = RISING_RAMP;
	} else {
		_direction = FALLING_RAMP;
	}
	_riseDelta = (1000.0 / _calcInterval) / _rampUp;
	_fallDelta = (1000.0 / _calcInterval) / _rampDown;
	_currentFloatIndex = 1.0; //this becomes subtracted by 1 upon look-up
	_halfwayReached = false;
}

boolean SpeedRamping::Calculate(int currPosition)
{
	unsigned long nowTime = millis();
	if((nowTime - _lastCalcTime) >= _calcInterval)
	{
		if(!_halfwayReached)
		{
			if(_direction == RISING_RAMP)
			{
				if(currPosition >= _halfwayPosition)
				{
					if(!_halfwayReached)
					_halfwayReached = true;
				}
			} else {
				if(currPosition <= _halfwayPosition)
				{
					if(!_halfwayReached)
					_halfwayReached = true;
				}
			}
			_currentFloatIndex = min(2.0, _currentFloatIndex + _riseDelta);
		} else {
			_peakDuration = _peakDuration - _calcInterval;
			if(_peakDuration < 0.0)
			{
				_currentFloatIndex = max(1.0, _currentFloatIndex - _fallDelta);
			} else {
				_currentFloatIndex = 2.0;
			}
		}
	 
		_lastCalcTime = nowTime;
		return true;
	} else {
		return false;
	}
}

void SpeedRamping::Stop()
{

}

int SpeedRamping::GetCurrentValue()
{
	int index;
	float result;
	index = max(0, static_cast<int>(floor((_currentFloatIndex - 1.0) * TABLE_SIZE)) - 1);
	result = (kRampTable[index] * _speedRange * _speedScaling) + _minSpeed;
	return static_cast<int>(result);
}

void SpeedRamping::SetMinSpeed(int speed)
{
	_minSpeed = speed;
}

void SpeedRamping::SetMaxSpeed(int speed)
{
	_maxSpeed = speed;
}

void SpeedRamping::SetSpeedScaling(float speedScaling)
{
	_speedScaling = constrain(speedScaling, 0.0, 1.0);
}

void SpeedRamping::SetRampUp(float value)
{
	_rampUp = max(value, 0.1);
}

void SpeedRamping::SetRampDown(float value)
{
	_rampDown = max(value, 0.1);
}

