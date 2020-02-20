#ifndef SpeedRamping_h
#define SpeedRamping_h

#include <Arduino.h>
#include "PinneAPI.h"
class SpeedRamping {
	public:
	SpeedRamping(int minSpeed, int maxSpeed);

	const static unsigned long DEFAULT_CALC_INTERVAL;
	enum {
		RISING_RAMP,
		FALLING_RAMP
	};
	void Start(int startValue, int endValue, int duration);
	void Stop();
	boolean Calculate(int currPosition);


	void SetMinSpeed(int speed);
	void SetMaxSpeed(int speed);
	void SetSpeedScaling(float speedScaling);
	void SetRampUp(float value);
	void SetRampDown(float value);

	int GetMinSpeed() {return static_cast<int>(_minSpeed); };
	int GetMaxSpeed() {return static_cast<int>(_maxSpeed); };
	float GetSpeedScaling() {return _speedScaling; };
	float GetRampUp() {return _rampUp; };
	float GetRampDown() {return _rampDown; };
	int GetCurrentValue();

	private:
	float _minSpeed;
	float _maxSpeed;
	float _speedScaling;
	float _rampUp;
	float _rampDown;

	int _halfwayPosition;
	int _startPosition;
	int _endPosition;
	float _riseDelta;
	float _fallDelta;

	boolean _halfwayReached;
	unsigned long _calcInterval;
	unsigned long _lastCalcTime;
	float _currentFloatIndex;
	int _direction;
	int _speedRange;
	int _peakDuration;
};


#endif
