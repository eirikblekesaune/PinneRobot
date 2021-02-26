#pragma once
#include <Metro.h>

class PositionMove{
	public:
          PositionMove(int startPosition, int targetPosition)
              : _startPosition(startPosition),
                _targetPosition(targetPosition){};
          void Update(){};

        protected:
          int _startPosition;
          int _targetPosition;

          Metro *_metro;
};
