#pragma once
#include <PID_v1.h>
#include <PositionMove.h>

class CurvedPositionMove : public PositionMove {
public:
  CurvedPositionMove(int startPosition, int targetPosition, double minSpeed,
                     double beta, double skirtRatio, int tickDuration)
      : PositionMove(startPosition, targetPosition) {}

private:
  PID *_pid;
};
