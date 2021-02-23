#include <TargetPositionMover.h>


int main(int args, char **argv) {
  TargetPositionMover mover();

  mover.StopMove();
  mover.StartMove();
  mover.GetNextSpeedValue();
  mover.GetNumTicks();
  mover.GetDuration();
  mover.GetMaxSpeed();
  mover.GetMinSpeed();

  mover.PlanMoveWithDuration(100, 400, 5000, 0.0);
  mover.PlanMoveWithMaxSpeed(100, 400, 3.0, 0.1);
  return 0;
}
