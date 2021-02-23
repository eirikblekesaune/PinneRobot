#include <TargetPositionMover.h>
#include <unity.h>

void test_function_IsBlocked() {
  TargetPositionMover mover();
  TEST_ASSERT_FALSE(mover.IsBlocked());
}

int main(int args, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_function_IsBlocked);

  UNITY_END();
  return 0;
}
