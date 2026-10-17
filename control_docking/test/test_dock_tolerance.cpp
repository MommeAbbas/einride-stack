#include "gtest/gtest.h"
#include "dock_tolerance.hpp"
TEST(DockTolerance, IndependentAxisLimits) {
  EXPECT_TRUE(control::withinDockTolerance(.003,.02,0.,.04,.005,.03,.05));
  EXPECT_FALSE(control::withinDockTolerance(.02,.003,0.,.04,.005,.03,.05));
  EXPECT_FALSE(control::withinDockTolerance(.003,.02,0.,.06,.005,.03,.05));
  EXPECT_FALSE(control::withinDockTolerance(.003,.031,0.,.04,.005,.03,.05));
}
TEST(DockTolerance, DockAxisRotation) {
  EXPECT_TRUE(control::withinDockTolerance(-.02,.003,1.5707963267948966,.04,.005,.03,.05));
}
