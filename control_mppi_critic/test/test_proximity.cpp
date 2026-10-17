#include "gtest/gtest.h"
#include "control_mppi_critic/proximity.hpp"
TEST(PredictionProximity, BroadSupportRemainsWeakButReal) {
  using control::proximity;
  const auto sharp=proximity(0,0,.01,0,.01,.536,1e-6);
  const auto broad=proximity(0,0,4,0,4,.536,1e-6);
  EXPECT_GT(sharp,broad);EXPECT_GT(broad,0.);
  EXPECT_GT(proximity(0,0,.1,0,.1,.536,1e-6),proximity(2,0,.1,0,.1,.536,1e-6));
}
