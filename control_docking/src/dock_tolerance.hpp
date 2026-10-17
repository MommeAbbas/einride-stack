#pragma once
#include <cmath>
namespace control {
inline bool withinDockTolerance(double dx,double dy,double dock_yaw,double yaw_error,
    double longitudinal,double lateral,double heading) {
  return std::abs(std::cos(dock_yaw)*dx+std::sin(dock_yaw)*dy)<longitudinal &&
         std::abs(-std::sin(dock_yaw)*dx+std::cos(dock_yaw)*dy)<lateral &&
         std::abs(yaw_error)<heading;
}
}
