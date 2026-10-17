#pragma once
#include <algorithm>
#include <cmath>
namespace control {
// PROVISIONAL soft score, not calibrated collision probability. Broad support stays nonzero.
inline double proximity(double dx, double dy, double xx, double xy, double yy,
                        double radius, double epsilon) {
  const double r2 = radius * radius;
  xx += r2; yy += r2;
  const double determinant = std::max(epsilon, xx * yy - xy * xy);
  const double mahalanobis = std::max(0.0, (yy*dx*dx - 2*xy*dx*dy + xx*dy*dy)/determinant);
  return r2 / std::sqrt(determinant) * std::exp(-0.5 * mahalanobis);
}
}
