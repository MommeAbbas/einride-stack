"""Measured chassis deadbands, compensated before the final collision guard."""
import math


def hardware_command(linear, angular, forward_limit, reverse_limit, angular_limit,
                     minimum_linear, minimum_angular, linear_deadband=0.02, angular_deadband=0.05):
    """Preserve gentle forward curves and exact zeros; never raise beyond a cap.

    Inputs/limits: m/s and rad/s. Thresholds are ROS parameters at each caller.
    Commands whose requested boost exceeds an authorised cap are suppressed.
    """
    if not all(math.isfinite(v) for v in (linear,angular,forward_limit,reverse_limit,angular_limit,
                                         minimum_linear,minimum_angular,linear_deadband,angular_deadband)):
        return 0.,0.
    if min(forward_limit,reverse_limit,angular_limit,minimum_linear,minimum_angular,linear_deadband,angular_deadband)<0:
        return 0.,0.
    v=max(-reverse_limit,min(forward_limit,linear));w=max(-angular_limit,min(angular_limit,angular))
    if linear_deadband<abs(v)<minimum_linear:
        if (reverse_limit if v<0 else forward_limit)<minimum_linear:return 0.,0.
        v=math.copysign(minimum_linear,v)
    if abs(v)<linear_deadband and angular_deadband<abs(w)<minimum_angular:
        if angular_limit<minimum_angular:return 0.,0.
        w=math.copysign(minimum_angular,w)
    return v,w
