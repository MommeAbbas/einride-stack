#include <cmath>
#include "dock_tolerance.hpp"
#include <string>
#include "opennav_docking/simple_non_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
namespace control {
// Installed SimpleNonChargingDock supplies pose filtering/staging/detection.
// Extension reason: upstream isDocked checks distance only, not dock-axis yaw;
// its single fixed detection topic also cannot select between eight tag IDs.
class TagDock : public opennav_docking::SimpleNonChargingDock {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override {
    SimpleNonChargingDock::configure(parent,name,tf);
    nav2_util::declare_parameter_if_not_declared(node_,name+".detection_topic",rclcpp::ParameterValue("/slam/live_docks/dock_0"));
    nav2_util::declare_parameter_if_not_declared(node_,name+".heading_tolerance",rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(node_,name+".lateral_tolerance",rclcpp::ParameterValue(0.03));
    node_->get_parameter(name+".heading_tolerance",heading_tolerance_);
    node_->get_parameter(name+".lateral_tolerance",lateral_tolerance_);
    std::string topic;node_->get_parameter(name+".detection_topic",topic);
    dock_pose_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>(topic,1,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr pose){detected_dock_pose_=*pose;});
  }
  bool isDocked() override {
    if (dock_pose_.header.frame_id.empty() ||
        (node_->now()-detected_dock_pose_.header.stamp).seconds()>external_detection_timeout_) return false;
    geometry_msgs::msg::PoseStamped base;base.header.frame_id=base_frame_id_;base.pose.orientation.w=1.;
    try {tf2_buffer_->transform(base,base,dock_pose_.header.frame_id);} catch(const tf2::TransformException &) {return false;}
    const double yaw=tf2::getYaw(dock_pose_.pose.orientation);
    const double dx=base.pose.position.x-dock_pose_.pose.position.x;
    const double dy=base.pose.position.y-dock_pose_.pose.position.y;
    return withinDockTolerance(dx,dy,yaw,
      angles::shortest_angular_distance(yaw,tf2::getYaw(base.pose.orientation)),
      docking_threshold_,lateral_tolerance_,heading_tolerance_);
  }
 private:
  double heading_tolerance_,lateral_tolerance_;
};
}
PLUGINLIB_EXPORT_CLASS(control::TagDock,opennav_docking_core::ChargingDock)
