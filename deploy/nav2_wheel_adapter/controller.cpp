#include "wheel_projection.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace r2b4_nav2 {
class WheelAwareRegulatedPurePursuitController
    : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    RegulatedPurePursuitController::configure(parent, name, tf, costmap);
    const auto node = parent.lock();
    auto value = [&](const char *key) {
      const auto parameter = name+"."+key;
      // Missing geometry is a configure failure; no guessed physical limits.
      nav2_util::declare_parameter_if_not_declared(node, parameter, rclcpp::ParameterValue(0.));
      double result = 0.; node->get_parameter(parameter, result);
      if (!std::isfinite(result) || result <= 0.)
        throw std::runtime_error("Missing/invalid wheel adapter parameter: "+parameter);
      return result;
    };
    limits_ = {value("track_width_m"), value("wheel_minimum_mps"), value("wheel_maximum_mps"),
               value("platform_max_v_mps"), value("platform_max_omega_rad_s"), value("max_curvature_rad_per_m")};
    if (limits_.minimum > limits_.maximum)
      throw std::runtime_error("Wheel minimum exceeds maximum");
    nav2_util::declare_parameter_if_not_declared(node, name+".mission_max_v_mps",
                                               rclcpp::ParameterValue(limits_.max_v));
    nav2_util::declare_parameter_if_not_declared(node, name+".mission_max_omega_rad_s",
                                               rclcpp::ParameterValue(limits_.max_omega));
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *goal_checker) override {
    // Keep all upstream planning, regulation and original collision checking.
    auto command = RegulatedPurePursuitController::computeVelocityCommands(pose, speed, goal_checker);
    auto limits = limits_;
    const auto node = node_.lock();
    const double max_v = node->get_parameter(plugin_name_+".mission_max_v_mps").as_double();
    const double max_omega = node->get_parameter(plugin_name_+".mission_max_omega_rad_s").as_double();
    if (!std::isfinite(max_v) || !std::isfinite(max_omega) || max_v <= 0. || max_omega <= 0.)
      throw nav2_core::NoValidControl("Invalid mission envelope");
    limits.max_v = std::min(limits.max_v, max_v);
    limits.max_omega = std::min(limits.max_omega, max_omega);
    const auto feasible = limits.project(command.twist.linear.x, command.twist.angular.z);
    command.twist.linear.x = feasible.first;
    command.twist.angular.z = feasible.second;
    // Projection changes curvature. Recheck that exact steady proposal with
    // Nav2's current full-footprint costmap before it leaves the controller.
    auto *map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    if (collision_checker_->isCollisionImminent(pose, feasible.first, feasible.second,
                                               std::numeric_limits<double>::infinity()))
      throw nav2_core::NoValidControl("Wheel-feasible R2B4 command collides in local costmap");
    return command;
  }
 private:
  WheelLimits limits_{};
};
}  // namespace r2b4_nav2

PLUGINLIB_EXPORT_CLASS(r2b4_nav2::WheelAwareRegulatedPurePursuitController, nav2_core::Controller)
