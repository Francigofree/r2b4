#include "wheel_projection.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace r2b4_nav2 {
// Recheck the changed steady command, including t=0 and the endpoint. The
// one-second horizon matches the previous adapter; spacing is <= half a cell
// and <= .05 rad. Unknown/out-of-map is unavailable, never free space.
bool commandCollides(nav2_costmap_2d::Costmap2D *map,
    const nav2_costmap_2d::Footprint &footprint, double x, double y, double yaw,
    double v, double omega) {
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::max(
      std::abs(v) / (map->getResolution() * .5), std::abs(omega) / .05))));
  if (steps > 64) return true;
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps, angle = yaw + omega*t;
    const double px = x + (std::abs(omega) < 1e-9 ? v*t*std::cos(yaw)
        : v/omega*(std::sin(angle)-std::sin(yaw)));
    const double py = y + (std::abs(omega) < 1e-9 ? v*t*std::sin(yaw)
        : v/omega*(std::cos(yaw)-std::cos(angle)));
    unsigned int mx, my;
    if (!map->worldToMap(px, py, mx, my) ||
        map->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return true;
    const double cost = checker.footprintCostAtPose(px, py, angle, footprint);
    if (cost < 0. || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) return true;
  }
  return false;
}

class WheelAwareMPPIController : public nav2_mppi_controller::MPPIController {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    MPPIController::configure(parent, name, tf, costmap);
    const auto node = parent.lock();
    auto value = [&](const char *key) {
      const auto parameter = name+"."+key;
      nav2_util::declare_parameter_if_not_declared(node, parameter, rclcpp::ParameterValue(0.));
      const double result = node->get_parameter(parameter).as_double();
      if (!std::isfinite(result) || result <= 0.)
        throw std::runtime_error("Missing/invalid wheel adapter parameter: "+parameter);
      return result;
    };
    limits_ = {value("track_width_m"), value("wheel_minimum_mps"), value("wheel_maximum_mps"),
               value("platform_max_v_mps"), value("platform_max_omega_rad_s"), value("max_curvature_rad_per_m")};
    if (limits_.minimum > limits_.maximum)
      throw std::runtime_error("Wheel minimum exceeds maximum");
    auto getParam = parameters_handler_->getParamGetter(name);
    getParam(mission_max_v_, "mission_max_v_mps", limits_.max_v);
    getParam(mission_max_omega_, "mission_max_omega_rad_s", limits_.max_omega);
    raw_publisher_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/r2b4/nav2/raw_cmd_vel", rclcpp::QoS(1));
  }

  void activate() override { MPPIController::activate(); raw_publisher_->on_activate(); }
  void deactivate() override { raw_publisher_->on_deactivate(); MPPIController::deactivate(); }
  void cleanup() override { raw_publisher_.reset(); MPPIController::cleanup(); }

  void setSpeedLimit(const double &speed_limit, const bool &percentage) override {
    // A mission maximum caps the current nominal, including dynamic edits.
    std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
    const double nominal = parent_.lock()->get_parameter(name_+".vx_max").as_double();
    double ratio = 0.;
    if (std::isfinite(speed_limit) && nominal > 0.) {
      ratio = speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT ? 100.
          : (percentage ? std::clamp(speed_limit, 0., 100.)
                        : std::clamp(speed_limit / nominal, 0., 1.) * 100.);
    }
    optimizer_.setSpeedLimit(ratio, true);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *goal_checker) override {
    auto command = MPPIController::computeVelocityCommands(pose, speed, goal_checker);
    raw_publisher_->publish(command);
    auto limits = limits_;
    const auto node = parent_.lock();
    const double max_v = node->get_parameter(name_+".mission_max_v_mps").as_double();
    const double max_omega = node->get_parameter(name_+".mission_max_omega_rad_s").as_double();
    if (!std::isfinite(max_v) || !std::isfinite(max_omega) || max_v <= 0. || max_omega <= 0.)
      throw nav2_core::NoValidControl("Invalid mission envelope");
    limits.max_v = std::min(limits.max_v, max_v);
    limits.max_omega = std::min(limits.max_omega, max_omega);
    const auto feasible = limits.project(command.twist.linear.x, command.twist.angular.z);
    command.twist.linear.x = feasible.first;
    command.twist.angular.z = feasible.second;
    auto *map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto &q = pose.pose.orientation;
    const double yaw = std::atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z));
    if (commandCollides(map, costmap_ros_->getRobotFootprint(), pose.pose.position.x,
                       pose.pose.position.y, yaw, feasible.first, feasible.second))
      throw nav2_core::NoValidControl("Wheel-feasible R2B4 command collides in local costmap");
    return command;
  }
 private:
  WheelLimits limits_{};
  double mission_max_v_{}, mission_max_omega_{};
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr raw_publisher_;
};
}  // namespace r2b4_nav2

PLUGINLIB_EXPORT_CLASS(r2b4_nav2::WheelAwareMPPIController, nav2_core::Controller)
