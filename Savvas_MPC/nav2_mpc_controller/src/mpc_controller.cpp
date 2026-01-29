#include "nav2_mpc_controller/mpc_controller.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_mpc_controller
{

void MpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MpcController: failed to lock node");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Parameters (namespaced by plugin name) per Nav2 tutorial convention :contentReference[oaicite:2]{index=2}
  using nav2_util::declare_parameter_if_not_declared;

  declare_parameter_if_not_declared(node, plugin_name_ + ".N", rclcpp::ParameterValue(p_.N));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dt", rclcpp::ParameterValue(p_.dt));

  declare_parameter_if_not_declared(node, plugin_name_ + ".v_max", rclcpp::ParameterValue(p_.v_max));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_min", rclcpp::ParameterValue(p_.v_min));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_max", rclcpp::ParameterValue(p_.w_max));

  declare_parameter_if_not_declared(node, plugin_name_ + ".dv_max", rclcpp::ParameterValue(p_.dv_max));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dw_max", rclcpp::ParameterValue(p_.dw_max));

  declare_parameter_if_not_declared(node, plugin_name_ + ".w_pos", rclcpp::ParameterValue(p_.w_pos));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_yaw", rclcpp::ParameterValue(p_.w_yaw));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_v", rclcpp::ParameterValue(p_.w_v));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_w", rclcpp::ParameterValue(p_.w_w));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_dv", rclcpp::ParameterValue(p_.w_dv));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_dw", rclcpp::ParameterValue(p_.w_dw));

  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_iters", rclcpp::ParameterValue(p_.opt_iters));
  declare_parameter_if_not_declared(node, plugin_name_ + ".step_size", rclcpp::ParameterValue(p_.step_size));
  declare_parameter_if_not_declared(node, plugin_name_ + ".grad_eps", rclcpp::ParameterValue(p_.grad_eps));

  declare_parameter_if_not_declared(node, plugin_name_ + ".prune_dist", rclcpp::ParameterValue(p_.prune_dist));
  declare_parameter_if_not_declared(node, plugin_name_ + ".ref_stride", rclcpp::ParameterValue(p_.ref_stride));
  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(p_.transform_tolerance));

  declare_parameter_if_not_declared(node, plugin_name_ + ".cmd_smoothing", rclcpp::ParameterValue(p_.cmd_smoothing));

  node->get_parameter(plugin_name_ + ".N", p_.N);
  node->get_parameter(plugin_name_ + ".dt", p_.dt);

  node->get_parameter(plugin_name_ + ".v_max", p_.v_max);
  node->get_parameter(plugin_name_ + ".v_min", p_.v_min);
  node->get_parameter(plugin_name_ + ".w_max", p_.w_max);

  node->get_parameter(plugin_name_ + ".dv_max", p_.dv_max);
  node->get_parameter(plugin_name_ + ".dw_max", p_.dw_max);

  node->get_parameter(plugin_name_ + ".w_pos", p_.w_pos);
  node->get_parameter(plugin_name_ + ".w_yaw", p_.w_yaw);
  node->get_parameter(plugin_name_ + ".w_v", p_.w_v);
  node->get_parameter(plugin_name_ + ".w_w", p_.w_w);
  node->get_parameter(plugin_name_ + ".w_dv", p_.w_dv);
  node->get_parameter(plugin_name_ + ".w_dw", p_.w_dw);

  node->get_parameter(plugin_name_ + ".opt_iters", p_.opt_iters);
  node->get_parameter(plugin_name_ + ".step_size", p_.step_size);
  node->get_parameter(plugin_name_ + ".grad_eps", p_.grad_eps);

  node->get_parameter(plugin_name_ + ".prune_dist", p_.prune_dist);
  node->get_parameter(plugin_name_ + ".ref_stride", p_.ref_stride);
  node->get_parameter(plugin_name_ + ".transform_tolerance", p_.transform_tolerance);

  node->get_parameter(plugin_name_ + ".cmd_smoothing", p_.cmd_smoothing);

  if (p_.N < 1) {
    throw std::runtime_error("MpcController: N must be >= 1");
  }

  u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
  last_u_.setZero();
  v_max_eff_ = p_.v_max;

  RCLCPP_INFO(logger_, "Configured MPC controller plugin '%s' (N=%d dt=%.3f)",
    plugin_name_.c_str(), p_.N, p_.dt);
}

void MpcController::cleanup()
{
  has_plan_ = false;
  global_plan_.poses.clear();
  u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
  last_u_.setZero();
}

void MpcController::activate()
{
  // Nothing required here for this simple implementation
}

void MpcController::deactivate()
{
  // Nothing required here for this simple implementation
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  has_plan_ = !global_plan_.poses.empty();

  // Reset warm start on new plan
  u_seq_.setZero();
  last_u_.setZero();
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  has_speed_limit_ = true;

  if (percentage) {
    // speed_limit in [0, 100] typically
    const double pct = std::max(0.0, std::min(100.0, speed_limit)) / 100.0;
    v_max_eff_ = pct * p_.v_max;
  } else {
    v_max_eff_ = std::max(0.0, speed_limit);
  }
}

nav_msgs::msg::Path MpcController::transformPlanToBaseFrame(
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  nav_msgs::msg::Path out;
  out.header.frame_id = costmap_ros_->getBaseFrameID();
  out.header.stamp = robot_pose.header.stamp;

  if (plan.poses.empty()) {
    return out;
  }

  // Transform each pose into base frame
  const std::string base = costmap_ros_->getBaseFrameID();

  // We'll stop after prune_dist in the plan to keep it small
  double acc_dist = 0.0;
  geometry_msgs::msg::PoseStamped prev = plan.poses.front();

  for (size_t i = 0; i < plan.poses.size(); ++i) {
    const auto & p_in = plan.poses[i];

    if (i > 0) {
      acc_dist += nav2_util::geometry_utils::euclidean_distance(prev, p_in);
      prev = p_in;
      if (acc_dist > p_.prune_dist) {
        break;
      }
    }

    geometry_msgs::msg::PoseStamped p_out;
    try {
      tf_->transform(p_in, p_out, base,
        tf2::durationFromSec(p_.transform_tolerance));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "TF transform plan->base failed: %s", ex.what());
      break;
    }
    out.poses.push_back(p_out);
  }

  return out;
}

void MpcController::buildReferenceHorizon(
  const nav_msgs::msg::Path & local_plan_base,
  int N,
  Eigen::MatrixXd & xref) const
{
  xref = Eigen::MatrixXd::Zero(N, 3);

  if (local_plan_base.poses.empty()) {
    return;
  }

  // Sample points along the plan
  int idx = 0;
  for (int k = 0; k < N; ++k) {
    idx = std::min<int>(idx, static_cast<int>(local_plan_base.poses.size()) - 1);
    const auto & ps = local_plan_base.poses[idx];

    const double x = ps.pose.position.x;
    const double y = ps.pose.position.y;
    const double yaw = tf2::getYaw(ps.pose.orientation);

    xref(k, 0) = x;
    xref(k, 1) = y;
    xref(k, 2) = yaw;

    idx += std::max(1, p_.ref_stride);
    if (idx >= static_cast<int>(local_plan_base.poses.size())) {
      idx = static_cast<int>(local_plan_base.poses.size()) - 1; // hold last
    }
  }
}

Eigen::Vector3d MpcController::stepDiffDrive(
  const Eigen::Vector3d & x,
  const Eigen::Vector2d & u,
  double dt)
{
  const double px = x(0);
  const double py = x(1);
  const double yaw = x(2);

  const double v = u(0);
  const double w = u(1);

  Eigen::Vector3d xn;
  xn(0) = px + dt * v * std::cos(yaw);
  xn(1) = py + dt * v * std::sin(yaw);
  xn(2) = yaw + dt * w;
  return xn;
}

Eigen::Vector2d MpcController::clampControl(const Eigen::Vector2d & u) const
{
  Eigen::Vector2d uc = u;

  const double v_max = v_max_eff_;
  uc(0) = std::max(p_.v_min, std::min(v_max, uc(0)));
  uc(1) = std::max(-p_.w_max, std::min(p_.w_max, uc(1)));
  return uc;
}

double MpcController::rolloutCost(
  const Eigen::Vector3d & x0,
  const Eigen::MatrixXd & u_seq,
  const Eigen::MatrixXd & xref,
  const Eigen::Vector2d & u_prev) const
{
  Eigen::Vector3d x = x0;
  Eigen::Vector2d u_last = u_prev;

  double J = 0.0;

  for (int k = 0; k < u_seq.rows(); ++k) {
    Eigen::Vector2d u(u_seq(k, 0), u_seq(k, 1));
    u = clampControl(u);

    // Tracking error
    const double ex = x(0) - xref(k, 0);
    const double ey = x(1) - xref(k, 1);

    // Yaw error wrapped
    double epsi = x(2) - xref(k, 2);
    while (epsi > M_PI) epsi -= 2.0 * M_PI;
    while (epsi < -M_PI) epsi += 2.0 * M_PI;

    // Input smoothness (approx accel)
    const double dv = (u(0) - u_last(0)) / p_.dt;
    const double dw = (u(1) - u_last(1)) / p_.dt;

    J += p_.w_pos * (ex * ex + ey * ey)
      + p_.w_yaw * (epsi * epsi)
      + p_.w_v * (u(0) * u(0))
      + p_.w_w * (u(1) * u(1))
      + p_.w_dv * (dv * dv)
      + p_.w_dw * (dw * dw);

    // Propagate dynamics
    x = stepDiffDrive(x, u, p_.dt);
    u_last = u;
  }

  return J;
}

void MpcController::optimizeControlSequence(
  const Eigen::Vector3d & x0,
  const Eigen::MatrixXd & xref,
  const Eigen::Vector2d & u_prev,
  Eigen::MatrixXd & u_seq_io)
{
  // Projected finite-difference gradient descent on u sequence
  const int N = static_cast<int>(u_seq_io.rows());
  const double eps = p_.grad_eps;

  for (int it = 0; it < p_.opt_iters; ++it) {
    const double J0 = rolloutCost(x0, u_seq_io, xref, u_prev);

    Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(N, 2);

    for (int k = 0; k < N; ++k) {
      for (int j = 0; j < 2; ++j) {
        Eigen::MatrixXd u_pert = u_seq_io;
        u_pert(k, j) += eps;
        const double Jp = rolloutCost(x0, u_pert, xref, u_prev);
        grad(k, j) = (Jp - J0) / eps;
      }
    }

    u_seq_io -= p_.step_size * grad;

    // Clamp each control after update
    for (int k = 0; k < N; ++k) {
      Eigen::Vector2d u(u_seq_io(k, 0), u_seq_io(k, 1));
      u = clampControl(u);
      u_seq_io(k, 0) = u(0);
      u_seq_io(k, 1) = u(1);
    }
  }
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  if (!has_plan_ || global_plan_.poses.empty()) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // Transform plan to base frame
  nav_msgs::msg::Path local_plan = transformPlanToBaseFrame(global_plan_, pose);
  if (local_plan.poses.size() < 2) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // Reference horizon in base frame (so x0 is near zero-ish)
  Eigen::MatrixXd xref;
  buildReferenceHorizon(local_plan, p_.N, xref);

  // State in base frame: robot is at origin with yaw=0
  // (because plan was transformed into base frame)
  Eigen::Vector3d x0(0.0, 0.0, 0.0);

  // Warm-start shift (receding horizon)
  // shift u_seq_ up by one, keep last
  if (u_seq_.rows() == p_.N) {
    for (int k = 0; k < p_.N - 1; ++k) {
      u_seq_.row(k) = u_seq_.row(k + 1);
    }
    u_seq_.row(p_.N - 1) = u_seq_.row(p_.N - 2);
  } else {
    u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
  }

  // Previous command (for accel penalty)
  Eigen::Vector2d u_prev(last_u_(0), last_u_(1));

  // Optimize
  optimizeControlSequence(x0, xref, u_prev, u_seq_);

  // Take first control
  Eigen::Vector2d u0(u_seq_(0, 0), u_seq_(0, 1));
  u0 = clampControl(u0);

  // Simple command smoothing
  const double a = std::max(0.0, std::min(1.0, p_.cmd_smoothing));
  Eigen::Vector2d u_smoothed = a * last_u_ + (1.0 - a) * u0;

  // Also respect incoming current velocity (optional):
  (void)velocity; // not used here, but you can incorporate it in the cost.

  last_u_ = u_smoothed;

  cmd.twist.linear.x = u_smoothed(0);
  cmd.twist.angular.z = u_smoothed(1);
  return cmd;
}

}  // namespace nav2_mpc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MpcController, nav2_core::Controller)
