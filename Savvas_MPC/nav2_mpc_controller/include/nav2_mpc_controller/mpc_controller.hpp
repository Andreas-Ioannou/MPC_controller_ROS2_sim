#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

#include <Eigen/Dense>

namespace nav2_mpc_controller
{

class MpcController : public nav2_core::Controller
{
public:
  MpcController() = default;
  ~MpcController() override = default;

  // ---- nav2_core::Controller lifecycle hooks ----
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Plan arrives here (new Nav2 API)
  void setPlan(const nav_msgs::msg::Path & path) override;

  // Compute command using stored plan
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct Params
  {
    // MPC timing
    int N{15};              // horizon steps
    double dt{0.1};         // seconds

    // Limits
    double v_max{0.5};
    double v_min{-0.1};     // allow slight reverse if you want
    double w_max{1.5};

    // Optional acceleration limits (softly enforced by cost)
    double dv_max{0.5};     // m/s^2
    double dw_max{2.5};     // rad/s^2

    // Cost weights
    double w_pos{5.0};
    double w_yaw{2.0};
    double w_v{0.2};
    double w_w{0.2};
    double w_dv{0.5};
    double w_dw{0.5};

    // Optimization
    int opt_iters{20};
    double step_size{0.3};      // gradient descent step
    double grad_eps{1e-3};      // finite-diff epsilon

    // Path handling
    double prune_dist{2.5};     // meters of plan to consider
    int ref_stride{1};          // take every k-th pose as ref
    double transform_tolerance{0.2};

    // Output smoothing
    double cmd_smoothing{0.2};  // 0..1 (higher = smoother, slower response)
  };

  // --- internal helpers ---
  nav_msgs::msg::Path transformPlanToBaseFrame(
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  void buildReferenceHorizon(
    const nav_msgs::msg::Path & local_plan_base,
    int N,
    Eigen::MatrixXd & xref /* Nx3 */) const;

  // State x = [x, y, yaw] in base frame. Controls u = [v, w]
  static Eigen::Vector3d stepDiffDrive(
    const Eigen::Vector3d & x,
    const Eigen::Vector2d & u,
    double dt);

  double rolloutCost(
    const Eigen::Vector3d & x0,
    const Eigen::MatrixXd & u_seq /* Nx2 */,
    const Eigen::MatrixXd & xref   /* Nx3 */,
    const Eigen::Vector2d & u_prev) const;

  void optimizeControlSequence(
    const Eigen::Vector3d & x0,
    const Eigen::MatrixXd & xref,
    const Eigen::Vector2d & u_prev,
    Eigen::MatrixXd & u_seq_io);

  Eigen::Vector2d clampControl(const Eigen::Vector2d & u) const;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_mpc_controller")};
  rclcpp::Clock::SharedPtr clock_;

  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  Params p_;

  nav_msgs::msg::Path global_plan_;   // stored plan
  bool has_plan_{false};

  // Receding horizon warm-start and smoothing
  Eigen::MatrixXd u_seq_;             // Nx2
  Eigen::Vector2d last_u_{0.0, 0.0};

  // Speed limit override from setSpeedLimit()
  bool has_speed_limit_{false};
  double speed_limit_{0.0};           // either absolute or percent handled into v_max_eff_
  double v_max_eff_{0.5};
};

}  // namespace nav2_mpc_controller
