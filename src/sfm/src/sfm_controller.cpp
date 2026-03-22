#include "sfm_controller_plugin/sfm_controller.hpp"
#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <utility>
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace sfm_controller_plugin
{

void SfmController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = node_.lock();
    plugin_name_ = name;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf;
    
    if (!node) {
        throw std::runtime_error("Failed to lock node");
    }
    
    clock_ = node->get_clock();
    
    // Get base frame from costmap
    base_frame_id_ = costmap_ros_->getBaseFrameID();
    
    // === SFM Parameters (Paper nomenclature: A, B, d, α, β, γ, δ, λ) ===
    // Paper eq. (5): f_i,q^int = A_q * exp((d_q - d_i,q) / B_q) * n^_i,q
    // Paper eq. (14): F_r = α*f_r,dest^goal + β*f_r,i^goal + γ*F_r^per + δ*F_r^obs
    
    // Repulsion parameters (A, B, d)
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".A", rclcpp::ParameterValue(10.0));  // A: strength/amplitude
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".B", rclcpp::ParameterValue(1.3));   // B: range/decay
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".d", rclcpp::ParameterValue(0.7));  // d: contact distance
    
    // Force combination weights (α, β, γ, δ)
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".alpha", rclcpp::ParameterValue(1.01));  // α: weight to destination
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".beta", rclcpp::ParameterValue(0.0));   // β: weight to target person (reserved)
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".gamma", rclcpp::ParameterValue(0.0));  // γ: weight person repulsion (reserved)
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".delta", rclcpp::ParameterValue(0.99));  // δ: weight obstacle repulsion
    
    // Anisotropy (λ)
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".lambda", rclcpp::ParameterValue(1.0));  // λ: field of view (1.0 = isotropic)
    
    // Other SFM parameters
    // Default to LETHAL_OBSTACLE (254) to only consider real obstacles, not inflated areas
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".obstacle_cost_threshold", rclcpp::ParameterValue(254.0));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".dist_threshold", rclcpp::ParameterValue(0.12));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".end_force", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".goal_force_scale", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_repulsive_magnitude", rclcpp::ParameterValue(0.0));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_obstacles", rclcpp::ParameterValue(500));
    // === Relaxation Gain (Paper: k) ===
    // Paper eq. (3): f_i^goal = k * (v_i^0 - v_i)
    //   k: Inverse of relaxation time, controls velocity adaptation speed
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".k_lin", rclcpp::ParameterValue(2.0));  // k: linear relaxation gain
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".k_ang", rclcpp::ParameterValue(2.0));  // k: angular relaxation gain
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".d_lin", rclcpp::ParameterValue(0.35));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".d_ang", rclcpp::ParameterValue(0.35));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.45));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(0.55));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".tolerance", rclcpp::ParameterValue(0.15));
    
    // Read all SFM parameters (paper names only)
    SFMParameters params;
    node->get_parameter(plugin_name_ + ".A", params.A);
    node->get_parameter(plugin_name_ + ".B", params.B);
    node->get_parameter(plugin_name_ + ".d", params.d);
    node->get_parameter(plugin_name_ + ".alpha", params.alpha);
    node->get_parameter(plugin_name_ + ".beta", params.beta);
    node->get_parameter(plugin_name_ + ".gamma", params.gamma);
    node->get_parameter(plugin_name_ + ".delta", params.delta);
    node->get_parameter(plugin_name_ + ".lambda", params.lambda);
    node->get_parameter(plugin_name_ + ".obstacle_cost_threshold", params.obstacle_cost_threshold);
    node->get_parameter(plugin_name_ + ".dist_threshold", params.dist_threshold);
    node->get_parameter(plugin_name_ + ".end_force", params.end_force);
    node->get_parameter(plugin_name_ + ".goal_force_scale", params.goal_force_scale);
    node->get_parameter(plugin_name_ + ".max_repulsive_magnitude", params.max_repulsive_magnitude);
    int max_obstacles_param = 500;
    node->get_parameter(plugin_name_ + ".max_obstacles", max_obstacles_param);
    params.max_obstacles = static_cast<unsigned int>(std::max(1, max_obstacles_param));
    
    // Relaxation gain (k)
    node->get_parameter(plugin_name_ + ".k_lin", pi_controller_.k_lin);
    node->get_parameter(plugin_name_ + ".k_ang", pi_controller_.k_ang);
    node->get_parameter(plugin_name_ + ".d_lin", pi_controller_.d_lin);
    node->get_parameter(plugin_name_ + ".d_ang", pi_controller_.d_ang);
    node->get_parameter(plugin_name_ + ".max_linear_vel", pi_controller_.max_linear_vel);
    node->get_parameter(plugin_name_ + ".max_angular_vel", pi_controller_.max_angular_vel);
    node->get_parameter(plugin_name_ + ".tolerance", pi_controller_.tolerance);
    
    // Store original max speeds for setSpeedLimit
    pi_controller_.original_max_linear_vel = pi_controller_.max_linear_vel;
    pi_controller_.original_max_angular_vel = pi_controller_.max_angular_vel;
    
    // Set SFM parameters
    sfm_algorithm_.setParameters(params);
    
    // Path following parameters
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".allow_backward", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".path_bias_gain", rclcpp::ParameterValue(3.0));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_goal_relaxation", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".prune_distance", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".slow_down_dist", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".stop_dist", rclcpp::ParameterValue(0.12));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_adaptive_lookahead", rclcpp::ParameterValue(true));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_lookahead_distance", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_lookahead_distance", rclcpp::ParameterValue(1.5));
    node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
    node->get_parameter(plugin_name_ + ".use_adaptive_lookahead", use_adaptive_lookahead_);
    node->get_parameter(plugin_name_ + ".min_lookahead_distance", min_lookahead_distance_);
    node->get_parameter(plugin_name_ + ".max_lookahead_distance", max_lookahead_distance_);
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_sec_);
    node->get_parameter(plugin_name_ + ".allow_backward", allow_backward_);
    node->get_parameter(plugin_name_ + ".path_bias_gain", path_bias_gain_);
    node->get_parameter(plugin_name_ + ".use_goal_relaxation", use_goal_relaxation_);
    node->get_parameter(plugin_name_ + ".prune_distance", prune_distance_);
    node->get_parameter(plugin_name_ + ".slow_down_dist", slow_down_dist_);
    node->get_parameter(plugin_name_ + ".stop_dist", stop_dist_);

    // Debug marker parameters
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".publish_debug_markers", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".publish_debug_stats", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".publish_param_stats", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".marker_scale", rclcpp::ParameterValue(0.5));  // Increased from 0.2 for visibility
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".marker_line_scale", rclcpp::ParameterValue(0.08));  // Increased from 0.03 for visibility
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".marker_text_scale", rclcpp::ParameterValue(0.15));  // Increased from 0.12 for readability
    node->get_parameter(plugin_name_ + ".publish_debug_markers", publish_debug_markers_);
    node->get_parameter(plugin_name_ + ".publish_debug_stats", publish_debug_stats_);
    node->get_parameter(plugin_name_ + ".publish_param_stats", publish_param_stats_);
    node->get_parameter(plugin_name_ + ".marker_scale", marker_scale_);
    node->get_parameter(plugin_name_ + ".marker_line_scale", marker_line_scale_);
    node->get_parameter(plugin_name_ + ".marker_text_scale", marker_text_scale_);

    // Create publishers in configure() (works in Nav2 when parent node is active)
    if (publish_debug_markers_) {
        force_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            plugin_name_ + "/force_markers", rclcpp::QoS(10));
        RCLCPP_INFO(node->get_logger(), 
                    "Created force markers publisher: %s/force_markers", plugin_name_.c_str());
    }
    if (publish_debug_stats_) {
        debug_stats_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            plugin_name_ + "/force_stats", rclcpp::QoS(10));
        repulsive_mag_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/repulsive_mag", rclcpp::QoS(10));
        attractive_mag_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/attractive_mag", rclcpp::QoS(10));
        total_mag_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/total_mag", rclcpp::QoS(10));
        cmd_vx_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/cmd_vx", rclcpp::QoS(10));
        cmd_wz_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/cmd_wz", rclcpp::QoS(10));
        goal_dist_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/goal_dist", rclcpp::QoS(10));
        nearest_obstacle_dist_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/nearest_obstacle_dist", rclcpp::QoS(10));
    }
    if (publish_param_stats_) {
        sfm_params_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            plugin_name_ + "/sfm_params", rclcpp::QoS(10));
        alpha_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/alpha", rclcpp::QoS(10));
        beta_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/beta", rclcpp::QoS(10));
        gamma_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/gamma", rclcpp::QoS(10));
        delta_pub_ = node->create_publisher<std_msgs::msg::Float64>(
            plugin_name_ + "/stats/delta", rclcpp::QoS(10));
    }

    // Dynamic parameter callback: allows runtime tuning via ros2 param set or rqt_reconfigure
    param_callback_handle_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            auto sfm_params = sfm_algorithm_.getParameters();
            bool sfm_changed = false;

            for (const auto & param : parameters) {
                const std::string & name = param.get_name();

                // SFM paper parameters
                if (name == plugin_name_ + ".A") {
                    sfm_params.A = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".B") {
                    sfm_params.B = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".d") {
                    sfm_params.d = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".alpha") {
                    sfm_params.alpha = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".delta") {
                    sfm_params.delta = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".lambda") {
                    sfm_params.lambda = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".max_repulsive_magnitude") {
                    sfm_params.max_repulsive_magnitude = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".goal_force_scale") {
                    sfm_params.goal_force_scale = param.as_double(); sfm_changed = true;
                } else if (name == plugin_name_ + ".obstacle_cost_threshold") {
                    sfm_params.obstacle_cost_threshold = param.as_double(); sfm_changed = true;
                }
                // Controller parameters
                else if (name == plugin_name_ + ".path_bias_gain") {
                    path_bias_gain_ = param.as_double();
                } else if (name == plugin_name_ + ".slow_down_dist") {
                    slow_down_dist_ = param.as_double();
                } else if (name == plugin_name_ + ".stop_dist") {
                    stop_dist_ = param.as_double();
                } else if (name == plugin_name_ + ".lookahead_distance") {
                    lookahead_distance_ = param.as_double();
                } else if (name == plugin_name_ + ".k_lin") {
                    pi_controller_.k_lin = param.as_double();
                } else if (name == plugin_name_ + ".k_ang") {
                    pi_controller_.k_ang = param.as_double();
                } else if (name == plugin_name_ + ".d_lin") {
                    pi_controller_.d_lin = param.as_double();
                } else if (name == plugin_name_ + ".d_ang") {
                    pi_controller_.d_ang = param.as_double();
                } else if (name == plugin_name_ + ".max_linear_vel") {
                    pi_controller_.max_linear_vel = param.as_double();
                } else if (name == plugin_name_ + ".max_angular_vel") {
                    pi_controller_.max_angular_vel = param.as_double();
                }
            }

            if (sfm_changed) {
                sfm_params.validate();
                sfm_algorithm_.setParameters(sfm_params);
            }
            return result;
        });

    RCLCPP_INFO(
        node->get_logger(),
        "SfmController configured: A=%.2f, B=%.2f, d=%.2f, alpha=%.2f, delta=%.2f, lambda=%.2f",
        params.A, params.B, params.d, params.alpha, params.delta, params.lambda);
}

void SfmController::cleanup()
{
    RCLCPP_INFO(
        rclcpp::get_logger("SfmController"),
        "Cleaning up controller: %s", plugin_name_.c_str());
}

void SfmController::activate()
{
    RCLCPP_INFO(
        rclcpp::get_logger("SfmController"),
        "Activating controller: %s", plugin_name_.c_str());
}

void SfmController::deactivate()
{
    RCLCPP_INFO(
        rclcpp::get_logger("SfmController"),
        "Deactivating controller: %s", plugin_name_.c_str());
}

void SfmController::setPlan(const nav_msgs::msg::Path & path)
{
    global_plan_ = path;
}

Point2D SfmController::getLookaheadPointPurePursuit(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double front_obstacle_dist) const
{
    // Pure Pursuit style lookahead: transform plan to base_link, prune, find first point ahead.
    // When front_obstacle_dist < 0.5 (narrow door), use min_lookahead for better entry.
    
    if (global_plan_.poses.empty()) {
        return Point2D(0.0, 0.0);
    }
    
    auto node = node_.lock();
    if (!node) {
        return Point2D(0.0, 0.0);
    }
    
    // Step 1: Transform entire plan to base_link frame
    nav_msgs::msg::Path plan_base;
    plan_base.header.frame_id = base_frame_id_;
    plan_base.header.stamp = robot_pose.header.stamp;
    
    try {
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(transform_tolerance_sec_));
        tf2::Duration tf2_tolerance(nanoseconds);
        
        for (const auto & plan_pose : global_plan_.poses) {
            geometry_msgs::msg::PoseStamped pose_in_base;
            if (plan_pose.header.frame_id != base_frame_id_) {
                pose_in_base = tf_buffer_->transform(plan_pose, base_frame_id_, tf2_tolerance);
            } else {
                pose_in_base = plan_pose;
            }
            plan_base.poses.push_back(pose_in_base);
        }
    } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
            node->get_logger(), *clock_, 2000,
            "Failed to transform plan to base_link: %s", ex.what());
        return Point2D(0.0, 0.0);
    }
    
    if (plan_base.poses.empty()) {
        return Point2D(0.0, 0.0);
    }
    
    // Step 2: Prune points that are too close (already passed)
    nav_msgs::msg::Path plan_pruned;
    plan_pruned.header = plan_base.header;
    
    for (const auto & pose : plan_base.poses) {
        double dist = std::hypot(pose.pose.position.x, pose.pose.position.y);
        if (dist > prune_distance_) {
            plan_pruned.poses.push_back(pose);
        }
    }
    
    if (plan_pruned.poses.empty()) {
        plan_pruned = plan_base;
    }
    
    // Step 3: Effective lookahead (door reduction + optional curve detection)
    double effective_lookahead = lookahead_distance_;
    
    // In narrow door / close frontal obstacle, use short lookahead so we don't aim too far inside
    if (front_obstacle_dist < 0.5) {
        effective_lookahead = min_lookahead_distance_;
    } else if (use_adaptive_lookahead_ && plan_pruned.poses.size() >= 2) {
        // Detect curve: check heading change in next few points
        // If path turns sharply (> 30°), reduce lookahead for better door/corner entry
        double max_heading_change = 0.0;
        const size_t check_points = std::min(static_cast<size_t>(5), plan_pruned.poses.size());
        
        for (size_t i = 0; i < check_points - 1; ++i) {
            double dx1 = plan_pruned.poses[i + 1].pose.position.x - plan_pruned.poses[i].pose.position.x;
            double dy1 = plan_pruned.poses[i + 1].pose.position.y - plan_pruned.poses[i].pose.position.y;
            if (i + 2 < plan_pruned.poses.size()) {
                double dx2 = plan_pruned.poses[i + 2].pose.position.x - plan_pruned.poses[i + 1].pose.position.x;
                double dy2 = plan_pruned.poses[i + 2].pose.position.y - plan_pruned.poses[i + 1].pose.position.y;
                
                double theta1 = std::atan2(dy1, dx1);
                double theta2 = std::atan2(dy2, dx2);
                double heading_change = std::abs(theta2 - theta1);
                while (heading_change > M_PI) heading_change -= 2.0 * M_PI;
                heading_change = std::abs(heading_change);
                
                max_heading_change = std::max(max_heading_change, heading_change);
            }
        }
        
        // Reduce lookahead if curve is tight (> 30° = ~0.52 rad)
        if (max_heading_change > 0.52) {
            // Scale lookahead: tighter curve → smaller lookahead
            double reduction = std::min(max_heading_change / (M_PI / 2.0), 1.0);  // Max reduction at 90°
            effective_lookahead = lookahead_distance_ * (1.0 - reduction * 0.5);  // Reduce up to 50%
            effective_lookahead = std::clamp(effective_lookahead, min_lookahead_distance_, max_lookahead_distance_);
        } else {
            // Straight path: use configured lookahead (clamped to max)
            effective_lookahead = std::clamp(lookahead_distance_, min_lookahead_distance_, max_lookahead_distance_);
        }
    }
    
    // Step 4: Find first point at effective_lookahead_distance
    for (const auto & pose : plan_pruned.poses) {
        double dist = std::hypot(pose.pose.position.x, pose.pose.position.y);
        if (dist >= effective_lookahead) {
            return Point2D(pose.pose.position.x, pose.pose.position.y);
        }
    }
    
    // Fallback: use last point
    if (!plan_pruned.poses.empty()) {
        const auto & last = plan_pruned.poses.back();
        return Point2D(last.pose.position.x, last.pose.position.y);
    }
    
    return Point2D(0.0, 0.0);
}

geometry_msgs::msg::TwistStamped SfmController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/)
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("Failed to lock node");
    }

    // Extract obstacles first (needed for frontal distance and for lookahead adaptation at doors)
    rclcpp::Time transform_time = pose.header.stamp;
    if (transform_time.nanoseconds() == 0) {
        transform_time = clock_->now();
    }
    std::vector<Point2D> obstacles = extractObstaclesFromCostmap(transform_time);
    if (!costmap_transform_ok_) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = base_frame_id_;
        cmd_vel.header.stamp = clock_->now();
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        RCLCPP_WARN_THROTTLE(
            node->get_logger(), *clock_, 2000,
            "Costmap transform unavailable (%s -> %s): stopping and no markers. Check TF and use_sim_time.",
            costmap_ros_->getGlobalFrameID().c_str(), base_frame_id_.c_str());
        return cmd_vel;
    }

    // Get lookahead point first (needed for e_r direction and raycast front_dist)
    // Use a preliminary front_dist=inf; we refine after computing obstacles
    Point2D goal_point = getLookaheadPointPurePursuit(pose, std::numeric_limits<double>::infinity());

    // Calculate obstacle distances:
    //   nearest_obstacle_distance : global minimum (for debug)
    //   front_dist_lethal         : raycast toward goal, only LETHAL obstacles → used for STOP
    //   front_dist_inflated       : raycast toward goal, all extracted obstacles → used for SLOWDOWN
    Point2D nearest_obstacle;
    double nearest_obstacle_distance = std::numeric_limits<double>::infinity();

    // Direction toward goal (for raycast and anisotropy e_r)
    const double goal_len = std::hypot(goal_point.x, goal_point.y);
    const double goal_ux = (goal_len > 1e-6) ? goal_point.x / goal_len : 1.0;
    const double goal_uy = (goal_len > 1e-6) ? goal_point.y / goal_len : 0.0;

    // Raycast half-width: must be less than typical corridor wall distance (~0.24m)
    // so lateral walls don't get flagged as frontal obstacles.
    static constexpr double kRayHalfWidth = 0.15;
    double front_dist_inflated = std::numeric_limits<double>::infinity();
    Point2D nearest_front_obstacle;

    for (const auto & obstacle : obstacles) {
        double dist = std::hypot(obstacle.x, obstacle.y);
        if (dist < 1e-6) continue;

        if (dist < nearest_obstacle_distance) {
            nearest_obstacle_distance = dist;
            nearest_obstacle = obstacle;
        }

        // Raycast toward goal: project obstacle onto goal direction ray
        // t = dot(obs, goal_unit) = how far along the ray
        // perp = |obs - t * goal_unit| = perpendicular distance to ray
        double t = obstacle.x * goal_ux + obstacle.y * goal_uy;
        if (t > 0.0) {  // Only obstacles ahead along the goal direction
            double perp_x = obstacle.x - t * goal_ux;
            double perp_y = obstacle.y - t * goal_uy;
            double perp_dist = std::hypot(perp_x, perp_y);
            if (perp_dist < kRayHalfWidth && t < front_dist_inflated) {
                front_dist_inflated = t;  // Distance along ray
                nearest_front_obstacle = obstacle;
            }
        }
    }

    // For STOP decision, use a higher threshold: only consider truly lethal cells
    // We approximate this by using a distance floor: if front_dist_inflated is > 0.3m
    // and the costmap cost at that point was likely inflation, don't trigger hard stop
    double nearest_front_distance = front_dist_inflated;
    if (!std::isfinite(nearest_front_distance)) {
        nearest_front_distance = nearest_obstacle_distance;
        nearest_front_obstacle = nearest_obstacle;
    }

    // If front obstacle is close, re-evaluate lookahead with reduced distance
    if (nearest_front_distance < 0.5) {
        goal_point = getLookaheadPointPurePursuit(pose, nearest_front_distance);
    }
    
    static size_t goal_log_counter = 0;
    if (++goal_log_counter % 50 == 0) {
        RCLCPP_DEBUG(
            node->get_logger(),
            "Goal point (Pure Pursuit): (%.3f, %.3f), distance=%.3f m",
            goal_point.x, goal_point.y, std::hypot(goal_point.x, goal_point.y));
    }
    
    // Reference point: robot position in base_link (0,0)
    Point2D reference_point(0.0, 0.0);
    
    // Note: In base_link frame, robot heading is +X (heading angle = 0)
    // Anisotropic weighting uses phi = theta directly (no need to subtract heading)
    
    // Verify goal_point is valid (not too close to robot, which would cause no movement)
    double goal_dist_check = std::hypot(goal_point.x, goal_point.y);
    if (goal_dist_check < 0.05 && !global_plan_.poses.empty()) {
        // Goal too close or zero: try to use a point further ahead in plan
        RCLCPP_WARN_THROTTLE(
            node->get_logger(), *clock_, 2000,
            "Goal point too close (%.3f m), using point further ahead...", goal_dist_check);
        
        // Try to find a point at least 0.3m ahead
        for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
            const auto & candidate_pose = global_plan_.poses[i];
            try {
                geometry_msgs::msg::PoseStamped candidate_in_base;
                if (candidate_pose.header.frame_id != base_frame_id_) {
                    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(transform_tolerance_sec_));
                    tf2::Duration tf2_tolerance(nanoseconds);
                    candidate_in_base = tf_buffer_->transform(candidate_pose, base_frame_id_, tf2_tolerance);
                } else {
                    candidate_in_base = candidate_pose;
                }
                double candidate_dist = std::hypot(
                    candidate_in_base.pose.position.x, 
                    candidate_in_base.pose.position.y);
                if (candidate_dist >= 0.3) {
                    goal_point = Point2D(
                        candidate_in_base.pose.position.x, 
                        candidate_in_base.pose.position.y);
                    RCLCPP_DEBUG(node->get_logger(), "Using waypoint %zu at distance %.3f m", i, candidate_dist);
                    break;
                }
            } catch (const std::exception & ex) {
                continue;  // Try next point
            }
        }
    }
    
    // Compute SFM forces. Paper Eq. 9: goal force = k_r * (v_desired - v_current); else spring-like attraction.
    // Pass e_r = unit(goal - robot) as desired direction for anisotropy (Eq. 7: cos φ = -n · e_r)
    Point2D desired_direction(goal_ux, goal_uy);
    Force2D rep_force = sfm_algorithm_.repulsiveForce(obstacles, reference_point, desired_direction);
    Force2D attr_force;
    if (use_goal_relaxation_) {
        attr_force = sfm_algorithm_.goalForceRelaxation(
            goal_point, reference_point,
            velocity.linear.x, velocity.linear.y,
            pi_controller_.max_linear_vel,
            pi_controller_.k_lin);
    } else {
        attr_force = sfm_algorithm_.attractiveForce(goal_point, reference_point);
    }

    // Combine forces (paper Eq. 14)
    Force2D total_force = sfm_algorithm_.combineForces(attr_force, rep_force);

    // Optional path bias: add a constant pull toward the goal direction.
    // With paper-pure repulsion + correct anisotropy, this should be small or zero.
    Force2D desired_force = total_force;
    const double goal_dist = std::hypot(goal_point.x, goal_point.y);
    if (goal_dist > 1e-3 && path_bias_gain_ > 0.0) {
        desired_force.fx += path_bias_gain_ * (goal_point.x / goal_dist);
        desired_force.fy += path_bias_gain_ * (goal_point.y / goal_dist);
    }

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = base_frame_id_;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist = forceToCmdVel(desired_force, goal_point, rep_force, nearest_front_obstacle, nearest_front_distance, nearest_obstacle_distance);

    // Store previous command
    prev_cmd_ = cmd_vel.twist;
    has_prev_cmd_ = true;

    double goal_distance = std::hypot(goal_point.x, goal_point.y);
    
    if (publish_debug_markers_ && force_markers_pub_) {
        publishForceMarkers(
            rep_force, attr_force, desired_force,  // desired_force = force actually used for cmd
            goal_point, nearest_obstacle,
            goal_distance, nearest_obstacle_distance,
            nearest_front_distance,
            cmd_vel.twist);
    }

    if (publish_debug_stats_ && debug_stats_pub_) {
        publishDebugStats(
            rep_force, attr_force, total_force,
            goal_distance, nearest_obstacle_distance,
            cmd_vel.twist);
    }

    if (publish_param_stats_ && sfm_params_pub_) {
        const auto & params = sfm_algorithm_.getParameters();
        // Publish paper parameters (alpha, beta, gamma, delta) instead of legacy names
        std_msgs::msg::Float64MultiArray param_msg;
        param_msg.data = {params.alpha, params.beta, params.gamma, params.delta};
        sfm_params_pub_->publish(param_msg);

        auto publish_scalar = [](const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub,
                                 double value) {
            if (!pub) {
                return;
            }
            std_msgs::msg::Float64 scalar;
            scalar.data = value;
            pub->publish(scalar);
        };
        // Publish paper parameters with correct names
        publish_scalar(alpha_pub_, params.alpha);  // α: weight to destination
        publish_scalar(beta_pub_, params.beta);    // β: weight to target person (reserved)
        publish_scalar(gamma_pub_, params.gamma);  // γ: weight person repulsion (reserved)
        publish_scalar(delta_pub_, params.delta);  // δ: weight obstacle repulsion
    }
    
    return cmd_vel;
}

std::vector<Point2D> SfmController::extractObstaclesFromCostmap(const rclcpp::Time & transform_time) const
{
    // Extract obstacles while holding lock (minimal time)
    std::vector<Point2D> obstacles;
    {
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap_ros_->getCostmap()->getMutex()));
        
        const nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
        obstacles = sfm_algorithm_.extractObstacles(*costmap);
    } // Lock released here - TF lookup and transform happen outside lock
    
    // Log extraction statistics for debugging (throttled to avoid spam)
    // Use getters to access debug info stored during extraction
    static size_t stride_log_counter = 0;
    if (++stride_log_counter % 100 == 0) {
        auto node = node_.lock();
        if (node) {
            unsigned int stride = sfm_algorithm_.getLastStride();
            auto [raw_hits, merged_rejected] = sfm_algorithm_.getLastExtractionStats();
            
            RCLCPP_DEBUG(
                node->get_logger(),
                "Costmap extraction: stride=%u, raw_hits=%zu, merged_rejected=%zu, final=%zu",
                stride, raw_hits, merged_rejected, obstacles.size());
        }
    }
    
    auto node = node_.lock();
    if (!node) {
        costmap_transform_ok_ = true;
        return obstacles;
    }

    const std::string costmap_frame_id = costmap_ros_->getGlobalFrameID();
    if (costmap_frame_id == base_frame_id_) {
        costmap_transform_ok_ = true;
        return obstacles;
    }

    // TF lookup and transform outside costmap lock
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(transform_tolerance_sec_));
    tf2::Duration tf2_tolerance(nanoseconds);

    auto do_transform = [&](tf2::TimePoint time_pt) {
        auto transform = tf_buffer_->lookupTransform(
            base_frame_id_, costmap_frame_id, time_pt, tf2_tolerance);
        tf2::Transform tf2_transform;
        tf2::fromMsg(transform.transform, tf2_transform);
        std::vector<Point2D> out;
        out.reserve(obstacles.size());
        for (const auto & p : obstacles) {
            tf2::Vector3 v(p.x, p.y, 0.0);
            tf2::Vector3 v_base = tf2_transform * v;
            out.emplace_back(v_base.x(), v_base.y());
        }
        return out;
    };

    tf2::TimePoint tf_time_point =
        (transform_time.nanoseconds() > 0)
            ? tf2::TimePoint(std::chrono::nanoseconds(transform_time.nanoseconds()))
            : tf2::TimePointZero;

    try {
        costmap_transform_ok_ = true;
        return do_transform(tf_time_point);
    } catch (const std::exception & ex) {
        try {
            costmap_transform_ok_ = true;
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *clock_, 5000,
                "Costmap TF at pose time failed (%s -> %s): %s; using latest transform.",
                costmap_frame_id.c_str(), base_frame_id_.c_str(), ex.what());
            return do_transform(tf2::TimePointZero);
        } catch (const std::exception & ex2) {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *clock_, 2000,
                "Costmap TF failed (%s -> %s): %s. Check TF tree and use_sim_time.",
                costmap_frame_id.c_str(), base_frame_id_.c_str(), ex2.what());
            costmap_transform_ok_ = false;
            return {};
        }
    }
}

geometry_msgs::msg::Twist SfmController::forceToCmdVel(
    const Force2D & force,
    const Point2D & goal_point,
    const Force2D & repulsive_force,
    const Point2D & /* nearest_front_obstacle */,
    double nearest_front_distance,
    double nearest_obstacle_distance)
{
    static constexpr double kCreepFast = 0.10;        // Creep velocity when path is wide open (m/s)
    static constexpr double kCreepSlow = 0.05;        // Creep velocity near obstacles (m/s)
    static constexpr double kGoalInFrontAngle = 1.4;  // ~80°: max angle to consider goal "in front"
    static constexpr double kGoalMinX = 0.05;         // Min goal.x to count as "ahead"

    geometry_msgs::msg::Twist cmd_vel;
    
    // Step 1: Calculate desired heading using goal direction with repulsion bias (for angular control)
    double goal_dist = std::hypot(goal_point.x, goal_point.y);
    double theta_goal = 0.0;
    if (goal_dist > 1e-6) {
        theta_goal = std::atan2(goal_point.y, goal_point.x);
    }
    
    // Heading = theta_goal only. No repulsion bias on yaw.
    // Paper-aligned: repulsion affects trajectory through force magnitude (Eq. 14),
    // not through angular corrections. Mixing repulsion into w causes zig-zag in corridors.
    double theta_desired = theta_goal;
    (void)repulsive_force;  // Repulsion only affects v through force.fx, not w
    
    // Step 2: Calculate time derivative for PI controller
    auto now = clock_->now();
    double dt = 0.0;
    if (has_last_time_) {
        dt = (now - last_cmd_time_).seconds();
    }
    if (dt <= 1e-3) {
        dt = 1.0 / 20.0;
    }
    last_cmd_time_ = now;
    has_last_time_ = true;
    
    double delta_theta = theta_desired - prev_theta_;
    while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
    while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;
    double d_theta = delta_theta / dt;
    prev_theta_ = theta_desired;
    
    // Step 3: Rotate-to-face — EMERGENCY ONLY.
    // Only activate when frontal obstacle is at actual collision distance (stop_dist)
    // AND the force model says "don't go forward". This prevents blocking in corridors
    // where front_dist is small due to inflation but the path is passable.
    if (nearest_front_distance <= stop_dist_ && force.fx <= 0.0) {
        double w = pi_controller_.k_ang * theta_desired + pi_controller_.d_ang * d_theta;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = std::clamp(w, -pi_controller_.max_angular_vel, pi_controller_.max_angular_vel);
        return cmd_vel;
    }
    
    // Step 4: Calculate linear velocity using force.fx directly (component forward in base_link)
    // This is the correct way: force.fx is the forward component, not cos(theta_desired)
    // Forward component is directly force.fx (X component in base_link, where +X is forward)
    double forward_component = force.fx;
    forward_component = std::max(0.0, forward_component);  // Clamp to non-negative
    
    // Step 5: Calculate slowdown based on FRONTAL obstacle distance (not lateral)
    // In corridors, lateral obstacles are close but shouldn't cause slowdown
    double slow = 1.0;
    if (nearest_front_distance < slow_down_dist_) {
        if (nearest_front_distance <= stop_dist_) {
            slow = 0.0;  // Stop completely if frontal obstacle is very close
        } else {
            // Linear slowdown between stop_dist_ and slow_down_dist_ for frontal obstacles
            slow = (nearest_front_distance - stop_dist_) / (slow_down_dist_ - stop_dist_);
            slow = std::clamp(slow, 0.0, 1.0);
        }
    }
    // If no frontal obstacle (inf), no slowdown (allows passage through corridors)
    
    // Step 6: Calculate linear velocity
    double v = pi_controller_.k_lin * forward_component * slow;
    
    // Goal is "forward" if it has a positive X component (anywhere in front hemisphere)
    // For creep: use wider angle (~80°) to allow entry into corridors at an angle
    const bool goal_in_front = (goal_point.x > kGoalMinX) && (std::abs(theta_goal) < kGoalInFrontAngle);
    
    // Creep forward: if goal is in front, ensure minimum progress.
    // Use nearest_obstacle_distance (actual closest obstacle, any direction) as safety check
    // instead of front_dist alone, because front_dist can be small from lateral wall cells
    // caught by the raycast in narrow corridors.
    if (goal_in_front && nearest_obstacle_distance > stop_dist_) {
        double creep = (nearest_front_distance > slow_down_dist_) ? kCreepFast : kCreepSlow;
        v = std::max(v, creep);
    }
    
    // Prevent backward motion: only block if goal is truly behind the robot (>90°)
    if (force.fx < 0.0 && !goal_in_front) {
        if (!allow_backward_) {
            v = 0.0;
        }
    }
    
    // Step 7: Calculate angular velocity
    // Paper-aligned: w tracks theta_goal (direction to lookahead) with derivative damping.
    // The repulsion affects the robot through force.fx (linear speed), NOT through w.
    // Adding repulsion correction to w causes zig-zag in corridors (oscillation between walls).
    double w = pi_controller_.k_ang * theta_goal + pi_controller_.d_ang * d_theta;
    
    // Step 8: Clamp to limits
    cmd_vel.linear.x = std::clamp(v, -pi_controller_.max_linear_vel * 0.7, pi_controller_.max_linear_vel);
    cmd_vel.angular.z = std::clamp(w, -pi_controller_.max_angular_vel, pi_controller_.max_angular_vel);
    
    return cmd_vel;
}

void SfmController::publishForceMarkers(
    const Force2D & repulsive,
    const Force2D & attractive,
    const Force2D & total_used_for_cmd,
    const Point2D & goal_point,
    const Point2D & nearest_obstacle,
    double goal_distance,
    double nearest_obstacle_distance,
    double nearest_front_distance,
    const geometry_msgs::msg::Twist & cmd_vel)
{
    auto node = node_.lock();
    if (!node || !force_markers_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    const auto stamp = clock_->now();
    const double marker_lifetime = 0.2;  // Short lifetime so markers don't accumulate in RViz

    auto make_arrow = [&](int id, const std::string & ns,
                          const Force2D & force,
                          float r, float g, float b, double z) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_id_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.frame_locked = true;
        marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

        // Normalize arrow length: clamp magnitude to [min_len, max_len] for visibility
        // Direction reflects force direction, length reflects magnitude (but always visible)
        const double min_len = 0.3;  // Minimum arrow length (always visible)
        const double max_len = 2.0;   // Maximum arrow length
        const double k = 0.5;         // Scale factor for magnitude
        
        double force_mag = std::hypot(force.fx, force.fy);
        double arrow_len = std::clamp(k * force_mag, min_len, max_len);
        
        // Unit vector in force direction
        double ux = 0.0, uy = 0.0;
        if (force_mag > 1e-6) {
            ux = force.fx / force_mag;
            uy = force.fy / force_mag;
        } else {
            ux = 1.0;  // Default forward if zero force
            uy = 0.0;
        }

        geometry_msgs::msg::Point p0;
        p0.x = 0.0;
        p0.y = 0.0;
        p0.z = z;
        geometry_msgs::msg::Point p1;
        p1.x = ux * arrow_len;
        p1.y = uy * arrow_len;
        p1.z = z;
        marker.points = {p0, p1};

        marker.scale.x = marker_line_scale_;  // shaft diameter
        marker.scale.y = marker_line_scale_ * 1.5;  // head diameter
        marker.scale.z = marker_line_scale_ * 2.5;  // head length
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.9;
        return marker;
    };

    auto make_text = [&](int id, const std::string & ns,
                         const std::string & text,
                         double x, double y, double z,
                         float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_id_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.frame_locked = true;
        marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.scale.z = marker_text_scale_;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.9;
        marker.text = text;
        return marker;
    };

    auto force_text = [](const std::string & label, const Force2D & force) {
        std::ostringstream stream;
        stream << label << "  fx=" << std::fixed << std::setprecision(2) << force.fx
               << " fy=" << force.fy
               << " |F|=" << std::hypot(force.fx, force.fy);
        return stream.str();
    };

    // Arrows: "total" is the force actually used for cmd (desired_force), so visualization matches behavior
    array.markers.push_back(make_arrow(0, "sfm_repulsive", repulsive, 1.0f, 0.2f, 0.2f, 0.5));
    array.markers.push_back(make_arrow(1, "sfm_attractive", attractive, 0.2f, 1.0f, 0.2f, 0.55));
    array.markers.push_back(make_arrow(2, "sfm_total", total_used_for_cmd, 0.2f, 0.4f, 1.0f, 0.6));

    array.markers.push_back(make_text(
        10, "sfm_total_label",
        force_text("total(cmd)", total_used_for_cmd),
        0.8, -0.2, 0.1, 0.2f, 0.4f, 1.0f));
    array.markers.push_back(make_text(
        11, "sfm_attractive_label",
        force_text("attractive", attractive),
        0.8, -0.4, 0.12, 0.2f, 1.0f, 0.2f));
    array.markers.push_back(make_text(
        12, "sfm_repulsive_label",
        force_text("repulsive", repulsive),
        0.8, -0.6, 0.14, 1.0f, 0.2f, 0.2f));

    // Slow factor (same formula as forceToCmdVel) for debug
    double slow = 1.0;
    if (nearest_front_distance < slow_down_dist_) {
        if (nearest_front_distance <= stop_dist_) {
            slow = 0.0;
        } else {
            slow = (nearest_front_distance - stop_dist_) / (slow_down_dist_ - stop_dist_);
            slow = std::clamp(slow, 0.0, 1.0);
        }
    }
    std::ostringstream legend;
    legend << "goal_dist=" << std::fixed << std::setprecision(2) << goal_distance
           << "  min_obs=";
    if (std::isfinite(nearest_obstacle_distance)) {
        legend << std::fixed << std::setprecision(2) << nearest_obstacle_distance;
    } else {
        legend << "inf";
    }
    legend << "  front_dist=";
    if (std::isfinite(nearest_front_distance)) {
        legend << std::fixed << std::setprecision(2) << nearest_front_distance;
    } else {
        legend << "inf";
    }
    legend << "  slow=" << std::fixed << std::setprecision(2) << slow
           << "  Fx(cmd)=" << total_used_for_cmd.fx
           << "\ncmd: v=" << std::fixed << std::setprecision(2) << cmd_vel.linear.x
           << "  w=" << cmd_vel.angular.z;
    // Summary text below robot, BLACK text for better visibility on light backgrounds
    array.markers.push_back(make_text(
        20, "sfm_summary",
        legend.str(),
        0.0, -1.0, 0.2,  // Centered below robot, even further down to avoid overlap
        0.0f, 0.0f, 0.0f));  // BLACK text instead of white
    
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = base_frame_id_;
    goal_marker.header.stamp = stamp;
    goal_marker.ns = "sfm_goal";
    goal_marker.id = 30;
    goal_marker.frame_locked = true;
    goal_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);
    if (std::hypot(goal_point.x, goal_point.y) > 0.0) {
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose.position.x = goal_point.x;
        goal_marker.pose.position.y = goal_point.y;
        goal_marker.pose.position.z = 0.05;
        goal_marker.scale.x = marker_line_scale_ * 4.0;
        goal_marker.scale.y = marker_line_scale_ * 4.0;
        goal_marker.scale.z = marker_line_scale_ * 4.0;
        goal_marker.color.r = 0.2f;
        goal_marker.color.g = 1.0f;
        goal_marker.color.b = 1.0f;
        goal_marker.color.a = 0.9f;
    } else {
        goal_marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    array.markers.push_back(goal_marker);

    visualization_msgs::msg::Marker obstacle_marker;
    obstacle_marker.header.frame_id = base_frame_id_;
    obstacle_marker.header.stamp = stamp;
    obstacle_marker.ns = "sfm_nearest_obstacle";
    obstacle_marker.id = 31;
    obstacle_marker.frame_locked = true;
    obstacle_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);
    if (std::isfinite(nearest_obstacle_distance)) {
        obstacle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
        obstacle_marker.pose.position.x = nearest_obstacle.x;
        obstacle_marker.pose.position.y = nearest_obstacle.y;
        obstacle_marker.pose.position.z = 0.05;
        obstacle_marker.scale.x = marker_line_scale_ * 3.0;
        obstacle_marker.scale.y = marker_line_scale_ * 3.0;
        obstacle_marker.scale.z = marker_line_scale_ * 3.0;
        obstacle_marker.color.r = 1.0f;
        obstacle_marker.color.g = 0.8f;
        obstacle_marker.color.b = 0.2f;
        obstacle_marker.color.a = 0.9f;
    } else {
        obstacle_marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    array.markers.push_back(obstacle_marker);

    // Publish markers
    force_markers_pub_->publish(array);
    
    // Debug: log publication (throttled to avoid spam)
    static size_t publish_counter = 0;
    if (++publish_counter % 50 == 0) {
        RCLCPP_DEBUG(node->get_logger(), 
                     "Published %zu force markers (rep: %.2f, attr: %.2f, total: %.2f)",
                     array.markers.size(),
                     std::hypot(repulsive.fx, repulsive.fy),
                     std::hypot(attractive.fx, attractive.fy),
                     std::hypot(total_used_for_cmd.fx, total_used_for_cmd.fy));
    }
}

void SfmController::publishDebugStats(
    const Force2D & repulsive,
    const Force2D & attractive,
    const Force2D & total,
    double goal_distance,
    double nearest_obstacle_distance,
    const geometry_msgs::msg::Twist & cmd_vel)
{
    if (!debug_stats_pub_) {
        return;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        repulsive.fx,
        repulsive.fy,
        std::hypot(repulsive.fx, repulsive.fy),
        attractive.fx,
        attractive.fy,
        std::hypot(attractive.fx, attractive.fy),
        total.fx,
        total.fy,
        std::hypot(total.fx, total.fy),
        cmd_vel.linear.x,
        cmd_vel.angular.z,
        goal_distance,
        nearest_obstacle_distance
    };
    debug_stats_pub_->publish(msg);

    auto publish_scalar = [](const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub,
                             double value) {
        if (!pub) {
            return;
        }
        std_msgs::msg::Float64 scalar;
        scalar.data = value;
        pub->publish(scalar);
    };
    publish_scalar(repulsive_mag_pub_, msg.data[2]);
    publish_scalar(attractive_mag_pub_, msg.data[5]);
    publish_scalar(total_mag_pub_, msg.data[8]);
    publish_scalar(cmd_vx_pub_, msg.data[9]);
    publish_scalar(cmd_wz_pub_, msg.data[10]);
    publish_scalar(goal_dist_pub_, msg.data[11]);
    publish_scalar(nearest_obstacle_dist_pub_, msg.data[12]);
}

Point2D SfmController::transformToBaseLink(const geometry_msgs::msg::PoseStamped & pose) const
{
    // Transform pose to base_link frame if needed
    // For now, assume pose is already in base_link
    return Point2D(pose.pose.position.x, pose.pose.position.y);
}

void SfmController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    auto node = node_.lock();
    if (!node) {
        return;
    }
    
    if (percentage) {
        // Apply percentage to original max speeds
        pi_controller_.max_linear_vel = pi_controller_.original_max_linear_vel * (speed_limit / 100.0);
        pi_controller_.max_angular_vel = pi_controller_.original_max_angular_vel * (speed_limit / 100.0);
    } else {
        // Set absolute speed limit (apply to linear speed)
        // Guard against division by zero and negative values
        const double base_v = std::max(1e-6, pi_controller_.original_max_linear_vel);
        const double ratio = pi_controller_.original_max_angular_vel / base_v;
        const double v_lim = std::max(0.0, speed_limit);  // Clamp to non-negative
        
        // Note: original_max_* should remain as baseline, only max_* are modified
        pi_controller_.max_linear_vel = v_lim;
        pi_controller_.max_angular_vel = v_lim * ratio;
    }
    
    RCLCPP_INFO(
        node->get_logger(),
        "SfmController speed limit set: linear=%.2f m/s, angular=%.2f rad/s (percentage=%s)",
        pi_controller_.max_linear_vel, pi_controller_.max_angular_vel,
        percentage ? "true" : "false");
}


}  // namespace sfm_controller_plugin

PLUGINLIB_EXPORT_CLASS(sfm_controller_plugin::SfmController, nav2_core::Controller)

