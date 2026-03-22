#ifndef SFM_CONTROLLER_PLUGIN__SFM_CONTROLLER_HPP_
#define SFM_CONTROLLER_PLUGIN__SFM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"

#include "sfm_controller_plugin/sfm_algorithm.hpp"

namespace sfm_controller_plugin
{

/**
 * @brief Nav2 Controller plugin implementing Social Force Model
 * 
 * This plugin implements the SFM algorithm as a Nav2 controller.
 * It follows the same pattern as the Nav2 tutorial example.
 * 
 * Tribute to KukaTxan in the implementation.
 */
class SfmController : public nav2_core::Controller
{
public:
    SfmController() = default;
    
    /**
     * @brief Configure the controller
     */
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
    /**
     * @brief Cleanup the controller
     */
    void cleanup() override;
    
    /**
     * @brief Activate the controller
     */
    void activate() override;
    
    /**
     * @brief Deactivate the controller
     */
    void deactivate() override;
    
    /**
     * @brief Compute velocity commands using SFM
     * 
     * This is the main method that Nav2 calls to get velocity commands.
     * It:
     * 1. Extracts obstacles from the local costmap
     * 2. Calculates SFM forces
     * 3. Converts forces to cmd_vel using PI controller
     * 4. Returns TwistStamped
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;
    
    /**
     * @brief Set the plan for the controller
     */
    void setPlan(const nav_msgs::msg::Path & path) override;
    
    /**
     * @brief Set speed limit
     */
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
    /**
     * @brief Extract obstacles from costmap
     */
    std::vector<Point2D> extractObstaclesFromCostmap(const rclcpp::Time & transform_time = rclcpp::Time(0)) const;
    
    /**
     * @brief Convert force to cmd_vel using PI controller
     * 
     * Paper-aligned: v = max(0, projection) except allow backward only in contact (nearest < d).
     * Uses goal-based heading with repulsion bias, slowdown by obstacle distance, and rotate-to-face.
     * 
     * @param force Total SFM force
     * @param goal_point Lookahead point (attractor) in base_link frame
     * @param repulsive_force Repulsive force vector (for heading bias)
     * @param nearest_front_obstacle Nearest FRONTAL obstacle point in base_link frame (for slowdown/stop)
     * @param nearest_front_distance Distance to nearest FRONTAL obstacle (only obstacles in ±60° cone)
     */
    geometry_msgs::msg::Twist forceToCmdVel(
        const Force2D & force,
        const Point2D & goal_point,
        const Force2D & repulsive_force,
        const Point2D & nearest_front_obstacle,
        double nearest_front_distance = std::numeric_limits<double>::infinity(),
        double nearest_obstacle_distance = std::numeric_limits<double>::infinity());

    /**
     * @brief Publish force markers for RViz visualization (optional)
     * @param total_used_for_cmd Force actually used for cmd_vel (e.g. desired_force = total + path_bias)
     * @param nearest_front_distance Distance to nearest frontal obstacle (for summary: front_dist, slow)
     */
    void publishForceMarkers(
        const Force2D & repulsive,
        const Force2D & attractive,
        const Force2D & total_used_for_cmd,
        const Point2D & goal_point,
        const Point2D & nearest_obstacle,
        double goal_distance,
        double nearest_obstacle_distance,
        double nearest_front_distance,
        const geometry_msgs::msg::Twist & cmd_vel);

    /**
     * @brief Publish numeric debug stats for plotting (optional)
     */
    void publishDebugStats(
        const Force2D & repulsive,
        const Force2D & attractive,
        const Force2D & total,
        double goal_distance,
        double nearest_obstacle_distance,
        const geometry_msgs::msg::Twist & cmd_vel);

    
    /**
     * @brief Transform point from costmap frame to base_link frame
     */
    Point2D transformToBaseLink(const geometry_msgs::msg::PoseStamped & pose) const;
    
    /**
     * @brief Get lookahead point using Pure Pursuit style (transform plan to base_link, prune, find first point ahead)
     * 
     * When front_obstacle_dist < 0.5 (e.g. narrow door), uses min_lookahead_distance for better entry.
     * 
     * @param robot_pose Current robot pose
     * @param front_obstacle_dist Distance to nearest frontal obstacle (inf if none); used to reduce lookahead at doors
     * @return Lookahead point in base_link frame, or (0,0) if no valid point found
     */
    Point2D getLookaheadPointPurePursuit(
        const geometry_msgs::msg::PoseStamped & robot_pose,
        double front_obstacle_dist = std::numeric_limits<double>::infinity()) const;

private:
    // Nav2 components
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Clock::SharedPtr clock_;
    std::string plugin_name_;
    std::string base_frame_id_;

    // Debug visualization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr force_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_stats_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr repulsive_mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr attractive_mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vx_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_wz_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr goal_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nearest_obstacle_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sfm_params_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alpha_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr beta_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delta_pub_;
    bool publish_debug_markers_{false};
    bool publish_debug_stats_{false};
    bool publish_param_stats_{false};
    double marker_scale_{0.2};
    double marker_line_scale_{0.03};
    double marker_text_scale_{0.12};

    // Transform status
    mutable bool costmap_transform_ok_{true};
    
    // SFM algorithm
    SocialForceModelAlgorithm sfm_algorithm_;
    
    // PI Controller parameters (from iri_sfm_based_robot_movement_controller)
    struct PIController
    {
        double k_lin = 2.0;
        double k_ang = 2.0;
        double d_lin = 0.15;
        double d_ang = 0.15;
        double max_linear_vel = 0.45;
        double max_angular_vel = 0.55;
        double tolerance = 0.15;
        double original_max_linear_vel = 0.45;
        double original_max_angular_vel = 0.55;
    } pi_controller_;
    
    // Path
    nav_msgs::msg::Path global_plan_;
    
    // Lookahead distance for following intermediate waypoints (meters)
    double lookahead_distance_{1.0};
    
    // Path alignment parameters
    
    // Transform tolerance (in seconds, converted to tf2::Duration when needed)
    double transform_tolerance_sec_{0.1};

    // If false, never command backward (v >= 0); avoids "retreat at door" loop when force points backward
    bool allow_backward_{false};

    // Extra "pull" toward path (goal lookahead) so the robot enters doors and advances in corridors.
    double path_bias_gain_{0.0};
    // Paper Eq. 9: goal force = k_r * (v_desired - v_current). If true, use this instead of spring-like attraction.
    bool use_goal_relaxation_{true};
    
    // Pure Pursuit style lookahead: prune distance (remove points already passed)
    double prune_distance_{0.3};
    
    // Adaptive lookahead: reduce in tight curves/doors to improve entry
    bool use_adaptive_lookahead_{true};
    double min_lookahead_distance_{0.5};  // Minimum lookahead (for tight curves/doors)
    double max_lookahead_distance_{1.5};  // Maximum lookahead (for straight paths)
    
    // Obstacle avoidance parameters for forceToCmdVel
    double slow_down_dist_{0.5};   // Start slowing down when frontal obstacle is closer than this
    double stop_dist_{0.12};       // Stop completely when frontal obstacle is closer than this

    // Time tracking for derivative terms
    bool has_last_time_{false};
    rclcpp::Time last_cmd_time_;
    double prev_theta_{0.0};
    bool has_prev_cmd_{false};
    geometry_msgs::msg::Twist prev_cmd_;

    // Dynamic parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace sfm_controller_plugin

#endif  // SFM_CONTROLLER_PLUGIN__SFM_CONTROLLER_HPP_

