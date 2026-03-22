#ifndef SFM_CONTROLLER_PLUGIN__SFM_ALGORITHM_HPP_
#define SFM_CONTROLLER_PLUGIN__SFM_ALGORITHM_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <tuple>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace sfm_controller_plugin
{

/**
 * @brief 2D point representation
 */
struct Point2D
{
    double x;
    double y;
    
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}
};

/**
 * @brief 2D force representation
 */
struct Force2D
{
    double fx;
    double fy;
    
    Force2D() : fx(0.0), fy(0.0) {}
    Force2D(double fx_val, double fy_val) : fx(fx_val), fy(fy_val) {}
};

/**
 * @brief Parameters for Social Force Model calculations
 * 
 * Nomenclature follows Helbing's Social Force Model paper:
 * - A, B, d: Repulsion parameters (eq. 5, p.2)
 * - k: Relaxation gain (eq. 3, p.2)
 * - λ: Anisotropy/field of view (eq. 6, p.2)
 * - α, β, γ, δ: Force combination weights (eq. 14, p.3)
 */
struct SFMParameters
{
    // === Repulsion Parameters (Paper: A, B, d) ===
    // Paper eq. (5): f_i,q^int = A_q * exp((d_q - d_i,q) / B_q) * n^_i,q
    // 
    // A: Strength/amplitude of repulsion
    //   Higher = stronger repulsion, more "aggressive" obstacle avoidance
    //   Paper: A_q (strength parameter for entity q)
    double A = 10.0;
    
    // B: Range/decay of repulsion (exponential decay distance)
    double B = 1.3;
    
    // d: Contact distance (sum of radii, repulsion peaks here)
    double d = 0.7;
    
    // === Force Combination Weights (Paper: α, β, γ, δ) ===
    // Paper eq. (14): F_r = α*f_r,dest^goal + β*f_r,i^goal + γ*F_r^per + δ*F_r^obs
    //
    // alpha: Weight towards destination (α in paper)
    //   Paper: α (weight for destination/predicted goal)
    double alpha = 1.01;
    
    // beta: Weight for following/accompanying target person (β in paper)
    //   Paper: β (weight for approaching/accompanying target person)
    //   Note: Reserved for future use (person following)
    double beta = 0.0;  // Not currently used
    
    // gamma: Weight for person repulsion (γ in paper)
    //   Paper: γ (weight for repulsion by people)
    //   Note: Reserved for future use (person avoidance)
    double gamma = 0.0;  // Not currently used
    
    // delta: Weight for obstacle repulsion (δ in paper)
    //   Paper: δ (weight for obstacle repulsion)
    double delta = 0.99;
    
    // === Anisotropy Parameter (Paper: λ) ===
    // Paper eq. (6): w(φ) = λ + (1-λ) * (1 + cos(φ)) / 2
    //   λ: Controls field of view weighting
    //   λ → 1: Almost isotropic (equal weight front/back)
    //   λ → 0: More human-like (less weight behind, but may reduce safety)
    // Note: Currently not implemented, but parameter reserved for future use
    double lambda = 1.0;  // Default: isotropic (safety-first)
    
    // === Implementation-specific parameters ===
    double dist_threshold = 0.12;  // Threshold for merging close obstacles (lower = more points, better wall representation)
    double approach = 1.0;        // Approach radius for braking (calculated from d + B)
    double end_force = 0.1;       // Minimum force at end of range
    double obstacle_cost_threshold = 254.0;  // Costmap threshold for obstacles (254 = LETHAL only, prevents blocking traversable areas)
    unsigned int max_obstacles = 500;  // Maximum number of obstacles to consider (prevents excessive forces from many points)
    double goal_force_scale = 1.5;     // Scale for attractive force magnitude (goal pull); >1 makes attraction compete better with summed repulsion
    double max_repulsive_magnitude = 0.0;  // Cap on |F_rep| (0 = no cap)
    
    void validate()
    {
        if (A <= 0) A = 10.0;
        if (d < 0) d = 0.7;
        if (B <= 0) B = 1.3;
        if (alpha <= 0) alpha = 1.01;
        if (delta <= 0) delta = 0.99;
        if (lambda < 0 || lambda > 1) lambda = 1.0;
        if (beta < 0) beta = 0.0;
        if (gamma < 0) gamma = 0.0;
        approach = d + B;
        if (dist_threshold < 0) dist_threshold = 0.01;
        if (approach <= 0) approach = 0.1;
        if (end_force <= 0 || end_force >= 1) end_force = 0.1;
        if (obstacle_cost_threshold < 1) obstacle_cost_threshold = 1;
        if (obstacle_cost_threshold > 255) obstacle_cost_threshold = 255;
        if (goal_force_scale <= 0.0) goal_force_scale = 1.0;
        if (max_repulsive_magnitude < 0.0) max_repulsive_magnitude = 0.0;
        if (max_obstacles < 1u) max_obstacles = 1u;
    }
};

/**
 * @brief Core Social Force Model algorithm implementation
 * 
 * This class contains the pure algorithm logic without any ROS2 dependencies.
 * Ported from Python implementation (sfm_algorithm.py) maintaining the same logic.
 */
class SocialForceModelAlgorithm
{
public:
    explicit SocialForceModelAlgorithm(const SFMParameters & params = SFMParameters())
    : params_(params)
    {
        params_.validate();
    }
    
    /**
     * @brief Extract obstacle positions from costmap
     * 
     * @param costmap Costmap2D from Nav2
     * @return List of obstacle positions (x, y) in meters relative to robot center
     */
    std::vector<Point2D> extractObstacles(const nav2_costmap_2d::Costmap2D & costmap) const;
    
    /**
     * @brief Get last stride used in obstacle extraction (for debugging)
     * 
     * @return Stride value used in last extractObstacles() call
     */
    unsigned int getLastStride() const { return last_stride_.load(); }
    
    /**
     * @brief Get last extraction statistics (for debugging)
     * 
     * @return Pair of (raw_hits, merged_rejected) from last extractObstacles() call
     *         merged_rejected = raw_hits - final_count (points removed by merge)
     */
    std::pair<size_t, size_t> getLastExtractionStats() const { 
        return {last_raw_hits_.load(), last_merged_rejected_.load()}; 
    }
    
    /**
     * @brief Compute aggregated repulsive force from obstacles (paper Eq. 5+6+7)
     * 
     * f_{r,o} = A * exp((d - d_{r,o}) / B) * w(φ) * n_{r,o}
     * w(φ) = λ + (1-λ) * (1 + cos φ) / 2            (Eq. 6)
     * cos φ = -n_{i,q} · e_r                          (Eq. 7)
     * 
     * where e_r is the desired direction of movement (unit vector toward goal).
     * 
     * @param obstacles Obstacle positions in base_link frame
     * @param reference_point Robot position (typically 0,0 in base_link)
     * @param desired_direction Unit vector of desired movement direction (e_r in paper).
     *        Defaults to (1,0) = forward; pass unit(goal - robot) for paper-correct anisotropy.
     * @return Aggregated repulsive force
     */
    Force2D repulsiveForce(
        const std::vector<Point2D> & obstacles,
        const Point2D & reference_point,
        const Point2D & desired_direction = Point2D(1.0, 0.0)
    ) const;
    
    /**
     * @brief Compute attractive force towards goal (spring-like, magnitude by distance)
     * 
     * Constant force outside approach radius, linear braking inside.
     * Use goalForceRelaxation() for paper Eq. 9 (relaxation to desired velocity).
     * 
     * @param goal Goal position
     * @param reference_point Reference point (robot position or human-robot centroid)
     * @return Attractive force towards goal
     */
    Force2D attractiveForce(
        const Point2D & goal,
        const Point2D & reference_point
    ) const;
    
    /**
     * @brief Goal force as relaxation to desired velocity (Paper Eq. 9)
     * 
     * f_r,dest^goal = k_r * (v_r^0 - v_r)
     * v_r^0 = desired velocity vector toward goal (magnitude v_desired_mag)
     * v_r = current robot velocity in base_link
     * 
     * This "pushes" to maintain forward progress; repulsion acts as correction.
     * 
     * @param goal Goal (lookahead) position in base_link
     * @param reference_point Reference point (typically 0,0)
     * @param v_current_x Current velocity x (base_link)
     * @param v_current_y Current velocity y (base_link)
     * @param v_desired_mag Desired speed magnitude (e.g. max_linear_vel)
     * @param k_r Relaxation gain (paper k, e.g. k_lin)
     * @return Force toward goal (relaxation form)
     */
    Force2D goalForceRelaxation(
        const Point2D & goal,
        const Point2D & reference_point,
        double v_current_x,
        double v_current_y,
        double v_desired_mag,
        double k_r
    ) const;
    
    /**
     * @brief Combine attractive and repulsive forces with weights
     * 
     * Paper eq. (14): F_r = α*f_r,dest^goal + δ*F_r^obs
     * Direct weighted sum WITHOUT normalization (as per paper)
     * 
     * @param attractive Attractive force
     * @param repulsive Repulsive force
     * @return Combined total force
     */
    Force2D combineForces(
        const Force2D & attractive,
        const Force2D & repulsive
    ) const;
    
    /**
     * @brief Compute all SFM forces in one call
     * 
     * @param obstacles List of obstacle positions (must be in base_link frame)
     * @param goal Goal position (must be in base_link frame)
     * @param reference_point Reference point (default: robot origin 0,0 in base_link)
     * @return Tuple of (repulsive_force, attractive_force, total_force)
     */
    std::tuple<Force2D, Force2D, Force2D> computeForces(
        const std::vector<Point2D> & obstacles,
        const Point2D & goal,
        const Point2D & reference_point = Point2D(0.0, 0.0)
    ) const;
    
    /**
     * @brief Get current parameters
     */
    const SFMParameters & getParameters() const { return params_; }
    
    /**
     * @brief Set parameters
     */
    void setParameters(const SFMParameters & params)
    {
        params_ = params;
        params_.validate();
    }

private:
    SFMParameters params_;
    
    // Debug statistics from last extraction (for observability)
    // Use atomic for thread-safety (readable from other threads while extraction runs)
    mutable std::atomic<unsigned int> last_stride_{1};
    mutable std::atomic<size_t> last_raw_hits_{0};
    mutable std::atomic<size_t> last_merged_rejected_{0};  // Points removed by merge
    
    /**
     * @brief Compute optimal stride for costmap sampling (private utility)
     * 
     * Calculates stride to ensure spatial step (stride * resolution) <= dist_threshold
     * This prevents "holes" in wall representation while optimizing performance
     * 
     * @param resolution Costmap resolution in meters per cell
     * @param width Costmap width in cells
     * @param height Costmap height in cells
     * @return Stride value (1, 2, or capped at 2)
     */
    unsigned int computeStride(double resolution, unsigned int width, unsigned int height) const;
    
    /**
     * @brief Calculate Euclidean distance between two points
     */
    static double distance(const Point2D & p1, const Point2D & p2)
    {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }
};

}  // namespace sfm_controller_plugin

#endif  // SFM_CONTROLLER_PLUGIN__SFM_ALGORITHM_HPP_

