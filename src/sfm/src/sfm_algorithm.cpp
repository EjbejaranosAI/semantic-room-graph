/**
 * @file sfm_algorithm.cpp
 * @brief Implementation of SocialForceModelAlgorithm (pure algorithm, no ROS2).
 * 
 * This file implements extractObstacles, repulsiveForce, attractiveForce, combineForces.
 * The controller plugin (sfm_controller.cpp) links to this library and uses these methods.
 */

#include "sfm_controller_plugin/sfm_algorithm.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace sfm_controller_plugin
{

unsigned int SocialForceModelAlgorithm::computeStride(
    double resolution,
    unsigned int width,
    unsigned int height) const
{
    (void)resolution;
    (void)width;
    (void)height;
    // Stride = 1 always: local costmap is a window, cost is acceptable. Stride > 1 smooths corners
    // and makes repulsion appear late → robot "eats" the corner then scrapes the wall.
    return 1u;
}

std::vector<Point2D> SocialForceModelAlgorithm::extractObstacles(
    const nav2_costmap_2d::Costmap2D & costmap) const
{
    const unsigned int width = costmap.getSizeInCellsX();
    const unsigned int height = costmap.getSizeInCellsY();
    const double resolution = costmap.getResolution();
    if (width == 0u || height == 0u || resolution <= 0.0) {
        last_stride_.store(1u);
        last_raw_hits_.store(0u);
        last_merged_rejected_.store(0u);
        return {};
    }

    const unsigned int stride = computeStride(resolution, width, height);
    last_stride_.store(stride);

    std::vector<Point2D> positions;
    positions.reserve((width / stride) * (height / stride) / 2u);

    const unsigned char threshold = static_cast<unsigned char>(
        std::min(255, std::max(0, static_cast<int>(params_.obstacle_cost_threshold))));

    for (unsigned int cy = 0u; cy < height; cy += stride) {
        for (unsigned int cx = 0u; cx < width; cx += stride) {
            const unsigned char cost = costmap.getCost(cx, cy);
            if (cost >= threshold) {
                double wx = 0.0, wy = 0.0;
                costmap.mapToWorld(cx, cy, wx, wy);
                positions.emplace_back(wx, wy);
            }
        }
    }

    const size_t raw_hits = positions.size();
    last_raw_hits_.store(raw_hits);

    // Merge points within dist_threshold. Keep first in cluster (do NOT use "closest to origin"):
    // points are still in costmap frame (map/odom), so origin (0,0) is not the robot and would bias walls/corners.
    if (positions.empty() || params_.dist_threshold <= 0.0) {
        last_merged_rejected_.store(0u);
        if (positions.size() > params_.max_obstacles) {
            positions.resize(params_.max_obstacles);
        }
        return positions;
    }

    std::vector<Point2D> merged;
    merged.reserve(positions.size());
    const double d2 = params_.dist_threshold * params_.dist_threshold;

    for (const Point2D & p : positions) {
        bool absorbed = false;
        for (const Point2D & m : merged) {
            const double dx = p.x - m.x;
            const double dy = p.y - m.y;
            if (dx * dx + dy * dy <= d2) {
                absorbed = true;
                break;  // keep first point in cluster; do not replace by "closer to origin"
            }
        }
        if (!absorbed) {
            merged.push_back(p);
        }
    }

    last_merged_rejected_.store(raw_hits - merged.size());

    if (merged.size() > params_.max_obstacles) {
        merged.resize(params_.max_obstacles);
    }
    return merged;
}

Force2D SocialForceModelAlgorithm::repulsiveForce(
    const std::vector<Point2D> & obstacles,
    const Point2D & reference_point,
    const Point2D & desired_direction) const
{
    // Paper Eq. 5 + Eq. 6 + Eq. 7, applied to REPRESENTATIVE obstacles.
    //
    // The paper assumes discrete entities (people, posts). A costmap wall is hundreds
    // of cells → raw sum explodes. To approximate discrete obstacles from a continuous
    // wall, we keep only the CLOSEST point in each of N angular sectors. Each sector
    // represents one "obstacle entity" as the paper intends.
    //
    // For each representative obstacle:
    //   f = A * exp((d - dist) / B) * w(φ) * n        (Eq. 5)
    //   w(φ) = λ + (1-λ) * (1 + cos φ) / 2            (Eq. 6)
    //   cos φ = -n · e_r                                (Eq. 7, e_r = desired direction)

    Force2D total(0.0, 0.0);
    if (params_.B <= 0.0 || params_.A <= 0.0) {
        return total;
    }

    const double margin_max = params_.d + params_.B;

    // e_r: desired direction of movement (for anisotropy, Eq. 7)
    const double er_len = std::hypot(desired_direction.x, desired_direction.y);
    const double er_x = (er_len > 1e-6) ? desired_direction.x / er_len : 1.0;
    const double er_y = (er_len > 1e-6) ? desired_direction.y / er_len : 0.0;

    // Sector clustering: keep only closest obstacle per angular sector.
    // 8 sectors ≈ 45° each. Fewer sectors = fewer representative obstacles from continuous
    // walls = lower total repulsion. 12 was too many for corridor entrances.
    constexpr int num_sectors = 8;
    struct SectorEntry { Point2D point; double dist; };
    std::array<SectorEntry, num_sectors> sectors;
    for (auto & s : sectors) { s.dist = std::numeric_limits<double>::infinity(); }

    for (const Point2D & obs : obstacles) {
        const double dist = std::hypot(obs.x - reference_point.x, obs.y - reference_point.y);
        if (dist < 1e-6 || dist > margin_max) {
            continue;
        }
        // Angle from robot to obstacle → sector index
        const double angle = std::atan2(obs.y - reference_point.y, obs.x - reference_point.x);
        int idx = static_cast<int>(((angle + M_PI) / (2.0 * M_PI)) * num_sectors);
        idx = std::clamp(idx, 0, num_sectors - 1);

        if (dist < sectors[idx].dist) {
            sectors[idx].point = obs;
            sectors[idx].dist = dist;
        }
    }

    // Apply paper formula to each representative obstacle
    for (const auto & sec : sectors) {
        if (sec.dist >= std::numeric_limits<double>::infinity()) {
            continue;
        }
        const double dist = sec.dist;

        // n: unit vector FROM obstacle TO agent (repulsion direction, Eq. 5)
        const double nx = (reference_point.x - sec.point.x) / dist;
        const double ny = (reference_point.y - sec.point.y) / dist;

        // Eq. 5: magnitude
        const double mag = params_.A * std::exp((params_.d - dist) / params_.B);

        // Eq. 7: cos φ = -n · e_r  (-n points FROM agent TO obstacle)
        const double cos_phi = (-nx) * er_x + (-ny) * er_y;

        // Eq. 6: anisotropic weight
        const double w = params_.lambda + (1.0 - params_.lambda) * (1.0 + cos_phi) * 0.5;

        total.fx += mag * w * nx;
        total.fy += mag * w * ny;
    }

    // Safety cap on total magnitude (prevents runaway in very dense costmaps)
    if (params_.max_repulsive_magnitude > 0.0) {
        const double total_mag = std::hypot(total.fx, total.fy);
        if (total_mag > params_.max_repulsive_magnitude && total_mag > 1e-9) {
            const double scale = params_.max_repulsive_magnitude / total_mag;
            total.fx *= scale;
            total.fy *= scale;
        }
    }
    return total;
}

Force2D SocialForceModelAlgorithm::attractiveForce(
    const Point2D & goal,
    const Point2D & reference_point) const
{
    const double dx = goal.x - reference_point.x;
    const double dy = goal.y - reference_point.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) {
        return Force2D(0.0, 0.0);
    }
    const double nx = dx / dist;
    const double ny = dy / dist;
    // Constant magnitude outside approach, linear inside (braking)
    // goal_force_scale lets attraction compete with summed repulsion (default 1.5)
    const double A_att = params_.A * params_.goal_force_scale;
    double mag;
    if (dist >= params_.approach) {
        mag = A_att;
    } else {
        mag = (dist / params_.approach) * A_att;
        if (mag < params_.end_force) {
            mag = params_.end_force;
        }
    }
    return Force2D(mag * nx, mag * ny);
}

Force2D SocialForceModelAlgorithm::goalForceRelaxation(
    const Point2D & goal,
    const Point2D & reference_point,
    double v_current_x,
    double v_current_y,
    double v_desired_mag,
    double k_r) const
{
    const double dx = goal.x - reference_point.x;
    const double dy = goal.y - reference_point.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) {
        return Force2D(0.0, 0.0);
    }
    const double ux = dx / dist;
    const double uy = dy / dist;
    const double v_desired_x = ux * v_desired_mag;
    const double v_desired_y = uy * v_desired_mag;
    return Force2D(
        k_r * (v_desired_x - v_current_x),
        k_r * (v_desired_y - v_current_y));
}

Force2D SocialForceModelAlgorithm::combineForces(
    const Force2D & attractive,
    const Force2D & repulsive) const
{
    return Force2D(
        params_.alpha * attractive.fx + params_.delta * repulsive.fx,
        params_.alpha * attractive.fy + params_.delta * repulsive.fy);
}

std::tuple<Force2D, Force2D, Force2D> SocialForceModelAlgorithm::computeForces(
    const std::vector<Point2D> & obstacles,
    const Point2D & goal,
    const Point2D & reference_point) const
{
    // e_r = unit vector toward goal (for anisotropy Eq. 7)
    const double gl = std::hypot(goal.x - reference_point.x, goal.y - reference_point.y);
    Point2D e_r(1.0, 0.0);
    if (gl > 1e-6) {
        e_r = Point2D((goal.x - reference_point.x) / gl, (goal.y - reference_point.y) / gl);
    }
    Force2D rep = repulsiveForce(obstacles, reference_point, e_r);
    Force2D attr = attractiveForce(goal, reference_point);
    Force2D total = combineForces(attr, rep);
    return std::make_tuple(rep, attr, total);
}

}  // namespace sfm_controller_plugin
