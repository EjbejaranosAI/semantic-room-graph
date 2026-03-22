# SFM (Social Force Model) -- Nav2 Controller Plugin

Nav2 controller plugin that implements the Social Force Model (Helbing, 1995; Ferrer et al., 2013) for reactive robot navigation. Written in C++ as a standard `nav2_core::Controller` plugin.

Tribute to **KukaTxan** in the implementation.

<p align="center">
  <img src="docs/papers/18_02_2026_sfm_navigation_working.gif" alt="SFM Navigation Demo" width="800"/>
</p>

---

## Overview

The Social Force Model treats navigation as a physics simulation: the robot is a particle subject to **attractive** and **repulsive** forces. The sum of these forces determines the robot's velocity at each control cycle.

This plugin receives a global plan from the Nav2 planner, extracts obstacle information from the local costmap, computes SFM forces, and outputs `cmd_vel` commands. All parameters are tunable at runtime via `ros2 param set` or `rqt_reconfigure`.

---

## The Social Force Model

### 1. Obstacle Extraction (Costmap to Discrete Obstacles)

The local costmap is a 2D grid where each cell has a cost (0--255). The plugin:

1. Scans all cells with cost >= `obstacle_cost_threshold` (default 253 = inscribed + lethal).
2. Merges nearby points within `dist_threshold` (default 0.12 m) to reduce redundancy.
3. Transforms the resulting points from the costmap frame to `base_link` via TF.

The result is a set of obstacle positions `{o_1, o_2, ..., o_N}` in the robot's body frame. These feed into the repulsive force calculation.

### 2. Lookahead Point (Attractor)

The robot does **not** navigate directly toward the final goal. Instead, it follows a **lookahead point** on the global plan -- a point `lookahead_distance` meters ahead of the robot along the path. This is the **attractor** in the SFM.

The lookahead is computed Pure Pursuit-style:

1. Transform the global plan to `base_link`.
2. Prune points already passed (behind the robot by `prune_distance`).
3. Select the first point at distance >= `effective_lookahead`.

Adaptive behavior:
- If a frontal obstacle is closer than 0.5 m (e.g., a door), the lookahead shrinks to `min_lookahead_distance` for tighter tracking.
- If the path curves sharply (> 30 deg heading change), the lookahead reduces up to 50%.

### 3. Repulsive Force (Paper Eq. 5, 6, 7)

Each obstacle exerts a repulsive force on the robot:

```
f_rep = A * exp((d - dist) / B) * w(phi) * n
```

Where:
- `A`: repulsion amplitude
- `B`: exponential decay range
- `d`: contact distance (sum of radii)
- `dist`: Euclidean distance robot-to-obstacle
- `n`: unit vector from obstacle toward robot (repulsion direction)
- `w(phi)`: anisotropic weight (Eq. 6)

**Anisotropic weighting** makes the robot care more about obstacles in front and less about those behind:

```
w(phi) = lambda + (1 - lambda) * (1 + cos(phi)) / 2
```

Where `cos(phi) = -n . e_r` (Eq. 7), and `e_r` is the unit vector toward the lookahead (desired direction). Lower `lambda` = stronger anisotropy.

**Sector clustering**: The paper assumes discrete obstacles (pedestrians). A costmap wall is hundreds of cells. To approximate discrete obstacles, the algorithm divides the 360 deg around the robot into **8 angular sectors** and keeps only the **closest** obstacle in each sector. This prevents force explosion from continuous walls while preserving the paper's formulation.

The total repulsive force is the sum over all sector representatives, capped at `max_repulsive_magnitude` as a safety net.

### 4. Attractive Force (Paper Eq. 9)

With `use_goal_relaxation: true` (default, recommended), the goal force follows the paper's relaxation model:

```
f_goal = k_r * (v_desired - v_current)
```

Where:
- `k_r`: relaxation gain (`k_lin`, paper k ~ 2.3)
- `v_desired`: unit vector toward lookahead * `max_linear_vel`
- `v_current`: current robot velocity from odometry

This creates a force that accelerates the robot toward the lookahead and decays naturally as the robot reaches cruising speed.

Alternative: with `use_goal_relaxation: false`, a simpler spring-like force is used with constant magnitude scaled by `goal_force_scale`.

### 5. Force Combination (Paper Eq. 14)

The forces are combined as a weighted sum:

```
F_total = alpha * f_goal + delta * f_rep
```

Where `alpha` and `delta` are the paper's combination weights. An optional `path_bias_gain` adds a constant pull toward the lookahead direction to help in narrow passages.

### 6. Force to Velocity Conversion

The SFM paper models pedestrians, not wheeled robots. Converting the force vector `F_total` to differential-drive `cmd_vel = (v, w)` requires a mapping:

**Linear velocity `v`:**

1. Take the forward component: `forward = max(0, F_total.fx)` (in `base_link`, +X = forward).
2. Apply a **slowdown factor** based on frontal obstacle distance (raycast along goal direction):
   - `front_dist > slow_down_dist`: no slowdown (`slow = 1.0`).
   - `stop_dist < front_dist < slow_down_dist`: linear ramp from 0 to 1.
   - `front_dist <= stop_dist`: full stop (`slow = 0.0`).
3. `v = k_lin * forward * slow`.
4. **Creep forward**: if the goal is in front (within ~80 deg) and the nearest obstacle is farther than `stop_dist`, enforce a minimum velocity (0.10 m/s if the path is wide open, 0.05 m/s if near obstacles). This prevents the robot from freezing at corridor entrances where repulsion temporarily exceeds attraction.
5. **Backward prevention**: if `F_total.fx < 0` and the goal is behind, `v = 0` (unless `allow_backward: true`).

**Angular velocity `w`:**

```
w = k_ang * theta_goal + d_ang * d(theta_goal)/dt
```

Where `theta_goal = atan2(goal.y, goal.x)` is the angle to the lookahead. The repulsive force does **not** affect `w` -- it only affects `v` through the force magnitude. This prevents zig-zag oscillation in corridors where lateral repulsion would flip the heading left/right each cycle.

**Emergency rotate-to-face**: if `front_dist <= stop_dist` AND `F_total.fx <= 0`, the robot stops and rotates in place toward the goal.

**Raycast frontal detection**: `front_dist` is computed by projecting all obstacle points onto the goal direction ray. Only points within 0.15 m perpendicular distance (half robot body width) count as "frontal". This avoids lateral walls triggering slowdown in corridors.

---

## Installation

```bash
cd ~/ros_sfm_ws/src
git clone <repository-url> sfm
cd ~/ros_sfm_ws
colcon build --packages-select sfm
source install/setup.bash
```

Verify registration:
```bash
ros2 plugin list | grep -F "sfm_controller_plugin::SfmController"
```

---

## Configuration

### Tiago Pro / PAL Robots

Copy the included preset:
```bash
sudo cp ~/ros_sfm_ws/src/sfm/config/mppi_omni.yaml \
  /opt/pal/alum/share/pal_navigation_cfg/presets/nav2_controller/mppi_omni.yaml
```

### Other Robots (Nav2 Standard)

Add to your `nav2_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "sfm_controller_plugin::SfmController"
      # See config/nav2_controller_sfm.yaml for all parameters
```

---

## Parameters

### Paper Parameters

| Symbol | Parameter | Description | Default |
|--------|-----------|-------------|---------|
| A | `A` | Repulsion amplitude | 2.7 |
| B | `B` | Repulsion decay range | 0.8 |
| d | `d` | Contact distance | 0.4 |
| alpha | `alpha` | Goal force weight (Eq. 14) | 1.5 |
| delta | `delta` | Repulsion weight (Eq. 14) | 0.7 |
| lambda | `lambda` | Anisotropy (0=front-only, 1=isotropic) | 0.4 |
| k | `k_lin`, `k_ang` | Relaxation gain (Eq. 9) | 2.3 |

### Obstacle Extraction

| Parameter | Description | Default |
|-----------|-------------|---------|
| `obstacle_cost_threshold` | Min costmap cost to be an obstacle (253=inscribed+lethal) | 253 |
| `dist_threshold` | Merge radius for nearby points (m) | 0.12 |
| `max_obstacles` | Max obstacle points to consider | 500 |
| `max_repulsive_magnitude` | Safety cap on total repulsive force magnitude | 5.0 |

### Path Following

| Parameter | Description | Default |
|-----------|-------------|---------|
| `lookahead_distance` | Pure Pursuit lookahead distance (m) | 1.0 |
| `use_goal_relaxation` | Use paper Eq. 9 for goal force | true |
| `path_bias_gain` | Extra pull toward lookahead direction | 1.0 |
| `slow_down_dist` | Start slowing when frontal obstacle < this (m) | 0.5 |
| `stop_dist` | Hard stop when frontal obstacle < this (m) | 0.12 |
| `allow_backward` | Allow negative linear velocity | false |

### Velocity Controller

| Parameter | Description | Default |
|-----------|-------------|---------|
| `k_lin` | Linear velocity gain | 2.3 |
| `k_ang` | Angular velocity gain | 2.3 |
| `d_lin` | Linear derivative gain | 0.15 |
| `d_ang` | Angular derivative gain | 0.15 |
| `max_linear_vel` | Max linear speed (m/s) | 0.45 |
| `max_angular_vel` | Max angular speed (rad/s) | 0.55 |

### Visualization

| Parameter | Description | Default |
|-----------|-------------|---------|
| `publish_debug_markers` | Publish force arrows to RViz | false |
| `publish_debug_stats` | Publish scalar stats for rqt_plot | false |
| `marker_scale` | Arrow length scale | 0.5 |
| `marker_line_scale` | Arrow shaft thickness | 0.08 |
| `marker_text_scale` | Debug text size | 0.15 |

See `config/mppi_omni.yaml` or `config/nav2_controller_sfm.yaml` for complete examples.

---

## Dynamic Parameter Tuning

All key parameters can be changed at runtime without restarting:

```bash
ros2 param set /controller_server FollowPath.alpha 2.0
ros2 param set /controller_server FollowPath.delta 0.5
ros2 param set /controller_server FollowPath.lambda 0.3
ros2 param set /controller_server FollowPath.max_repulsive_magnitude 4.0
ros2 param set /controller_server FollowPath.path_bias_gain 1.5
ros2 param set /controller_server FollowPath.slow_down_dist 0.4
```

For a graphical interface with sliders:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

Navigate to `/controller_server` to see all `FollowPath.*` parameters.

---

## Visualization

### Force Arrows in RViz

1. Set `publish_debug_markers: true` in the configuration.
2. In RViz: Add -> MarkerArray -> Topic: `/FollowPath/force_markers`.
3. You will see:
   - **Red arrow**: Repulsive force (from obstacles)
   - **Green arrow**: Attractive force (toward lookahead)
   - **Blue arrow**: Total force used for cmd_vel
   - **Cyan sphere**: Lookahead point (attractor)
   - **Yellow sphere**: Nearest obstacle
   - **Text overlay**: `goal_dist`, `min_obs`, `front_dist`, `slow`, `Fx(cmd)`, `v`, `w`

### Plotting with rqt_plot

With `publish_debug_stats: true`:
```bash
rqt_plot /FollowPath/stats/repulsive_mag \
         /FollowPath/stats/attractive_mag \
         /FollowPath/stats/total_mag \
         /FollowPath/stats/cmd_vx
```

---

## Project Structure

```
SFM/
├── include/sfm_controller_plugin/
│   ├── sfm_algorithm.hpp      # Pure SFM algorithm (no ROS dependencies)
│   └── sfm_controller.hpp     # Nav2 controller plugin interface
├── src/
│   ├── sfm_algorithm.cpp      # Force calculations (Eq. 5, 6, 7, 9, 14)
│   └── sfm_controller.cpp     # Obstacle extraction, lookahead, force-to-cmd_vel
├── config/
│   ├── nav2_controller_sfm.yaml    # Minimal config example
│   └── mppi_omni.yaml              # Full PAL preset (includes MPPI fallback)
├── docs/
│   └── papers/
└── README.md
```

---

## Troubleshooting

**Robot does not move**: Check `Fx(cmd)` in the debug text. If negative, repulsion dominates attraction. Increase `alpha`, decrease `delta`, or lower `max_repulsive_magnitude`.

**Robot zig-zags in corridors**: Lower `d_ang` (e.g., 0.10). The angular controller should be smooth, not reactive.

**Robot stops at corridor entrance**: Check `front_dist` and `slow`. If `slow = 0` but path is clear, the raycast may be catching lateral walls. Verify `stop_dist` is small enough (0.10--0.12 m).

**Arrows not visible in RViz**: Increase `marker_scale` (0.5 -> 1.0). Set Fixed Frame to `map` or `base_link`.

**Robot retreats at doors**: Ensure `allow_backward: false`. Increase `path_bias_gain` or `alpha`.

---

## References

- Helbing, D. & Molnar, P. (1995). "Social force model for pedestrian dynamics." *Physical Review E*.
- Ferrer, G. et al. (2013). "Robot Companion: A Social-Force based approach with Human Awareness-Navigation in Crowded Environments."
- [Nav2 Controller Plugin Tutorial](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)

---

## License

MIT License -- See `LICENSE`
