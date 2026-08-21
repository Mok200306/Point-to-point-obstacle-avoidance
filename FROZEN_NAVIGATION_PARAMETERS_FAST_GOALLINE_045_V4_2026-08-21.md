# fast_goalline_045_v4 参数冻结

更新时间：2026-08-21

## 冻结对象

这是当前 `indoor_obstacle_course_large.world` A -> B benchmark 的推荐快速 profile。
它不覆盖旧 profile；`fast_north_045_v3`、`fast_goalline_045_v2` 和旧 baseline 仍可用
于对照和复现。

| 项目 | 冻结值 |
| --- | --- |
| profile | `fast_goalline_045_v4` |
| 运行时 HEAD | `452b45f` |
| 最终可复现代码 | `78bb860` |
| world | `indoor_obstacle_course_large.world` |
| 起点 / 目标 | `(-8.5, 0.0, 0)` -> `(8.5, 0.0, 0)` |
| mode | `online=true`, `localization=false`, `reset_db=true` |
| planner | `rtabmap_tb3_nav/GoalLineSmacPlanner` |
| controller | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| inflation radius | global/local `0.45 m` |
| footprint | `0.60 x 0.48 m`，padding `0.03 m` |
| collision monitor | `PolygonSlow` 和 `PolygonStop` 保持启用 |
| online settle | `5 s`，不计入导航 wall 时间 |

## 覆盖参数

基础参数来自 `src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml`，profile 在
`src/rtabmap_tb3_nav/launch/demo.launch.py` 中覆盖以下字段：

```yaml
profile: fast_goalline_045_v4
controller_server:
  FollowPath:
    desired_linear_vel: 0.30
    lookahead_dist: 0.80
    min_lookahead_dist: 0.62
    max_lookahead_dist: 1.20
    lookahead_time: 1.7
    inflation_cost_scaling_factor: 4.5
velocity_smoother:
  max_velocity: [0.30, 0.0, 0.90]
planner_server:
  GridBased:
    side_bias_world_x_min: -7.2
    side_bias_world_x_max: 7.9
    side_bias_target_world_y_enabled: true
    side_bias_reference_world_y: 0.0
    side_bias_target_world_y: 0.60
    side_bias_target_offset: 0.60
    side_bias_target_max_cost: 200.0
    side_bias_target_distance_scale: 0.45
    side_bias_target_exponent: 2.0
    side_bias_target_schedule_enabled: true
    side_bias_target_schedule_x: [-7.2, -3.4, -2.6, -2.25, 2.75, 3.20, 3.50, 7.40, 7.90]
    side_bias_target_schedule_y: [0.95, 0.95, 0.75, 0.60, 0.60, 0.68, 0.58, 0.58, 0.00]
local_costmap:
  inflation_layer:
    cost_scaling_factor: 4.5
global_costmap:
  inflation_layer:
    cost_scaling_factor: 4.5
```

未列出的参数保持基础 YAML，尤其是 `inflation_radius=0.45 m`、footprint、
`cost_travel_multiplier=6.0`、RPP collision detection、`PolygonStop` 和 RGB-D 点云
范围均没有被 v4 弱化。

## 回归结果

证据：[v4 优化报告](NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md)。

| run | wall [s] | Gazebo path [m] | XY error [m] | status | contacts |
| ---: | ---: | ---: | ---: | ---: | --- |
| 01 | 79.01 | 17.336 | 0.361 | 4 | none |
| 02 | 78.26 | 17.451 | 0.234 | 4 | none |
| 03 | 85.94 | 17.359 | 0.274 | 4 | none |
| 平均 | **81.07** | **17.382** | **0.290** | **3/3** | **0/3** |

## 复现与回退

推荐 profile 的 GUI 启动：

```bash
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

旧 profile 的参数文档和 Git 提交仍可用于回退；正式结果归档和已清理目录见
[EXPERIMENT_ARCHIVE_INDEX.md](EXPERIMENT_ARCHIVE_INDEX.md)。保留的正式结果目录保存了
`git_commit`、基础 `nav2_rgbd_params.yaml`、`collision_monitor_rgbd_params.yaml`、
`world.sdf` 和 `profile_overrides.yaml`。运行时只想回退参数时可直接指定旧 profile，例如：

```bash
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true navigation_profile:=fast_north_045_v3'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true navigation_profile:=fast_goalline_045_v2'
```

若要代码级精确复现旧实验，应读取对应目录的 `experiment.yaml`，使用记录的
`git_commit` 建立独立 worktree 后再构建容器工作区。仅在新代码上选择同名 profile
可以恢复参数意图，但不能保证 C++ planner 或 launch 行为与旧提交逐字一致。
