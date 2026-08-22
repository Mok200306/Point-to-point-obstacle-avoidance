# fast_goalline_045_v2 候选参数冻结

更新时间：2026-08-21

## 冻结范围

这是 `fast_goalline_045_v2` 的候选冻结文件，不改变默认 profile。当前默认仍为
`fast_north_045_v3`，因为它在现有 3 次正式结果中路线方差更小；v2 用于“无碰撞后优先
缩短时间”的速度对照。

| 项目 | 值 |
| --- | --- |
| 代码提交 | `452b45f` |
| profile | `fast_goalline_045_v2` |
| 世界 | `indoor_obstacle_course_large` |
| 起点 / 目标 | `(-8.5, 0.0)` -> `(8.5, 0.0, 0)` |
| 模式 | `online=true`, `localization=false`, `reset_db=true` |
| 规划器 | `rtabmap_tb3_nav/GoalLineSmacPlanner` |
| 控制器 | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| inflation radius | 全局/局部 `0.45 m` |
| footprint | `0.60 x 0.48 m`，padding `0.03 m` |
| stop polygon | 前方约 `0.38 m`，保持启用 |
| slowdown polygon | 前方约 `1.05 m`，ratio `0.75`，保持启用 |

## v2 覆盖参数

基础参数来自 `src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml`，v2 由
`src/rtabmap_tb3_nav/launch/demo.launch.py` 覆盖：

```yaml
controller_server:
  FollowPath:
    desired_linear_vel: 0.28
    inflation_cost_scaling_factor: 4.5
velocity_smoother:
  max_velocity: [0.28, 0.0, 0.90]
planner_server:
  GridBased:
    side_bias_world_x_min: -7.2
    side_bias_world_x_max: 7.35
    side_bias_target_world_y_enabled: true
    side_bias_target_world_y: 0.75
    side_bias_target_max_cost: 140.0
    side_bias_target_distance_scale: 0.50
    side_bias_target_schedule_enabled: true
    side_bias_target_schedule_x: [-7.2, -2.9, -2.35, 2.8, 3.35, 7.35]
    side_bias_target_schedule_y: [0.75, 0.75, 0.30, 0.30, 0.50, 0.50]
local_costmap:
  inflation_layer:
    cost_scaling_factor: 4.5
global_costmap:
  inflation_layer:
    cost_scaling_factor: 4.5
```

没有覆盖的参数保持基础 YAML：`cost_travel_multiplier=6.0`、RPP collision detection、
`PolygonStop`、goal checker、footprint 和 RGB-D 点云范围均不变。

## 回归验收

v2 三次 A -> B：wall `85.46 / 93.85 / 87.53 s`，平均 `88.95 s`；Gazebo 路径
`17.24 / 17.18 / 17.50 m`，平均 `17.31 m`；Nav2 `3/3 status=4`；非地面
Gazebo contacts `0/3`。run3 最大 `y=1.37 m`，说明在线未知地图的偶发路线变化仍在，
所以这个文件是“候选冻结”，不是“任意场景最优”声明。

## 运行

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v2'
```

需要恢复默认快速基线时，把 profile 改回 `fast_north_045_v3`；需要恢复 v1，使用
`fast_goalline_045_v1`。需要代码级恢复时按结果目录 `experiment.yaml` 的
`git_commit` 建立 worktree。

