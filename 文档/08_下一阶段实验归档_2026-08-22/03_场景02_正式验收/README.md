# 场景 02 正式验收文档索引

世界文件：`src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_cross_scene_02.world`
更新时间：2026-08-24

## 当前正式结果

| 结果 | profile | 任务 | 结论 |
| --- | --- | --- | --- |
| [M→N 原参数基线](../../../results/05_跨场景验证/场景02/01_正式验收/场景02_MN原参数基线_2026-08-23) | `adaptive_goal_line_045` | M→N × 3 | 3/3，0/3 非地面 contacts |
| [四点闭环 v5](../../../results/05_跨场景验证/场景02/01_正式验收/场景02_四点闭环_恢复增强v5_2026-08-23) | `adaptive_goal_line_050_recovery_v5` | M→N→X→Y→M × 3 | 3/3，12/12 段，0/3 contacts |
| [四点闭环 v13](../../../results/05_跨场景验证/场景02/01_正式验收/场景02_四点闭环_目标线轻偏好v13_2026-08-23) | `adaptive_goal_line_050_recovery_v13_line_tiebreaker` | M→N→X→Y→M × 3 | 3/3，12/12 段，0/3 contacts |

v13 三次总耗时为 `539.848 / 540.191 / 449.438 s`，平均 `509.826 s`；N→X 三次
均约 `21 m`。平均值接近 510 秒，但 M→N 仍有在线建图带来的方差。

## 正式报告和复现

- [场景 02环境设计说明](阶段2_跨场景场景02环境设计说明_2026-08-22.md)
- [M→N原参数三次复测](阶段2_场景02_MN原参数基线三次复测报告_2026-08-23.md)
- [v5实验报告](阶段2_场景02_四点闭环恢复增强实验报告_2026-08-23.md)
- [v5轨迹图索引](阶段2_场景02_四点闭环恢复增强轨迹图索引_2026-08-23.md)
- [v5参数冻结与复现](阶段2_场景02_恢复增强v5参数冻结与复现手册_2026-08-23.md)
- [v13异常回环优化报告](阶段2_场景02_南侧回环原因与周期重规划优化报告_2026-08-23.md)
- [v13三次实验总结](阶段2_场景02_周期重规划三次实验总结_2026-08-23.md)
- [v13轨迹图索引](阶段2_场景02_周期重规划轨迹图索引_2026-08-23.md)
- [v13参数冻结与复现](阶段2_场景02_恢复增强最终参数与复现手册_2026-08-23.md)
- [三次汇总 CSV](阶段2_场景02_三次实验汇总.csv)

## 复现 v13

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true online:=true localization:=false world_file:=/workspaces/rtabmap_tb3_nav/src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_cross_scene_02.world navigation_profile:=adaptive_goal_line_050_recovery_v13_line_tiebreaker x_pose:=-8.5 y_pose:=0.0'
```

另开终端运行新的结果标签：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/multi_waypoint_regression.sh \
  --start-name M --start-x -8.5 --start-y 0.0 \
  --goal N:8.5:0.0:0.0 --goal X:-3.0:-4.0:0.0 \
  --goal Y:5.0:5.0:0.0 --goal M:-8.5:0.0:0.0 \
  --profile adaptive_goal_line_050_recovery_v13_line_tiebreaker \
  --world-file src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_cross_scene_02.world \
  --label 05_跨场景验证/场景02/03_新实验/复现_v13_$(date +%Y%m%d_%H%M%S) \
  --contact-timeout 1200 --settle-seconds 5'
```

每个新 label 会创建独立目录，不覆盖 `01_正式验收` 中的三次正式结果。
