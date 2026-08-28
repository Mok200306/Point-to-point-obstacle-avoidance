# Gate 4 生命周期隔离验证

日期：2026-08-28
分支：`exp/oracle-g4-critic-2026-08-28`
PredictionCritic 提交：`73872bc5db65c56ad4f7aefa3e25fe63b92bf0ac`

## 结论

**PredictionCritic 在 controller 生命周期中的功能路径通过；整套 launch 联动退出保留工程 caveat。**

已观察到的证据：

- 正式 Oracle run 的 `/controller_server`、`/planner_server`、`/bt_navigator` 和
  `/collision_monitor` 运行时均为 `active [3]`；
- controller 配置阶段加载 `PredictionCritic`，并在运行中持续输出
  `PredictionCritic status=active`；
- 独立 controller 生命周期检查中，`deactivate` 成功，随后 `cleanup` 成功，状态回到
  `unconfigured [1]`；
- 正式 3+3 导航期间没有 controller crash、NaN、插件加载错误或导航失败。

## Caveat

通过 Nav2 lifecycle manager 联动关闭整个 launch 时，planner 在 teardown 阶段出现额外
错误。该错误发生在导航完成后的整套进程退出路径，没有证据表明它来自 PredictionCritic
的 configure、activate 或 score；正式运行时 planner 和 controller 都已经正常工作并
完成目标。

因此 Gate 4 状态写为：

```text
硬功能验收：PASS
整套 Nav2 launch teardown：PASS WITH CAVEAT
```

进入 Gate 5 前，建议补做不依赖整套 launch 联动退出的三节点隔离
`deactivate -> cleanup` 检查，并把每个服务返回码和最终状态单独保存。该检查不改变
Gate 4 已冻结的插件、参数或零风险回归数据。
