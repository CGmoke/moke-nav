# 实验记录

规则：**一次只改一个参数**，改动前后都跑同一组 10 个目标点；没有 CSV 数据的结论视为无效。

## 记录模板（复制一份改标题）

### EXP-000 · 基线

| 项 | 内容 |
|---|---|
| 日期 | YYYY-MM-DD |
| 目的 | 建立未调参基线 |
| 改动 | 无（默认参数） |
| 场景 | room_10x10 |
| planner / controller | NavFn / MPPI |
| 关键参数 | `inflation_radius=0.35`, `max_vel_x=0.5`, `controller_frequency=20` |
| 结果 CSV | `outputs/eval/exp000.csv` |
| 成功率 | — |
| 平均耗时 | — |
| 路径效率 | — |
| 最小间距 | — |
| 现象 / 观察 | |
| 结论 / 下一步 | |

## 实验索引

| 编号 | 日期 | 变更点 | 成功率 | 平均耗时 | 结论 |
|---|---|---|---|---|---|
| EXP-000 | | 基线 | | | |
| EXP-001 | | | | | |

## 待做实验清单

- [ ] `inflation_radius`：0.25 / 0.35 / 0.45
- [ ] `max_vel_x`：0.3 / 0.5 / 0.8
- [ ] 控制器：MPPI vs DWB
- [ ] planner：NavFn vs SmacPlanner2D
- [ ] 恢复行为：默认 BT vs 缩短 recovery 次数
- [ ] 自研插件 vs 默认算法
