# nav_bot_tools — 工具层（阶段 5）

把"导航跑得好不好"变成可量化的数据，以及自研算法的落地位置。

## 目录职责

| 路径 | 放什么 | 状态 |
|---|---|---|
| `scripts/eval_nav.py` | `nav2_simple_commander` 批量跑目标点，输出 CSV | 待建 |
| `scripts/record_bag.sh` | 实验前统一录 bag（`-a` 全话题） | 待建 |
| `config/goals.yaml` | 评测用目标点集（10 个随机点 + 场景化点位） | 待建 |
| `plugins/`（待建） | 自研 `nav2_core::GlobalPlanner` / `Controller` 插件 | 空 |

## 评测输出字段

`run_id, start_x, start_y, goal_x, goal_y, success, duration_s, path_length_m, straight_dist_m, efficiency, min_obstacle_dist_m, recoveries, planner, controller, max_vel`

## 指标目标

| 指标 | 目标 |
|---|---|
| 成功率 | ≥ 90% |
| 平均耗时 | < 40 s |
| 路径效率（实际/直线） | < 1.3 |
| 最小障碍间距 | > 0.15 m |

## 参数实验矩阵（这份矩阵就是报告的核心图）

- `inflation_radius`：0.25 / 0.35 / 0.45
- `max_vel_x`：0.3 / 0.5 / 0.8
- 控制器：MPPI vs DWB

固定其余参数，一次只变一个，结果写进 `docs/experiments.md`。
