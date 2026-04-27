# Path Follower (MPC)

这个包提供一个二维轨迹跟踪节点，用于跟踪 Hybrid A* 规划的 `nav_msgs/Path`。控制量通过 `/cmd_vel` 下发，适配足式机器人，可控制 `vx`、`vy` 和 `w`。

## 订阅与发布

- 订阅:
  - `path_topic` (默认 `/searched_path`) : `nav_msgs/Path`
  - `odom_topic` (默认 `/gazebo_odom`) : `nav_msgs/Odometry`
- 发布:
  - `cmd_vel_topic` (默认 `/cmd_vel`) : `geometry_msgs/Twist`

## 思路说明

- 从 `Path` 中选取距离当前位姿最近的点作为起点，按索引采样出长度为 $N$ 的参考轨迹。
- 使用参考航向对二维平移进行线性化，构建最小二乘问题求解速度序列 `(vx, vy)`。
- 以同样方式对航向角速度 `w` 构建一维最小二乘问题。
- 取求解得到的第一个控制量作为当前时刻输出，并进行速度限幅。

这是一个轻量级的线性化 MPC 版本，适合快速验证。若需要更严格的约束或更高精度，可替换为带约束的 QP 求解器。

## 参数

- `control_rate` (默认 10.0): 控制频率
- `dt` (默认 0.1): 预测步长，建议设置为 $1/\text{control\_rate}$
- `horizon_steps` (默认 10): 预测步数
- `lambda` (默认 0.1): 正则化系数
- `max_vx` / `max_vy` / `max_w` : 速度限幅

## 调参建议

- 先只调 `max_vx/max_vy/max_w`，确保速度不超过机器人能力和仿真稳定范围。
- 设定 `control_rate` 后，用 $dt = 1/\text{control\_rate}$ 保持预测步长一致。
- `horizon_steps` 可从 8-15 开始：过小易抖动，过大会增大计算量和滞后。
- 轨迹过密时，适当降低 `control_rate` 或在上游做稀疏化，避免近邻点抖动。
- 跟踪发散时增大 `lambda` 稳定控制；跟踪滞后时减小 `lambda` 增强响应。

## 运行

```bash
roslaunch path_follower mpc_path_follower.launch
```

可通过 launch 文件的参数重映射话题或调整 MPC 参数。
