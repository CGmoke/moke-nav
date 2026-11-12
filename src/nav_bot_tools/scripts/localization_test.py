"""T6 定位鲁棒性测试：AMCL 误差定量 + 初始位姿扰动 + 绑架重定位。

复用 scenario_test 基础设施（监视节点 / gz 真值 / 证据链）。

核心方法：同步采样 (map 系位姿, gz 世界真值) 对，Umeyama 拟合刚体变换
  map = R·world + t；拟合残差 = AMCL 真实定位误差（剥掉 SLAM 建图漂移
  造成的 map↔world 定义偏移 —— 该偏移本身也是 T5 发现的重要量化项）。

阶段：
  A 标定+误差基线：巡逻 P1→P2→P3→P4 驱动覆盖全屋，拟合变换，
    报告全屋 RMS + 门洞区域 RMS（门洞 map 中心 (-0.75,2.0) 附近 1.2m，
    即 T5 P4 规划失败的上游区域）
  B 初始位姿扰动：/initialpose 注入扰动 → 立即发近距导航目标，
    测收敛时间（位置误差<0.15m 且航向<0.25rad 持续 2s）与成功率
  C 绑架重定位：远距 belief 重置（≈2.1m+1rad），先量化"未察觉期"
    （AMCL 粒子云紧贴、无全局重定位能力 → 误差持续），再给 0.4m 提示
    测恢复收敛

用法（先 source jazzy 与 install）：
  python3 localization_test.py            # A+B+C 全跑
  python3 localization_test.py --a        # 仅标定+误差基线
"""
import math
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

import scenario_test as st

EX = st.EX
WAYPOINTS = [
    ('T6-P1', 2.5, -1.4),
    ('T6-P2', -6.8, -1.4),
    ('T6-P3', -1.5, 2.7),
    ('T6-P4', 0.0, 0.0),
]
DOOR_MAP = (-0.75, 2.0)   # 北屋门洞 map 系中心（T5 清障探针：该处自由半径仅 0.25m）
DOOR_R = 1.2              # 门洞区域半径（world 系，经标定变换映射）
CONV_POS = 0.15           # 收敛判据：位置误差 m
CONV_YAW = 0.25           # 收敛判据：航向误差 rad
CONV_HOLD = 2.0           # 收敛需持续秒数

# ── 采样对（map 位姿 ↔ 世界真值） ──
pairs = []                # [(t_wall, mx,my,myaw, tx,ty,tyaw)]
pairs_lock = threading.Lock()


def map_pose():
    """map 系位姿 = map->odom ⊕ odom->base_footprint（见 patrol_test 同名函数）。"""
    with st.data_lock:
        mo = st.map_odom_buf[-1] if st.map_odom_buf else None
        ob = st.tf_buf[-1] if st.tf_buf else None
    if mo is None or ob is None:
        return None
    mx, my, myaw = mo[1], mo[2], mo[3]
    ox, oy, oyaw = ob[1], ob[2], ob[3]
    x = mx + math.cos(myaw) * ox - math.sin(myaw) * oy
    y = my + math.sin(myaw) * ox + math.cos(myaw) * oy
    return x, y, st.wrap(myaw + oyaw)


def pair_sampler():
    while not st.stop_flag.is_set():
        mp = map_pose()
        try:
            tp = st.latest_truth(timeout=1.0)
        except RuntimeError:
            time.sleep(0.3)
            continue
        if mp is not None:
            with pairs_lock:
                pairs.append((time.time(), mp[0], mp[1], mp[2],
                              tp[1], tp[2], tp[4]))
        time.sleep(0.2)


def latest_pair():
    with pairs_lock:
        return pairs[-1] if pairs else None


def fit_transform(t0, t1=None):
    """Umeyama：拟合 map = R·world + t，返回 dict(R, t, theta, rms_pos, rms_yaw, n)。"""
    with pairs_lock:
        pts = [p for p in pairs if (t0 is None or p[0] >= t0)
               and (t1 is None or p[0] <= t1)]
    if len(pts) < 30:
        return None
    M = np.array([[p[1], p[2]] for p in pts])          # map
    T = np.array([[p[4], p[5]] for p in pts])          # truth(world)
    mc, tc = M.mean(axis=0), T.mean(axis=0)
    H = (M - mc).T @ (T - tc) / len(pts)               # 2x2 互协方差
    U, S, Vt = np.linalg.svd(H)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    t = mc - R @ tc
    theta = math.atan2(R[1, 0], R[0, 0])
    # 残差
    pred = (R @ T.T).T + t
    pos_err = np.linalg.norm(M - pred, axis=1)
    yaw_err = np.array([st.wrap(p[3] - p[6] - theta) for p in pts])
    return dict(R=R, t=t, theta=theta,
                rms_pos=float(np.sqrt((pos_err ** 2).mean())),
                rms_yaw=float(np.sqrt((yaw_err ** 2).mean())),
                n=len(pts))


def aligned_err(fit, pair):
    """单采样对的拟合后误差 (pos_m, yaw_rad)。"""
    _, mx, my, myaw, tx, ty, tyaw = pair
    px = fit['R'][0, 0] * tx + fit['R'][0, 1] * ty + fit['t'][0]
    py = fit['R'][1, 0] * tx + fit['R'][1, 1] * ty + fit['t'][1]
    return math.hypot(mx - px, my - py), st.wrap(myaw - tyaw - fit['theta'])


def world_of_map(fit, x, y):
    """map → world 逆变换（门洞中心等）。"""
    d = np.array([x, y]) - fit['t']
    w = fit['R'].T @ d
    return float(w[0]), float(w[1])


def current_error(fit):
    p = latest_pair()
    if p is None or fit is None:
        return None, None
    return aligned_err(fit, p)


def wait_convergence(fit, deadline, pub_fn=None):
    """等待误差收敛到 (CONV_POS, CONV_YAW) 且持续 CONV_HOLD，返回耗时或 None。"""
    t0 = time.time()
    ok_since = None
    while time.time() < deadline:
        e = current_error(fit)
        if e[0] is not None and e[0] < CONV_POS and abs(e[1]) < CONV_YAW:
            if ok_since is None:
                ok_since = time.time()
            elif time.time() - ok_since >= CONV_HOLD:
                return time.time() - t0
        else:
            ok_since = None
        time.sleep(0.2)
    return None


# ── /initialpose 发布（belief 重置/扰动注入） ──

helper_node = None
ip_pub = None


def init_publisher():
    global helper_node, ip_pub
    helper_node = rclpy.create_node('t6_loc_helper')
    ip_pub = helper_node.create_publisher(PoseWithCovarianceStamped,
                                          '/initialpose', 10)
    # ★ 必须用独立 executor：rclpy.spin 用全局 executor，与 monitor 的 spin
    #   冲突抛 "Executor is already spinning"，导致 /initialpose 从未发出
    #   （首轮 T6 的 B/C 阶段因此测了个寂寞）
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(helper_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.5)


def publish_initialpose(x, y, yaw):
    cov = [0.0] * 36
    cov[0] = 0.25       # x
    cov[7] = 0.25       # y
    cov[35] = 0.10      # yaw
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = helper_node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.w = float(math.cos(yaw / 2))
    msg.pose.pose.orientation.z = float(math.sin(yaw / 2))
    msg.pose.covariance = cov
    for _ in range(3):  # 连发 3 次防丢
        ip_pub.publish(msg)
        time.sleep(0.1)


def publish_perturbed(fit, dx, dy, dyaw, from_truth=False):
    """在参考位姿基础上注入扰动。
    from_truth=False：基于当前 belief（Phase B，belief≈真值时正确）；
    from_truth=True：基于真值经 fit 映射到 map（Phase C 绑架后 belief
    本身偏 2.2m，hint 必须以真值为基准——首轮 bug：hint=错 belief+0.4m，
    实际仍偏 1.8m，永远收敛不了）。"""
    p = latest_pair()
    if from_truth:
        mx = fit['R'][0, 0] * p[4] + fit['R'][0, 1] * p[5] + fit['t'][0]
        my = fit['R'][1, 0] * p[4] + fit['R'][1, 1] * p[5] + fit['t'][1]
        myaw = p[6] + fit['theta']
    else:
        mx, my, myaw = p[1], p[2], p[3]
    publish_initialpose(mx + dx, my + dy, myaw + dyaw)


# ── 阶段 ──

def phase_a(procs):
    """标定 + 误差基线 + 门洞定量。"""
    print('== A: 标定驱动 + 误差基线 ==', flush=True)
    t0 = time.time()
    goals = []
    for name, x, y in WAYPOINTS:
        ok, detail = st.nav_goal(x, y, name)
        goals.append((name, ok))
        print(f'  [{name}] {detail}', flush=True)
    t1 = time.time()
    fit = fit_transform(t0, t1)
    if fit is None:
        print('✗ 标定失败：采样对不足', flush=True)
        return None
    print(f'  map↔world 变换: θ={math.degrees(fit["theta"]):+.2f}° '
          f't=({fit["t"][0]:+.3f},{fit["t"][1]:+.3f})  n={fit["n"]}', flush=True)
    print(f'  全屋定位误差: RMS_pos={fit["rms_pos"]:.3f}m '
          f'RMS_yaw={math.degrees(fit["rms_yaw"]):.2f}°', flush=True)
    # 门洞区域定量
    dx_w, dy_w = world_of_map(fit, *DOOR_MAP)
    print(f'  门洞 world 坐标≈({dx_w:.2f},{dy_w:.2f})', flush=True)
    with pairs_lock:
        pts = [p for p in pairs if t0 <= p[0] <= t1]
    dz, oz = [], []
    for p in pts:
        d = math.hypot(p[4] - dx_w, p[5] - dy_w)
        e = aligned_err(fit, p)
        (dz if d <= DOOR_R else oz).append(e)
    for label, arr in (('门洞区', dz), ('非门洞', oz)):
        if arr:
            pe = [e[0] for e in arr]
            ye = [abs(e[1]) for e in arr]
            print(f'  {label}: n={len(arr)} RMS_pos={np.sqrt(np.mean(np.square(pe))):.3f}m '
                  f'max={max(pe):.3f}m RMS_yaw={math.degrees(np.mean(ye)):.2f}°', flush=True)
        else:
            print(f'  {label}: 无样本', flush=True)
    fit['goals'] = goals
    return fit


def farthest_waypoint():
    """距当前位姿最远的航点（重定位测试必须有真实运动激励；
    首轮 bug：选了脚下航点 → 车不动 → AMCL 无激励 → 假性未收敛）。"""
    p = latest_pair()
    best, bd = WAYPOINTS[0], -1
    for w in WAYPOINTS:
        d = math.hypot(p[1] - w[1], p[2] - w[2])
        if d > bd:
            best, bd = w, d
    return best


def phase_b(fit):
    """初始位姿扰动矩阵。"""
    print('== B: 初始位姿扰动 ==', flush=True)
    perturbs = [
        ('零扰动对照', 0.0, 0.0, 0.0),
        ('平移0.5m', 0.5, 0.0, 0.0),
        ('旋转0.5rad', 0.0, 0.0, 0.5),
        ('组合1m+0.7rad', 0.8, -0.6, 0.7),
    ]
    results = []
    for i, (name, dx, dy, dyaw) in enumerate(perturbs):
        tgt = WAYPOINTS[i % len(WAYPOINTS)]  # 轮换航点保证运动激励
        publish_perturbed(fit, dx, dy, dyaw)
        time.sleep(1.0)
        # 立即导航触发运动辅助收敛
        goal_th = threading.Thread(
            target=st.nav_goal, args=(tgt[1], tgt[2], f'B-{name}'),
            daemon=True)
        goal_th.start()
        conv = wait_convergence(fit, deadline=time.time() + 120)
        goal_th.join(timeout=150)
        results.append((name, conv))
        conv_s = f'{conv:.1f}s' if conv is not None else '✗未收敛(120s)'
        print(f'  [{name}] 收敛={conv_s}（判据 {CONV_POS}m/{CONV_YAW}rad 持续{CONV_HOLD}s）',
              flush=True)
        time.sleep(2)
    return results


def phase_c(fit):
    """绑架重定位：远距 belief 重置 → 未察觉期量化 → 提示恢复。"""
    print('== C: 绑架重定位 ==', flush=True)
    # 1) 远距 belief 重置，静止观察 15s：AMCL 无全局重定位能力，误差应持续
    publish_perturbed(fit, 2.0, 1.0, 1.0)
    time.sleep(1.0)
    errs = []
    t0 = time.time()
    while time.time() - t0 < 15:
        e = current_error(fit)
        if e[0] is not None:
            errs.append(e[0])
        time.sleep(0.3)
    print(f'  未察觉期(15s静止): 误差均值={np.mean(errs):.2f}m '
          f'最小={min(errs):.2f}m 最大={max(errs):.2f}m'
          f'{"（误差持续=AMCL无自愈，符合预期）" if np.mean(errs) > 1.0 else "（警告：意外自愈？）"}',
          flush=True)
    # 2) 提示重定位（基于真值 +0.4m 误差）+ 导航到最远航点（运动激励）
    tgt = farthest_waypoint()
    publish_perturbed(fit, 0.4, 0.0, 0.1, from_truth=True)
    time.sleep(1.0)
    goal_th = threading.Thread(
        target=st.nav_goal, args=(tgt[1], tgt[2], 'C-提示恢复'), daemon=True)
    goal_th.start()
    conv = wait_convergence(fit, deadline=time.time() + 120)
    goal_th.join(timeout=150)
    conv_s = f'{conv:.1f}s' if conv is not None else '✗未收敛(120s)'
    print(f'  提示恢复(0.4m): 收敛={conv_s}', flush=True)


def main():
    args = sys.argv[1:]
    mode = 'full' if '--full' in args or not args else None
    if '--a' in args:
        mode = 'a'

    rclpy.init()
    st.monitor_node = st.Monitor()
    threading.Thread(target=st.ros_spin, daemon=True).start()
    threading.Thread(target=st.truth_reader, daemon=True).start()
    threading.Thread(target=st.truth_health_watch, daemon=True).start()
    threading.Thread(target=st.lock_detector, daemon=True).start()
    threading.Thread(target=pair_sampler, daemon=True).start()
    init_publisher()
    with open(f'{EX}/lock_events.log', 'a') as f:
        f.write(f"==== T6定位测试 {time.strftime('%Y-%m-%d %H:%M:%S')} ====\n")

    procs = []
    try:
        if not st.start_sim(procs, 't6'):
            print('✗ 仿真未就绪', flush=True)
            sys.exit(1)
        nav = subprocess.Popen(
            ['ros2', 'launch', 'nav_bot_navigation',
             'navigation.launch.py', 'use_rviz:=false'],
            stdout=open(f'{EX}/t6_nav.log', 'w'), stderr=subprocess.STDOUT,
            start_new_session=True)
        procs.append(nav)
        if not st.wait_nav_ready():
            print('✗ 导航栈未就绪', flush=True)
            sys.exit(1)
        time.sleep(5)
        st.arm_grace()

        fit = phase_a(procs)
        if fit is None:
            sys.exit(1)
        if mode in ('full',):
            phase_b(fit)
            phase_c(fit)
    finally:
        st.active_scenario = None
        st.kill_stack(procs)
        st.close_episode('t6_shutdown')
        with open(f'{EX}/lock_events.log', 'a') as f:
            f.write(f"==== T6结束 {time.strftime('%Y-%m-%d %H:%M:%S')} ====\n")

    # 采样对落盘供离线分析
    with pairs_lock:
        dump = list(pairs)
    with open(f'{EX}/t6_pairs.txt', 'w') as f:
        for p in dump:
            f.write(' '.join(f'{v:.4f}' for v in p) + '\n')
    print(f'\n采样对已落盘: {EX}/t6_pairs.txt ({len(dump)} 对)', flush=True)
    st.stop_flag.set()


if __name__ == '__main__':
    main()
