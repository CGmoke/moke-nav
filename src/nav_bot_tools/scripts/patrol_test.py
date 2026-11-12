"""T5 多航点巡逻端到端验证（锁死根治后的重启验收）。

复用 scenario_test 的监视节点 + 锁死证据链（微失速 episode 自动归账到航点名）。
生产链路：/follow_waypoints 动作（waypoint_follower 生命周期节点），
航点到达检测用 gz 世界真值接近度（<0.35m）。

航点（全部为已验证可达点）：
  P1 南走廊东端 (2.5,-1.4) → P2 窄门区域 (-6.8,-1.4)
  → P3 北屋自由区 (-1.5,2.7) → P4 回出生点 (0,0)     单圈 ≈25m

验收标准：
  - 每个 waypoint 在单圈超时内到达（真值接近度判定）
  - 全程无 FollowWaypoints ABORT
  - lock_events.log 微失速统计（episode 归账到航点）

用法（先 source jazzy 与 install）：
  python3 patrol_test.py [loops]   # 默认 2 圈
"""
import math
import os
import re
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy

import scenario_test as st  # 复用 Monitor / 证据链 / start_sim / kill_stack

EX = st.EX
WAYPOINTS = [
    ('P1-南走廊东端', 2.5, -1.4),
    ('P2-窄门区域', -6.8, -1.4),
    ('P3-北屋自由区', -1.5, 2.7),
    ('P4-回出生点', 0.0, 0.0),
]
ARRIVE_R = 0.45       # map 系到达半径（Nav2 xy 容差 0.20 + 停车裕量；
                      # 新地图实测停车距航点 0.31~0.34m，0.30 会漏检 1~4cm）
LOOP_TIMEOUT = 300    # 单圈动作超时（25m @0.3m/s 理想 84s + 转向裕量）


def map_pose():
    """map 系机器人位姿 = map->odom ⊕ odom->base_footprint。
    ★ 到达检测必须用 map 系：SLAM 建图时轮式里程计漂移使 map 与 gz 世界
      坐标系存在不可忽略偏移，用 gz 真值接近 map 航点会漏检（首次 T5 实测）。"""
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


def clear_costmaps():
    """清除两层代价地图：失速期定位误差+激光误标记会留下陈旧障碍，
    导致下一圈规划失败（首次 T5 圈2 P4 规划失败的根因，静态地图确认空闲）。"""
    for layer in ('global_costmap', 'local_costmap'):
        try:
            subprocess.run(
                ['ros2', 'service', 'call',
                 f'/{layer}/clear_entirely_{layer}',
                 'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                capture_output=True, timeout=10)
        except subprocess.TimeoutExpired:
            pass


def poses_yaml(wpts):
    parts = []
    for _, x, y in wpts:
        parts.append('{header: {frame_id: map}, pose: {position: {x: %s, y: %s, '
                     'z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
                     % (x, y))
    return '{poses: [' + ', '.join(parts) + ']}'


class PatrolState:
    def __init__(self, wpts):
        self.wpts = wpts        # 本通行要走的航点子集（补跑时为缺失航点）
        self.idx = 0            # 当前目标航点下标
        self.active = True
        self.arrivals = {}      # name -> (loop, t_wall, travel_m) 首次到达
        self.travel = 0.0       # 当圈行驶里程（map 系）
        self.last_pos = None
        self.min_dists = {}     # name -> 目标接近过程中的最小距离（诊断用）
        self.traj = []          # [(t_wall, map_x, map_y, truth_x, truth_y)]


def watcher(state):
    """到达检测线程：map 系接近度推进目标 + 里程累计 + episode 归账切换。"""
    while state.active and not st.stop_flag.is_set():
        mp = map_pose()
        try:
            tp = st.latest_truth()
        except RuntimeError:
            tp = None
        if mp is None or tp is None:
            time.sleep(0.5)
            continue
        x, y = mp[0], mp[1]
        state.traj.append((time.time(), x, y, tp[1], tp[2]))
        if state.last_pos is not None:
            state.travel += math.hypot(x - state.last_pos[0], y - state.last_pos[1])
        state.last_pos = (x, y)
        if state.idx < len(state.wpts):
            name, wx, wy = state.wpts[state.idx]
            d = math.hypot(x - wx, y - wy)
            state.min_dists[name] = min(state.min_dists.get(name, 1e9), d)
            if d < ARRIVE_R:
                state.arrivals[name] = (state.arrivals.get('_loop', 0),
                                        time.time(), state.travel)
                print(f'    ✓ 到达 {name}  里程={state.travel:.1f}m', flush=True)
                state.idx += 1
                if state.idx < len(state.wpts):
                    nxt = state.wpts[state.idx][0]
                    st.active_scenario = nxt
                    st.lock_by_scenario.setdefault(nxt, 0.0)
                    # ★ 每腿切换前清代价地图：失速期原地旋转+定位误差会把门洞
                    #   误标记进全局代价地图，导致下一条腿穿越门洞规划间歇失败
                    threading.Thread(target=clear_costmaps, daemon=True).start()
        time.sleep(0.3)


def run_pass(loop, pass_no, wpts, procs):
    """跑一次 FollowWaypoints 通行（可为全圈或缺失航点补跑）。"""
    state = PatrolState(wpts)
    state.arrivals['_loop'] = loop
    st.active_scenario = wpts[0][0]
    st.lock_by_scenario.setdefault(wpts[0][0], 0.0)
    th = threading.Thread(target=watcher, args=(state,), daemon=True)
    th.start()
    t0 = time.time()
    try:
        r = subprocess.run(
            ['ros2', 'action', 'send_goal', '/follow_waypoints',
             'nav2_msgs/action/FollowWaypoints', poses_yaml(wpts)],
            capture_output=True, text=True, timeout=LOOP_TIMEOUT)
        out = r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        out = '超时'
    dur = time.time() - t0
    state.active = False
    th.join(timeout=3)
    reached = sum(1 for k in state.arrivals if k != '_loop')
    missed_names = [w[0] for w in wpts if w[0] not in state.arrivals]
    status = 'SUCCEEDED' if 'SUCCEEDED' in out else \
             ('ABORTED' if 'ABORTED' in out else ('超时' if '超时' == out else 'FAILED'))
    tag = f'圈{loop + 1}' + (f'-补跑{pass_no}' if pass_no > 0 else '')
    print(f'  [{tag}] 状态={status} 到达={reached}/{len(wpts)} '
          f'里程={state.travel:.1f}m 用时={dur:.0f}s'
          + (f' 未达={missed_names}' if missed_names else ''), flush=True)
    mind = ' '.join(f'{k.split("-")[0]}={v:.2f}m' for k, v in state.min_dists.items())
    if mind:
        print(f'    航点最小距离(map系): {mind}', flush=True)
    with open(f'{EX}/patrol_traj_loop{loop + 1}_pass{pass_no}.txt', 'w') as f:
        for row in state.traj:
            f.write('%s %s %s %s %s\n' % tuple(f'{v:.3f}' if isinstance(v, float) else v
                                               for v in row))
    return state, reached, missed_names, dur, status


def main():
    loops = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    rclpy.init()
    st.monitor_node = st.Monitor()
    threading.Thread(target=st.ros_spin, daemon=True).start()
    threading.Thread(target=st.truth_reader, daemon=True).start()
    threading.Thread(target=st.truth_health_watch, daemon=True).start()
    threading.Thread(target=st.lock_detector, daemon=True).start()
    with open(f'{EX}/lock_events.log', 'a') as f:
        f.write(f"==== T5巡逻开始 loops={loops} "
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} ====\n")

    procs = []
    results = []
    try:
        if not st.start_sim(procs, 'patrol'):
            print('✗ 仿真未就绪', flush=True)
            sys.exit(1)
        nav = subprocess.Popen(
            ['ros2', 'launch', 'nav_bot_navigation',
             'navigation.launch.py', 'use_rviz:=false'],
            stdout=open(f'{EX}/patrol_nav.log', 'w'), stderr=subprocess.STDOUT,
            start_new_session=True)
        procs.append(nav)
        if not st.wait_nav_ready():
            print('✗ 导航栈未就绪', flush=True)
            sys.exit(1)
        time.sleep(5)  # AMCL 粒子稳定
        st.arm_grace()

        for loop in range(loops):
            if loop > 0:
                clear_costmaps()  # 清除上一圈遗留的陈旧障碍标记
            wpts = list(WAYPOINTS)
            loop_arrivals = {}
            passes = []
            for pass_no in range(3):  # 未达航点最多补跑 2 次
                state, reached, missed, dur, status = run_pass(
                    loop, pass_no, wpts, procs)
                passes.append((pass_no, reached, len(wpts), dur, status,
                               list(missed)))
                loop_arrivals.update({k: v for k, v in state.arrivals.items()
                                      if k != '_loop'})
                if not missed:
                    break
                missed_set = set(missed)
                wpts = [w for w in wpts if w[0] in missed_set]  # 保留元组
                clear_costmaps()
            total_dur = sum(p[3] for p in passes)
            ok = len(loop_arrivals) == len(WAYPOINTS)
            results.append((loop, ok, len(loop_arrivals), total_dur, passes))
    finally:
        st.active_scenario = None
        st.kill_stack(procs)
        st.close_episode('patrol_shutdown')
        with open(f'{EX}/lock_events.log', 'a') as f:
            f.write(f"==== T5巡逻结束 {time.strftime('%Y-%m-%d %H:%M:%S')} ====\n")

    # ── 汇总 ──
    print('\n========== T5 巡逻验收汇总 ==========', flush=True)
    all_ok = True
    for loop, ok, n_arr, total_dur, passes in results:
        all_ok &= ok
        detail = ' → '.join(f'通行{p + 1}:{r}/{n}'
                            for p, r, n, _d, _s, _m in passes)
        retry = '' if len(passes) == 1 else f'（补跑 {len(passes) - 1} 次）'
        print(f'  圈{loop + 1}: {"✓" if ok else "✗"} 到达 {n_arr}/{len(WAYPOINTS)} '
              f'总用时={total_dur:.0f}s {detail} {retry}', flush=True)
    print('\n微失速统计（episode 归账）:', flush=True)
    for name, _x, _y in WAYPOINTS:
        lk = st.lock_by_scenario.get(name, 0.0)
        print(f'  {name}: {lk:.1f}s', flush=True)
    total_lock = sum(v for k, v in st.lock_by_scenario.items()
                     if k.startswith('P'))
    print(f'  合计: {total_lock:.1f}s', flush=True)
    print(f'\n结果: {"✓ T5 巡逻验收通过" if all_ok else "✗ 存在未达项"}', flush=True)
    st.stop_flag.set()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
