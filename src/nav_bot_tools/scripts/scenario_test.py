"""旋转锁死全栈场景验收（slip 顺应性根治方案验证）。

架构（吸取动态位姿洪流教训）：
  - ROS 侧 rclpy 监视节点：/imu/data（陀螺仪 = 旋转真值，见项目记忆）、
    /cmd_vel（指令）、/tf（EKF 融合 odom->base_footprint 位姿，Nav2 场景用）
  - gz 世界真值（dynamic_pose/info）单订阅线程：xy/yaw 绝对真值，带卡死检测
  - 锁死判定：指令 |wz|>0.25 持续 1.5s 期间，陀螺仪 |wz| < 0.1×指令
    （历史特征：健康 ratio≈0.5，锁死 ratio<0.05）

场景：
  物理层（仅仿真，直发 /cmd_vel）：
    S1 原地旋转 720°       —— 纯旋转锁死主战场（历史 3/3 复现）
    S2 窄空间转向 8 段     —— 90° 正反交替（模拟窄门调整朝向）
    S3 多障碍环绕（蛇形）  —— 左右弧线交替（模拟绕障）
    S4 连续路径跟踪 40s    —— 双向弧线（模拟 MPPI 持续输出）
  导航层（sim + navigation.launch.py）：
    N1 南走廊直线 (2.5,-1.4) / N2 窄门区域 (-6.8,-1.4) / N3 北屋穿行 (-4.5,2.6)

用法（先 source /opt/ros/jazzy/setup.bash 与 install/setup.bash）：
  python3 scenario_test.py --physics | --nav | --full
"""
import math
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

WS = '/home/moke/moke-nav_ws'
EX = f'{WS}/experiments'
TRUTH_TOPIC = '/world/default/dynamic_pose/info'
# 物理层场景用空地世界（确定性），导航层用 small_house（真实环境）
EMPTY_WORLD = f'{WS}/install/nav_bot_gazebo/share/nav_bot_gazebo/worlds/empty_test.world'

# ── 监视数据（全部有锁保护） ──
data_lock = threading.Lock()
imu_buf = []      # [(t_sim, gz)]  陀螺仪 z
cmd_buf = []      # [(t_wall, vx, wz)]
tf_buf = []       # [(t_sim, x, y, yaw)]  EKF odom->base_footprint
truth_buf = []    # [(t_sim, x, y, yaw)]  gz 世界真值
truth_alive = True

active_scenario = None
lock_by_scenario = {}
stop_flag = threading.Event()


def wrap(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ── ROS 监视节点 ──

class Monitor(Node):
    def __init__(self):
        super().__init__('scenario_monitor')
        self.create_subscription(Imu, '/imu/data', self.on_imu, 30)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 30)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with data_lock:
            imu_buf.append((t, msg.angular_velocity.z))
            if len(imu_buf) > 4000:
                del imu_buf[:2000]

    def on_cmd(self, msg):
        with data_lock:
            cmd_buf.append((time.time(), msg.linear.x, msg.angular.z))
            if len(cmd_buf) > 2000:
                del cmd_buf[:1000]

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                q = tr.transform.rotation
                p = tr.transform.translation
                yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z ** 2)
                with data_lock:
                    tf_buf.append((t, p.x, p.y, yaw))
                    if len(tf_buf) > 4000:
                        del tf_buf[:2000]


def ros_spin():
    rclpy.spin(monitor_node)


# ── gz 世界真值（单订阅 + 自动重连 + 卡死检测） ──

def truth_reader():
    global truth_alive
    dbg = open(f'{EX}/truth_reader_dbg.log', 'w')
    while not stop_flag.is_set():
        p = subprocess.Popen(['gz', 'topic', '-e', '-t', TRUTH_TOPIC],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        t_sim = 0.0
        cur = None
        section = None
        got_line = False
        n_lines = 0
        dbg.write(f'[{time.time():.0f}] CLI 启动 pid={p.pid}\n'); dbg.flush()
        for line in p.stdout:
            if stop_flag.is_set():
                break
            got_line = True
            n_lines += 1
            if n_lines % 2000 == 0:
                dbg.write(f'[{time.time():.0f}] 行数={n_lines}\n'); dbg.flush()
            s = line.strip()
            if s.startswith('sec:') or s.startswith('nsec:'):
                v = float(s.split(':')[1])
                t_sim = v if s.startswith('sec:') else t_sim + v * 1e-9
                continue
            if s.startswith('name:'):
                cur = {} if 'nav_bot' in s else None
                section = None
                continue
            if cur is None:
                continue
            if s == 'position {':
                section = 'position'
                continue
            if s == 'orientation {':
                section = 'orientation'
                continue
            m = re.match(r'(x|y|z|w): (\S+)', s)
            if m and section:
                prefix = 'p' if section == 'position' else 'q'
                cur[prefix + m.group(1)] = float(m.group(2))
                if all(k in cur for k in ('px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw')):
                    yaw = math.atan2(2 * (cur['qw'] * cur['qz']),
                                     1 - 2 * cur['qz'] ** 2)
                    with data_lock:
                        truth_buf.append((t_sim, cur['px'], cur['py'], cur['pz'], yaw))
                        if len(truth_buf) > 6000:
                            del truth_buf[:3000]
                    cur = {}
                    section = None
        p.kill()
        dbg.write(f'[{time.time():.0f}] CLI 退出，行数={n_lines}\n'); dbg.flush()
        if got_line and not stop_flag.is_set():
            # 原本有数据流但 EOF（server 退出）：短暂等待重连
            time.sleep(0.5)
        elif not stop_flag.is_set():
            time.sleep(1.0)


def truth_health_watch():
    """真值流卡死检测：10s 无新帧 → 标记失活。"""
    global truth_alive
    last_n = 0
    while not stop_flag.is_set():
        time.sleep(2)
        with data_lock:
            n = len(truth_buf)
        if n == last_n and n > 0:
            truth_alive = False
        elif n > last_n:
            truth_alive = True
        last_n = n


# ── 取数工具 ──

def latest_truth(timeout=20):
    """最新 gz 真值帧（阻塞等待）。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        with data_lock:
            if truth_buf:
                return truth_buf[-1]
        time.sleep(0.3)
    raise RuntimeError('gz 真值流 20s 无数据')


def truth_since(t_sim0):
    with data_lock:
        return [f for f in truth_buf if f[0] >= t_sim0]


def imu_window(t_sim0, t_sim1=None):
    with data_lock:
        return [(t, g) for t, g in imu_buf
                if t >= t_sim0 and (t_sim1 is None or t <= t_sim1)]


def latest_imu_t():
    with data_lock:
        return imu_buf[-1][0] if imu_buf else 0.0


def publish_cmd(vx, wz, duration):
    """10Hz 直发 /cmd_vel duration 墙钟秒，返回 (t_wall0, t_wall1)。"""
    t0 = time.time()
    p = subprocess.Popen(
        ['ros2', 'topic', 'pub', '-r', '10', '/cmd_vel',
         'geometry_msgs/msg/Twist',
         '{linear: {x: %f}, angular: {z: %f}}' % (vx, wz)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    while time.time() - t0 < duration:
        time.sleep(0.2)
    p.terminate()
    p.wait(timeout=5)
    return t0, time.time()


def stop_cmd():
    publish_cmd(0.0, 0.0, 1.0)


# ── 锁死检测（陀螺仪为旋转真值） ──

grace_until = [0.0]  # 每场景前 1.2s 指令建立瞬态不计锁死


def lock_detector():
    while not stop_flag.is_set():
        time.sleep(0.2)
        now = time.time()
        with data_lock:
            recent_cmd = [(t, vx, wz) for t, vx, wz in cmd_buf if now - t < 1.2]
            imu = list(imu_buf[-300:])
        if not recent_cmd or len(imu) < 20:
            continue
        if now < grace_until[0]:
            continue
        t0, t1 = imu[0][0], imu[-1][0]
        if t1 - t0 < 1.5:
            continue
        wz_cmd = max(abs(wz) for _, _, wz in recent_cmd)
        if wz_cmd < 0.25:
            continue
        # 指令窗口对应 imu 末段 1.5s 的平均 |gz|
        tail = [g for t, g in imu if t >= t1 - 1.5]
        if not tail:
            continue
        gz_mean = sum(abs(g) for g in tail) / len(tail)
        if gz_mean < 0.1 * wz_cmd:
            sc = active_scenario
            if sc:
                lock_by_scenario[sc] = lock_by_scenario.get(sc, 0.0) + 0.2


def arm_grace(seconds=1.2):
    """场景开始时调用：给指令建立留出宽限窗。"""
    global grace_until
    grace_until[0] = time.time() + seconds


# ── 场景装饰器 ──

def scenario(fn):
    def wrapper(name, *args, **kw):
        global active_scenario
        active_scenario = name
        lock_by_scenario.setdefault(name, 0.0)
        arm_grace()
        try:
            p_start = latest_truth()
        except RuntimeError:
            p_start = None
        t0 = time.time()
        try:
            ok, detail = fn(*args, **kw)
        except RuntimeError as e:
            ok, detail = False, str(e)
        dur = time.time() - t0
        active_scenario = None
        lock = lock_by_scenario.get(name, 0.0)
        verdict = '✓' if ok else '✗'
        pos = ''
        try:
            p_end = latest_truth()
            pos = f' 位姿=({p_end[1]:.2f},{p_end[2]:.2f},z={p_end[3]:.3f})'
            if p_start:
                drift = math.hypot(p_end[1] - p_start[1], p_end[2] - p_start[2])
                pos += f' 场景位移={drift:.2f}m'
        except RuntimeError:
            pass
        print(f'  [{verdict}] {name}: {detail}{pos}  锁死={lock:.1f}s 耗时={dur:.0f}s',
              flush=True)
        return ok
    return wrapper


def gyro_yaw_change(t_sim0, t_sim1):
    """陀螺仪积分得到 t_sim0→t_sim1 的累计偏航（rad，符号）。"""
    w = imu_window(t_sim0, t_sim1)
    if len(w) < 5:
        raise RuntimeError('IMU 样本不足')
    ys = 0.0
    for (ta, ga), (tb, gb) in zip(w, w[1:]):
        dt = tb - ta
        if 0 < dt < 0.5:
            ys += (ga + gb) / 2 * dt
    return ys


# ── 物理层场景 ──

@scenario
def s1_spin720():
    """原地旋转 720°（两圈连续，wz=1.0×16s）。"""
    t0 = latest_truth()[0]
    w0 = latest_imu_t()
    publish_cmd(0.0, 1.0, 16)
    stop_cmd()
    time.sleep(0.5)
    w1 = latest_imu_t()
    dy = gyro_yaw_change(w0, w1)
    deg = math.degrees(abs(dy))
    # 健康 ratio≈0.46±0.06 → 16s 约 420±55°（实测 379~430°）；
    # 锁死基线仅 90°。阈值 360°（ratio 0.35）与锁死清晰分离。
    return deg > 360, f'陀螺仪累计偏航={deg:.0f}°（目标>360°）'


@scenario
def s2_narrow_segments():
    """窄空间转向：8 段正反交替（wz=±0.8×4s，段间停 1s）。
    健康 ratio≈0.46 → 段均约 84°；variance 下限 40° 判过。"""
    segs = []
    for i in range(8):
        wz = 0.8 if i % 2 == 0 else -0.8
        w0 = latest_imu_t()
        publish_cmd(0.0, wz, 4.0)
        stop_cmd()
        time.sleep(1.0)
        w1 = latest_imu_t()
        dy = gyro_yaw_change(w0, w1)
        segs.append(abs(math.degrees(dy)))
    ok = all(s > 40 for s in segs)
    return ok, '段偏航°=' + ','.join(f'{s:.0f}' for s in segs)


@scenario
def s3_snake():
    """多障碍环绕（锯齿蛇形）：6 段左右弧交替（v=0.25, wz=±0.35×3.5s），
    路径沿 +x 走廊锯齿前进，避免开环弧线跑进房间。"""
    arcs = []
    p_first = latest_truth()
    for i in range(6):
        wz = 0.35 if i % 2 == 0 else -0.35
        p0 = latest_truth()
        publish_cmd(0.25, wz, 3.5)
        p1 = latest_truth()
        d = math.hypot(p1[1] - p0[1], p1[2] - p0[2])
        arcs.append(d)
    p_last = latest_truth()
    total = math.hypot(p_last[1] - p_first[1], p_last[2] - p_first[2])
    ok = all(a > 0.5 for a in arcs) and total > 2.0
    return ok, (f'弧位移m=' + ','.join(f'{a:.2f}' for a in arcs) +
                f' 净位移={total:.2f}m')


@scenario
def s4_curve_track():
    """连续路径跟踪：S 形大弧（v=0.3, wz=±0.4×12s×2），空地世界无障碍。"""
    p0 = latest_truth()
    publish_cmd(0.3, 0.4, 12)
    publish_cmd(0.3, -0.4, 12)
    stop_cmd()
    p1 = latest_truth()
    d = math.hypot(p1[1] - p0[1], p1[2] - p0[2])
    ok = d > 4.0
    return ok, f'净位移={d:.1f}m（目标>4m）'


PHYSICS = [
    # 路径场景在前：出生点朝向 +x（东走廊），开环路径可预测；
    # 纯旋转场景（S2/S1）不依赖朝向且会打乱朝向，放最后。
    ('S3-多障碍环绕蛇形', s3_snake),
    ('S4-连续路径跟踪', s4_curve_track),
    ('S2-窄空间转向8段', s2_narrow_segments),
    ('S1-原地旋转720°', s1_spin720),
]


# ── 导航层场景 ──

def nav_goal(x, y, name, timeout=150):
    """发 NavigateToPose 目标并等待结果（锁死归账到本目标）。"""
    global active_scenario
    active_scenario = name
    lock_by_scenario.setdefault(name, 0.0)
    p0 = latest_truth()
    cmd = (f'{{pose: {{header: {{frame_id: map}}, pose: '
           f'{{position: {{x: {x}, y: {y}, z: 0.0}}, '
           f'orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}}}')
    try:
        r = subprocess.run(
            ['ros2', 'action', 'send_goal', '/navigate_to_pose',
             'nav2_msgs/action/NavigateToPose', cmd],
            capture_output=True, text=True, timeout=timeout)
        ok = 'SUCCEEDED' in (r.stdout + r.stderr)
    except subprocess.TimeoutExpired:
        ok = False
    p1 = latest_truth()
    d = math.hypot(p1[1] - p0[1], p1[2] - p0[2])
    active_scenario = None
    return ok, f'结果={"成功" if ok else "失败"} 位移={d:.1f}m'


def wait_nav_ready(timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = subprocess.run(['ros2', 'lifecycle', 'get', '/bt_navigator'],
                           capture_output=True, text=True, timeout=10)
        if 'active' in r.stdout.lower():
            return True
        time.sleep(2)
    return False


def start_sim(procs, log_prefix, world=None):
    cmd = ['ros2', 'launch', 'nav_bot_gazebo', 'sim.launch.py', 'gui:=false']
    if world:
        cmd.append(f'world:={world}')
    sim = subprocess.Popen(
        cmd,
        stdout=open(f'{EX}/{log_prefix}_sim.log', 'w'), stderr=subprocess.STDOUT,
        start_new_session=True)
    procs.append(sim)
    t0 = time.time()
    while time.time() - t0 < 45:
        with data_lock:
            if len(truth_buf) >= 6:
                return True
        time.sleep(1)
    return False


def kill_stack(procs):
    for p in reversed(procs):
        try:
            p.send_signal(signal.SIGINT)
        except Exception:
            pass
    time.sleep(4)
    for p in reversed(procs):
        try:
            p.terminate()
        except Exception:
            pass


def main():
    global active_scenario
    args = sys.argv[1:]
    mode = 'full'
    if '--physics' in args: mode = 'physics'
    if '--nav' in args: mode = 'nav'

    rclpy.init()
    global monitor_node
    monitor_node = Monitor()
    threading.Thread(target=ros_spin, daemon=True).start()
    threading.Thread(target=truth_reader, daemon=True).start()
    threading.Thread(target=truth_health_watch, daemon=True).start()
    threading.Thread(target=lock_detector, daemon=True).start()

    results = {}
    procs = []

    try:
        if mode in ('physics', 'full'):
            print('== 物理层场景（空地世界，Nav2 不在场）==', flush=True)
            if not start_sim(procs, 'phys', world=EMPTY_WORLD):
                print('✗ 仿真真值流未就绪', flush=True)
                kill_stack(procs)
                sys.exit(1)
            time.sleep(3)
            for name, fn in PHYSICS:
                results[name] = fn(name)
            kill_stack(procs)
            procs.clear()
            time.sleep(3)

        if mode in ('nav', 'full'):
            print('== 导航层场景（Nav2 全栈）==', flush=True)
            if not start_sim(procs, 'nav'):
                print('✗ 仿真真值流未就绪', flush=True)
                kill_stack(procs)
                sys.exit(1)
            nav = subprocess.Popen(
                ['ros2', 'launch', 'nav_bot_navigation',
                 'navigation.launch.py', 'use_rviz:=false'],
                stdout=open(f'{EX}/nav_nav.log', 'w'), stderr=subprocess.STDOUT,
                start_new_session=True)
            procs.append(nav)
            if not wait_nav_ready():
                print('✗ 导航栈未就绪', flush=True)
                kill_stack(procs)
                sys.exit(1)
            time.sleep(5)  # AMCL 粒子稳定
            for name, x, y in [('N1-南走廊直线', 2.5, -1.4),
                               ('N2-窄门区域', -6.8, -1.4),
                               ('N3-北屋穿行', -4.5, 2.6)]:
                ok, detail = nav_goal(x, y, name)
                results[name] = ok
                print(f'  [{"✓" if ok else "✗"}] {name}: {detail}  '
                      f'锁死={lock_by_scenario.get(name, 0):.1f}s', flush=True)
            kill_stack(procs)
    finally:
        kill_stack(procs)
        # 轨迹留档（事后分析：撞家具/下沉/漂移定位）
        try:
            with data_lock:
                traj = list(truth_buf)
            with open(f'{EX}/{mode}_trajectory.txt', 'w') as f:
                for t, x, y, z, yaw in traj[::20]:  # 1/20 抽样
                    f.write(f'{t:.2f} {x:.3f} {y:.3f} {z:.4f} {yaw:.3f}\n')
        except Exception:
            pass

    print('\n========== 验收汇总 ==========', flush=True)
    npass = 0
    for name, ok in results.items():
        npass += ok
        print(f'  {"✓" if ok else "✗"} {name}  (锁死 {lock_by_scenario.get(name, 0):.1f}s)')
    print(f'\n通过 {npass}/{len(results)}', flush=True)
    stop_flag.set()
    sys.exit(0 if npass == len(results) else 1)


if __name__ == '__main__':
    main()
