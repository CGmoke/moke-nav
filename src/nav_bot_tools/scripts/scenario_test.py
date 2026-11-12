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
from sensor_msgs.msg import Imu, JointState
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
map_odom_buf = [] # [(t_sim, x, y, yaw)]  AMCL map->odom（巡逻到达检测用 map 系位姿）
truth_buf = []    # [(t_sim, x, y, z, yaw)]  gz 世界真值
js_buf = []       # [(t_sim, {joint: velocity})]  轮关节速度（锁死证据链）
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
        self.create_subscription(JointState, '/joint_states', self.on_js, 30)

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
            elif tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                q = tr.transform.rotation
                p = tr.transform.translation
                yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z ** 2)
                with data_lock:
                    map_odom_buf.append((t, p.x, p.y, yaw))
                    if len(map_odom_buf) > 1500:
                        del map_odom_buf[:800]

    def on_js(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vel = dict(zip(msg.name, msg.velocity))
        with data_lock:
            js_buf.append((t, vel))
            if len(js_buf) > 1500:
                del js_buf[:800]


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


# ── 锁死检测（陀螺仪为旋转真值）+ 证据链记录器 ──
#
# 判定哲学：区分"真锁死"与"瞬态"，靠三路独立证据交叉验证：
#   ① /cmd_vel    指令（取最新值而非窗口 max —— 修复减速段残留旧值的窗口错位误计）
#   ② /imu/data   陀螺仪 gz（车体旋转真值）
#   ③ gz 世界真值 yaw 速率（第二旋转证人，校验陀螺仪）
#   ④ /joint_states 轮速（真锁死历史特征：轮速满速执行 + 车体不转）
#
# 纪元（epoch）对齐：wz 指令从 <0.25 跨到 ≥0.25 视为新纪元；
#   纪元前 0.8s 是物理爬坡期（az_max=3.5 → 0.43s 达满速 + 传感器滞后），不计锁死。
#   真锁死会持续 2.5s+ 且 gz≈0，不受爬坡豁免影响。

grace_until = [0.0]  # 每场景前 1.2s 指令建立瞬态不计锁死
EPOCH_MIN_WZ = 0.25  # 低于此视为非旋转指令
SPINUP_SKIP = 0.8    # 纪元开始后的物理爬坡豁免窗
LOCK_RATIO = 0.1     # gz < 10%×wz_cmd 判疑似
WHEELS = ('left_front_wheel_joint', 'right_front_wheel_joint',
          'left_rear_wheel_joint', 'right_rear_wheel_joint')
TRACK_HALF, WHEEL_R = 0.24, 0.09   # 纯旋转期望轮速 = wz × TRACK_HALF / WHEEL_R

episode = {'open': False, 'scenario': None, 'idx': 0, 'samples': []}


def truth_motion(t_sim, dur=1.0):
    """gz 真值在 t_sim 前后 dur 窗内的 (yaw_rate, lin_speed)，样本不足返回 (None, None)。"""
    with data_lock:
        frames = [f for f in truth_buf if t_sim - dur <= f[0] <= t_sim + 0.2]
    if len(frames) < 3:
        return None, None
    dy = wrap(frames[-1][4] - frames[0][4])
    dt = frames[-1][0] - frames[0][0]
    if dt <= 0:
        return None, None
    d = math.hypot(frames[-1][1] - frames[0][1], frames[-1][2] - frames[0][2])
    return abs(dy) / dt, d / dt


def wheel_max_speed(t_sim):
    """t_sim ±0.3s 内四轮最大轮速（rad/s）。"""
    with data_lock:
        frames = [f for f in js_buf if abs(f[0] - t_sim) < 0.3]
    if not frames:
        return None
    return max(abs(vel.get(w, 0.0)) for _, vel in frames for w in WHEELS)


def close_episode(reason):
    """结束一个疑似锁死 episode：计算判据、写事件日志与 CSV 证据。"""
    if not episode['open']:
        return
    episode['open'] = False
    samples = episode['samples']
    if not samples:
        return
    dur = samples[-1][0] - samples[0][0]
    wz = max(s[1] for s in samples)
    # 列序: (t, wz, vx, gz, truth_yaw_rate, truth_speed, wheel_max)
    gz_ratio = min(s[3] / s[1] for s in samples if s[1] > 0.05)
    ty = [s[4] for s in samples if s[4] is not None]
    ty_ratio = (sum(ty) / len(ty)) / wz if ty and wz > 0.05 else None
    ws = [s[6] for s in samples if s[6] is not None]
    ws_ratio = (max(ws) / (wz * TRACK_HALF / WHEEL_R)) if ws and wz > 0.05 else None
    # 自动判定（真值不转+轮子在转即锁死，与时长无关；阴性对照 S4 短 episode 验证）
    if (gz_ratio < LOCK_RATIO
            and (ty_ratio is None or ty_ratio < 0.15)
            and (ws_ratio is None or ws_ratio > 0.6)):
        verdict = '真锁死(高度疑似)' + ('' if dur >= 2.5 else '(短时)')
    elif ty_ratio is not None and ty_ratio >= 0.3:
        verdict = '瞬态(车体确实在转:陀螺仪滞后/爬坡)'
    elif dur < 2.5:
        verdict = '瞬态(短时爬坡/减速错位)'
    else:
        verdict = '瞬态(证据不足,需人工看CSV)'
    episode['idx'] += 1
    csv_path = f"{EX}/lock_ep_{episode['idx']:02d}_{episode['scenario']}.csv"
    fmt = lambda v: 'na' if v is None else f'{v:.2f}'
    line = (f"[{time.strftime('%m-%d %H:%M:%S')}] [{episode['scenario']}] "
            f"时长={dur:.1f}s wz={wz:.2f} vx={samples[-1][2]:.2f} "
            f"gz/指令={fmt(gz_ratio)} 真值w/指令={fmt(ty_ratio)} "
            f"轮速/期望={fmt(ws_ratio)} ({reason}) → {verdict}")
    with open(f'{EX}/lock_events.log', 'a') as f:
        f.write(line + f'\n  证据: {csv_path}\n')
    with open(csv_path, 'w') as f:
        f.write('t_wall,wz_cmd,vx_cmd,gz_mean,truth_yaw_rate,truth_speed,wheel_max\n')
        for s in samples:
            f.write(','.join('na' if v is None else f'{v:.3f}' for v in s) + '\n')


def lock_detector():
    epoch_start = None
    while not stop_flag.is_set():
        time.sleep(0.2)
        now = time.time()
        with data_lock:
            # ★ 取"最新指令"而非窗口 max：velocity_smoother 减速时窗口内残留
            #   大值会把已停转的车误判为锁死（N3 瞬态计数的假阳性根源）
            recent = [c for c in cmd_buf if now - c[0] < 0.4]
            imu = list(imu_buf[-400:])
        if now < grace_until[0] or len(imu) < 20:
            continue
        if recent:
            _, vx_now, wz_now = recent[-1]
            wz_now, vx_now = abs(wz_now), abs(vx_now)
        else:
            vx_now = wz_now = 0.0
        if wz_now < EPOCH_MIN_WZ:
            if episode['open']:
                close_episode('cmd_stop')
            epoch_start = None
            continue
        if epoch_start is None:
            epoch_start = now
        if now - epoch_start < SPINUP_SKIP and not episode['open']:
            continue  # 爬坡豁免
        imu_recent = [(t, g) for t, g in imu if imu[-1][0] - 1.0 <= t <= imu[-1][0]]
        # ★ 窗口锚必须是 imu 自己的仿真时间戳：gz server 启动延迟使仿真时间
        #   恒定落后墙钟若干秒，用墙钟 now 过滤会匹配 0 样本 → 判定器失明
        #   （阴性对照实验发现的 bug）。墙钟只用于 cmd_buf（同为墙钟）。
        if len(imu_recent) < 10:
            continue
        gz_mean = sum(abs(g) for _, g in imu_recent) / len(imu_recent)
        if gz_mean < LOCK_RATIO * max(wz_now, 0.05):
            if not episode['open']:
                episode['open'] = True
                episode['scenario'] = active_scenario or 'unknown'
                episode['samples'] = []
            t_imu = imu_recent[-1][0]
            ty, ts = truth_motion(t_imu)
            ws = wheel_max_speed(t_imu)
            episode['samples'].append((now, wz_now, vx_now, gz_mean, ty, ts, ws))
            sc = active_scenario
            if sc:
                lock_by_scenario[sc] = lock_by_scenario.get(sc, 0.0) + 0.2
        elif episode['open']:
            close_episode('recovered')


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
    with open(f'{EX}/lock_events.log', 'a') as f:
        f.write(f"==== 运行开始 mode={mode} {time.strftime('%Y-%m-%d %H:%M:%S')} ====\n")
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
                               ('N3-北屋穿行', -1.5, 2.7)]:
                # N3 目标 (-4.5,2.6) 位于床/衣柜膨胀区内，规划器直接失败
                # （T5 实测北屋可达自由空间 ≈ (-1, 2.7)），故改用该点。
                ok, detail = nav_goal(x, y, name)
                results[name] = ok
                print(f'  [{"✓" if ok else "✗"}] {name}: {detail}  '
                      f'锁死={lock_by_scenario.get(name, 0):.1f}s', flush=True)
            kill_stack(procs)
    finally:
        kill_stack(procs)
        close_episode('shutdown')  # 收尾未闭合的疑似锁死 episode
        with open(f'{EX}/lock_events.log', 'a') as f:
            f.write(f"==== 运行结束 {time.strftime('%Y-%m-%d %H:%M:%S')} ====\n")
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
