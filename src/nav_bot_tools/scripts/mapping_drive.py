"""T6 后续：重建地图（slip 修复后里程计质量提升，消除 SLAM 畸变地图）。

方法：T6 已证实旧地图非刚性扭曲（各区域局部旋转角 −0.3°~+19.4°），根因是
建图时用的漂移里程计。现在 slip=0.05 已根治旋转失速、EKF 陀螺仪修正里程计，
重建的地图应与世界坐标基本对齐。

驱动方式：真值引导跟踪 T5 已验证成功的巡逻轨迹（world 系 via-points），
激光安全制动（前方 ±30° 扇区 <0.28m 停），卡点超时跳过。
完成后 map_saver_cli 保存到 src/nav_bot_navigation/maps/small_house（旧图备份
为 small_house_old），并同步拷贝到 install share。

用法（先 source jazzy 与 install）：
  python3 mapping_drive.py
"""
import math
import os
import shutil
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import scenario_test as st

EX = st.EX
SRC_MAPS = f'{st.WS}/src/nav_bot_navigation/maps'
INST_MAPS = (f'{st.WS}/install/nav_bot_navigation/share/'
             'nav_bot_navigation/maps')
TRAJ = f'{EX}/patrol_traj_loop1_pass0.txt'
VIA_SPACING = 0.45      # via-point 抽稀间距 m
ARRIVE_R = 0.30         # via-point 到达半径
FRONT_STOP = 0.28       # 前方扇区安全制动距离
V_MAX = 0.22
WZ_MAX = 0.8
DRIVE_TIMEOUT = 480
STUCK_SKIP = 25         # 单 via-point 超时跳过

frontal_min = 99.0
scan_lock = threading.Lock()


class DriveNode(Node):
    def __init__(self):
        super().__init__('mapping_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, msg):
        global frontal_min
        lo, hi = math.radians(-30), math.radians(30)
        vals = []
        for i in range(len(msg.ranges)):
            a = msg.angle_min + i * msg.angle_increment
            r = msg.ranges[i]
            if lo <= a <= hi and msg.range_min < r < msg.range_max:
                vals.append(r)
        with scan_lock:
            frontal_min = min(vals) if vals else 99.0

    def drive(self, vx, wz):
        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        self.pub.publish(m)


def load_via_points():
    pts = []
    for line in open(TRAJ):
        v = line.split()
        if len(v) == 5:
            pts.append((float(v[3]), float(v[4])))  # truth_x, truth_y
    # 抽稀
    via = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - via[-1][0], p[1] - via[-1][1]) >= VIA_SPACING:
            via.append(p)
    # 去重尾（起点原点处密集重复）
    via.append(pts[-1])
    return via


def main():
    global frontal_min
    via = load_via_points()
    total_len = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(via, via[1:]))
    print(f'via-points: {len(via)} 个，总路径 {total_len:.1f}m', flush=True)

    rclpy.init()
    st.monitor_node = st.Monitor()
    threading.Thread(target=st.ros_spin, daemon=True).start()
    threading.Thread(target=st.truth_reader, daemon=True).start()
    threading.Thread(target=st.truth_health_watch, daemon=True).start()
    driver = DriveNode()
    threading.Thread(target=lambda: rclpy.spin(driver), daemon=True).start()

    procs = []
    try:
        if not st.start_sim(procs, 'mapping'):
            print('✗ 仿真未就绪', flush=True)
            sys.exit(1)
        slam = subprocess.Popen(
            ['ros2', 'launch', 'nav_bot_navigation', 'slam.launch.py',
             'use_rviz:=false'],
            stdout=open(f'{EX}/mapping_slam.log', 'w'), stderr=subprocess.STDOUT,
            start_new_session=True)
        procs.append(slam)
        time.sleep(12)  # slam_toolbox 就绪
        st.arm_grace()

        idx = 0
        t_idx = time.time()
        t0 = time.time()
        while idx < len(via) and time.time() - t0 < DRIVE_TIMEOUT:
            try:
                tp = st.latest_truth(timeout=1.0)
            except RuntimeError:
                driver.drive(0.0, 0.0)
                time.sleep(0.3)
                continue
            tx, ty, tyaw = tp[1], tp[2], tp[4]
            gx, gy = via[idx]
            d = math.hypot(gx - tx, gy - ty)
            if d < ARRIVE_R:
                idx += 1
                t_idx = time.time()
                print(f'    via {idx}/{len(via)}  累计里程≈{idx * VIA_SPACING:.0f}m',
                      flush=True)
                continue
            if time.time() - t_idx > STUCK_SKIP:
                print(f'    ⚠ via {idx} 卡点超时，跳过', flush=True)
                idx += 1
                t_idx = time.time()
                continue
            err = st.wrap(math.atan2(gy - ty, gx - tx) - tyaw)
            wz = max(-WZ_MAX, min(WZ_MAX, 1.5 * err))
            with scan_lock:
                fmin = frontal_min
            vx = V_MAX if abs(err) < 0.7 else 0.06
            if fmin < FRONT_STOP:
                vx = 0.0  # 安全制动（旋转仍允许，slip 已根治失速）
            driver.drive(vx, wz)
            time.sleep(0.1)
        driver.drive(0.0, 0.0)
        done = idx >= len(via)
        print(f'\n驱动结束: {"✓ 全程完成" if done else f"⚠ 超时，走到 via {idx}/{len(via)}"}',
              flush=True)
        time.sleep(3)  # 等 slam_toolbox 处理完最后一帧

        # ── 保存地图（旧图备份） ──
        for d_ in (SRC_MAPS, INST_MAPS):
            for ext in ('.pgm', '.yaml'):
                src = f'{d_}/small_house{ext}'
                if os.path.exists(src):
                    shutil.copy(src, f'{d_}/small_house_old{ext}')
        r = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', f'{SRC_MAPS}/small_house', '--fmt', 'trinary'],
            capture_output=True, text=True, timeout=60)
        print(r.stdout.strip()[-200:] or r.stderr.strip()[-200:], flush=True)
        ok_save = os.path.exists(f'{SRC_MAPS}/small_house.yaml')
        # 同步到 install share
        if ok_save:
            for ext in ('.pgm', '.yaml'):
                shutil.copy(f'{SRC_MAPS}/small_house{ext}',
                            f'{INST_MAPS}/small_house{ext}')
            print('✓ 新地图已保存并同步到 install（旧图 → small_house_old）',
                  flush=True)
    finally:
        st.kill_stack(procs)
        st.close_episode('mapping_shutdown')

    st.stop_flag.set()
    sys.exit(0 if done else 1)


if __name__ == '__main__':
    main()
