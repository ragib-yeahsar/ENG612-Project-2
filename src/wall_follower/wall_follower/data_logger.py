import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def beam(ranges, angle_min, inc, deg, half=3, rmax=3.5):
    """Median range around a bearing (0 = front, CCW positive)."""
    n = len(ranges)
    c = int(round((math.radians(deg) - angle_min) / inc)) % n
    vals = []
    for k in range(-half, half + 1):
        r = ranges[(c + k) % n]
        if not math.isfinite(r) or r < 0.05:
            r = rmax
        vals.append(r)
    vals.sort()
    return vals[len(vals) // 2]


def estimate_start(ranges, angle_min, inc, side, box):
    """Estimate the start pose in the box frame from one lidar scan.

    Box frame: the followed wall is a straight line, the robot's heading is
    close to +x. Left wall -> wall at y = box. Right wall -> wall at y = 0.
    Returns (x, y, yaw_rad, problems_list).
    """
    left = side == 'left'
    b = beam(ranges, angle_min, inc, 90 if left else 270)
    a = beam(ranges, angle_min, inc, 45 if left else 315)
    rear = beam(ranges, angle_min, inc, 180)
    th = math.radians(45)
    alpha = math.atan2(a * math.cos(th) - b, a * math.sin(th))
    d = b * math.cos(alpha)
    yaw = -alpha if left else alpha
    y = (box - d) if left else d
    x = rear * math.cos(yaw)
    problems = []
    if min(a, b) > 1.0:
        problems.append('no wall within 1 m on the %s side' % side)
    if abs(math.degrees(yaw)) > 15:
        problems.append('robot not parallel to the wall (%.0f deg)' % math.degrees(yaw))
    if abs(d - 0.30) > 0.10:
        problems.append('robot is %.2f m from the wall (expected about 0.30)' % d)
    if rear > 1.2:
        problems.append('no wall behind the robot; start_x taken from the parameter')
    return x, y, yaw, problems


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.declare_parameter('label', 'run')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw_deg', 0.0)
        self.declare_parameter('box_size', 2.0)
        self.declare_parameter('side', 'right')       # must match the follower
        self.declare_parameter('target', 0.3)
        self.declare_parameter('zero_odom', False)    # True on the real robot
        self.declare_parameter('auto_start', False)   # True: take start pose from the first scan
        self.declare_parameter('out_dir', os.path.expanduser('~/ENG612-Project-2/docs/logs'))

        self.sx = self.get_parameter('start_x').value
        self.sy = self.get_parameter('start_y').value
        self.syaw = math.radians(self.get_parameter('start_yaw_deg').value)
        self.box = self.get_parameter('box_size').value
        self.side = self.get_parameter('side').value
        self.sign = 1.0 if self.side == 'left' else -1.0
        self.target = self.get_parameter('target').value
        self.zero = self.get_parameter('zero_odom').value
        self.auto = self.get_parameter('auto_start').value
        self.first = None
        self.start_ready = not self.auto
        self.hist = []

        out_dir = self.get_parameter('out_dir').value
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(out_dir, f"{self.get_parameter('label').value}_{stamp}.csv")
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'yaw_deg', 'v_cmd', 'w_cmd',
                         'true_wall_dist', 'side_dist', 'err', 'abs_err_cm'])
        self.get_logger().info(f'logging to {path}')

        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.t0 = None
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        if self.auto:
            self.zero = True
            self.scan_sub = self.create_subscription(
                LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
            self.get_logger().info('auto_start: waiting for the first scan')

    def scan_cb(self, msg):
        if self.start_ready:
            return
        x, y, yaw, problems = estimate_start(
            msg.ranges, msg.angle_min, msg.angle_increment, self.side, self.box)
        blocking = [p for p in problems if 'behind' not in p]
        if blocking:
            self.hist = []
            self.get_logger().warn('Waiting for good placement: ' + '; '.join(blocking),
                                   throttle_duration_sec=2.0)
            return
        if any('behind' in p for p in problems):
            x = self.sx
        self.hist.append((x, y, yaw))
        self.hist = self.hist[-10:]
        if len(self.hist) < 10:
            return
        xs = [h[0] for h in self.hist]
        ys = [h[1] for h in self.hist]
        yws = [h[2] for h in self.hist]
        if max(xs) - min(xs) > 0.02 or max(ys) - min(ys) > 0.01 or max(yws) - min(yws) > 0.03:
            self.get_logger().info('Placement good, waiting for the robot to hold still...',
                                   throttle_duration_sec=2.0)
            return
        n = len(self.hist)
        self.sx, self.sy, self.syaw = sum(xs) / n, sum(ys) / n, sum(yws) / n
        self.start_ready = True
        self.get_logger().info(
            'PLACEMENT OK, logging started. start pose: x=%.3f y=%.3f yaw=%.1f deg'
            % (self.sx, self.sy, math.degrees(self.syaw)))
        self.destroy_subscription(self.scan_sub)

    def cmd_cb(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

    def side_distance(self, x, y, yaw):
        """Perpendicular distance to the wall hit by a ray cast sideways."""
        phi = yaw + self.sign * math.pi / 2.0
        dx, dy = math.cos(phi), math.sin(phi)
        hits = []
        if dx > 1e-6:
            hits.append(((self.box - x) / dx, self.box - x))
        elif dx < -1e-6:
            hits.append(((0.0 - x) / dx, x))
        if dy > 1e-6:
            hits.append(((self.box - y) / dy, self.box - y))
        elif dy < -1e-6:
            hits.append(((0.0 - y) / dy, y))
        return min(hits)[1]

    def odom_cb(self, msg):
        if not self.start_ready:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.zero:
            if self.first is None:
                self.first = (px, py, yaw)
            p0x, p0y, y0 = self.first
            c, s_ = math.cos(y0), math.sin(y0)
            lx = c * (px - p0x) + s_ * (py - p0y)
            ly = -s_ * (px - p0x) + c * (py - p0y)
            cs, ss = math.cos(self.syaw), math.sin(self.syaw)
            x = self.sx + cs * lx - ss * ly
            y = self.sy + ss * lx + cs * ly
            yaw = math.atan2(math.sin(yaw - y0 + self.syaw), math.cos(yaw - y0 + self.syaw))
        else:
            x = px + self.sx
            y = py + self.sy
        near = min(x, self.box - x, y, self.box - y)
        side = self.side_distance(x, y, yaw)
        err = side - self.target
        self.w.writerow([f'{now - self.t0:.3f}', f'{x:.4f}', f'{y:.4f}',
                         f'{math.degrees(yaw):.2f}', f'{self.v_cmd:.3f}',
                         f'{self.w_cmd:.3f}', f'{near:.4f}', f'{side:.4f}',
                         f'{err:.4f}', f'{abs(err) * 100:.2f}'])
        self.f.flush()


def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
