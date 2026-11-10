import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

# --- Tuning (m, m/s, rad/s) ---
WALL_DIST   = 0.45   # target distance to the wall we follow
FRONT_STOP  = 0.50   # wall ahead closer than this -> turn away from the wall
TURN_MARGIN = 0.15   # keep turning until front > FRONT_STOP + TURN_MARGIN
WALL_LOST   = 0.90   # nothing on the wall side closer than this -> the wall has ended
MIN_RANGE   = 0.12   # lidar minimum range, smaller values are noise
SPEED       = 0.25   # forward speed
TURN_SPEED  = 0.6    # turn-in-place speed
KP          = 1.0    # steering gain while following the wall

SPIN_EVERY  = 15.0   # s between each 360 degree look-around (camera only looks forward)
SPIN_SPEED  = 0.8    # rad/s, not too fast so the marker pose is still accurate

LOOP_TIME   = 30.0   # back at a place we were this long ago -> we are driving in a loop
LOOP_DIST   = 0.5    # how close counts as "the same place"

FIRE_STOP   = 1.2    # stop this close to the big fire (scoring needs < 2 m)
FIRE_SEE    = 4.0    # only drive straight to the fire if it is closer than this (> marker range)


class RobotHandler(Node):
    def __init__(self):
        super().__init__('robot_handler')
        self.declare_parameter('namespace', 'tb3_0')
        self.declare_parameter('side', 'right')    # which wall to follow: 'right' or 'left'
        ns = self.get_parameter('namespace').value
        side = self.get_parameter('side').value
        # +1: wall on the right, so "away from the wall" is a left (positive) turn
        # -1: wall on the left, everything is mirrored
        self.dir = 1 if side == 'right' else -1

        self.pub = self.create_publisher(Twist, '/' + ns + '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/' + ns + '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/' + ns + '/odom', self.odom_cb, 10)
        # Sent by the leader node to all robots
        self.create_subscription(Point, '/big_fire', self.fire_cb, 10)
        self.create_subscription(Bool, '/fire_done', self.fire_done_cb, 10)
        self.create_timer(0.1, self.loop)

        self.ranges = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.fire = None          # position of the big fire, None = not found yet
        self.fire_done = False    # both robots have been at the fire
        self.state = 'FIND'
        self.last_spin = self.now()
        self.spun = 0.0           # how much we have turned in the current spin
        self.last_yaw = 0.0
        self.history = []         # (time, x, y) saved every 2 seconds, used to detect loops
        self.get_logger().info(f'Started {ns}, following the {side} wall')

    # ---------- callbacks ----------
    def scan_cb(self, msg):
        self.ranges = msg.ranges

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def fire_cb(self, msg):
        if self.fire is None:
            self.get_logger().info(f'Big fire at ({msg.x:.1f}, {msg.y:.1f}), going there')
        self.fire = msg

    def fire_done_cb(self, msg):
        if msg.data and not self.fire_done:
            self.get_logger().info('Done at the big fire, back to exploring')
        self.fire_done = msg.data

    # ---------- helpers ----------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def closest(self, start_deg, end_deg):
        """Smallest lidar range between two angles (0 = straight ahead, positive = left)."""
        n = len(self.ranges)
        vals = [self.ranges[int((a % 360) * n / 360)] for a in range(start_deg, end_deg)]
        vals = [r for r in vals if math.isfinite(r) and r > MIN_RANGE]
        return min(vals, default=10.0)

    # ---------- behaviours ----------
    def go_to_fire(self, cmd):
        """Returns True if the robot is handling the big fire this step, False -> keep exploring."""
        dx = self.fire.x - self.x
        dy = self.fire.y - self.y
        dist = math.hypot(dx, dy)
        if dist < FIRE_STOP:
            self.state = 'AT_FIRE'          # just wait here, cmd stays zero
            return True
        if dist > FIRE_SEE:
            return False                    # too far away, keep following the wall
        err = math.atan2(dy, dx) - self.yaw
        err = math.atan2(math.sin(err), math.cos(err))
        # Is the straight line to the fire free? Check the lidar around the fire direction.
        deg = int(math.degrees(err))
        if self.closest(deg - 15, deg + 15) < dist - 0.4:
            return False                    # something in the way, keep following the wall
        self.state = 'TO_FIRE'
        if abs(err) > 0.3:
            cmd.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED
        else:
            cmd.linear.x = SPEED
            cmd.angular.z = err
        return True

    def follow_wall(self, cmd):
        front = self.closest(-30, 30)
        if self.dir == 1:
            wall = self.closest(-100, -30)    # right side + front-right
        else:
            wall = self.closest(30, 100)

        if self.state == 'WALL_END' and wall > WALL_LOST:
            # Going round the end of a wall: keep turning towards it until it shows up again
            if front < FRONT_STOP:
                cmd.angular.z = -self.dir * TURN_SPEED
            else:
                cmd.linear.x = SPEED
                cmd.angular.z = -self.dir * SPEED / WALL_DIST
        elif front < FRONT_STOP or (self.state == 'TURN' and front < FRONT_STOP + TURN_MARGIN):
            self.state = 'TURN'                 # wall ahead: turn away from the wall on the spot
            cmd.angular.z = self.dir * TURN_SPEED
        elif wall < WALL_LOST:
            self.state = 'FOLLOW'               # keep WALL_DIST to the wall
            cmd.linear.x = SPEED
            turn = self.dir * KP * (WALL_DIST - wall)
            cmd.angular.z = max(-TURN_SPEED, min(TURN_SPEED, turn))
        elif self.state == 'FIND':
            cmd.linear.x = SPEED                # no wall found yet: drive straight
        else:
            self.state = 'WALL_END'             # wall ended: arc around the end of it
            cmd.linear.x = SPEED
            cmd.angular.z = -self.dir * SPEED / WALL_DIST

    def spin(self, cmd):
        """Turn one full round on the spot so the camera can look for markers everywhere."""
        d = self.yaw - self.last_yaw
        self.spun += abs(math.atan2(math.sin(d), math.cos(d)))
        self.last_yaw = self.yaw
        if self.spun > 2 * math.pi:
            self.state = 'FOLLOW'
            self.last_spin = self.now()
        else:
            cmd.angular.z = SPIN_SPEED

    def check_loop(self):
        """If we come back to a place we visited long ago we are going around in a loop
        (e.g. around a free-standing wall), so start following the other wall instead."""
        t = self.now()
        if self.history and t - self.history[-1][0] < 2.0:
            return
        for (old_t, old_x, old_y) in self.history:
            if t - old_t > LOOP_TIME and math.hypot(self.x - old_x, self.y - old_y) < LOOP_DIST:
                self.dir = -self.dir
                self.state = 'FIND'       # drive straight until we hit a new wall
                self.history = []
                self.get_logger().info('Driving in a loop, switching to the other wall')
                return
        self.history.append((t, self.x, self.y))

    def loop(self):
        if self.ranges is None:
            self.get_logger().warn('Waiting for scan ...', throttle_duration_sec=2.0)
            return

        cmd = Twist()
        if self.fire is not None and not self.fire_done and self.go_to_fire(cmd):
            pass
        elif self.state == 'SPIN':
            self.spin(cmd)
        elif self.state == 'FOLLOW' and self.now() - self.last_spin > SPIN_EVERY:
            self.state = 'SPIN'
            self.spun = 0.0
            self.last_yaw = self.yaw
        else:
            if self.state in ('TO_FIRE', 'AT_FIRE'):
                self.state = 'FOLLOW'
            self.check_loop()
            self.follow_wall(cmd)

        self.get_logger().info(f'{self.state}', throttle_duration_sec=2.0)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())  # stop the robot
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
