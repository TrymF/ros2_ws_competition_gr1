import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64, Bool
from scoring_interfaces.srv import SetMarkerPosition

ROBOTS = ['tb3_0', 'tb3_1']
BIG_FIRE_ID = 4
MEET_DIST = 1.8        # both robots closer than this to the big fire -> bonus (scoring uses 2 m)
FIRE_TIMEOUT = 300.0   # never wait at the fire longer than this
ANSWER_TIMEOUT = 3.0   # no answer from the scoring node after this -> send the report again


class Leader(Node):
    def __init__(self):
        super().__init__('leader')
        self.last_pose = {}                      # latest marker pose seen by each robot
        self.robot_pos = {}                      # latest odom position of each robot
        self.reported = set()                    # marker ids accepted by the scoring node
        self.busy = False                        # waiting for an answer from the scoring node
        self.busy_since = 0.0
        self.fire = None                         # position of the big fire once accepted
        self.fire_time = 0.0
        self.fire_done = False

        for ns in ROBOTS:
            # default argument ns=ns so each lambda remembers its own robot name
            self.create_subscription(Pose, '/' + ns + '/marker_map_pose',
                                     lambda msg, ns=ns: self.pose_cb(msg, ns), 10)
            self.create_subscription(Int64, '/' + ns + '/marker_id',
                                     lambda msg, ns=ns: self.id_cb(msg, ns), 10)
            self.create_subscription(Odometry, '/' + ns + '/odom',
                                     lambda msg, ns=ns: self.odom_cb(msg, ns), 10)

        self.fire_pub = self.create_publisher(Point, '/big_fire', 10)
        self.done_pub = self.create_publisher(Bool, '/fire_done', 10)
        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        self.create_timer(1.0, self.loop)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pose_cb(self, msg, ns):
        self.last_pose[ns] = msg

    def odom_cb(self, msg, ns):
        self.robot_pos[ns] = msg.pose.pose.position

    def id_cb(self, msg, ns):
        # marker_pose publishes the pose right before the id, so last_pose belongs to this id
        marker_id = msg.data
        if marker_id < 0 or marker_id > 4:
            return                               # false detection, only ids 0-4 exist
        if self.busy and self.now() - self.busy_since > ANSWER_TIMEOUT:
            # The answer got lost (happens sometimes with ROS 2 Foxy services), so send it again
            self.get_logger().warn('No answer from the scoring node, sending again')
            self.busy = False
        if marker_id in self.reported or self.busy or ns not in self.last_pose:
            return
        if not self.client.service_is_ready():
            self.get_logger().warn('Scoring service not available', throttle_duration_sec=5.0)
            return

        req = SetMarkerPosition.Request()
        req.marker_id = marker_id
        req.marker_position = self.last_pose[ns].position
        self.busy = True
        self.busy_since = self.now()
        future = self.client.call_async(req)
        future.add_done_callback(lambda f: self.answer_cb(f, req, ns))

    def answer_cb(self, future, req, ns):
        self.busy = False
        p = req.marker_position
        if future.result().accepted:
            self.reported.add(req.marker_id)
            self.get_logger().info(
                f'{ns} found marker {req.marker_id} at ({p.x:.2f}, {p.y:.2f}) - accepted, '
                f'{len(self.reported)}/5 found')
            if req.marker_id == BIG_FIRE_ID:
                self.fire = Point(x=p.x, y=p.y, z=0.0)
                self.fire_time = self.now()
        else:
            # Probably a bad pose estimate, we just try again with the next detection
            self.get_logger().info(f'Marker {req.marker_id} not accepted, trying again later')

    def loop(self):
        if self.fire is None:
            return
        # Keep publishing so the robots always know where the fire is
        self.fire_pub.publish(self.fire)

        if not self.fire_done:
            dist = {ns: math.hypot(pos.x - self.fire.x, pos.y - self.fire.y)
                    for ns, pos in self.robot_pos.items()}
            close = sum(d < MEET_DIST for d in dist.values())
            if close >= len(ROBOTS):
                self.get_logger().info('Both robots are at the big fire!')
                self.fire_done = True
            elif self.now() - self.fire_time > FIRE_TIMEOUT:
                self.get_logger().info('Waited too long at the big fire, continue exploring')
                self.fire_done = True
        self.done_pub.publish(Bool(data=self.fire_done))


def main(args=None):
    rclpy.init(args=args)
    node = Leader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
