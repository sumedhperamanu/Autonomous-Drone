import math

from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class OdomReader(Node):
    """Read PX4 vehicle odometry and log position, speed, and heading once per second."""

    def __init__(self):
        super().__init__('odom_reader')
        topic = self.declare_parameter('topic', '/fmu/out/vehicle_odometry').value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(VehicleOdometry, topic, self.on_odometry, qos)
        self.last_log_time = 0.0
        self.get_logger().info(f'Listening for PX4 odometry on {topic}')

    def on_odometry(self, msg):
        """Convert one odometry message into readable numbers."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now

        north, east, down = msg.position
        vx, vy, vz = msg.velocity
        speed = math.sqrt((vx*vx) + (vy*vy) + (vz*vz))
        w, x, y, z = msg.q
        yaw = math.degrees(math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)))

        self.get_logger().info(
            f'N {north:6.2f} m, E {east:6.2f} m, Altitude {-down:5.2f} m '
            f'Speed {speed:4.2f} m/s, Heading {yaw:4.0f} deg'
        )


def main(args=None):
    """Start the node and keep it running until Ctrl+C."""
    rclpy.init(args=args)
    node = OdomReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
