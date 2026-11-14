"""Helper that talks to PX4: setpoints, commands, status, and odometry"""

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

PX4_QOS = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    durability = DurabilityPolicy.TRANSIENT_LOCAL,
    history = HistoryPolicy.KEEP_LAST,
    depth=1
)

DEFAULT_TOPICS = {
    'offboard_control_mode': '/fmu/in/offboard_control_mode',
    'trajectory_setpoint': '/fmu/in/trajectory_setpoint',
    'vehicle_command': '/fmu/in/vehicle_command',
    'vehicle_status': '/fmu/out/vehicle_status_v1',
    'vehicle_odometry': '/fmu/out/vehicle_odometry'
}

class Px4Commander:
    """Send setpoints and commands to PX4, and keep its latest status and position"""

    def __init__(self, node):
        self.node = node
        names = {
            key: node.declare_parameter(f'topics.{key}', default).value
            for key, default in DEFAULT_TOPICS.items()
        }
        self.mode_pub = node.create_publisher(
            OffboardControlMode, names['offboard_control_mode'], PX4_QOS)
        self.setpoint_pub = node.create_publisher(
            TrajectorySetpoint, names['trajectory_setpoint'], PX4_QOS)
        self.command_pub = node.create_publisher(
            VehicleCommand, names['vehicle_command'], PX4_QOS)
        node.create_subscription(
            VehicleStatus, names['vehicle_status'], self._on_status, PX4_QOS)
        node.create_subscription(
                    VehicleOdometry, names['vehicle_odometry'], self._on_odometry, PX4_QOS)
        self.status = None
        self.odometry = None
        self.odometry_time = None

    def _on_status(self, msg):
        self.status = msg
    
    def _on_odometry(self, msg):
        self.odometry = msg
        self.odometry_time = self.node.get_clock().now()
    
    def _now_us(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)

    #------What PX4 is doing-------#

    def has_data(self):
        """Return True once both odometry and status data have returned"""
        return self.status is not None and self.odometry is not None
    
    def is_armed(self):
        """Return True if PX4 reports the motors are armed"""
        return self.status is not None and (
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def in_offboard(self):
        """Return True if PX4 is currently in offboard mode"""
        return self.status is not None and (
            self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def position(self):
        """Return (North, East, Down) in meters from the takeoff point"""
        north, east, down = self.odometry.position
        return float(north), float(east), float(down)

    def altitude(self):
        """Return height above the takeoff point in meters (0 if unknown)"""
        if self.odometry is None:
            return 0.0
        return -float(self.odometry.position[2])

    def odometry_age(self):
        """Returns seconds since last odometry message (very large if none yet)"""
        if self.odometry_time is None:
            return 1e9
        return (self.node.get_clock().now() - self.odometry_time).nanoseconds / 1e9

    #--------Sending to PX4--------#

    def send_heartbeat(self):
        """Tell PX4 we control position; must be sent at 2Hz or faster"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.mode_pub.publish(msg)

    def send_position(self, north, east, down, yaw=0.0):
        """Ask PX4 to fly to a point (meters, NED frame), with a heading (radians)"""
        msg = TrajectorySetpoint()
        msg.position = [float(north), float(east), float(down)]
        msg.yaw = float(yaw)
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        """Send one vehicle command to PX4"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.command_pub.publish(msg)

    def arm(self):
        """Arm the motors"""
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def start_offboard(self):
        """Switch PX4 to offboard mode"""
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def land(self):
        """Land where the drone is now"""
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_home(self):
        """Fly back to the takeoff point and land"""
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)