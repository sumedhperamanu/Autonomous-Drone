from drone_px4_interface.px4_commander import Px4Commander
import rclpy
from rclpy.node import Node

class OffboardTakeoff(Node):
    """Arm, climb to a set height in offboard mode, takeoff, hold, then land"""

    def __init__(self):
        super().__init__('offboard_takeoff')
        self.px4 = Px4Commander(self)
        self.target_altitude = self.declare_parameter('takeoff_altitude', 2.0).value
        self.hold_seconds = self.declare_parameter('hold_seconds', 10.0).value
        self.climb_timeout = self.declare_parameter('climb_timeout', 20.0).value
        self.phase = 'wait_for_px4'
        self.phase_start = self.get_clock().now()
        self.warmup_ticks = 0
        self.create_timer(0.1, self.tick)

    def seconds_in_phase(self):
        """Return how long we have been in the current phase"""
        return (self.get_clock().now() - self.phase_start).nanoseconds / 1e9

    def change_phase(self, phase):
        """Log and switch to a new phase"""
        self.get_logger().info(f'{self.phase} -> {phase}')
        self.phase = phase
        self.phase_start = self.get_clock().now()

    def tick(self):
        """Runs ten times a second; keeps the heartbeat going and step through phases"""
        if self.phase in ('warmup', 'climb', 'hold'):
            self.px4.send_heartbeat()
            self.px4.send_position(0.0, 0.0, -self.target_altitude)

        if self.phase == 'wait_for_px4':
            if self.px4.has_data():
                self.change_phase('warmup')
        elif self.phase == 'warmup':
            self.warmup_ticks += 1
            if self.warmup_ticks >= 10:
                self.px4.start_offboard()
                self.px4.arm()
                self.change_phase('climb')
        elif self.phase == "climb":
            if self.px4.altitude() >= self.target_altitude - 0.1:
                self.change_phase('hold')
            elif self.seconds_in_phase() > self.climb_timeout:
                self.get_logger().warn('Climb took too long; landing.')
                self.px4.land()
                self.change_phase('landing')
        elif self.phase == 'hold':
            if self.seconds_in_phase() >= self.hold_seconds:
                self.px4.land()
                self.change_phase('landing')
        elif self.phase == 'landing':
            if not self.px4.is_armed() and self.seconds_in_phase() > 2.0:
                self.get_logger().info('Landed and disarmed. Done.')
                self.change_phase('done')
            
def main(args=None):
    """Start the node and keep it running until Ctrl + C"""

    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()