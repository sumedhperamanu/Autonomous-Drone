from enum import Enum

from drone_px4_interface.px4_commander import Px4Commander
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class State(Enum):
    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    SEARCH = 'SEARCH'
    APPROACH = 'APPROACH'
    LAND = 'LAND'
    RTL = 'RTL'
    ABORT = 'ABORT'

OFFBOARD_STATES = (State.TAKEOFF, State.SEARCH, State.APPROACH)
LANDING_STATES = (State.LAND, State.RTL, State.ABORT)

class Mission(Node):
    """Mission Skeleton: IDLE, TAKEOFF, SEARCH, APPROACH, LAND, RTL, ABORT"""

    def __init__(self):
        super().__init__('mission')
        self.px4 = Px4Commander(self)
        self.altitude = self.declare_parameter('altitude', 2.0).value
        self.climb_timeout = self.declare_parameter('climb_timeout', 20.0).value
        self.search_seconds = self.declare_parameter('search_seconds', 10.0).value
        self.approach_seconds = self.declare_parameter('approach_seconds', 5.0).value
        self.odometry_timeout = self.declare_parameter('odometry_timeout', 0.5).value
        self.state = State.IDLE
        self.state_start = self.get_clock().now()
        self.warmup_ticks = 0
        self.hold_point = (0.0, 0.0, -self.altitude)
        self.create_service(Trigger, 'mission/start', self.on_start)
        self.create_service(Trigger, 'mission/rtl', self.on_rtl)
        self.create_service(Trigger, 'mission/abort', self.on_abort)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Mission ready in IDLE. Call mission/start to fly.')

    def seconds_in_state(self):
        """Return how long we have been in the current state"""
        return(self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def change_state(self, new_state, reason):
        """Switch state, log why, and run the new state's one-time entry action"""
        self.get_logger().info(f'{self.state.value} -> {new_state.value}: {reason}')
        self.state = new_state
        self.state_start = self.get_clock().now()
        if new_state == State.TAKEOFF:
            self.warmup_ticks = 0
        elif new_state in (State.LAND, State.ABORT):
            self.px4.land()
        elif new_state == State.RTL:
            self.px4.return_home()

    def on_start(self, request, response):
        """Start the mission, only from IDLE and only with PX4 data flowing"""
        if self.state != State.IDLE:
            response.success = False
            response.message = f'Cannot start from {self.state.value}'
        elif not self.px4.has_data():
            response.success = False
            response.message = 'No status or odometry from PX4 yet'
        elif self.px4.is_armed():
            response.success = False
            response.message = "Drone is still armed; land it first"
        else:
            north, east, _ = self.px4.position()
            self.hold_point = (north, east, -self.altitude)
            self.change_state(State.TAKEOFF, 'start requested')
            response.success = True
            response.message = "Taking off"
        return response
        
    def on_rtl(self, request, response):
        """Return home, only while the mission is flying in offboard"""
        response.success = self.state in OFFBOARD_STATES
        if response.success:
            self.change_state(State.RTL, 'return home requested')
        response.message = f'State is now {self.state.value}'
        return response
    
    def on_abort(self, request, response):
        """Stop the mission and land where we are, only while flying in offboard"""
        response.success = self.state in OFFBOARD_STATES
        if response.success:
                self.change_state(State.ABORT, 'abort requested')
        response.message = f'State is now {self.state.value}'
        return response

    def tick(self):
        """Run ten times a second: safety checks first, then the current state's work"""
        if self.state in OFFBOARD_STATES:
            if self.px4.odometry_age() > self.odometry_timeout:
                self.change_state(State.ABORT, 'odometry lost')
                return
            if self.state != State.TAKEOFF and not self.px4.in_offboard():
                self.change_state(State.IDLE, 'PX4 left offboard: pilot or failsafe in control')
                return
            self.px4.send_heartbeat()
            self.px4.send_position(*self.hold_point)
        if self.state == State.TAKEOFF:
            self.warmup_ticks += 1
            if self.warmup_ticks == 10:
                self.px4.start_offboard()
                self.px4.arm()
            if self.px4.altitude() >= self.altitude - 0.1:
                self.change_state(State.SEARCH, 'reached take-off height')
            elif self.seconds_in_state() > self.climb_timeout:
                self.change_state(State.ABORT, 'climb timed out')
        elif self.state == State.SEARCH:
            # Replace this hover with lawnmower or spiral search pattern
            # Later: leave search when P3's detector reports the waypoint

            if self.seconds_in_state() >= self.search_seconds:
                self.change_state(State.APPROACH, 'placeholder: pretend waypoint found')
        elif self.state == State.APPROACH:
            # Fly over the waypoint using P3's waypoint position
            if self.seconds_in_state() >= self.approach_seconds:
                self.change_state(State.LAND, 'placeholder: pretend we are over waypoint')
        elif self.state in LANDING_STATES:
            if not self.px4.is_armed() and self.seconds_in_state() > 2.0:
                self.change_state(State.IDLE, 'landed and disarmed')

def main(args=None):
    """Start the mission node and keep it running until Ctrl+C"""
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()