"""
Status LEDs connected to the Robotino I/O interface.

The LEDs show the state of teleoperation as the Robotino View program
gamepad_control_gripper_safety.rvwx did:
- green: normal operation;
- red: an obstacle limits the speed or stops the robot (collision_monitor is active);
- blue: ready to grip, a roller lever switch is pressed and an object is between the fingers;
- emergency stop: only the red LED is on.

All channels and their polarity are parameters, an empty channel means that the LED is not
connected. The state of the LEDs is published on change and repeated periodically, so the LEDs are
restored after a restart of robotino_node.
"""

from control_msgs.msg import DynamicInterfaceGroupValues
from nav2_msgs.msg import CollisionMonitorState
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rto_io.gpio import Channel, make_command
from std_msgs.msg import Bool


class StatusLeds(Node):

    def __init__(self):
        super().__init__('status_leds')
        active_low = self.declare_parameter('leds_active_low', False).value
        self.red = Channel(self.declare_parameter('red_led', 'DQ4').value, active_low, output=True)
        self.green = Channel(
            self.declare_parameter('green_led', 'DQ5').value, active_low, output=True)
        self.blue = Channel(
            self.declare_parameter('blue_led', 'DQ6').value, active_low, output=True)
        refresh_period = self.declare_parameter('refresh_period', 1.0).value
        self.get_logger().info(f'LEDs: red {self.red}, green {self.green}, blue {self.blue}')

        self.emergency_stop = False
        self.obstacle = False
        self.contact = False
        self.object_detected = False
        self.leds = None

        self.gpio_pub = self.create_publisher(DynamicInterfaceGroupValues, 'gpio_commands', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, 'safety/emergency_stop', self.emergency_stop_callback, latched)
        self.create_subscription(
            CollisionMonitorState, 'collision_monitor_state', self.collision_monitor_callback, 10)
        self.create_subscription(Bool, 'gripper/contact', self.contact_callback, latched)
        self.create_subscription(
            Bool, 'gripper/object_detected', self.object_detected_callback, latched)
        self.update()
        if refresh_period > 0.0:
            self.create_timer(refresh_period, self.publish)

    def publish(self):
        self.gpio_pub.publish(make_command(zip((self.red, self.green, self.blue), self.leds)))

    def set_leds(self, red: bool, green: bool, blue: bool):
        leds = (red, green, blue)
        if leds == self.leds:
            return
        self.leds = leds
        self.publish()

    def update(self):
        if self.emergency_stop:
            self.set_leds(True, False, False)
        else:
            self.set_leds(self.obstacle, True, self.contact and self.object_detected)

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = msg.data
        self.update()

    def collision_monitor_callback(self, msg: CollisionMonitorState):
        self.obstacle = msg.action_type != CollisionMonitorState.DO_NOTHING
        self.update()

    def contact_callback(self, msg: Bool):
        self.contact = msg.data
        self.update()

    def object_detected_callback(self, msg: Bool):
        self.object_detected = msg.data
        self.update()


def main():
    # Without the signal handlers of rclpy the context stays valid on Ctrl+C,
    # so the LEDs can be switched off
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = StatusLeds()
        # spin_once with a timeout: Python handles Ctrl+C only between calls to rclpy
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.set_leds(False, False, False)
                node.gpio_pub.wait_for_all_acked(Duration(seconds=1.0))
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
