"""
Emergency stop of teleoperation by the bumper, with a latch.

Passes velocity commands of the teleop from cmd_vel_in to cmd_vel_out (input of nav2
collision_monitor) until the bumper is pressed. Then the robot is stopped and commands are dropped
until the emergency stop is reset with the reset service or by holding the reset buttons
of the gamepad.

The emergency stop is latched on a rising edge of the bumper, so after a reset the robot can move
away from the obstacle while the bumper is still pressed.

While a roller lever switch of the gripper is pressed (contact topic of the gripper node of
rto_io), the robot may only move backward to get away from the obstacle: forward and sideways
motion and rotation are dropped from the commands. While the operator tries such motion,
motion_blocked is true (status_leds shows it in yellow).
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def is_zero(twist: Twist) -> bool:
    return (twist.linear.x == 0.0 and twist.linear.y == 0.0 and twist.linear.z == 0.0
            and twist.angular.x == 0.0 and twist.angular.y == 0.0 and twist.angular.z == 0.0)


def backward_only(twist: Twist) -> Twist:
    """Keep only backward motion of the command."""
    result = Twist()
    result.linear.x = min(twist.linear.x, 0.0)
    return result


class TeleopSafety(Node):

    def __init__(self):
        super().__init__('teleop_safety')
        self.reset_buttons = list(self.declare_parameter('reset_buttons', [4, 6]).value)
        self.reset_hold_time = Duration(
            seconds=self.declare_parameter('reset_hold_time', 1.0).value)
        self.backward_only_on_contact = self.declare_parameter(
            'backward_only_on_contact', True).value
        # motion_blocked is reset if no blocked command comes within this time [s],
        # e.g. when the teleop stops publishing
        self.motion_blocked_timeout = Duration(
            seconds=self.declare_parameter('motion_blocked_timeout', 0.5).value)

        self.latched = False
        # After a reset commands pass only after a zero command,
        # so that the robot does not jerk if a stick is deflected
        self.wait_zero_cmd = False
        self.bumper_pressed = False
        self.reset_press_time = None
        self.reset_done = False
        self.contact = False
        self.motion_blocked = False
        self.motion_blocked_time = None

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.estop_pub = self.create_publisher(
            Bool, 'emergency_stop',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.estop_pub.publish(Bool(data=False))
        self.motion_blocked_pub = self.create_publisher(
            Bool, 'motion_blocked',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.motion_blocked_pub.publish(Bool(data=False))

        self.create_subscription(Twist, 'cmd_vel_in', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, 'bumper', self.bumper_callback, 10)
        self.create_subscription(
            Bool, 'contact', self.contact_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_service(Trigger, 'reset', self.reset_callback)
        self.create_timer(0.1, self.motion_blocked_timer_callback)

    def latch(self):
        if not self.latched:
            self.latched = True
            self.set_motion_blocked(False)
            self.cmd_vel_pub.publish(Twist())
            self.estop_pub.publish(Bool(data=True))
        self.get_logger().error(
            'Emergency stop: bumper is pressed. Hold the reset buttons of the gamepad '
            'or press Ctrl+R in keyboard_teleop to continue')

    def reset(self) -> str:
        if not self.latched:
            return 'Emergency stop is not active'
        self.latched = False
        self.wait_zero_cmd = True
        self.estop_pub.publish(Bool(data=False))
        message = 'Emergency stop reset'
        if self.bumper_pressed:
            message += ', bumper is still pressed: move away from the obstacle'
        self.get_logger().warning(message)
        return message

    def cmd_vel_callback(self, msg: Twist):
        if self.latched:
            return
        if self.wait_zero_cmd:
            if not is_zero(msg):
                return
            self.wait_zero_cmd = False
        if self.backward_only_on_contact and self.contact:
            allowed = backward_only(msg)
            blocked = allowed != msg
            if blocked:
                self.motion_blocked_time = self.get_clock().now()
            self.set_motion_blocked(blocked)
            msg = allowed
        self.cmd_vel_pub.publish(msg)

    def set_motion_blocked(self, blocked: bool):
        if blocked != self.motion_blocked:
            self.motion_blocked = blocked
            self.motion_blocked_pub.publish(Bool(data=blocked))

    def motion_blocked_timer_callback(self):
        if (self.motion_blocked and self.get_clock().now() - self.motion_blocked_time
                >= self.motion_blocked_timeout):
            self.set_motion_blocked(False)

    def bumper_callback(self, msg: Bool):
        if msg.data and not self.bumper_pressed:
            self.latch()
        self.bumper_pressed = msg.data

    def contact_callback(self, msg: Bool):
        if msg.data and not self.contact and self.backward_only_on_contact:
            self.get_logger().warning(
                'Roller lever switch is pressed: only backward motion is allowed')
        self.contact = msg.data
        if not self.contact:
            self.set_motion_blocked(False)

    def joy_callback(self, msg: Joy):
        pressed = bool(self.reset_buttons) and all(
            0 <= b < len(msg.buttons) and msg.buttons[b] for b in self.reset_buttons)
        if not pressed:
            self.reset_press_time = None
            self.reset_done = False
            return
        now = self.get_clock().now()
        if self.reset_press_time is None:
            self.reset_press_time = now
        if not self.reset_done and now - self.reset_press_time >= self.reset_hold_time:
            self.reset_done = True
            self.reset()

    def reset_callback(self, request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = self.reset()
        return response


def main():
    rclpy.init()
    node = TeleopSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
