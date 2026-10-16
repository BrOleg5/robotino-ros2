"""
Emergency stop of teleoperation by the bumper, with a latch.

Passes velocity commands of the teleop from cmd_vel_in to cmd_vel_out (input of nav2
collision_monitor) until the bumper is pressed. Then the robot is stopped and commands are dropped
until the emergency stop is reset with the reset service or by holding the reset buttons
of the gamepad.

The emergency stop is latched on a rising edge of the bumper, so after a reset the robot can move
away from the obstacle while the bumper is still pressed.
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


class TeleopSafety(Node):

    def __init__(self):
        super().__init__('teleop_safety')
        self.reset_buttons = list(self.declare_parameter('reset_buttons', [4, 6]).value)
        self.reset_hold_time = Duration(
            seconds=self.declare_parameter('reset_hold_time', 1.0).value)

        self.latched = False
        # After a reset commands pass only after a zero command,
        # so that the robot does not jerk if a stick is deflected
        self.wait_zero_cmd = False
        self.bumper_pressed = False
        self.reset_press_time = None
        self.reset_done = False

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.estop_pub = self.create_publisher(
            Bool, 'emergency_stop',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.estop_pub.publish(Bool(data=False))

        self.create_subscription(Twist, 'cmd_vel_in', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, 'bumper', self.bumper_callback, 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_service(Trigger, 'reset', self.reset_callback)

    def latch(self):
        if not self.latched:
            self.latched = True
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
        self.cmd_vel_pub.publish(msg)

    def bumper_callback(self, msg: Bool):
        if msg.data and not self.bumper_pressed:
            self.latch()
        self.bumper_pressed = msg.data

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
