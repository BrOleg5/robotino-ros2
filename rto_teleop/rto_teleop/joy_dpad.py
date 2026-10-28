"""
Driving with buttons of the gamepad (the D-pad): forward, backward, left and right.

Republishes joy with the axes of driving forward and left set by the pressed buttons, for
teleop_twist_joy: a button gives the deflection of the axis that teleop_twist_joy scales to the
speed of the buttons (own speed and turbo speed, may exceed those of the sticks), while the enable
button is held. Two buttons at once drive diagonally at the same speed. Without the buttons pressed
the axes of the sticks pass unchanged.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyDpad(Node):

    def __init__(self):
        super().__init__('joy_dpad')
        self.forward_axis = self.declare_parameter('forward_axis', 1).value
        self.left_axis = self.declare_parameter('left_axis', 0).value
        # Axis value of driving forward or left: -1.0 for an inverted axis
        self.forward_sign = self.declare_parameter('forward_sign', 1.0).value
        self.left_sign = self.declare_parameter('left_sign', 1.0).value
        # Buttons of driving forward, backward, left, right
        self.buttons = self.declare_parameter('buttons', [11, 12, 13, 14]).value
        # Axis deflection of the buttons: their speed / speed of the sticks, without and with turbo
        self.scale = self.declare_parameter('scale', 1.0).value
        self.scale_turbo = self.declare_parameter('scale_turbo', 1.0).value
        self.turbo_button = self.declare_parameter('turbo_button', 10).value

        self.publisher = self.create_publisher(Joy, 'joy_dpad', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def pressed(self, msg: Joy, button: int) -> bool:
        return 0 <= button < len(msg.buttons) and bool(msg.buttons[button])

    def joy_callback(self, msg: Joy):
        forward_button, backward_button, left_button, right_button = self.buttons
        forward = self.pressed(msg, forward_button) - self.pressed(msg, backward_button)
        left = self.pressed(msg, left_button) - self.pressed(msg, right_button)
        if forward or left:
            # Same speed in all directions, diagonals included
            turbo = self.pressed(msg, self.turbo_button)
            scale = (self.scale_turbo if turbo else self.scale) / math.hypot(forward, left)
            axes = list(msg.axes)
            axes += [0.0] * (max(self.forward_axis, self.left_axis) + 1 - len(axes))
            axes[self.forward_axis] = self.forward_sign * forward * scale
            axes[self.left_axis] = self.left_sign * left * scale
            msg.axes = axes
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = JoyDpad()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
