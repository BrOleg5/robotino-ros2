"""
Gamepad control of the gripper: open and close it on a press of a button.

Calls the open and close services of the gripper node of rto_io on a rising edge of the buttons.
The gripper refuses a command if its interlocks are not met, the reason is logged.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class GripperJoy(Node):

    def __init__(self):
        super().__init__('gripper_joy')
        self.open_button = self.declare_parameter('open_button', 1).value
        self.close_button = self.declare_parameter('close_button', 0).value

        self.button_clients = {
            self.open_button: self.create_client(Trigger, 'gripper/open'),
            self.close_button: self.create_client(Trigger, 'gripper/close'),
        }
        self.pressed = dict.fromkeys(self.button_clients, False)
        self.future = None

        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg: Joy):
        for button, client in self.button_clients.items():
            pressed = 0 <= button < len(msg.buttons) and bool(msg.buttons[button])
            if pressed and not self.pressed[button]:
                self.call(client)
            self.pressed[button] = pressed

    def call(self, client):
        if self.future is not None and not self.future.done():
            return
        if not client.service_is_ready():
            self.get_logger().warning(f'Service {client.srv_name} is not available')
            return
        self.future = client.call_async(Trigger.Request())
        self.future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        response = future.result()
        if response is not None and not response.success:
            self.get_logger().warning(response.message)


def main():
    rclpy.init()
    node = GripperJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
