"""
Keyboard teleoperation of the omnidirectional base of Robotino in a terminal.

The robot moves while a key is held: the terminal repeats a held key, and the robot stops when
the repeats stop. Ctrl+R resets the emergency stop of teleop_safety, o / c open / close the
gripper (gripper node of rto_io). Keys are bound by the parameters keys.<command> (see
key_bindings); a terminal reports only characters, so keys like arrows and Alt+ are not supported
and turbo is always Shift.
"""

import select
import signal
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .key_bindings import declare_key_bindings, help_lines, keys_label, MOTION, parse_key

KEY_QUIT = '\x03'  # Ctrl+C


def key_char(key: str):
    """Return the character of a key in a terminal, None if a terminal does not report it."""
    modifiers, name = parse_key(key)
    if name == 'space':
        name = ' '
    if len(name) != 1:
        return None
    if not modifiers:
        return name
    if modifiers == {'ctrl'} and 'a' <= name <= 'z':
        return chr(ord(name) - ord('a') + 1)
    return None


def help_text(bindings: dict) -> str:
    lines = help_lines(bindings, [c for c in bindings if c != 'turbo'])
    return '\n'.join(['', 'Robotino keyboard teleop. Hold a key to move:']
                     + [f'  {line}' for line in lines]
                     + ['  Shift + motion key: turbo', '  Ctrl+C: quit', ''])


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop_tty')
        self.linear_speed = self.declare_parameter('linear_speed', 0.2).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.6).value
        self.turbo_scale = self.declare_parameter('turbo_scale', 2.0).value
        # Stop if no key repeat comes within key_timeout; the first repeat comes after the
        # autorepeat delay of the terminal, so first_key_timeout is longer
        self.key_timeout = self.declare_parameter('key_timeout', 0.15).value
        self.first_key_timeout = self.declare_parameter('first_key_timeout', 0.6).value
        self.rate = self.declare_parameter('rate', 20.0).value
        self.bindings = declare_key_bindings(self)
        # Character -> command
        self.commands = {}
        for command, keys in self.bindings.items():
            if command == 'turbo':
                continue
            for key in list(keys):
                char = key_char(key)
                if char is None or char == KEY_QUIT:
                    self.get_logger().warning(
                        f'Key "{key}" of {command} is not supported in the terminal: ignored')
                    keys.remove(key)
                elif char in self.commands:
                    self.get_logger().warning(
                        f'Key "{key}" of {command} is already bound to '
                        f'{self.commands[char]}: ignored')
                    keys.remove(key)
                else:
                    self.commands[char] = command

        self.cmd_vel_pub = self.create_publisher(Twist, '/robotino/cmd_vel_teleop', 10)
        self.reset_client = self.create_client(Trigger, '/robotino/safety/reset')
        self.create_subscription(
            Bool, '/robotino/safety/emergency_stop', self.estop_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.gripper_clients = {
            action: self.create_client(Trigger, f'/robotino/gripper/{action}')
            for action in ('open', 'close')
        }
        self.gripper_future = None
        self.gripper_key = None
        self.gripper_key_time = 0.0

        self.key = None
        self.first_key_time = 0.0
        self.last_key_time = 0.0
        self.moving = False
        self.last_publish_time = 0.0

    def estop_callback(self, msg: Bool):
        if msg.data:
            print('\r\n*** EMERGENCY STOP: press '
                  f'{keys_label(self.bindings, "reset_estop")} to reset ***\r')
        else:
            print('\r\nEmergency stop is not active\r')

    def publish(self, twist: Twist):
        self.cmd_vel_pub.publish(twist)
        self.last_publish_time = time.monotonic()

    def stop(self):
        self.key = None
        self.publish(Twist())
        self.moving = False

    def request_reset(self):
        if not self.reset_client.service_is_ready():
            print('\r\nteleop_safety is not running: nothing to reset\r')
            return
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(self.reset_done)

    def request_gripper(self, action: str):
        client = self.gripper_clients[action]
        if not client.service_is_ready():
            print('\r\nThe gripper node is not running\r')
            return
        if self.gripper_future is not None and not self.gripper_future.done():
            return
        self.gripper_future = client.call_async(Trigger.Request())
        self.gripper_future.add_done_callback(self.gripper_done)

    def gripper_done(self, future):
        result = future.result()
        print(f'\r\n{result.message if result else "Gripper command failed"}\r')

    def reset_done(self, future):
        result = future.result()
        print(f'\r\n{result.message if result else "Reset failed"}\r')
        # teleop_safety passes commands after a reset only after a zero command
        self.publish(Twist())

    def on_key(self, key: str):
        now = time.monotonic()
        # Shift + a letter key gives an uppercase letter: turbo
        command = self.commands.get(key, self.commands.get(key.lower()))
        if command == 'reset_estop':
            self.request_reset()
        elif command == 'stop':
            self.stop()
        elif command in ('open_gripper', 'close_gripper'):
            # A held key is repeated by the terminal: send one command per press
            repeated = (key == self.gripper_key
                        and now - self.gripper_key_time < self.first_key_timeout)
            self.gripper_key = key
            self.gripper_key_time = now
            if not repeated:
                self.request_gripper(command.split('_')[0])
        elif command in MOTION:
            if key != self.key or now - self.last_key_time > self.first_key_timeout:
                # A new key press: the terminal starts repeating it after the autorepeat delay
                self.first_key_time = now
            self.key = key
            self.last_key_time = now

    def update(self):
        now = time.monotonic()
        if self.key is not None:
            timeout = (self.first_key_timeout if self.last_key_time == self.first_key_time
                       else self.key_timeout)
            if now - self.last_key_time > timeout:
                self.stop()
                return
            command = self.commands.get(self.key, self.commands.get(self.key.lower()))
            x, y, yaw = MOTION[command]
            scale = self.turbo_scale if self.key != self.key.lower() else 1.0
            twist = Twist()
            twist.linear.x = x * self.linear_speed * scale
            twist.linear.y = y * self.linear_speed * scale
            twist.angular.z = yaw * self.angular_speed * scale
            if now - self.last_publish_time >= 1.0 / self.rate:
                self.publish(twist)
                self.moving = True
        elif self.moving:
            self.stop()


def read_key(timeout: float):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def main():
    if not sys.stdin.isatty():
        print('keyboard_teleop_tty needs a terminal: '
              'start it with "ros2 run rto_teleop keyboard_teleop_tty"')
        return 1

    # Ctrl+C raises KeyboardInterrupt instead of shutting rclpy down, so the robot can be stopped
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(help_text(node.bindings))
    try:
        # cbreak: keys are read one by one without echo, Ctrl+C still raises KeyboardInterrupt
        tty.setcbreak(sys.stdin)
        while rclpy.ok():
            key = read_key(0.01)
            if key is not None:
                node.on_key(key)
            node.update()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 run forwards Ctrl+C once more: do not let it interrupt stopping the robot
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
