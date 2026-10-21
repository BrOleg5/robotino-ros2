"""
Keyboard teleoperation of the omnidirectional base of Robotino in a window.

Unlike a terminal, a window gets key press and release events, so held keys are combined:
W+Q drives an arc, W+D drives diagonally. The robot moves only while keys are held and stops
when the window loses focus. Ctrl+R resets the emergency stop of teleop_safety, O / C open / close
the gripper (gripper node of rto_io). Keys are bound by the parameters keys.<command>
(see key_bindings).
"""

import functools
import math
import signal
import sys

from geometry_msgs.msg import Twist
from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .key_bindings import declare_key_bindings, help_lines, keys_label, MOTION, normalize, \
    parse_key

Qt = QtCore.Qt
# Qt 6 has scoped enums only, Qt 5 has unscoped ones
Modifier = getattr(Qt, 'KeyboardModifier', Qt)
FocusPolicy = getattr(Qt, 'FocusPolicy', Qt)
Alignment = getattr(Qt, 'AlignmentFlag', Qt)
EventType = getattr(QtCore.QEvent, 'Type', QtCore.QEvent)


# Keys by X11 scan code in the US layout: do not depend on the keyboard layout, e.g. Russian
KEYS_BY_SCAN_CODE = {
    **{10 + i: k for i, k in enumerate('1234567890-=')},
    **{24 + i: k for i, k in enumerate('qwertyuiop[]')},
    **{38 + i: k for i, k in enumerate("asdfghjkl;'`")},
    51: '\\',
    **{52 + i: k for i, k in enumerate('zxcvbnm,./')},
    65: 'space',
}


def help_text(bindings: dict) -> str:
    return '\n'.join([
        'Hold keys to move, combine them for arcs and diagonals:',
        '    '.join(help_lines(bindings, MOTION)),
        '    '.join(help_lines(bindings, ['turbo', 'stop', 'reset_estop'])),
        '    '.join(help_lines(bindings, ['open_gripper', 'close_gripper'])),
        'The robot stops when this window loses focus',
    ])


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.linear_speed = self.declare_parameter('linear_speed', 0.2).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.6).value
        self.turbo_scale = self.declare_parameter('turbo_scale', 2.0).value
        self.rate = self.declare_parameter('rate', 20.0).value
        self.bindings = declare_key_bindings(self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/robotino/cmd_vel_teleop', 10)
        self.reset_client = self.create_client(Trigger, '/robotino/safety/reset')
        self.estop = None
        self.create_subscription(
            Bool, '/robotino/safety/emergency_stop', self.estop_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.message = ''

        self.gripper_clients = {
            'open': self.create_client(Trigger, '/robotino/gripper/open'),
            'close': self.create_client(Trigger, '/robotino/gripper/close'),
        }
        self.gripper_future = None
        self.gripper_state = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in ('is_open', 'object_detected', 'contact'):
            self.create_subscription(
                Bool, f'/robotino/gripper/{name}',
                functools.partial(self.gripper_state_callback, name), latched)

    def estop_callback(self, msg: Bool):
        self.estop = msg.data

    def twist(self, commands, turbo: bool) -> Twist:
        x = sum(MOTION[c][0] for c in commands if c in MOTION)
        y = sum(MOTION[c][1] for c in commands if c in MOTION)
        yaw = sum(MOTION[c][2] for c in commands if c in MOTION)
        scale = self.turbo_scale if turbo else 1.0
        # Diagonal motion is not faster than straight motion
        norm = math.hypot(x, y)
        twist = Twist()
        if norm > 0.0:
            twist.linear.x = x / norm * self.linear_speed * scale
            twist.linear.y = y / norm * self.linear_speed * scale
        twist.angular.z = yaw * self.angular_speed * scale
        return twist

    def request_reset(self):
        if not self.reset_client.service_is_ready():
            self.message = 'teleop_safety is not running: nothing to reset'
            return
        self.reset_client.call_async(Trigger.Request()).add_done_callback(self.reset_done)

    def gripper_state_callback(self, name: str, msg: Bool):
        self.gripper_state[name] = msg.data

    def request_gripper(self, action: str):
        client = self.gripper_clients[action]
        if not client.service_is_ready():
            self.message = 'The gripper node is not running'
            return
        if self.gripper_future is not None and not self.gripper_future.done():
            return
        self.gripper_future = client.call_async(Trigger.Request())
        self.gripper_future.add_done_callback(self.gripper_done)

    def gripper_done(self, future):
        result = future.result()
        self.message = result.message if result else 'Gripper command failed'

    def gripper_status(self) -> str:
        def state(name, yes, no):
            value = self.gripper_state.get(name)
            return '?' if value is None else (yes if value else no)

        return (f'Gripper: {state("is_open", "open", "closed")}   '
                f'object: {state("object_detected", "yes", "no")}   '
                f'contact: {state("contact", "yes", "no")}')

    def reset_done(self, future):
        result = future.result()
        self.message = result.message if result else 'Reset failed'
        # teleop_safety passes commands after a reset only after a zero command
        self.cmd_vel_pub.publish(Twist())


class TeleopWindow(QtWidgets.QWidget):

    def __init__(self, node: KeyboardTeleopNode):
        super().__init__()
        self.node = node
        # Held keys: key name -> motion or turbo command
        self.held = {}
        self.moving = False
        self.use_scan_codes = QtGui.QGuiApplication.platformName() == 'xcb'
        self.commands = {}
        for command, keys in node.bindings.items():
            for key in list(keys):
                binding = parse_key(key)
                if binding in self.commands:
                    node.get_logger().warning(
                        f'Key "{key}" of {command} is already bound to '
                        f'{self.commands[binding]}: ignored')
                    keys.remove(key)
                else:
                    self.commands[binding] = command
        # Modifiers bound to turbo (e.g. ctrl) are ignored when the other keys are matched
        self.turbo_modifiers = {key for mods, key in self.commands
                                if not mods and self.commands[(mods, key)] == 'turbo'}

        self.setWindowTitle('Robotino keyboard teleop')
        self.setFocusPolicy(FocusPolicy.StrongFocus)
        layout = QtWidgets.QVBoxLayout(self)
        self.estop_label = QtWidgets.QLabel()
        self.estop_label.setAlignment(Alignment.AlignCenter)
        self.estop_label.setMargin(8)
        self.cmd_label = QtWidgets.QLabel()
        self.cmd_label.setFont(QtGui.QFontDatabase.systemFont(
            getattr(QtGui.QFontDatabase, 'SystemFont', QtGui.QFontDatabase).FixedFont))
        self.gripper_label = QtWidgets.QLabel()
        self.message_label = QtWidgets.QLabel()
        layout.addWidget(self.estop_label)
        layout.addWidget(self.cmd_label)
        layout.addWidget(self.gripper_label)
        layout.addWidget(self.message_label)
        layout.addWidget(QtWidgets.QLabel(help_text(node.bindings)))

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_state)
        self.timer.start(int(1000 / node.rate))

    def key_name(self, event) -> str:
        if self.use_scan_codes and event.nativeScanCode() in KEYS_BY_SCAN_CODE:
            return KEYS_BY_SCAN_CODE[event.nativeScanCode()]
        return normalize(QtGui.QKeySequence(event.key()).toString())

    def command(self, event):
        key = self.key_name(event)
        modifiers = set()
        if event.modifiers() & Modifier.ControlModifier:
            modifiers.add('ctrl')
        if event.modifiers() & Modifier.AltModifier:
            modifiers.add('alt')
        # A pressed modifier key is reported among the modifiers too
        modifiers.discard(key)
        command = self.commands.get((frozenset(modifiers), key))
        if command is None:
            command = self.commands.get((frozenset(modifiers - self.turbo_modifiers), key))
        return key, command

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key, command = self.command(event)
        if command == 'reset_estop':
            self.stop()
            self.node.request_reset()
        elif command == 'stop':
            self.stop()
        elif command == 'open_gripper':
            self.node.request_gripper('open')
        elif command == 'close_gripper':
            self.node.request_gripper('close')
        elif command in MOTION or command == 'turbo':
            self.held[key] = command

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        self.held.pop(self.key_name(event), None)

    def changeEvent(self, event):
        # Released keys are not reported to an inactive window: stop the robot
        if event.type() == EventType.ActivationChange and not self.isActiveWindow():
            self.stop()
        super().changeEvent(event)

    def stop(self):
        self.held.clear()
        self.node.cmd_vel_pub.publish(Twist())
        self.moving = False

    def update_state(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        commands = set(self.held.values())
        twist = self.node.twist(commands, 'turbo' in commands)
        if commands & MOTION.keys():
            self.node.cmd_vel_pub.publish(twist)
            self.moving = True
        elif self.moving:
            self.stop()

        self.cmd_label.setText(
            f'x: {twist.linear.x:+.2f} m/s   y: {twist.linear.y:+.2f} m/s   '
            f'yaw: {twist.angular.z:+.2f} rad/s')
        self.gripper_label.setText(self.node.gripper_status())
        self.message_label.setText(self.node.message)
        if self.node.estop:
            self.estop_label.setText(
                'EMERGENCY STOP: press '
                f'{keys_label(self.node.bindings, "reset_estop")} to reset')
            self.estop_label.setStyleSheet('background: #c62828; color: white; font-weight: bold')
        elif self.node.estop is None:
            self.estop_label.setText('teleop_safety is not running: no safety layer')
            self.estop_label.setStyleSheet('background: #f9a825; color: black')
        else:
            self.estop_label.setText('Ready')
            self.estop_label.setStyleSheet('background: #2e7d32; color: white')


def main():
    # Ctrl+C is handled below to stop the robot before rclpy shuts down
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    app = QtWidgets.QApplication(sys.argv)
    node = KeyboardTeleopNode()
    window = TeleopWindow(node)
    window.show()
    # Ctrl+C in the terminal closes the window; the timer lets Python handle the signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        app.exec() if hasattr(app, 'exec') else app.exec_()
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
