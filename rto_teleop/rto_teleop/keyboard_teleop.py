"""
Keyboard teleoperation of the omnidirectional base of Robotino in a window.

Unlike a terminal, a window gets key press and release events, so held keys are combined:
W+Q drives an arc, W+D drives diagonally. The robot moves only while keys are held and stops
when the window loses focus. Ctrl+R resets the emergency stop of teleop_safety, O / C open / close
the gripper (gripper node of rto_io).
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

Qt = QtCore.Qt
# Qt 6 has scoped enums only, Qt 5 has unscoped ones
Key = getattr(Qt, 'Key', Qt)
Modifier = getattr(Qt, 'KeyboardModifier', Qt)
FocusPolicy = getattr(Qt, 'FocusPolicy', Qt)
Alignment = getattr(Qt, 'AlignmentFlag', Qt)
EventType = getattr(QtCore.QEvent, 'Type', QtCore.QEvent)


def enum_int(value) -> int:
    return value.value if hasattr(value, 'value') else int(value)


# Motion keys: (x, y, yaw) direction
MOTION = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
}
# Keys by Qt key code (depends on the keyboard layout)
KEYS_BY_CODE = {
    enum_int(getattr(Key, f'Key_{k.upper()}')): k for k in list(MOTION) + ['r', 'o', 'c']}
KEYS_BY_CODE[enum_int(Key.Key_Space)] = 'space'
# Keys by X11 scan code (do not depend on the keyboard layout, e.g. Russian)
KEYS_BY_SCAN_CODE = {
    24: 'q', 25: 'w', 26: 'e', 27: 'r', 32: 'o', 38: 'a', 39: 's', 40: 'd', 54: 'c', 65: 'space'}

HELP = (
    'Hold keys to move, combine them for arcs and diagonals:\n'
    'W / S: forward / backward    A / D: left / right    Q / E: rotate left / right\n'
    'Shift: turbo    Space: stop    Ctrl+R: reset the emergency stop\n'
    'O / C: open / close the gripper\n'
    'The robot stops when this window loses focus'
)


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.linear_speed = self.declare_parameter('linear_speed', 0.2).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.6).value
        self.turbo_scale = self.declare_parameter('turbo_scale', 2.0).value
        self.rate = self.declare_parameter('rate', 20.0).value

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

    def twist(self, keys: set, turbo: bool) -> Twist:
        x = sum(MOTION[k][0] for k in keys if k in MOTION)
        y = sum(MOTION[k][1] for k in keys if k in MOTION)
        yaw = sum(MOTION[k][2] for k in keys if k in MOTION)
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
        self.keys = set()
        self.turbo = False
        self.moving = False
        self.use_scan_codes = QtGui.QGuiApplication.platformName() == 'xcb'

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
        layout.addWidget(QtWidgets.QLabel(HELP))

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_state)
        self.timer.start(int(1000 / node.rate))

    def key_name(self, event):
        if self.use_scan_codes:
            return KEYS_BY_SCAN_CODE.get(event.nativeScanCode())
        return KEYS_BY_CODE.get(event.key())

    def keyPressEvent(self, event):
        if event.key() == enum_int(Key.Key_Shift):
            self.turbo = True
        if event.isAutoRepeat():
            return
        key = self.key_name(event)
        if key == 'r' and event.modifiers() & Modifier.ControlModifier:
            self.stop()
            self.node.request_reset()
        elif key == 'space':
            self.stop()
        elif key == 'o':
            self.node.request_gripper('open')
        elif key == 'c' and not event.modifiers() & Modifier.ControlModifier:
            self.node.request_gripper('close')
        elif key in MOTION:
            self.keys.add(key)

    def keyReleaseEvent(self, event):
        if event.key() == enum_int(Key.Key_Shift):
            self.turbo = False
        if event.isAutoRepeat():
            return
        self.keys.discard(self.key_name(event))

    def changeEvent(self, event):
        # Released keys are not reported to an inactive window: stop the robot
        if event.type() == EventType.ActivationChange and not self.isActiveWindow():
            self.stop()
        super().changeEvent(event)

    def stop(self):
        self.keys.clear()
        self.turbo = False
        self.node.cmd_vel_pub.publish(Twist())
        self.moving = False

    def update_state(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        twist = self.node.twist(self.keys, self.turbo)
        if self.keys:
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
            self.estop_label.setText('EMERGENCY STOP: press Ctrl+R to reset')
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
