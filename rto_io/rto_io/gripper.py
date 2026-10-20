"""
Gripper connected to the Robotino I/O interface.

The gripper is opened and closed by two outputs and reports its state with up to three kinds of
inputs: the gripper open sensor, the object sensor between the fingers and the roller lever
switches (contact with the object or the place in front of the robot). All channels and their
polarity are parameters; the defaults are the wiring of the Robotino View program
gamepad_control_gripper_safety.rvwx.

The open and close services check the interlocks of that program: the gripper opens only on
contact of a roller lever switch and when it is not open yet, it closes only when it is open, on
contact and with an object between the fingers. A check is skipped if its sensor is not connected.
"""

from typing import Optional

from control_msgs.msg import DynamicInterfaceGroupValues
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rto_io.gpio import Channel, make_command
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def open_blocked(is_open: Optional[bool], contact: Optional[bool],
                 require_contact: bool) -> Optional[str]:
    """Return why the gripper may not open, None if it may. None: the sensor is not connected."""
    if is_open:
        return 'the gripper is already open'
    if require_contact and contact is False:
        return 'no contact of the roller lever switches'
    return None


def close_blocked(is_open: Optional[bool], contact: Optional[bool],
                  object_detected: Optional[bool], require_contact: bool,
                  require_object: bool) -> Optional[str]:
    """Return why the gripper may not close, None if it may. None: the sensor is not connected."""
    if is_open is False:
        return 'the gripper is not open'
    if require_contact and contact is False:
        return 'no contact of the roller lever switches'
    if require_object and object_detected is False:
        return 'no object between the fingers'
    return None


class Gripper(Node):

    def __init__(self):
        super().__init__('gripper')
        outputs_active_low = self.declare_parameter('outputs_active_low', False).value
        self.open_output = Channel(
            self.declare_parameter('open_output', 'DQ1').value, outputs_active_low, output=True)
        self.close_output = Channel(
            self.declare_parameter('close_output', 'DQ2').value, outputs_active_low, output=True)
        if not self.open_output.connected or not self.close_output.connected:
            raise ValueError('open_output and close_output must be set')
        self.pulse_duration = self.declare_parameter('pulse_duration', 0.1).value

        self.open_sensor = Channel(
            self.declare_parameter('open_sensor', 'DI1').value,
            self.declare_parameter('open_sensor_active_low', False).value)
        self.object_sensor = Channel(
            self.declare_parameter('object_sensor', 'DI2').value,
            self.declare_parameter('object_sensor_active_low', True).value)
        contact_names = self.declare_parameter(
            'contact_sensors', ['DI3', 'DI4'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)).value
        contact_active_low = self.declare_parameter('contact_sensors_active_low', True).value
        self.contact_sensors = [
            Channel(name, contact_active_low) for name in contact_names if name.strip()]

        self.require_contact = self.declare_parameter('require_contact', True).value
        self.require_object_to_close = self.declare_parameter(
            'require_object_to_close', True).value

        self.get_logger().info(
            f'Outputs: open {self.open_output}, close {self.close_output}; '
            f'sensors: open {self.open_sensor}, object {self.object_sensor}, '
            f'contact {self.contact_sensors or "not connected"}')

        self.states = None
        self.published = {}
        self.pulse_timer = None

        self.gpio_pub = self.create_publisher(DynamicInterfaceGroupValues, 'gpio_commands', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pubs = {
            name: self.create_publisher(Bool, 'gripper/' + name, latched)
            for name in ('is_open', 'object_detected', 'contact')
        }
        self.create_subscription(
            DynamicInterfaceGroupValues, 'gpio_states', self.gpio_states_callback, 10)
        self.create_service(Trigger, 'gripper/open', self.open_callback)
        self.create_service(Trigger, 'gripper/close', self.close_callback)

    def is_open(self) -> Optional[bool]:
        return self.open_sensor.read(self.states) if self.states else None

    def object_detected(self) -> Optional[bool]:
        return self.object_sensor.read(self.states) if self.states else None

    def contact(self) -> Optional[bool]:
        if not self.states:
            return None
        values = [sensor.read(self.states) for sensor in self.contact_sensors]
        values = [value for value in values if value is not None]
        return any(values) if values else None

    def gpio_states_callback(self, msg: DynamicInterfaceGroupValues):
        self.states = msg
        for name, value in (('is_open', self.is_open()),
                            ('object_detected', self.object_detected()),
                            ('contact', self.contact())):
            if value is not None and self.published.get(name) != value:
                self.published[name] = value
                self.state_pubs[name].publish(Bool(data=value))

    def unknown_sensors(self) -> bool:
        """Sensors are connected, but robotino_node has not reported their states yet."""
        sensors = [self.open_sensor, self.object_sensor] + self.contact_sensors
        return self.states is None and any(sensor.connected for sensor in sensors)

    def set_outputs(self, open_state: bool, close_state: bool):
        self.gpio_pub.publish(make_command(
            [(self.open_output, open_state), (self.close_output, close_state)]))

    def command(self, open_gripper: bool, blocked: Optional[str],
                response: Trigger.Response) -> Trigger.Response:
        action = 'open' if open_gripper else 'close'
        if blocked is None and self.unknown_sensors():
            blocked = 'no states of the I/O interface from robotino_node'
        if blocked is not None:
            response.success = False
            response.message = f'Gripper does not {action}: {blocked}'
            self.get_logger().warning(response.message)
            return response

        if self.pulse_timer is not None:
            self.pulse_timer.cancel()
            self.destroy_timer(self.pulse_timer)
            self.pulse_timer = None
        self.set_outputs(open_gripper, not open_gripper)
        if self.pulse_duration > 0.0:
            self.pulse_timer = self.create_timer(self.pulse_duration, self.end_pulse)
        response.success = True
        response.message = f'Gripper {action}s'
        self.get_logger().info(response.message)
        return response

    def end_pulse(self):
        self.pulse_timer.cancel()
        self.destroy_timer(self.pulse_timer)
        self.pulse_timer = None
        self.set_outputs(False, False)

    def open_callback(self, request: Trigger.Request, response: Trigger.Response):
        blocked = open_blocked(self.is_open(), self.contact(), self.require_contact)
        return self.command(True, blocked, response)

    def close_callback(self, request: Trigger.Request, response: Trigger.Response):
        blocked = close_blocked(self.is_open(), self.contact(), self.object_detected(),
                                self.require_contact, self.require_object_to_close)
        return self.command(False, blocked, response)


def main():
    # Without the signal handlers of rclpy the context stays valid on Ctrl+C,
    # so the outputs can be switched off
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Gripper()
        # spin_once with a timeout: Python handles Ctrl+C only between calls to rclpy
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.set_outputs(False, False)
                node.gpio_pub.wait_for_all_acked(Duration(seconds=1.0))
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
