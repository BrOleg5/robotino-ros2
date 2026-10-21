from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
import pytest

from rto_io.gpio import Channel, make_command
from rto_io.gripper import close_blocked, open_blocked


def states(**channels):
    """States message of robotino_node with the given digital inputs, the other ones are 0."""
    names = [f'DI{i}' for i in range(1, 9)]
    return DynamicInterfaceGroupValues(
        interface_groups=['digital_inputs', 'digital_outputs'],
        interface_values=[
            InterfaceValue(interface_names=names,
                           values=[float(channels.get(name, 0)) for name in names]),
            InterfaceValue(interface_names=['DQ1'], values=[0.0]),
        ])


def test_channel_groups():
    assert Channel('DI3').group == 'digital_inputs'
    assert Channel('dq1', output=True).group == 'digital_outputs'
    assert Channel(' R2 ', output=True).group == 'relays'
    assert not Channel('').connected
    assert not Channel('', output=True).connected


@pytest.mark.parametrize('name, output', [
    ('DQ1', False), ('R1', False), ('DI1', True), ('AI1', False), ('X1', True), ('DQ', True),
])
def test_channel_invalid(name, output):
    with pytest.raises(ValueError):
        Channel(name, output=output)


def test_read():
    msg = states(DI2=1)
    assert Channel('DI2').read(msg) is True
    assert Channel('DI2', active_low=True).read(msg) is False
    assert Channel('DI3').read(msg) is False
    assert Channel('DI3', active_low=True).read(msg) is True
    assert Channel('').read(msg) is None
    assert Channel('DI9').read(msg) is None


def test_make_command():
    msg = make_command([
        (Channel('DQ1', output=True), True),
        (Channel('', output=True), True),
        (Channel('R2', output=True), True),
        (Channel('DQ4', active_low=True, output=True), True),
    ])
    assert msg.interface_groups == ['digital_outputs', 'relays']
    assert list(msg.interface_values[0].interface_names) == ['DQ1', 'DQ4']
    assert list(msg.interface_values[0].values) == [1.0, 0.0]
    assert list(msg.interface_values[1].interface_names) == ['R2']
    assert list(msg.interface_values[1].values) == [1.0]


def test_open_interlocks():
    assert open_blocked(is_open=False, contact=True, require_contact=True) is None
    assert open_blocked(is_open=True, contact=True, require_contact=True) is not None
    assert open_blocked(is_open=False, contact=False, require_contact=True) is not None
    assert open_blocked(is_open=False, contact=False, require_contact=False) is None
    # Sensors not connected
    assert open_blocked(is_open=None, contact=None, require_contact=True) is None


def test_close_interlocks():
    def blocked(is_open=True, contact=True, object_detected=True,
                require_contact=True, require_object=True):
        return close_blocked(is_open, contact, object_detected, require_contact, require_object)

    assert blocked() is None
    assert blocked(is_open=False) is not None
    assert blocked(contact=False) is not None
    assert blocked(object_detected=False) is not None
    assert blocked(contact=False, require_contact=False) is None
    assert blocked(object_detected=False, require_object=False) is None
    # Sensors not connected
    assert blocked(is_open=None, contact=None, object_detected=None) is None
