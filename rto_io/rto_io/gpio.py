"""
Channels of the Robotino I/O interface given by parameters.

robotino_node publishes the states of the I/O interface on gpio_states and takes commands on
gpio_commands as control_msgs/DynamicInterfaceGroupValues. A channel is named by its label on
the I/O interface connector: DI1..DI8 (digital inputs), DQ1..DQ8 (digital outputs),
R1, R2 (relays).
An empty name means that the signal is not connected.
"""

from typing import Iterable, Optional, Tuple

from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue

INPUT_GROUPS = {'DI': 'digital_inputs'}
OUTPUT_GROUPS = {'DQ': 'digital_outputs', 'R': 'relays'}


class Channel:
    """A digital channel of the I/O interface, possibly inverted (active low)."""

    def __init__(self, name: str, active_low: bool = False, output: bool = False):
        self.name = name.strip().upper()
        self.active_low = active_low
        self.group = None
        if not self.name:
            return
        groups = OUTPUT_GROUPS if output else INPUT_GROUPS
        for prefix, group in groups.items():
            number = self.name[len(prefix):]
            if self.name.startswith(prefix) and number.isdigit():
                self.group = group
                return
        kind = 'output (DQ1..DQ8, R1, R2)' if output else 'input (DI1..DI8)'
        raise ValueError(f"'{name}' is not a digital {kind}")

    @property
    def connected(self) -> bool:
        return self.group is not None

    def read(self, states: DynamicInterfaceGroupValues) -> Optional[bool]:
        """Return the logical state of the channel, None if it is not connected or not reported."""
        if not self.connected:
            return None
        for group, values in zip(states.interface_groups, states.interface_values):
            if group != self.group:
                continue
            for name, value in zip(values.interface_names, values.values):
                if name == self.name:
                    return (value != 0.0) != self.active_low
        return None

    def __repr__(self):
        if not self.connected:
            return 'not connected'
        return self.name + (' (active low)' if self.active_low else '')


def make_command(outputs: Iterable[Tuple[Channel, bool]]) -> DynamicInterfaceGroupValues:
    """Build a command setting the logical states of the outputs, skipping not connected ones."""
    msg = DynamicInterfaceGroupValues()
    for channel, state in outputs:
        if not channel.connected:
            continue
        if channel.group not in msg.interface_groups:
            msg.interface_groups.append(channel.group)
            msg.interface_values.append(InterfaceValue())
        values = msg.interface_values[msg.interface_groups.index(channel.group)]
        values.interface_names.append(channel.name)
        values.values.append(1.0 if state != channel.active_low else 0.0)
    return msg
