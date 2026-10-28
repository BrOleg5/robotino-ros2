"""
Gamepad bindings of joy_teleop.yaml: commands bound to buttons and axes by name.

Names are those of the SDL GameController layout published by joy's game_controller_node, which
is the same for all gamepads known to SDL (named as on an Xbox gamepad). Numbers are accepted too.
load_joy_config() converts the file to the parameters of teleop_twist_joy, joy_dpad, gripper_joy
and teleop_safety.
"""

import yaml

BUTTONS = {
    name: index for index, name in enumerate([
        'a', 'b', 'x', 'y', 'back', 'guide', 'start', 'left_stick', 'right_stick', 'lb', 'rb',
        'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right', 'misc1',
        'paddle1', 'paddle2', 'paddle3', 'paddle4', 'touchpad',
    ])
}
AXES = {
    name: index for index, name in enumerate(
        ['left_x', 'left_y', 'right_x', 'right_y', 'lt', 'rt'])
}


def button_index(command: str, name) -> int:
    if isinstance(name, int):
        return name
    key = str(name).strip().lower()
    if key not in BUTTONS:
        raise ValueError(
            f'joy_teleop.yaml: unknown button "{name}" of "{command}", '
            f'known buttons: {", ".join(BUTTONS)}')
    return BUTTONS[key]


def axis_index(command: str, name) -> tuple:
    """Return the axis number and 1.0 or -1.0 for an inverted axis ("-" before the name)."""
    if isinstance(name, int):
        return abs(name), -1.0 if name < 0 else 1.0
    key = str(name).strip().lower()
    sign = 1.0
    if key.startswith('-'):
        key, sign = key[1:].strip(), -1.0
    if key.isdigit():
        return int(key), sign
    if key not in AXES:
        raise ValueError(
            f'joy_teleop.yaml: unknown axis "{name}" of "{command}", '
            f'known axes: {", ".join(AXES)}')
    return AXES[key], sign


def load_joy_config(path: str) -> dict:
    """Read joy_teleop.yaml and return parameters by node name."""
    with open(path) as f:
        config = yaml.safe_load(f)
    bindings = config['bindings']
    speed = {name: float(value) for name, value in config['speed'].items()}

    forward, forward_sign = axis_index('forward', bindings['forward'])
    left, left_sign = axis_index('left', bindings['left'])
    rotate, rotate_sign = axis_index('rotate_left', bindings['rotate_left'])
    reset = bindings.get('reset_estop', [])
    if not isinstance(reset, list):
        reset = [reset]
    drive = bindings.get('drive_buttons', [])
    if drive and (not isinstance(drive, list) or len(drive) != 4):
        raise ValueError(
            'joy_teleop.yaml: "drive_buttons" are 4 buttons: forward, backward, left, right')
    if drive and not (speed['linear'] and speed['linear_turbo']):
        raise ValueError(
            'joy_teleop.yaml: "drive_buttons" need nonzero "linear" and "linear_turbo" speeds')

    return {
        'teleop_twist_joy_node': {
            'axis_linear.x': forward,
            'axis_linear.y': left,
            'axis_angular.yaw': rotate,
            'scale_linear.x': forward_sign * speed['linear'],
            'scale_linear.y': left_sign * speed['linear'],
            'scale_linear_turbo.x': forward_sign * speed['linear_turbo'],
            'scale_linear_turbo.y': left_sign * speed['linear_turbo'],
            'scale_angular.yaw': rotate_sign * speed['angular'],
            'scale_angular_turbo.yaw': rotate_sign * speed['angular_turbo'],
            'enable_button': button_index('enable', bindings['enable']),
            'enable_turbo_button': button_index('turbo', bindings['turbo']),
        },
        # None without drive_buttons: joy_dpad is not started
        'joy_dpad': {
            'forward_axis': forward,
            'left_axis': left,
            'forward_sign': forward_sign,
            'left_sign': left_sign,
            'buttons': [button_index('drive_buttons', b) for b in drive],
            # teleop_twist_joy multiplies the axes by the speed of the sticks
            'scale': speed.get('linear_buttons', speed['linear']) / speed['linear'],
            'scale_turbo': (speed.get('linear_buttons_turbo', speed['linear_turbo'])
                            / speed['linear_turbo']),
            'turbo_button': button_index('turbo', bindings['turbo']),
        } if drive else None,
        'gripper_joy': {
            'open_button': button_index('open_gripper', bindings['open_gripper']),
            'close_button': button_index('close_gripper', bindings['close_gripper']),
        },
        'teleop_safety': {
            # An empty list cannot be passed as a parameter: -1 is a button that is never pressed
            'reset_buttons': [button_index('reset_estop', b) for b in reset] or [-1],
        },
    }
