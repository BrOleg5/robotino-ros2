"""
Key bindings of keyboard_teleop and keyboard_teleop_tty.

Every command is bound to a list of keys by the parameter keys.<command>. A key is named in
lowercase: a letter, a digit or another character ("w", "1", ","), "space" or a name of Qt
("up", "f1", "shift", "esc"), with optional "ctrl+" and "alt+" prefixes ("ctrl+r").
"""

from rclpy.node import Node

# Motion commands: (x, y, yaw) direction
MOTION = {
    'forward': (1.0, 0.0, 0.0),
    'backward': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'rotate_left': (0.0, 0.0, 1.0),
    'rotate_right': (0.0, 0.0, -1.0),
}
DEFAULT_KEYS = {
    'forward': ['w'],
    'backward': ['s'],
    'left': ['a'],
    'right': ['d'],
    'rotate_left': ['q'],
    'rotate_right': ['e'],
    'turbo': ['shift'],
    'stop': ['space'],
    'reset_estop': ['ctrl+r'],
    'open_gripper': ['o'],
    'close_gripper': ['c'],
}
DESCRIPTIONS = {
    'forward': 'forward',
    'backward': 'backward',
    'left': 'left',
    'right': 'right',
    'rotate_left': 'rotate left',
    'rotate_right': 'rotate right',
    'turbo': 'turbo',
    'stop': 'stop',
    'reset_estop': 'reset the emergency stop',
    'open_gripper': 'open the gripper',
    'close_gripper': 'close the gripper',
}
MODIFIERS = ('ctrl', 'alt')
# Other names of keys -> names of Qt (QKeySequence.toString)
ALIASES = {
    ' ': 'space',
    'escape': 'esc',
    'enter': 'return',
    'delete': 'del',
    'insert': 'ins',
    'control': 'ctrl',
    'pageup': 'pgup',
    'pagedown': 'pgdown',
}


def normalize(key: str) -> str:
    key = key if key == ' ' else key.strip().lower()
    return ALIASES.get(key, key)


def parse_key(text: str) -> tuple:
    """Split a binding into its modifiers and its key: 'ctrl+r' -> (frozenset({'ctrl'}), 'r')."""
    text = text.strip().lower()
    modifiers = set()
    while True:
        prefix, sep, rest = text.partition('+')
        if not sep or not rest or normalize(prefix) not in MODIFIERS:
            break
        modifiers.add(normalize(prefix))
        text = rest
    return frozenset(modifiers), normalize(text)


def declare_key_bindings(node: Node) -> dict:
    """Declare the parameters keys.<command> and return the bindings by command."""
    return {
        command: [k for k in node.declare_parameter(f'keys.{command}', keys).value if k]
        for command, keys in DEFAULT_KEYS.items()
    }


def key_label(key: str) -> str:
    modifiers, name = parse_key(key)
    name = name.upper() if len(name) == 1 else name.capitalize()
    return '+'.join([m.capitalize() for m in sorted(modifiers)] + [name])


def keys_label(bindings: dict, command: str) -> str:
    keys = bindings.get(command)
    return ' or '.join(key_label(k) for k in keys) if keys else 'not bound'


def help_lines(bindings: dict, commands=None) -> list:
    """Return lines 'keys: description' of the commands."""
    return [f'{keys_label(bindings, c)}: {DESCRIPTIONS[c]}'
            for c in (commands or DEFAULT_KEYS) if bindings.get(c)]
