#!/usr/bin/env python3
"""
Make the simulated Robotino look like robotino_node on the ROS side.

- /robotino/cmd_vel (Twist) -> omni_wheel_drive_controller cmd_vel (TwistStamped)
- /robotino/sim/irN (LaserScan) -> /robotino/irN (Range)
- /robotino/sim/bumper (Contacts) -> /robotino/bumper (Bool)
"""

import math

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool


class SimAdapter(Node):

    def __init__(self):
        super().__init__('robotino_sim_adapter')
        self.declare_parameter('ir_count', 9)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('bumper_rate', 20.0)
        self.declare_parameter('bumper_timeout', 0.2)

        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.bumper_timeout = Duration(seconds=self.get_parameter('bumper_timeout').value)

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/omni_wheel_drive_controller/cmd_vel', 10)
        self.create_subscription(Twist, '/robotino/cmd_vel', self.cmd_vel_callback, 10)

        self.range_pubs = []
        for i in range(self.get_parameter('ir_count').value):
            pub = self.create_publisher(Range, f'/robotino/ir{i}', 10)
            self.range_pubs.append(pub)
            self.create_subscription(
                LaserScan, f'/robotino/sim/ir{i}',
                lambda msg, pub=pub: self.ir_callback(msg, pub), 10)

        # gz contact sensor publishes only while something is touching
        self.last_contact_time = None
        self.bumper_pub = self.create_publisher(Bool, '/robotino/bumper', 10)
        self.create_subscription(Contacts, '/robotino/sim/bumper', self.contacts_callback, 10)
        self.create_timer(1.0 / self.get_parameter('bumper_rate').value, self.publish_bumper)

    def cmd_vel_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.base_frame_id
        stamped.twist = msg
        self.cmd_vel_pub.publish(stamped)

    def ir_callback(self, scan: LaserScan, pub):
        rng = Range()
        rng.header = scan.header
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = scan.angle_max - scan.angle_min
        rng.min_range = scan.range_min
        rng.max_range = scan.range_max
        # gz reports -inf for obstacles closer than range_min and +inf for nothing within range_max.
        # nav2 collision_monitor rejects infinite ranges as malformed and ignores ranges out of
        # the sensor span, so an obstacle closer than range_min is reported at range_min and
        # nothing in range just beyond range_max; RViz shows no cone for it, like on the real robot
        rng.range = min((r for r in scan.ranges if not math.isnan(r)), default=math.inf)
        if rng.range < rng.min_range:
            rng.range = rng.min_range
        elif rng.range > rng.max_range:
            rng.range = rng.max_range + 0.01
        pub.publish(rng)

    def contacts_callback(self, msg: Contacts):
        if msg.contacts:
            self.last_contact_time = self.get_clock().now()

    def publish_bumper(self):
        pressed = (self.last_contact_time is not None
                   and self.get_clock().now() - self.last_contact_time < self.bumper_timeout)
        self.bumper_pub.publish(Bool(data=pressed))


def main():
    rclpy.init()
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
