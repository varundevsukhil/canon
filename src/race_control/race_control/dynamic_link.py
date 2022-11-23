#!/usr/bin/env python3
import sys

import rclpy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class RacecarTF:
    def __init__(
        self,
        node: Node,
        racecar_name: str,
        tf_pub=TransformBroadcaster,
    ) -> None:
        self.tf = TransformStamped()
        self.node = node
        self.tf_pub = tf_pub
        self.tf.header.frame_id = "odom"
        self.tf.child_frame_id = racecar_name

    def apply_transform(self, odom: Odometry) -> None:
        self.tf.transform = Transform(
            translation=Vector3(
                x=odom.pose.pose.position.x,
                y=odom.pose.pose.position.y,
            ),
            rotation=odom.pose.pose.orientation,
        )
        self.tf.header.stamp = self.node.get_clock().now().to_msg()
        self.tf_pub.sendTransform(self.tf)


class DynamicTF(Node):
    def __init__(self) -> None:
        self.tf_pub = TransformBroadcaster(self)
        self.racecars = [
            RacecarTF(self, str(sys.argv[i]), self.tf_pub)
            for i in range(1, len(sys.argv))
        ]
        self.subscribers = [
            self.create_subscription(
                Odometry,
                f"/{str(sys.argv[i])}/odom",
                self.racecars[i - 1].apply_transform,
                1,
            )
            for i in range(1, len(sys.argv))
        ]


def dynamic_link():
    rclpy.init()
    node = DynamicTF()
    rclpy.spin(node)
