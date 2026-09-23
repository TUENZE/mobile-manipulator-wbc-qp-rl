#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class AddTablePlane(Node):

    def __init__(self):
        super().__init__('add_table_plane')

        self.publisher = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_object)
        self.sent = False

    def publish_object(self):

        if self.sent:
            return

        obj = CollisionObject()

        # 使用机器人底座坐标系
        obj.header.frame_id = 'base_link'
        obj.id = 'table_plane'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX

        # 长、宽、厚度
        box.dimensions = [
            4.0,     # X
            4.0,     # Y
            0.01     # Z
        ]

        pose = Pose()

        pose.position.x = 0.0
        pose.position.y = 0.0

        # box 中心在 -1 cm
        # 因此顶部约为 -5 mm
        pose.position.z = -0.010

        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)

        obj.operation = CollisionObject.ADD

        self.publisher.publish(obj)

        self.get_logger().info(
            'Added table collision plane below base_link.'
        )

        self.sent = True


def main(args=None):

    rclpy.init(args=args)

    node = AddTablePlane()

    # 留几秒让 MoveIt 收到消息
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
