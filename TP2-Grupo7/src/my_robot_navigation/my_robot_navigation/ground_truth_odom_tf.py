#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthOdomTF(Node):
    """Publica odom->base_footprint usando /odom_real (pose real de Gazebo)
    en vez de la odometria integrada por el DiffDrive. Solo debe correr
    cuando DiffDrive tiene publish_diffdrive_tf:=false, para no pisarse
    con esa otra fuente de la misma transformada."""

    def __init__(self):
        super().__init__('ground_truth_odom_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom_real', self.odom_cb, 10)

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GroundTruthOdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
