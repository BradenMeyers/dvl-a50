#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from scipy.spatial.transform import Rotation as R

class DvlStaticTFPub(Node):

    def __init__(self):
        super().__init__('dvl_static_tf_pub')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('child_frame_id', 'dvl_link')
        
        dvl_tf = TransformStamped()
        dvl_tf.header.stamp = self.get_clock().now().to_msg()  # Use first odom timestamp
        dvl_tf.header.frame_id = self.get_parameter('frame_id').value
        dvl_tf.child_frame_id = self.get_parameter('child_frame_id').value

        self.declare_parameter('transform_rotation', [1.0, 0.0, 0.0, 0.0])  # Defualt Quaternion for 180 deg rotation around X (Z down to Z up)
        quat = self.get_parameter('transform_rotation').value
        self.declare_parameter('transform_translation', [0.0, 0.0, 0.0])  # Default No translation
        translation = self.get_parameter('transform_translation').value
        quat = self.get_parameter('transform_rotation').value
        if len(quat) != 4:
            raise ValueError("transform_rotation must be 4 elements [x, y, z, w]")

        translation = self.get_parameter('transform_translation').value
        if len(translation) != 3:
            raise ValueError("transform_translation must be 3 elements [x, y, z]")

        # Translation 
        dvl_tf.transform.translation.x = translation[0]
        dvl_tf.transform.translation.y = translation[1]
        dvl_tf.transform.translation.z = translation[2]

        # Rotation quaternion 
        dvl_tf.transform.rotation.x = quat[0]
        dvl_tf.transform.rotation.y = quat[1]
        dvl_tf.transform.rotation.z = quat[2]
        dvl_tf.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(dvl_tf)

        self.get_logger().info(f'Published static transforms from {dvl_tf.header.frame_id} to {dvl_tf.child_frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = DvlStaticTFPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()