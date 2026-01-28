#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVL, DVLDR
from builtin_interfaces.msg import Time


class DVLNavInterface(Node):

    def __init__(self):
        super().__init__('dvl_nav_interface')
        
        self.declare_parameter('odom_frame_id', 'dvl_odom')
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.declare_parameter('child_frame_id', 'base_link')
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.declare_parameter('orientation_variance', 0.15)  # radians^2
        self.orientation_var = self.get_parameter('orientation_variance').value
        self.declare_parameter('publish_tf', True)
        self.publish_tf = self.get_parameter('publish_tf').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            DVL,
            'dvl/data',
            self.dvl_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            'dvl/twist',
            10
        )

        self.sub_odom = self.create_subscription(
            DVLDR,
            'dvl/position',
            self.position_callback,
            qos_profile_sensor_data
        )
        self.pub_odom = self.create_publisher(
            Odometry,
            'dvl/odom',
            10
        )

        self.get_logger().info('DVL Navigation Interface started')


    def position_callback(self, msg: DVLDR):
        """
        Convert DVL dead-reckoned position to nav_msgs/Odometry.
        Converts the Odometry into the base link frame
        Also publishes the odom → base_link TF (if enabled)
        """

        odom = Odometry()

        # ---------- Header ----------
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.child_frame_id

        # Get transfrom from msg.header.frame_id to self.child_frame_id
        # CHECK: I dont think translation matters here since it starts from an arbitrary point
        try:
            T_base_dvl = self.tf_buffer.lookup_transform(
                self.child_frame_id,
                msg.header.frame_id,
                msg.header.stamp
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return
        
        R_base_dvl = R.from_quat([
            T_base_dvl.transform.rotation.x,
            T_base_dvl.transform.rotation.y,
            T_base_dvl.transform.rotation.z,
            T_base_dvl.transform.rotation.w,
        ]).as_matrix()


        # DVLDR time is float64 seconds
        sec = int(msg.time)
        nanosec = int((msg.time - sec) * 1e9)
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec

        # ---------- Position ----------
        # Rotation from DVL frame to base_link frame
        dvl_pos = np.array([[msg.position.x],
                            [msg.position.y],
                            [msg.position.z]])
        enu_pos = R_base_dvl @ dvl_pos

        odom.pose.pose.position.x = enu_pos[0, 0]
        odom.pose.pose.position.y = enu_pos[1, 0]
        odom.pose.pose.position.z = enu_pos[2, 0]

        # ---------- Orientation (Euler → quaternion via scipy) ----------
        ROT = R.from_euler('xyz', [msg.roll, msg.pitch, msg.yaw], degrees=True).as_matrix()
        R_Corrected = R_base_dvl @ ROT @ R_base_dvl.T
        quat = R.from_matrix(R_Corrected).as_quat()


        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # ---------- Pose covariance ----------
        # Position variance from pos_std (assumed isotropic)
        var = msg.pos_std ** 2
        odom.pose.covariance[0] = var
        odom.pose.covariance[7] = var
        odom.pose.covariance[14] = var

        odom.pose.covariance[21] = self.orientation_var
        odom.pose.covariance[28] = self.orientation_var
        odom.pose.covariance[35] = self.orientation_var

        self.pub_odom.publish(odom)

        if self.publish_tf:
            self.publish_odom_tf(odom)

    def publish_odom_tf(self, odom: Odometry):
        """
        Publish TF using the odometry message.
        """

        tf = TransformStamped()

        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = odom.header.frame_id
        tf.child_frame_id = odom.child_frame_id

        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z

        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)

    def dvl_callback(self, msg: DVL):

        if not msg.velocity_valid:
            self.get_logger().warn('Received invalid DVL velocity')
            return

        twist_msg = TwistWithCovarianceStamped()

        # ---------- Header ----------
        twist_msg.header.frame_id = msg.header.frame_id
        twist_msg.header.stamp = self._time_from_microseconds(
            msg.time_of_validity
        )

        # ---------- Linear velocity -------------
        twist_msg.twist.twist.linear.x = msg.velocity.x
        twist_msg.twist.twist.linear.y = msg.velocity.y
        twist_msg.twist.twist.linear.z = msg.velocity.z

        # ---------- Covariance -------------------
        # Assumes msg.covariance is 3x3 (row-major) for linear velocity
        # Populate the 6x6 Twist covariance matrix
        twist_msg.twist.covariance[0:3] = msg.covariance[0:3]
        twist_msg.twist.covariance[6:9] = msg.covariance[3:6]
        twist_msg.twist.covariance[12:15] = msg.covariance[6:9]

        self.pub.publish(twist_msg)

    @staticmethod
    def _time_from_microseconds(us: int) -> Time:
        t = Time()
        t.sec = us // 1_000_000
        t.nanosec = (us % 1_000_000) * 1_000
        return t


def main(args=None):
    rclpy.init(args=args)
    node = DVLNavInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
