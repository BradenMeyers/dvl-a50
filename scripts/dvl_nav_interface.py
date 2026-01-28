#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math

from geometry_msgs.msg import TwistWithCovarianceStamped, Pose, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVL, DVLDR
from builtin_interfaces.msg import Time


class DVLNavInterface(Node):

    def __init__(self):
        super().__init__('dvl_nav_interface')
        
        self.declare_parameter('odom_frame_id', 'dvl_odom')
        self.declare_parameter('ned_odom_frame_id', 'dvl_odom_ned')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('orientation_variance', 0.15)  # radians^2
        self.declare_parameter('publish_tf', True)
        
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.ned_odom_frame_id = self.get_parameter('ned_odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.orientation_var = self.get_parameter('orientation_variance').value
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
            T_dvl_to_base = self.tf_buffer.lookup_transform(
                self.child_frame_id,
                msg.header.frame_id,
                msg.header.stamp
            )
            T_zup_zdown = self.tf_buffer.lookup_transform(
                self.ned_odom_frame_id,
                self.odom_frame_id,
                msg.header.stamp
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return
        

        # DVLDR time is float64 seconds
        sec = int(msg.time)
        nanosec = int((msg.time - sec) * 1e9)
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec

        # ------------ Pose --------------
        # Pose calculation using tf2 do_transform_pose
        q = self.quaternion_from_euler(msg.roll, msg.pitch, msg.yaw, degrees=True)

        ned_dvl_pose_in_dvl = Pose()
        ned_dvl_pose_in_dvl.position.x = msg.position.x
        ned_dvl_pose_in_dvl.position.y = msg.position.y
        ned_dvl_pose_in_dvl.position.z = msg.position.z
        ned_dvl_pose_in_dvl.orientation = q

        ned_dvl_pose_in_base = do_transform_pose(ned_dvl_pose_in_dvl, T_dvl_to_base)
       
        # Tedious unpacking to TransformStamped for do_transform_pose
        ned_dvl_transform_in_base = TransformStamped()
        ned_dvl_transform_in_base.transform.translation.x = ned_dvl_pose_in_base.position.x
        ned_dvl_transform_in_base.transform.translation.y = ned_dvl_pose_in_base.position.y
        ned_dvl_transform_in_base.transform.translation.z = ned_dvl_pose_in_base.position.z
        ned_dvl_transform_in_base.transform.rotation = ned_dvl_pose_in_base.orientation

        pose_zup_zdown = Pose()
        pose_zup_zdown.position.x = T_zup_zdown.transform.translation.x
        pose_zup_zdown.position.y = T_zup_zdown.transform.translation.y
        pose_zup_zdown.position.z = T_zup_zdown.transform.translation.z
        pose_zup_zdown.orientation = T_zup_zdown.transform.rotation

        dvl_pose_in_base = do_transform_pose(pose_zup_zdown, ned_dvl_transform_in_base)

        odom.pose.pose = dvl_pose_in_base

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
    
    def quaternion_from_euler(self, roll, pitch, yaw, degrees=False):
        """
        Convert Euler angles to a Quaternion message.

        :param roll: Roll angle in radians.
        :param pitch: Pitch angle in radians.
        :param yaw: Yaw angle in radians.
        :return: Populated geometry_msgs/Quaternion message.
        """
        if degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


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
