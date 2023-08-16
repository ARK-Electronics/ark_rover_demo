#!/usr/bin/env python
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from nav_msgs.msg import Path
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, TrajectorySetpoint

def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id=frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    return pose_msg

class PX4Visualizer(Node):

    def __init__(self):
        super().__init__('px4_visualizer')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile)

        self.vehicle_pose_pub = self.create_publisher(PoseStamped, '/px4_visualizer/vehicle_pose', 10)
        self.vehicle_vel_pub = self.create_publisher(Marker, '/px4_visualizer/vehicle_velocity', 10)
        self.vehicle_path_pub = self.create_publisher(Path, '/px4_visualizer/vehicle_path', 10)
        self.setpoint_path_pub = self.create_publisher(Path, '/px4_visualizer/setpoint_path', 10)

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_msg = Path()
        self.setpoint_path_msg = Path()
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vehicle_attitude_callback(self, msg):
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz

    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position[0] = msg.position[0]
        self.setpoint_position[1] = -msg.position[1]
        self.setpoint_position[2] = -msg.position[2]

    def cmdloop_callback(self):
        vehicle_pose_msg = vector2PoseMsg('map', self.vehicle_local_position, self.vehicle_attitude)
        self.vehicle_pose_pub.publish(vehicle_pose_msg)

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "base_link"
        transform_stamped.transform.translation.x = self.vehicle_local_position[0]
        transform_stamped.transform.translation.y = self.vehicle_local_position[1]
        transform_stamped.transform.translation.z = self.vehicle_local_position[2]
        transform_stamped.transform.rotation.w = self.vehicle_attitude[0]
        transform_stamped.transform.rotation.x = self.vehicle_attitude[1]
        transform_stamped.transform.rotation.y = self.vehicle_attitude[2]
        transform_stamped.transform.rotation.z = self.vehicle_attitude[3]
        self.tf_broadcaster.sendTransform(transform_stamped)

        self.vehicle_path_msg.header = vehicle_pose_msg.header
        self.vehicle_path_msg.poses.append(vehicle_pose_msg)
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        setpoint_pose_msg = vector2PoseMsg('map', self.setpoint_position, self.vehicle_attitude)
        self.setpoint_path_msg.header = setpoint_pose_msg.header
        self.setpoint_path_msg.poses.append(setpoint_pose_msg)
        self.setpoint_path_pub.publish(self.setpoint_path_msg)

        velocity_msg = self.create_arrow_marker(1, self.vehicle_local_position, self.vehicle_local_velocity)
        self.vehicle_vel_pub.publish(velocity_msg)

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
        msg.type = Marker.ARROW
        msg.id = id
        msg.scale.x = 0.1
        msg.scale.y = 0.3
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.points.append(Point(x=tail[0], y=tail[1], z=tail[2]))
        msg.points.append(Point(x=tail[0]+vector[0], y=tail[1]+vector[1], z=tail[2]+vector[2]))
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PX4Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
