#!/usr/bin/env python3
from enum import Enum
from time import sleep

import rclpy
import tf_transformations as tf_trans
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from riegl_vz_interfaces.srv import GetPose
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer,
    ConnectivityException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
    TransformStamped,
)
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class State(Enum):
    WAIT_FOR_PATH = 0
    RIEGL_SCAN = 1
    WAIT_FOR_RIEGL_SCAN = 2
    MAP_TF_UPDATE = 3
    SEND_WAYPOINT = 4
    WAIT_TO_REACH_WAYPOINT = 5


def transform_to_matrix(transform):
    """
    Convert a TransformStamped to a 4x4 matrix.
    """
    translation = [
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ]
    rotation = [
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ]
    return tf_trans.translation_matrix(translation) @ tf_trans.quaternion_matrix(
        rotation
    )


class ExecutePlannedPath(Node):
    def __init__(self):
        super().__init__("follow_planned_path")
        self.declare_parameter("goal_topic", "/controller_goal")
        self.declare_parameter("goal_active_srv", "/is_goal_active")
        self.declare_parameter("scan_service", "/scan")
        self.declare_parameter("pose_service", "/get_vop")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("map_frame_id", "map")

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.goal_active_srv = str(self.get_parameter("goal_active_srv").value)
        self.scan_service = str(self.get_parameter("scan_service").value)
        self.pose_service = str(self.get_parameter("pose_service").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.is_goal_active_srv = self.create_client(Trigger, self.goal_active_srv)
        self.scan_trigger = self.create_client(Trigger, self.scan_service)
        self.get_pose = self.create_client(GetPose, self.pose_service)

        self.path = Path()
        for i in range(2):
            next_waypoint = PoseStamped()
            next_waypoint.header.frame_id = "map"
            next_waypoint.pose.position.x = float(i + 1)
            # next_waypoint.pose.position.y = (i + 1) / 2.0
            self.path.poses.append(next_waypoint)

        # Node State
        self.cur_state = State.WAIT_FOR_PATH
        self.nav_goal_active = False
        self.cur_wpt_idx = 0

        self.is_goal_active_future = None
        self.get_pose_future = None
        self.timer = self.create_timer(0.2, self.step_plan)

        self.get_logger().info(f"Node Ready. Stepping through plan")

    def compose_transforms(self, t1, t2):
        """
        Compose two TransformStamped messages: t1 * t2.
        Returns a new TransformStamped.
        """
        # Convert TransformStamped to 4x4 matrices
        m1 = transform_to_matrix(t1)
        m2 = transform_to_matrix(t2)

        # Multiply the matrices
        m_composed = tf_trans.concatenate_matrices(m1, m2)

        # Convert back to TransformStamped
        composed_transform = TransformStamped()
        composed_transform.header.stamp = self.get_clock().now().to_msg()
        composed_transform.header.frame_id = t1.header.frame_id
        composed_transform.child_frame_id = t2.child_frame_id

        # Extract translation and rotation from the composed matrix
        translation = tf_trans.translation_from_matrix(m_composed)
        quaternion = tf_trans.quaternion_from_matrix(m_composed)

        composed_transform.transform.translation.x = translation[0]
        composed_transform.transform.translation.y = translation[1]
        composed_transform.transform.translation.z = translation[2]
        composed_transform.transform.rotation.x = quaternion[0]
        composed_transform.transform.rotation.y = quaternion[1]
        composed_transform.transform.rotation.z = quaternion[2]
        composed_transform.transform.rotation.w = quaternion[3]

        return composed_transform

    def wait_for_path(self):
        if self.path is not None:
            self.cur_state = State.RIEGL_SCAN
            self.cur_wpt_idx = 0

        return True

    def trigger_riegl_scan(self):
        self.get_logger().info(f"Triggered Riegl scan ...")
        self.is_scan_triggered_future = self.scan_trigger.call_async(Trigger.Request())
        self.cur_state = State.WAIT_FOR_RIEGL_SCAN
        return True

    def wait_for_riegl_scan(self):
        received, msg = wait_for_message(DiagnosticArray, self, "/diagnostics")
        if not received:
            return

        scanner_status = [
            stat for stat in msg.status if stat.name == "riegl_vz_node: scanner"
        ]
        if len(scanner_status) == 0:
            return

        scan_opstate = {val.key: val.value for val in scanner_status[0].values}[
            "opstate"
        ]
        if scan_opstate == "waiting":
            self.cur_state = State.MAP_TF_UPDATE

    def riegl_map_update(self):
        if self.get_pose_future is None:
            self.get_pose_future = self.get_pose.call_async(GetPose.Request())

        if not self.get_pose_future.done():
            return

        response = self.get_pose_future.result()
        self.get_pose_future = None
        if response is None or not response.success:
            self.get_logger().warning(
                f"Did not receive any pose update from the Riegl. No correction applied to accumulated drift."
            )
            return False
        ps = response.pose

        transform_map_to_base_link = TransformStamped()
        transform_map_to_base_link.header = ps.header
        transform_map_to_base_link.header.frame_id = "map"
        transform_map_to_base_link.child_frame_id = "base_link"
        transform_map_to_base_link.transform.translation.x = ps.pose.position.x
        transform_map_to_base_link.transform.translation.y = ps.pose.position.y
        transform_map_to_base_link.transform.translation.z = ps.pose.position.z
        transform_map_to_base_link.transform.rotation = ps.pose.orientation

        print("transform_map_to_base_link")
        print(transform_map_to_base_link)
        print()

        try:
            transform_base_link_to_odom = self.tf_buffer.lookup_transform(
                "base_link",
                "odom",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=4.0),
            )
        except (LookupException, ConnectivityException):
            self.get_logger().error(
                f"Failed to find transform from 'base_link' to 'odom'"
            )
            return

        print("transform_base_link_to_odom")
        print(transform_base_link_to_odom)
        print()

        # Compose the transforms
        composed_transform_stamped = self.compose_transforms(
            transform_map_to_base_link, transform_base_link_to_odom
        )

        print("composed_transform_stamped")
        print(composed_transform_stamped)
        print()

        # Broadcast map-odom tf transform
        self.tf_static_broadcaster.sendTransform(composed_transform_stamped)

        self.cur_state = State.SEND_WAYPOINT
        return True

    def navigate_to_next_waypoint(self):
        waypoint = self.path.poses[self.cur_wpt_idx]
        self.get_logger().info(
            f"Navigating to the next waypoint {self.cur_wpt_idx}: ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f} in {waypoint.header.frame_id})"
        )
        self.goal_pub.publish(waypoint)
        self.nav_goal_active = True
        self.is_goal_active_future = self.is_goal_active_srv.call_async(
            Trigger.Request()
        )
        self.cur_state = State.WAIT_TO_REACH_WAYPOINT

    def wait_to_reach_waypoint(self):
        if self.is_goal_active_future.done():
            res = self.is_goal_active_future.result()
            self.nav_goal_active = res.success

        if not self.nav_goal_active:
            self.get_logger().info(f"Reached waypoint {self.cur_wpt_idx}.")
            self.cur_wpt_idx += 1
            if (self.cur_wpt_idx) == len(self.path.poses):
                self.get_logger().info(f"Done with plan execution.")
                self.path = None
                self.cur_state = State.WAIT_FOR_PATH
            else:
                self.cur_state = State.RIEGL_SCAN

        self.is_goal_active_future = self.is_goal_active_srv.call_async(
            Trigger.Request()
        )

    def step_plan(self):
        self.get_logger().info(
            f"Stepping through plan. Current state: {self.cur_state}"
        )
        {
            State.WAIT_FOR_PATH: self.wait_for_path,
            State.RIEGL_SCAN: self.trigger_riegl_scan,
            State.WAIT_FOR_RIEGL_SCAN: self.wait_for_riegl_scan,
            State.MAP_TF_UPDATE: self.riegl_map_update,
            State.SEND_WAYPOINT: self.navigate_to_next_waypoint,
            State.WAIT_TO_REACH_WAYPOINT: self.wait_to_reach_waypoint,
        }[self.cur_state]()


def main(args=None):
    rclpy.init(args=args)
    node = ExecutePlannedPath()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop_goal_pose = PoseStamped()
    stop_goal_pose.header.frame_id = "base_link"
    node.goal_pub.publish(stop_goal_pose)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
