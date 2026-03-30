#!/usr/bin/env python3
from enum import Enum
from time import sleep

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
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

from path_utils import *

class State(Enum):
    WAIT_FOR_PATH = 0
    RIEGL_SCAN = 1
    WAIT_FOR_RIEGL_SCAN = 2
    MAP_TF_UPDATE = 3
    SEND_WAYPOINT = 4
    WAIT_TO_REACH_WAYPOINT = 5


class ExecutePlannedPath(Node):
    def __init__(self):
        super().__init__("follow_planned_path")
        self.declare_parameter("follow_path_server", "/follow_path")
        self.declare_parameter("scan_service", "/scan")
        self.declare_parameter("pose_service", "/get_vop")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("fake_riegl", False)

        self.fake_riegl = self.get_parameter("fake_riegl").value
        print(self.get_parameter("map_frame_id").value)
        print(self.get_parameter("fake_riegl").value)

        self.follow_path_server = str(self.get_parameter("follow_path_server").value)
        self.scan_service = str(self.get_parameter("scan_service").value)
        self.pose_service = str(self.get_parameter("pose_service").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.follow_path_action_client = ActionClient(self, FollowPath, self.follow_path_server)
        self.scan_trigger = self.create_client(Trigger, self.scan_service)
        if not self.fake_riegl:
            from riegl_vz_interfaces.srv import GetPose
            self.get_pose = self.create_client(GetPose, self.pose_service)

        self.map_waypoints = [
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
        ]
        # for i in range(2):
        #     next_waypoint = PoseStamped()
        #     next_waypoint.header.frame_id = "map"
        #     next_waypoint.pose.position.x = float(i + 1)
        #     # next_waypoint.pose.position.y = (i + 1) / 2.0
        #     self.map_waypoints.poses.append(next_waypoint)

        # Node State
        self.cur_state = State.WAIT_FOR_PATH
        self.cur_wpt_idx = 0

        self.follow_path_future_goal = None
        self.follow_path_future_result = None
        self.get_pose_future = None
        self.timer = self.create_timer(0.2, self.step_plan)

        self.get_logger().info(f"Node Ready. Stepping through plan")

    def wait_for_path(self):
        if self.map_waypoints is not None:
            self.cur_state = State.RIEGL_SCAN
            self.cur_wpt_idx = 0

        return True

    def trigger_riegl_scan(self):
        if self.fake_riegl:
            self.get_logger().info(f'Fake-triggered Riegl scan ...')
            sleep(.5)
            self.cur_state = State.MAP_TF_UPDATE
            return True
        else:
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
        if self.fake_riegl:
            # Fake it using odometry
            received, msg = wait_for_message(Odometry, self, "/nav/odom")

            if not received:
                self.get_logger().warning(f"Did not receive any pose update from the Odometry (faking Riegl). No correction applied to accumulated drift.")
                return False

            # Create a PoseStamped for interface consistency with Riegl call
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
        else:
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
        self.transform_map_to_base_link = transform_map_to_base_link
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
        composed_transform_stamped = compose_transforms(
            transform_map_to_base_link,
            transform_base_link_to_odom,
            self.get_clock().now().to_msg()
        )

        print("composed_transform_stamped")
        print(composed_transform_stamped)
        print()

        # Broadcast map-odom tf transform
        self.tf_static_broadcaster.sendTransform(composed_transform_stamped)

        self.cur_state = State.SEND_WAYPOINT
        return True

    def navigate_to_next_waypoint(self):
        waypoint = self.map_waypoints[self.cur_wpt_idx]
        self.get_logger().info(
            f"Navigating to the next waypoint {self.cur_wpt_idx}: ({waypoint[0]:.2f}, {waypoint[1]:.2f} in map)"
        )

        follow_path_goal = FollowPath.Goal()
        follow_path_goal.path.header.frame_id = "odom"
        follow_path_goal.path.poses = densify_path(
            (
                self.transform_map_to_base_link.transform.translation.x,
                self.transform_map_to_base_link.transform.translation.y,
                0.0
            ),
            waypoint,
            transform_map_to_base_link.header
        )

        self.follow_path_action_client.wait_for_server()
        self.follow_path_future_goal = self.follow_path_action_client.send_goal_async(follow_path_goal)

        self.cur_state = State.WAIT_TO_REACH_WAYPOINT

    def wait_to_reach_waypoint(self):
        if self.follow_path_future_goal is not None and self.follow_path_future_goal.done():
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info(f"Waypoint {self.cur_wpt_idx} Goal rejected. Trying again...")
                self.cur_state = State.SEND_WAYPOINT
                return

            self.follow_path_future_result = goal_handle.get_result_async()
            self.follow_path_future_goal = None

        if self.follow_path_future_result is not None and self.follow_path_future_result.done():
            self.follow_path_future_result = None
            self.get_logger().info(f"Reached waypoint {self.cur_wpt_idx}.")
            self.cur_wpt_idx += 1
            if (self.cur_wpt_idx) == len(self.map_waypoints):
                self.get_logger().info(f"Done with plan execution.")
                self.map_waypoints = None
                self.cur_state = State.WAIT_FOR_PATH
            else:
                self.cur_state = State.RIEGL_SCAN

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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
