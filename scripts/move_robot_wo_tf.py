import math
import numpy as np
import time

from geometry_msgs.msg import Pose,Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node


def quaternion_to_yaw(q) -> float:
    "Converts a quaternion into yaw (rotation about Z axis). Returns yaw in radians."
    siny_cosp = 2*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1 - 2*(q.y**2 +q.z**2)
    return math.atan2(siny_cosp, cosy_cosp)


class FrameListener(Node):

    def __init__(self):
        super().__init__('odom_listener')
        """
        self.cli = self.create_client(Trigger, 'scan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """
        # Create velocity publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.callback,
            10
        )

        self.target_pose = None
        self.trajectory = [(0.1,0.0), (0.0, 0.2), (0.0, 0.1)]
        self.cur_traj_idx = 0

        print("Done with init")

    def distance(self, p1, p2):
        return math.sqrt(
            (p1.position.x - p2.position.x) ** 2 +
            (p1.position.y - p2.position.y) ** 2)

    def angular_distance(self, p1, p2):
        return math.atan2(
            p1.position.y - p2.position.y,
            p1.position.x - p2.position.x)

    def callback(self, msg):
        cur_pose = msg.pose.pose

        if not self.target_pose:
            if len(self.trajectory) == self.cur_traj_idx:
                self.get_logger().info("Done with the trajectory")
                return

            self.target_pose = Pose()
            self.target_pose.position.x = cur_pose.position.x + self.trajectory[self.cur_traj_idx][0]
            self.target_pose.position.y = cur_pose.position.y + self.trajectory[self.cur_traj_idx][1]
            self.get_logger().info(f"target_pose {self.target_pose.position.x}, {self.target_pose.position.y}")
            self.get_logger().info(f"cur_pose {cur_pose.position.x}, {cur_pose.position.y}")

            ang_dist = self.angular_distance(cur_pose, self.target_pose)
            self.target_yaw = (quaternion_to_yaw(cur_pose.orientation) + ang_dist)
            self.cur_traj_idx += 1

        if self.distance(cur_pose, self.target_pose) < 0.05:
           self.target_pose = None
           """
           self.get_logger().info("start scanning")
           req = Trigger.Request()
           # self.get_logger().info("1")
           self.future = self.cli.call_async(req)
           # self.get_logger().info("2")
           # rclpy.spin_until_future_complete(self, self.future)
           sleep_time = 120
           self.get_logger().info(f"sleeping for: {sleep_time}s")
           time.sleep(sleep_time)
           self.get_logger().info("sleeping done")
           """
           return

        self.get_logger().info("Current distance %f" % self.distance(cur_pose, self.target_pose))

        msg = Twist()

        cur_yaw = quaternion_to_yaw(cur_pose.orientation)
        self.target_yaw = cur_yaw
        self.get_logger().info("Current yaw %f /  %f" % (cur_yaw, self.target_yaw))
        ang_dist = self.angular_distance(cur_pose, self.target_pose)
        #if abs(cur_yaw - self.target_yaw) > 0.05:
        #    msg.angular.z = math.copysign(0.1, -ang_dist) 
        if abs(ang_dist) > 0.1:
            for i in range(14):
                msg.angular.z = math.copysign(0.25, -ang_dist)
                self.publisher.publish(msg)
                time.sleep(0.1)
        else:
            msg.linear.x = math.copysign(0.05, self.target_pose.position.x - cur_pose.position.x)
     
        #self.get_logger().info('Moving x:%f' % msg.linear.x)
        print(msg)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

main()
