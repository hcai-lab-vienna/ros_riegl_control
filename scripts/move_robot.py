import math
import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('odom_frame_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #time.sleep(2)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

        from_frame_rel = 'base_link'
        to_frame_rel = 'odom'

        while rclpy.ok():
            try:
                self.start_transform = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
                break
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                time.sleep(1.0)
                continue

        self.target_transform = self.start_transform
        self.target_transform.transform.translation.x += 1

    def distance(self, t1, t2):
        return math.sqrt(
            (t1.transform.translation.x - t2.transform.translation.x) ** 2 +
            (t1.transform.translation.y - t2.transform.translation.y) ** 2)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'base_link'
        to_frame_rel = 'odom'

        try:
            cur_transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        if self.distance(cur_transform, self.target_transform) < 0.1:
           return

        msg = Twist()
        scale_rotation_rate = 1.0
        msg.angular.z = scale_rotation_rate * math.atan2(
            cur_transform.transform.translation.y,
            cur_transform.transform.translation.x)

        scale_forward_speed = 0.2
        # msg.linear.x = scale_forward_speed * math.sqrt(
        #     cur_transform.transform.translation.x ** 2 +
        #     cur_transform.transform.translation.y ** 2)
        msg.linear.x = 0.05

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
