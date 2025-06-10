#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# implementation of the hand on wall rule, https://en.wikipedia.org/wiki/Maze-solving_algorithm
# takes ~17 mins to solve the given 10x10 maze!
# currently this does not have the "stop when maze ends" logic and it continues to follow the outer boundary wall even after the maze ends
class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")

        # this is how we give or "publish" the desired velocity to our bot
        # https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # this is the LIDAR sensor, we "subscribe" to get readings from the sensor
        # https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/
        # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.regions = {
            "right": float("inf"),
            "front": float("inf"),
            "left": float("inf"),
        }
        self.state = "find_wall"  # initial state
        self.wall_found_side = None  # 'left' or 'right'

    def scan_callback(self, msg):
        # LaserScan.ranges has clockwise 360 readings from 0 (front) to 359 (just before front again)
        ranges = msg.ranges

        self.regions = {
            "right": min(min(ranges[270:290]), 10.0),
            "front": min(min(ranges[0:20] + ranges[340:360]), 10.0),
            "left": min(min(ranges[70:90]), 10.0),
        }

    def control_loop(self):
        msg = Twist()
        linear_speed = 0.15
        angular_speed = 0.3
        d = 0.3  # desired distance from wall
        front_threshold = 0.4  # threshold to consider obstacle in front

        # get current readings
        front = self.regions["front"]
        left = self.regions["left"]
        right = self.regions["right"]

        # log the current readings
        self.get_logger().info(f"state: {self.state}, front: {front:.2f}, left: {left:.2f}, right: {right:.2f}")

        if self.state == "find_wall":
            # initial state -> move forward until we find a wall
            msg.linear.x = linear_speed
            msg.angular.z = 0.0

            # check if we found a wall on any side
            if right < d * 1.5:
                self.state = "follow_wall"
                self.wall_found_side = "right"
                self.get_logger().info("found wall on right: start following")
            elif left < d * 1.5:
                self.state = "follow_wall"
                self.wall_found_side = "left"
                self.get_logger().info("found wall on left: start following")
            elif front < front_threshold:
                # wall directly in front -> turn left
                self.state = "follow_wall"
                self.wall_found_side = "left"
                msg.linear.x = 0.0
                msg.angular.z = angular_speed
                self.get_logger().info("wall directly ahead: turning left")

        elif self.state == "follow_wall":
            if self.wall_found_side == "right":
                if front < front_threshold:
                    # obstacle in front -> turn left
                    msg.linear.x = 0.0
                    msg.angular.z = angular_speed
                    self.get_logger().info("obstacle ahead: turning left")
                elif right > d * 1.2:
                    # far from right wall -> turn right
                    msg.linear.x = linear_speed * 0.5
                    msg.angular.z = -angular_speed * 0.7
                    self.get_logger().info("far from right wall: adjusting right")
                elif right < d * 0.8:
                    # close to right wall -> turn left
                    msg.linear.x = linear_speed * 0.5
                    msg.angular.z = angular_speed * 0.7
                    self.get_logger().info("close to right wall: adjusting left")
                else:
                    # just right -> go forward
                    msg.linear.x = linear_speed
                    msg.angular.z = 0.0
                    self.get_logger().info("following right wall: moving forward")

            elif self.wall_found_side == "left":
                if front < front_threshold:
                    # obstacle in front -> turn right
                    msg.linear.x = 0.0
                    msg.angular.z = -angular_speed
                    self.get_logger().info("obstacle ahead: turning right")
                elif left > d * 1.2:
                    # far from left wall -> turn left
                    msg.linear.x = linear_speed * 0.5
                    msg.angular.z = angular_speed * 0.7
                    self.get_logger().info("far from left wall: adjusting left")
                elif left < d * 0.8:
                    # close to left wall -> turn right
                    msg.linear.x = linear_speed * 0.5
                    msg.angular.z = -angular_speed * 0.7
                    self.get_logger().info("close to left wall: adjusting right")
                else:
                    # just left -> go forward
                    msg.linear.x = linear_speed
                    msg.angular.z = 0.0
                    self.get_logger().info("following left wall: moving forward")

        self.cmd_pub.publish(msg)


def main(args=None):
    # this is some standard stuff
    # refer https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
