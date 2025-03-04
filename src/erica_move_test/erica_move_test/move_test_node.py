#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TestMoveNode(Node):

    def __init__(self):
        super().__init__("test_move")
        self.count = 0
        self.mov_cmd_pub_ = self.create_publisher(JointState, "/joint_states", 10)
        self.timer_ = self.create_timer(0.1, self.send_move_command)  # Timer to trigger send_move_command every 5 seconds



    def send_move_command(self):
        msg = JointState()  # Instantiate JointState correctly
        msg.header.stamp = self.get_clock().now().to_msg()  # Get current time using the clock
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        
        # Ensure self.count is within a valid range, and explicitly convert it to float
        # Here I'm assuming self.count is incremented properly, but adding checks for sanity.
        self.count = float(self.count)  # Make sure count is a float, as required
        
        # Set position values as floats
        msg.position = [0.0, 0.0, self.count/2, self.count, 0.0, 0.0]
        
        # Uncomment below lines if you need to set velocity and effort
        msg.velocity = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        # msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.count += 0.1  # Increment the count for position of the 4th joint
        self.mov_cmd_pub_.publish(msg)  # Publish the message to the topic


def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy
    node = TestMoveNode()  # Instantiate the node
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shutdown rclpy when done