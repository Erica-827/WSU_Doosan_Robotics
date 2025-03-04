 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import RobotState
from dsr_msgs2.srv import MoveLine, MoveJoint
import math
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('r_controller')
        
        # Get namespace from node
        self.robot_ns = self.get_namespace().strip('/')
        if not self.robot_ns:
            self.robot_ns = 'dsr01'  # Default namespace
        
        # Publishers
        self.joint_publisher = self.create_publisher(
            Float64MultiArray, 
            '/dsr_controller2/commands',  # Control interface
            10
        )
        
        # Subscribers
        self.robot_state_sub = self.create_subscription(
            RobotState,
            f'/{self.robot_ns}/state',
            self.robot_state_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Service clients
        self.move_line_client = self.create_client(
            MoveLine,
            f'/{self.robot_ns}/motion/move_line'
        )
        
        self.move_joint_client = self.create_client(
            MoveJoint,
            f'/{self.robot_ns}/motion/move_joint'
        )
        
        # State variables
        self.current_state = None
        self.current_joints = None
        self.get_logger().info('Robot controller initialized')
    
    def robot_state_callback(self, msg):
        """Process robot state updates"""
        self.current_state = msg
        
    def joint_state_callback(self, msg):
        """Track current joint positions"""
        self.current_joints = msg.position
        
    def publish_joint_positions(self, positions):
        """Publish joint positions to the controller"""
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_publisher.publish(msg)
        self.get_logger().info(f'Published joint positions: {positions}')
    
    def move_to_home(self):
        """Move robot to home position"""
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_joint_positions(home_position)
    
    async def move_to_pose(self, x, y, z, vel=100):
        """Move end-effector to a cartesian pose"""
        if not self.move_line_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Move service not available')
            return False
            
        request = MoveLine.Request()
        request.pos = [x, y, z, 0.0, 0.0, 0.0]  # [x, y, z, rx, ry, rz]
        request.vel = [vel]  # velocity in mm/s
        request.acc = [150]  # acceleration in mm/s^2
        request.time = 0.0
        request.ref = 0  # Reference coordinate system (0: base)
        request.mode = 0  # Movement mode (0: absolute)
        request.radius = 0.0
        request.blending = 0
        request.sync_type = 0
        
        self.get_logger().info(f'Moving to position: x={x}, y={y}, z={z}')
        future = self.move_line_client.call_async(request)
        
        try:
            response = await future
            return response
        except Exception as e:
            self.get_logger().error(f'Move failed: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
