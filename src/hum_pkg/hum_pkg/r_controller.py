import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import RobotState
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('r_controller')
        
        # Get namespace from node
        self.robot_ns = self.get_namespace().strip('/')
        if not self.robot_ns:
            self.robot_ns = 'dsr01'  # Default namespace
        
        # Subscribe to robot state
        self.robot_state_sub = self.create_subscription(
            RobotState,
            f'/{self.robot_ns}/state',
            self.robot_state_callback,
            10
        )
        
        self.get_logger().info('Robot controller initialized')
        self.current_state = None
        
    def robot_state_callback(self, msg):
        self.current_state = msg

def main():
    rclpy.init()
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

#!/usr/bin/env python3
# Save as src/hum_pkg/hum_pkg/robot_control.py

#!/usr/bin/env python3
# src/hum_pkg/hum_pkg/r_controller.py




# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState
# import math
# import time

# class RobotController(Node):
#     def __init__(self):
#         super().__init__('r_controller')
        
#         # Create publisher for joint positions
#         self.joint_publisher = self.create_publisher(
#             Float64MultiArray, 
#             '/forward_position_controller/commands', # This topic controls joint positions
#             10
#         )
        
#         # Subscribe to joint states to monitor robot
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )
        
        
#         self.get_logger().info('Robot controller initialized')
#         self.current_joints = None

#         # Start movement sequence after a short delay
#         #self.create_timer(2.0, self.move_sequence)

#         ### add services here. to trigger arm movement

#     def joint_state_callback(self, msg):     # Monitor joint states but don't override them
#         self.current_joints = msg.position
        
#     def publish_joint_positions(self, positions):  # Helper function to publish joint positions"
#         msg = Float64MultiArray()
#         msg.data = positions
#         self.joint_publisher.publish(msg)
#         self.get_logger().info(f'Published joint positions: {positions}')

#     # def move_sequence(self):
#     #     """Execute a sequence of joint movements"""
#     #     positions = [
#     #         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],                    # Home
#     #         [math.pi/4, 0.0, 0.0, 0.0, 0.0, 0.0],             # Rotate base
#     #         [math.pi/4, math.pi/4, -math.pi/4, 0.0, 0.0, 0.0], # Wave
#     #         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]                     # Return home
#     #     ]
        
#     #     for pos in positions:
#     #         self.publish_joint_positions(pos)
#     #         time.sleep(3.0)  # Wait between movements

#     def move_to_home(self):
#         """Move robot to home position when explicitly called"""
#         home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         self.publish_joint_positions(home_position)

# def main(args=None):
#     rclpy.init(args=args)
#     controller = RobotController()
    
#     try:
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState
# from dsr_msgs2.msg import RobotState
# from dsr_msgs2.srv import MoveLine, MoveJoint
# import math
# import time
# import asyncio
# import threading

# class RobotController(Node):
#     def __init__(self):
#         super().__init__('r_controller')
        
#         # Get namespace from node
#         self.robot_ns = self.get_namespace().strip('/')
#         if not self.robot_ns:
#             self.robot_ns = 'dsr01'  # Default namespace
            
#         # Create publisher for joint positions
#         self.joint_publisher = self.create_publisher(
#             Float64MultiArray, 
#             '/dsr_controller2/commands',  # Corrected topic name
#             10
#         )
        
#         # Subscribe to robot state
#         self.robot_state_sub = self.create_subscription(
#             RobotState,
#             f'/{self.robot_ns}/state',
#             self.robot_state_callback,
#             10
#         )
        
#         # Subscribe to joint states
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',  # Standard topic for joint states
#             self.joint_state_callback,
#             10
#         )
        
#         self.current_state = None
#         self.current_joints = None
#         self.get_logger().info('Robot controller initialized')

#     def robot_state_callback(self, msg):
#         self.current_state = msg
#         self.get_logger().debug('Received robot state update')

#     def joint_state_callback(self, msg):
#         self.current_joints = msg.position
#         self.get_logger().debug('Joint states updated')

#     def publish_joint_positions(self, positions):
#         """Publish joint positions to the controller"""
#         msg = Float64MultiArray()
#         msg.data = positions
#         self.joint_publisher.publish(msg)
#         self.get_logger().info(f'Published joint positions: {positions}')

#     def move_to_pose(self, x, y, z, vel=100):
#         """Move end-effector to a cartesian pose"""
#         self.get_logger().info(f'Moving to position: x={x}, y={y}, z={z}')

# def main(args=None):
#     rclpy.init(args=args)
#     controller = RobotController()
    
#     try:
#         # Home position
#         controller.publish_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()