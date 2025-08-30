#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import signal

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        # Publisher - use cmd_vel_input so emergency_stop can intercept
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_input', 10)
        
        # Subscribe to emergency stop status
        self.emergency_subscriber = self.create_subscription(
            Bool,
            '/emergency_stop_status',
            self.emergency_callback,
            10
        )
        
        # Control state
        self.emergency_active = False
        self.shutdown_requested = False
        
        # Timer for publishing commands (you can modify this for your control logic)
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info('Simple Robot Controller Started')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info('Publishing to /cmd_vel_input')

    def emergency_callback(self, msg):
        """Handle emergency stop status updates"""
        self.emergency_active = msg.data
        if self.emergency_active:
            self.get_logger().warn('Emergency stop activated - stopping commands')
        else:
            self.get_logger().info('Emergency stop deactivated - can resume')

    def control_callback(self):
        """Main control loop - modify this for your robot behavior"""
        if self.shutdown_requested or self.emergency_active:
            return
        
        # Simple example: publish stop command (modify for your needs)
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        # You can add your control logic here
        # For example:
        # - Keyboard input processing
        # - Autonomous navigation
        # - Follow waypoints
        # - etc.
        
        self.cmd_vel_publisher.publish(cmd)

    def move_forward(self, duration=1.0):
        """Helper function to move forward"""
        if self.emergency_active:
            return
            
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        self.cmd_vel_publisher.publish(cmd)

    def turn_left(self, duration=1.0):
        """Helper function to turn left"""
        if self.emergency_active:
            return
            
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(cmd)

    def turn_right(self, duration=1.0):
        """Helper function to turn right"""
        if self.emergency_active:
            return
            
        cmd = Twist()
        cmd.angular.z = -self.angular_speed
        self.cmd_vel_publisher.publish(cmd)

    def stop(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)

    def shutdown_gracefully(self):
        """Handle graceful shutdown"""
        self.shutdown_requested = True
        
        # Send stop command
        self.stop()
        
        self.get_logger().info('Simple robot controller shutting down gracefully...')

def signal_handler(signum, frame, node):
    """Handle shutdown signals"""
    node.shutdown_gracefully()

def main(args=None):
    rclpy.init(args=args)
    
    controller = SimpleRobotController()
    
    # Set up signal handler
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, controller))
    signal.signal(signal.SIGTERM, lambda s, f: signal_handler(s, f, controller))
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Received KeyboardInterrupt')
    finally:
        controller.shutdown_gracefully()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()