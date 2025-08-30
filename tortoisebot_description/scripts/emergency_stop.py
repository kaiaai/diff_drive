#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        
        # Try to declare parameter, handle if already exists
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            self.get_logger().info("use_sim_time parameter already declared, using existing value")
        
        # Emergency stop state
        self.emergency_stop_active = False
        
        # Publishers
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_callback, 10)
        
        # Timer for emergency stop publishing
        self.timer = self.create_timer(0.1, self.publish_emergency_status)
        
        # Safety parameters
        self.min_safe_distance = 0.3  # meters
        
        self.get_logger().info('Safety Monitor Node started')
    
    def laser_callback(self, msg):
        """Check laser scan for obstacles"""
        if len(msg.ranges) == 0:
            return
            
        # Check front arc (assume laser points forward at index len/2)
        front_ranges = []
        total_points = len(msg.ranges)
        
        # Check front 60 degrees (30 degrees each side)
        front_arc_start = int(total_points * 0.4)  # 40% of total range
        front_arc_end = int(total_points * 0.6)    # 60% of total range
        
        for i in range(front_arc_start, front_arc_end):
            if msg.ranges[i] > 0.1:  # Ignore invalid readings
                front_ranges.append(msg.ranges[i])
        
        if front_ranges:
            min_distance = min(front_ranges)
            if min_distance < self.min_safe_distance:
                if not self.emergency_stop_active:
                    self.get_logger().warn(f'EMERGENCY STOP! Obstacle detected at {min_distance:.2f}m')
                    self.emergency_stop_active = True
            else:
                if self.emergency_stop_active:
                    self.get_logger().info('Path clear, disabling emergency stop')
                    self.emergency_stop_active = False
    
    def cmd_vel_callback(self, msg):
        """Filter and republish velocity commands based on safety status"""
        if self.emergency_stop_active:
            # Publish stop command
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        else:
            # Forward the command
            self.cmd_vel_pub.publish(msg)
    
    def publish_emergency_status(self):
        """Publish emergency stop status"""
        msg = Bool()
        msg.data = self.emergency_stop_active
        self.emergency_stop_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SafetyMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()