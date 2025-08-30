#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sys

class EmergencyStopClient(Node):
    def __init__(self):
        super().__init__('emergency_stop_client')
        self.client = self.create_client(SetBool, 'emergency_stop')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Emergency stop service not available, waiting...')

    def send_request(self, activate):
        request = SetBool.Request()
        request.data = activate
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Emergency stop result: {future.result().message}')
            return future.result()
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 2 or sys.argv[1] not in ['on', 'off']:
        print("Usage: ros2 run tortoisebot_description estop_client.py [on|off]")
        return
    
    activate = sys.argv[1] == 'on'
    
    client = EmergencyStopClient()
    result = client.send_request(activate)
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()