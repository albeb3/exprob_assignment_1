#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/camera_joint_controller/commands', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = [0.01]
        self.data = [0]

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.layout.dim = []  # Layout is empty in this case
        
        msg.data = [float (self.data[0] + self.i[0])]  # Concatenate data lists
        if float(msg.data[0])>=2*3.14:
        	self.data[0]=0
        else:
        	self.data[0]=float(msg.data[0])
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')  # Use f-string for cleaner logging"
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

