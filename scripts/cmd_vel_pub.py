#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32


class CmdVelPublisher(Node):
    
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        # Subscription to '/action' topic
        self.subscription = self.create_subscription(
            String,
            '/action',
            self.cmd_vel_callback,
            10
        )
        self.sub_pos_marker_x_on_camera = self.create_subscription(
            Float32,
            '/pos_marker_x_on_camera',
            self.pos_marker_x_on_camera_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Publisher to 'cmd_vel' topic
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10  # Queue size
        )
        
        self.control_vel=Twist()
        self.control_vel.angular.z =0.5
        self.publisher_.publish(self.control_vel)

    def pos_marker_x_on_camera_callback(self, msg):
        #self.get_logger().info(f'I heard: "{msg.data}"')
        try:
            # Attempt to convert the received string to a float
           self.sub_pos_marker_x_on_camera = float(msg.data)  # Solo float, non un oggetto Subscription
           self.sub_pos_marker_x_on_camera = abs(self.sub_pos_marker_x_on_camera)
        except ValueError:
            # If conversion fails, log a warning
            self.get_logger().warn(f"Invalid velocity value received: {msg.data}")

        
    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

        # Create a Twist message based on the received command
        
        if msg.data == "rotate":
            self.control_vel.angular.z = 0.5 # Example: move forward
        elif msg.data == "align":
             self.get_logger().warn(f"Velocity value : {self.sub_pos_marker_x_on_camera}")
             self.control_vel.angular.z=self.sub_pos_marker_x_on_camera
             if self.control_vel.angular.z < 0.2:
                 self.control_vel.angular.z = 0.2 
        elif msg.data == "stop":
            self.control_vel.angular.z = 0.0
        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')
            return

        # Publish the message
        self.publisher_.publish(self.control_vel)
        self.get_logger().info('cmd_vel published.')
    
    

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    try:
        rclpy.spin(cmd_vel_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_publisher.destroy_node()
        rclpy.shutdown()

   


if __name__ == '__main__':
    main()

