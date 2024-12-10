#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray  # Usato per velocità della camera

class CameraVelocityPublisher(Node):
    
    def __init__(self):
        super().__init__('camera_velocity_publisher')

        # Subscription to '/action' topic
        self.subscription = self.create_subscription(
            String,
            '/action',
            self.cmd_vel_callback,
            10
        )
        
        # Subscription to '/pos_marker_x_on_camera' for the marker position
        self.sub_pos_marker_x_on_camera = self.create_subscription(
            Float32,
            '/pos_marker_x_on_camera',
            self.pos_marker_x_on_camera_callback,
            10
        )

        # Publisher to publish the camera velocity
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/camera_velocity_controller/commands',
            10  # Queue size
        )
        
        # Initialize velocity data
        self.camera_velocity = Float64MultiArray()
        self.camera_velocity.data = [0.0, 0.0]  # Example: [linear velocity, angular velocity]

    def pos_marker_x_on_camera_callback(self, msg):
        try:
            # Convert the received message data to a float
            self.sub_pos_marker_x_on_camera = float(msg.data)
            self.sub_pos_marker_x_on_camera = abs(self.sub_pos_marker_x_on_camera)
        except ValueError:
            # Log a warning if conversion fails
            self.get_logger().warn(f"Invalid velocity value received: {msg.data}")

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

        # Handle different commands
        if msg.data == "rotate":
            self.camera_velocity.data = [0.0, 0.5]  # Example: no linear velocity, angular velocity for rotation
        elif msg.data == "align":
            self.get_logger().warn(f"Velocity value: {self.sub_pos_marker_x_on_camera}")
            self.camera_velocity.data = [0.0, self.sub_pos_marker_x_on_camera]
            if self.camera_velocity.data[1] < 0.2:
                self.camera_velocity.data[1] = 0.2  # Ensure minimum angular velocity for alignment
        elif msg.data == "stop":
            self.camera_velocity.data = [0.0, 0.0]  # Stop the camera
        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')
            return

        # Publish the camera velocity message
        self.publisher_.publish(self.camera_velocity)
        self.get_logger().info('Camera velocity published.')

def main(args=None):
    rclpy.init(args=args)
    camera_velocity_publisher = CameraVelocityPublisher()
    try:
        rclpy.spin(camera_velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

