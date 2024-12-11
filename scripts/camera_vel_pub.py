#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Float32

class CameraVelocityPublisher(Node):
    def __init__(self):
        super().__init__('camera_vel_pub')
        
        # Crea un subscriber per ricevere comandi dal topic /action
        self.subscription = self.create_subscription(
            String, 
            '/action', 
            self.cmd_vel_callback, 
            10)
        
        self.sub_pos_marker_x_on_camera = self.create_subscription(
            Float32,
            '/pos_marker_x_on_camera',
            self.pos_marker_x_on_camera_callback,
            10
        )
        self.subscription

        # Crea un publisher per il topic /camera_vel_controller/commands
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/camera_vel_controller/commands',
              10)

        self.control_vel = 0.5  # Velocità di default

    def pos_marker_x_on_camera_callback(self, msg):
        #
        self.get_logger().info(f'I heard: "{msg.data}"')
        try:
        # Attempt to convert the received string to a float
            self.sub_pos_marker_x_on_camera = float(msg.data)  # Solo float, non un oggetto Subscription
            self.sub_pos_marker_x_on_camera = abs(self.sub_pos_marker_x_on_camera)
        except ValueError:
        # If conversion fails, log a warning
            self.get_logger().warn(f"Invalid velocity value received: {msg.data}")

    def publish_velocity(self, velocity):
        # Crea un messaggio Float64MultiArray
        msg = Float64MultiArray()
        msg.data = [velocity]  # Assegna la velocità
        self.publisher_.publish(msg)  # Pubblica il messaggio
        self.get_logger().info(f'Published velocity: {velocity}')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
        self.get_logger().info(f'control_vel: {self.control_vel}')

        # Create a Twist message based on the received command
        
        if msg.data == "rotate":
            self.publish_velocity(0.5)
        elif msg.data == "align":
            self.publish_velocity(self.control_vel)
            self.control_vel= self.sub_pos_marker_x_on_camera
            if self.control_vel< 0.2:
                self.control_vel = 0.2 
            
        elif msg.data == "stop":
            self.publish_velocity(0.0)
        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')
            return

        # Publish the message
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraVelocityPublisher()
    try:
        rclpy.spin(node)  # Mantieni il nodo attivo per ascoltare e pubblicare
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
