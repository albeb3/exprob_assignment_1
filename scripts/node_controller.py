#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Cambia con il tipo di messaggio richiesto
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import time

class NodeController(Node):
    def __init__(self):
        super().__init__('node_controller')  # Nome del nodo
        
         # Inizializzazione variabili come attributi della classe
        self.selected_marker_id = 0
        self.marker_list = []
        self.marker_removed_list = []
        self.marker_showed_list = []
        self.action = String(data="rotate")
        self.trigger_remove = 0
        self.trigger_stop = 0
        self.trigger_image = 0
        self.bridge = CvBridge()
        
        # Publisher
        self.action_pub = self.create_publisher(String, 'action', 10)
        
        # Subscriber
        self.aruco_markers_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.listener_aruco_markers_callback,
            10
        )

        self.marker_detector_sub = self.create_subscription(
            Image,
            'image_with_circle',
            self.marker_detector_callback,
            10
        )
        self.sub_pos_marker_x_on_camera = self.create_subscription(
            Float32,
            '/pos_marker_x_on_camera',
            self.pos_marker_x_on_camera_callback,
            10
        )

  
    
    def listener_aruco_markers_callback(self, msg):
        
        self.selected_marker_id=msg.marker_ids[0]
        self.action_pub.publish(self.action)
        

        if self.selected_marker_id not in self.marker_list and self.selected_marker_id not in self.marker_removed_list:
            
            self.action.data="align"
            self.action_pub.publish(self.action)      
            self.marker_list.append(self.selected_marker_id)
            self.marker_list.sort()
            self.action.data="rotate"
            self.action_pub.publish(self.action)
       
        elif self.selected_marker_id in self.marker_list and  len(self.marker_list)==5 and self.trigger_remove==0:
            self.trigger_remove=1   
            self.trigger_image= 0
        
        elif self.selected_marker_id in self.marker_list and self.trigger_remove==1:
            
            self.action.data="align"
            self.action_pub.publish(self.action)
            if self.selected_marker_id == self.marker_list[0] :
                self.trigger_image= 1
                self.marker_removed_list.append(self.selected_marker_id)    
                self.marker_list.remove(self.selected_marker_id)
                self.action.data="rotate"
                self.action_pub.publish(self.action)
            
            if len(self.marker_list)==0:
                self.trigger_stop=1
                
        elif self.selected_marker_id in self.marker_removed_list and self.trigger_stop==1:
            self.action.data="stop"
            self.action_pub.publish(self.action)
        
        else:
            self.action.data="rotate"
            self.action_pub.publish(self.action)
        

        self.get_logger().info(f'Markers ID rilevati: {self.marker_list}')
        self.get_logger().info(f'Markers ID eliminati: {self.marker_removed_list}')
        self.action_pub.publish(self.action)
                
    
    
    def pos_marker_x_on_camera_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
        try:
            # Attempt to convert the received string to a float
           self.sub_pos_marker_x_on_camera = abs(msg.data)  # Solo float, non un oggetto Subscription
           
        except ValueError:
            # If conversion fails, log a warning
            self.get_logger().warn(f"Invalid velocity value received: {msg.data}")
   
    def marker_detector_callback(self, msg):
        # Converti l'immagine ROS in un formato OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.trigger_image==1 and self.selected_marker_id not in self.marker_showed_list and abs(self.sub_pos_marker_x_on_camera)<0.05 :
            #self.get_logger().info(f'Markers ID STAMPATO: {self.selected_marker_id}')
            #self.get_logger().info(f'Markers ID DI CONTROLLO: {self.marker_list[0]}')
            if self.selected_marker_id == self.marker_removed_list[-1]:
                self.marker_showed_list.append(self.selected_marker_id)
                # Mostra l'immagine con il cerchio
                cv2.imshow('Aruco Detection', cv_image)
                # Attendi un po' per aggiornare la finestra
                cv2.waitKey(4)

        


def main(args=None):
    rclpy.init(args=args)  # Inizializza il sistema ROS 2
    node = NodeController()       # Istanzia il nodo
    rotate_action = String(data="rotate")
    time.sleep(5)
    node.action_pub.publish(rotate_action)
    node.get_logger().info('Azione iniziale "rotate" pubblicata.')
    try:
        rclpy.spin(node)  # Esegue il nodo
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()  # Distrugge il nodo
        rclpy.shutdown()     # Arresta ROS 2

if __name__ == '__main__':
    main()

