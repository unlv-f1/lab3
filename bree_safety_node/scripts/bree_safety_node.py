#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('bree_safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        #Create /odom Subscriber
        
        #Create /scan Subscriber

        #Create /drive Publisher
	

        
    def odom_callback(self, odom_msg):
        

    def scan_callback(self, scan_msg):
            

def main(args=None):
    #init ROS2 COMM and FEATURES
    rclpy.init(args=args)
    #create node inherited from node class SafetyNode
    bree_safety_node = SafetyNode()
    #Node spin -> keeps it alive until we kill [communication]
    rclpy.spin(bree_safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bree_safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
