#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )
        self.bubble = 90
        self.last_proc_ranges = np.zeros((1080,),dtype = np.float32)
        self.current_proc_ranges = np.zeros((1080,),dtype = np.float32)

    def preprocess_lidar(self, ranges):

        max_value = 1.4
        proc_ranges = np.array(ranges, dtype=np.float32)
        self.current_proc_ranges = (proc_ranges+self.last_proc_ranges)/2
        self.last_proc_ranges = proc_ranges

        filter_idx = self.current_proc_ranges>max_value
        self.current_proc_ranges[filter_idx] = max_value

    def find_max_gap(self, free_space_ranges):
        split_idx = np.where(free_space_ranges == 0.0)[0]
        sranges = np.split(free_space_ranges,split_idx)
        len_sranges = np.array([len(x) for x in sranges])
        max_idx = np.argmax(len_sranges)
        if max_idx == 0:
            start_i = 0
            end_i = len_sranges[0]-1
        else:
            start_i = np.sum(len_sranges[:max_idx])
            end_i = start_i+len_sranges[max_idx]-1
        max_length_ranges = sranges[max_idx]
        return start_i, end_i, max_length_ranges
    
    def find_best_point(self, start_i, end_i, ranges):
        idx_list = np.where(ranges == np.max(ranges))[0]
        best_idx = start_i + idx_list[round(len(idx_list)/2)]
        return best_idx

    def lidar_callback(self, data):
        ranges = data.ranges
        self.preprocess_lidar(ranges)

        cl_pt_idx = np.argmin(self.current_proc_ranges)
        bubble_idx = np.array(range(cl_pt_idx-self.bubble,cl_pt_idx+self.bubble))
        np.clip(bubble_idx,0,len(self.current_proc_ranges)-1,out = bubble_idx) #clip the indexes to min and max indexes
        bubble_idx = np.unique(bubble_idx) #remove the repeated index
        self.current_proc_ranges[bubble_idx] = 0
        start_i, end_i, max_length_ranges = self.find_max_gap(self.current_proc_ranges)
        best_idx = self.find_best_point(start_i,end_i,max_length_ranges)
        angle_array = np.arange(data.angle_min,data.angle_max,data.angle_increment)
        angle = angle_array[best_idx]
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        if abs(drive_msg.drive.steering_angle) >= np.radians(0) and abs(drive_msg.drive.steering_angle) < np.radians(10):
            drive_msg.drive.speed = 2.0
        elif abs(drive_msg.drive.steering_angle) >= np.radians(10) and abs(drive_msg.drive.steering_angle) < np.radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5
            # drive_msg.drive.acceleration = -0.1
        # drive_msg.drive.steering_angle_velocity = 1000.0
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
