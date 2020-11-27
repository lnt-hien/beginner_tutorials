#!/usr/bin/env python3

import numpy as np
import math

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    
    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        rospy.loginfo("Publisher has been started!")
        
        self.sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, callback_laser)
        rospy.loginfo("Subscriber has been started!")

        self.self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }

        self.state = 0
        self.state_dict = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }

    # TODO:
    # Write your callback functions here.

    def callback_laser(data):
        self.region = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),
        }
        take_action()

    def change_state(state):
        if state is not self.state:
            print(f'Wall follower - [{state}] - {self.state_dict[state]}') 
            self.state = state

    def take_action():
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        linear_x = 0
        angular_z = 0

        state_description = ''
    
        d = 1.5
        
        if self.regions['front'] > d and self.regions['fleft'] > d and self.regions['fright'] > d:
            state_description = 'case 1 - nothing'
            change_state(0)
        elif self.regions['front'] < d and self.regions['fleft'] > d and self.regions['fright'] > d:
            state_description = 'case 2 - front'
            change_state(1)
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['fright'] < d:
            state_description = 'case 3 - fright'
            change_state(2)
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['fright'] > d:
            state_description = 'case 4 - fleft'
            change_state(0)
        elif self.regions['front'] < d and self.regions['fleft'] > d and self.regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            change_state(1)
        elif self.regions['front'] < d and self.regions['fleft'] < d and self.regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            change_state(1)
        elif self.regions['front'] < d and self.regions['fleft'] < d and self.regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            change_state(1)
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(self.self.regions)
    
    def find_wall():
        msg = AckermannDriveStamped()
        msg.linear_x = 0.2
        msg.angular_z = -0.3
        return msg

    def turn_left():
        msg = AckermannDriveStamped()
        msg.angular_z = 0.3
        return msg
    
    def follow_wall():
        msg = AckermannDriveStamped()
        msg.linear_x = 0.5
        return msg


if __name__ == "__main__":
    try:
        rospy.init_node('wall_follower')
        wall_follower = WallFollower()
        
        while not rospy.is_shutdown():
            msg = AckermannDriveStamped()
            if wall_follower.state == 0:
                msg = find_wall()
            elif wall_follower.state == 1:
                msg = turn_left()
            elif wall_follower.state == 2:
                msg = follow_wall()
                pass
            else:
                rospy.logerr("Unknown state!")
    except rospy.ROSInterruptException:
        pass