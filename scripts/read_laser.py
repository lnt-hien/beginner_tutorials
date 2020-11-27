#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # 720/5 = 144
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    range_min = msg.range_min
    range_max = msg.range_max
    regions = msg.ranges
               
    print(f'Min angle: {angle_min}')
    print(f'Max angle: {angle_max}')
    print(f'Angle increment: {angle_increment}')
    print(f'Min range: {range_min}')
    print(f'Max range: {range_max}')
    print(f'Region: {regions}')

def main():
    rospy.init_node('reading_laser')
    sub= rospy.Subscriber("/scan", LaserScan, clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()