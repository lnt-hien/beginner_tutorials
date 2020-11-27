#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import math

# global constants
global YAW_TOLERANCE, DISTANCE_TOLERANCE
YAW_TOLERANCE = math.pi * 10 / 180 # +/- 10 degree allowed
DISTANCE_TOLERANCE = 0.5


# global variables
stop = False
current_yaw = 0
current_position = Point()
desired_position = Point()

# -----------CALLBACKS----------
# get information from Odometry
def callback_odom(msg):
    global current_position, current_yaw
    current_position = msg.pose.pose.position

    # current_yaw
    quaternion = (msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    current_yaw = euler[2]

# get information from 2D Nav Goal
def callback_pose(msg):
    global desired_position
    desired_position = msg.pose.position
# --------------------------------

def getCurrentPos(current_position):
    return f"Current position:\t{current_position}"

def getDesiredPos(desired_position):
    return f"Desired position:\t{desired_position}"

def getDriveMsg(steering_angle, speed):
    return f"Current drive msg:\tspeed {speed}\tsteering angle {steering_angle}"

# ----------------COMMAND MESSAGES----------------
def steer_right(pub_drive):
    drive_msg = AckermannDrive(steering_angle=0.4, speed=0.6)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(getDriveMsg(drive_msg.steering_angle, drive_msg.speed))
    print(getCurrentPos(current_position))

def steer_left(pub_drive):
    drive_msg = AckermannDrive(steering_angle=-0.4, speed=0.6)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(getDriveMsg(drive_msg.steering_angle, drive_msg.speed))
    print(getCurrentPos(current_position))

def go_straight(pub_drive):
    drive_msg = AckermannDrive(steering_angle=0.0, speed=2.0)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(getDriveMsg(drive_msg.steering_angle, drive_msg.speed))
    print(getCurrentPos(current_position))

def stop(pub_drive):
    drive_msg = AckermannDrive(steering_angle=0, speed=0)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(getCurrentPos(current_position))
    print("Reached desired position!!!")
# -----------------------------------------------

def check_conditions(desired_position):
    desired_yaw = math.atan2(desired_position.y - current_position.y, desired_position.x - current_position.x)
    yaw_err = desired_yaw - current_yaw
    abs_yaw_err = math.fabs(yaw_err)
    pos_err = math.sqrt(pow(desired_position.y - current_position.y, 2) + pow(desired_position.x - current_position.x, 2))
    
    print(getDesiredPos(desired_position))
    
    if abs_yaw_err > YAW_TOLERANCE:
        if yaw_err > 0:
            print(f'Yaw error: [{abs_yaw_err}] > Yaw precision: [{YAW_TOLERANCE}]')
            steer_right(pub_drive)
        else:
            print(f'Yaw error: [{abs_yaw_err}] > Yaw precision: [{YAW_TOLERANCE}]')
            steer_left(pub_drive)
        
        if pos_err < DISTANCE_TOLERANCE:
            print(f'Position error: [{pos_err}] <= Distance precision: [{DISTANCE_TOLERANCE}]')
            stop(pub_drive)
            if pos_err > DISTANCE_TOLERANCE:
                check_conditions(desired_position)
    else:
        print(f'Yaw error: [{abs_yaw_err}] <= Yaw precision: [{YAW_TOLERANCE}]')
        if pos_err > DISTANCE_TOLERANCE:
            print(f'Position error: [{pos_err}] > Distance precision: [{DISTANCE_TOLERANCE}]')
            go_straight(pub_drive)
            if abs_yaw_err > YAW_TOLERANCE:
                check_conditions(desired_position)
        else:
            print(f'Position error: [{pos_err}] <= Distance precision: [{DISTANCE_TOLERANCE}]')
            stop(pub_drive)
            if pos_err > DISTANCE_TOLERANCE:
                check_conditions(desired_position)


    
if __name__ == '__main__':
    pub_drive = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

    rospy.init_node('go_to_point')
   
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.wait_for_message('move_base_simple/goal', PoseStamped)
    sub_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, callback_pose)
    
    rate = rospy.Rate(1)   # Publish rate 10Hz
    while not rospy.is_shutdown():
        check_conditions(desired_position)
        print()
        rate.sleep()