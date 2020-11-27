#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
import utils as Utils

import math

# global constants
global YAW_TOLERANCE, DISTANCE_TOLERANCE
YAW_TOLERANCE = math.pi * 10 / 180 # +/- 10 degree allowed
DISTANCE_TOLERANCE = 0.5

# global variables
stop = False
case = 0
current_yaw = 0
current_position = Point()
desired_position = Point()
scanned_regions = {
    'right': 0,
    'front_right': 0,
    'front': 0,
    'front_left': 0,
    'left': 0,
}

# -----------------------CALLBACKS--------------------------------
# ----------------------------------------------------------------
# get information from Odometry
def callback_odom(msg):
    global current_position, current_yaw
    current_position = msg.pose.pose.position

    # current_yaw
    quaternion = (msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w)

    current_yaw = Utils.angle_from_quaternion(quaternion)

# get information from 2D Nav Goal
def callback_pose(msg):
    global desired_position
    desired_position = msg.pose.position

# get information from laser scan
def callback_laserscan(msg):
    scanned_regions['right']       = min(min(msg.ranges[0:19]), 10)
    scanned_regions['front_right'] = min(min(msg.ranges[20:39]), 10)
    scanned_regions['front']       = min(min(msg.ranges[40:59]), 10)
    scanned_regions['front_left']  = min(min(msg.ranges[60:79]), 10)
    scanned_regions['left']        = min(min(msg.ranges[80:99]), 10)
# ----------------------------------------------------------------

# -------------------------COMMAND MESSAGES-------------------------
# ------------------------------------------------------------------
def steer_right(pub_drive, desired_position, scanned_regions):
    drive_msg = AckermannDrive(steering_angle=0.4, speed=0.6)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(Utils.getDriveMsg(drive_msg.steering_angle, drive_msg.speed))
    print(Utils.getDesiredPos(desired_position))
    print(Utils.getCurrentPos(current_position))

    if isObstacled(scanned_regions):
        avoid_obstacle(pub_drive)

def steer_left(pub_drive, desired_position, scanned_regions):
    drive_msg = AckermannDrive(steering_angle=-0.4, speed=0.6)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(Utils.getDriveMsg(drive_msg.steering_angle, drive_msg.speed))
    print(Utils.getDesiredPos(desired_position))
    print(Utils.getCurrentPos(current_position))

    if isObstacled(scanned_regions):
        avoid_obstacle(pub_drive)

def move_forward(pub_drive, desired_position, scanned_regions):
    drive_msg = AckermannDrive(steering_angle=0.0, speed=2.0)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(Utils.getDriveMsg(drive_msg.steering_angle, drive_msg.speed))
    print(Utils.getDesiredPos(desired_position))
    print(Utils.getCurrentPos(current_position))

    if isObstacled(scanned_regions):
        avoid_obstacle(pub_drive)

def stop(pub_drive, desired_position):
    drive_msg = AckermannDrive(steering_angle=0, speed=0)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(Utils.getDesiredPos(desired_position))
    print(Utils.getCurrentPos(current_position))
    print("Reached desired position!!!")

def avoid_obstacle(pub_drive):
    speed = 0
    steering_angle = 0

    if case == 1:
        speed = 1
        steering_angle = 0  
    elif case == 2:
        speed = 1
        if scanned_regions['front_left'] > scanned_regions['front_right']:
            steering_angle = -2
        else:
            steering_angle = 2
    elif case == 3:
        speed = 1
        steering_angle = -0.6
    elif case == 4:
        speed = 1
        steering_angle = 0.6
    elif case == 5:
        speed = 1
        steering_angle = -2
    elif case == 6:
        speed = 1
        steering_angle = 2
    elif case == 7:
        speed = 1
        steering_angle = 0
    elif case == 8:
        speed = -2
        if scanned_regions['front_left'] > scanned_regions['front_right']:
            steering_angle = -2
        else:
            steering_angle = 2  
    else:
        state_description = "Unknown case"

    drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed)
    pub_drive.publish(AckermannDriveStamped(drive=drive_msg))
    print(Utils.getDriveMsg(drive_msg.steering_angle, drive_msg.speed))
# ------------------------------------------------------------------

def check_conditions(desired_position):
    desired_yaw = math.atan2(desired_position.y - current_position.y, desired_position.x - current_position.x)
    yaw_err = Utils.normalize_angle(desired_yaw - current_yaw)
    abs_yaw_err = math.fabs(yaw_err)
    distance = math.sqrt(pow(desired_position.y - current_position.y, 2) + pow(desired_position.x - current_position.x, 2))  # Euclidean distance
    isRight = yaw_err > 0

    if abs_yaw_err > YAW_TOLERANCE:
        if distance > DISTANCE_TOLERANCE:
            if isRight:
                print(f'Yaw error: [{abs_yaw_err}] > Yaw precision: [{YAW_TOLERANCE}]')
                steer_right(pub_drive, desired_position, scanned_regions)
            else:
                print(f'Yaw error: [{abs_yaw_err}] > Yaw precision: [{YAW_TOLERANCE}]')
                steer_left(pub_drive, desired_position, scanned_regions)
        else:
            print(f'Position error: [{distance}] <= Distance precision: [{DISTANCE_TOLERANCE}]')
            stop(pub_drive, desired_position)
            if distance > DISTANCE_TOLERANCE:
                check_conditions(desired_position)
    else:
        print(f'Yaw error: [{abs_yaw_err}] <= Yaw precision: [{YAW_TOLERANCE}]')
        if distance > DISTANCE_TOLERANCE:
            print(f'Position error: [{distance}] > Distance precision: [{DISTANCE_TOLERANCE}]')
            move_forward(pub_drive, desired_position, scanned_regions)
            if abs_yaw_err > YAW_TOLERANCE:
                check_conditions(desired_position)
        else:
            print(f'Position error: [{distance}] <= Distance precision: [{DISTANCE_TOLERANCE}]')
            stop(pub_drive, desired_position)
            if distance > DISTANCE_TOLERANCE:
                check_conditions(desired_position)

def isObstacled(scanned_regions):
    global case
    obstacle_distance_allowed = 0.75
    print(f'Scanned Regions: {scanned_regions}')

    if (scanned_regions['front_left']  > obstacle_distance_allowed and 
        scanned_regions['front']       > obstacle_distance_allowed and 
        scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 1 - nothing"
        case = 1
        return False
    elif (scanned_regions['front_left']  > obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 2 - obstacle in front"
        case = 2
        return True
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       > obstacle_distance_allowed and 
          scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 3 - obstacle on front left"
        case = 3
        return True
    elif (scanned_regions['front_left']  > obstacle_distance_allowed and 
          scanned_regions['front']       > obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 4 - obstacle on front right"
        case = 4
        return True
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 5 - obstacle on front and front left"
        case = 5
        return True
    elif (scanned_regions['front_left']  > obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 6 - obstacle on front and front right"
        case = 6
        return True
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       > obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 7 - obstacle on front left and front right"
        case = 7
        return True
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 8 - obstacle on front and front left and front right"
        case = 8
        return True   
    else:
        state_description = "Unknown case"
    
    print(state_description)

    
if __name__ == '__main__':
    pub_drive = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    rospy.init_node('go_to_point')
   
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)
    sub_scan = rospy.Subscriber('/scan', LaserScan, callback_laserscan)

    # rospy.wait_for_message('move_base_simple/goal', PoseStamped)
    sub_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, callback_pose)
    
    rate = rospy.Rate(5)   # Publish rate 5Hz
    while not rospy.is_shutdown():
        check_conditions(desired_position)
        print()
        rate.sleep()