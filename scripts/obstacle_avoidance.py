#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

def getDriveMsg(steering_angle, speed):
    return f"Current drive msg:\tspeed {speed}\tsteering angle {steering_angle}"

scanned_regions = {
    'right': 0,
    'front_right': 0,
    'front': 0,
    'front_left': 0,
    'left': 0,
}

def callback_laserscan(msg):
    # scanned_regions = dict({
    #     'right': min(min(msg.ranges[0:19]), 10),
    #     'front_right': min(min(msg.ranges[20:39]), 10),
    #     'front': min(min(msg.ranges[40:59]), 10),
    #     'front_left': min(min(msg.ranges[60:79]), 10),
    #     'left': min(min(msg.ranges[80:99]), 10),
    # })

    scanned_regions['right'] = min(min(msg.ranges[0:19]), 10)
    scanned_regions['front_right'] = min(min(msg.ranges[20:39]), 10)
    scanned_regions['front'] = min(min(msg.ranges[40:59]), 10)
    scanned_regions['front_left'] = min(min(msg.ranges[60:79]), 10)
    scanned_regions['left'] = min(min(msg.ranges[80:99]), 10)

def avoid_obstacle(scanned_regions):
    speed = 0
    steering_angle = 0
    obstacle_distance_allowed = 0.75
    state_description = ""

    if (scanned_regions['front_left']  > obstacle_distance_allowed and 
        scanned_regions['front']       > obstacle_distance_allowed and 
        scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 1 - nothing"
        speed = 1
        steering_angle = 0
    elif (scanned_regions['front_left']  > obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 2 - obstacle in front"
        speed = 1
        if scanned_regions['front_left'] > scanned_regions['front_right']:
            steering_angle = -2
        else:
            steering_angle = 2
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       > obstacle_distance_allowed and 
          scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 3 - obstacle on front left"
        speed = 1
        steering_angle = -0.6
    elif (scanned_regions['front_left']  > obstacle_distance_allowed and 
          scanned_regions['front']       > obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 4 - obstacle on front right"
        speed = 1
        steering_angle = 0.6
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] > obstacle_distance_allowed):
        state_description = "Case 5 - obstacle on front and front left"
        speed = 1
        steering_angle = -2
    elif (scanned_regions['front_left']  > obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 6 - obstacle on front and front right"
        speed = 1
        steering_angle = 2
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       > obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 7 - obstacle on front left and front right"
        speed = 1
        steering_angle = 0
    elif (scanned_regions['front_left']  < obstacle_distance_allowed and 
          scanned_regions['front']       < obstacle_distance_allowed and 
          scanned_regions['front_right'] < obstacle_distance_allowed):
        state_description = "Case 8 - obstacle on front and front left and front right"
        speed = -2
        if scanned_regions['front_left'] > scanned_regions['front_right']:
            steering_angle = -2
        else:
            steering_angle = 2    
    else:
        state_description = "Unknown case"
    
    # rospy.loginfo(state_description)
    print(state_description)
    drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed)
    pub.publish(AckermannDriveStamped(drive=drive_msg))
    print(getDriveMsg(drive_msg.steering_angle, drive_msg.speed))



if __name__ == '__main__':
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

    rospy.init_node('obstacle_avoidance')

    sub = rospy.Subscriber('/scan', LaserScan, callback_laserscan)

    rate = rospy.Rate(10)   # Publish rate 5Hz
    while not rospy.is_shutdown():
        print(f'Scanned Regions: {scanned_regions}')
        avoid_obstacle(scanned_regions)
        rate.sleep()
