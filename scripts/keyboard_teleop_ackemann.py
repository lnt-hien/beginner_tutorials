#!/usr/bin/env python3
import rospy

from ackermann_msgs.msg import AckermannDriveStamped

import sys
import select
import termios
import tty

banner = """
Reading from the keyboard  and Publishing to AckermannDriveStamped!
---------------------------
Moving around:
        w
   a    s    d

u/i : increase/decrease max speeds by 10%
j/k : increase/decrease only angular speed by 10%
space key, x : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    'd': (1, -1),
    'a': (1, 1),
    's': (-1, 0),
}

speedBindings={
    'u':(1.1,1.1),
    'i':(.9,.9),
    'j':(1,1.1),
    'k':(1,.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = 0.5
turn = 1


def getVels(speed, turn):
    return f"Currently:\tspeed {speed}\tturn {turn}" 


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.init_node('keyboard_teleop_ackermann')

    x = 0
    theta = 0
    status = 0
    count = 0

    try:
        print(getVels(speed, turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                theta = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(getVels(speed, turn))
                if status == 14:
                    # print(twist)
                    pass
                status = (status + 1) % 15
            elif key == ' ' or key == 'k':    
                x = 0
                theta = 0
            else:
                count = count +1
                if count > 4:
                    x = 0
                    theta = 0
                if (key == '\x03'):
                    break

            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "base_link"

            drive_msg.drive.speed = x * speed
            drive_msg.drive.acceleration = 1
            drive_msg.drive.jerk = 1
            drive_msg.drive.steering_angle = theta * turn
            drive_msg.drive.steering_angle_velocity = 1

            pub.publish(drive_msg)

    except Exception as e:
        print(e)

    finally:
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"

        drive_msg.drive.speed = 0
        drive_msg.drive.acceleration = 1
        drive_msg.drive.jerk = 1
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.steering_angle_velocity = 1
        pub.publish(drive_msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)