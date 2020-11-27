import tf
import math
from geometry_msgs.msg import Quaternion

def getCurrentPos(current_position):
    return f"Current position:\tx: {current_position.x}\ty: {current_position.y}\tz: {current_position.z}"

def getDesiredPos(desired_position):
    return f"Desired position:\tx: {desired_position.x}\ty: {desired_position.y}\tz: {desired_position.z}"

def getDriveMsg(steering_angle, speed):
    return f"Current drive msg:\tSpeed: {speed}\tSteering angle: {steering_angle}"

def normalize_angle(angle):
    """
    Get the right angle for the case where angle is larger than pi.
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def quaternion_from_angle(angle):
    """
    Convert an angle in radians into a quaternion _message_.
    """
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def angle_from_quaternion(quaternion):
    """
    Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion.
    """

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw
