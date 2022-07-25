#!/usr/bin/env python3
from tkinter import Y
import rospy
import math
import numpy as np
from datetime import datetime

import tf

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState

#moves alphabot2 in the simulation in squares ( each side = 12m)

global yaw
global x, y

def callback(data):
    global yaw, x, Y
    w = data.transforms[0].transform.rotation.w
    z = data.transforms[0].transform.rotation.z

    x[1] = data.transforms[0].transform.translation.x
    y[1] = data.transforms[0].transform.translation.y

    q = data.transforms[0].transform.rotation

    quaternion = ( q.x, q.y, q.z, q.w )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]



def move():
    global twist, yaw, x, y
    i = 0
    wanted_yaw = [0.00000000, -math.pi/2, math.pi, math.pi/2]
    distance = 12
    while i < 4:
        #turn right 90º degrees
        turn ( wanted_yaw[i], 0 )

        twist.linear.x = 0
        twist.angular.z = 0
        pubMovement.publish( twist )
        rospy.loginfo("STOP")
        rospy.sleep ( 1 )

        x[0] = x[1] #xinitial
        y[0] = y[1] #yinitial

        rospy.loginfo("FORWARD")
        prev = -1
        while ( math.sqrt( ( x[1] - x[0] )**2 + ( y[1] - y[0] )**2 ) ) < distance:
            prev = moveForward ( wanted_yaw[i], 1, prev )
            rospy.sleep ( 0.1 )

        twist.linear.x = 0
        twist.angular.z = 0
        pubMovement.publish( twist )
        rospy.sleep ( 5 )

        i += 1

def moveForward ( wanted_yaw, x, prev):
    global yaw, twist
    speed = 0.005
    error = [ 0.0001, 0.001 ]
    twist.linear.x = x
    rospy.loginfo ( "%f", yaw - wanted_yaw)
    if ( abs (yaw - wanted_yaw ) < error[0] ) or ( abs ( yaw - wanted_yaw ) > 2 * math.pi - error[0] ):
        if prev != 0:
            twist.angular.z = 0
            pubMovement.publish ( twist )
            rospy.loginfo ("0")
        return 0
    if ( yaw - wanted_yaw > 0 ) or ( yaw - wanted_yaw < -6 ): #turn right
        if prev != 2:
            twist.angular.z = +speed
            pubMovement.publish ( twist )
        rospy.loginfo("2 - right")
        return 2
    if ( yaw - wanted_yaw < 0 ): #turn left
        if prev != 1:
            twist.angular.z = -speed
            pubMovement.publish ( twist )
        rospy.loginfo("1 - left")
        return 1


def turn ( wanted_yaw, i):
    global twist, yaw, x, y
    error = [ 0.000005, 0.0002, 0.002,  0.01,  0.02, 0.3, 1.0, 2.0 ]
    speed = [ 0.00005, 0.0004,  0.003,  0.005, 0.06, 0.3, 0.5, 1.0 ]
    twist.linear.x = 0
    rospy.loginfo("TURN start")
    while (not abs (yaw - wanted_yaw ) < error[i] ) and (not abs ( yaw - wanted_yaw ) > 2 * math.pi - error[i] ) :
        if i+1 != len( error ):
            if (not abs (yaw - wanted_yaw ) < error[i+1] ) and (not abs ( yaw - wanted_yaw ) > 2 * math.pi - error[i+1] ):
                turn ( wanted_yaw, i+1 )
        if twist.angular.z != speed[i]:
            twist.angular.z = speed[i]
            pubMovement.publish ( twist )
    if i == 0:
        twist.angular.z = 0
        pubMovement.publish ( twist )
        rospy.loginfo( "yaw: " + str( yaw ) )
        rospy.loginfo("TURN end")

def Reset ():
    msg = ModelState()
    msg.model_name = 'alphabot2'
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    msg.pose.orientation.w = 0
    msg.twist.linear.x = 0
    msg.twist.linear.y = 0
    msg.twist.linear.z = 0
    msg.twist.angular.x = 0
    msg.twist.angular.y = 0
    msg.twist.angular.z = 0

    pubReset = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1 )

    rospy.sleep ( 1 )

    pubReset.publish ( msg )

    rospy.loginfo ( "published Reset")



if __name__ == '__main__':
    rospy.init_node('gazebo_move', anonymous=True)
  
    pubMovement = rospy.Publisher('/alphabot2/control', Twist, queue_size=1 )

    Reset()

    yaw = 0
    x = [0, 0]
    y = [0, 0]
    prev = 0

    rospy.Subscriber("tf", TFMessage , callback)

    twist = Twist()

    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.angular.z = 0

    pubMovement.publish( twist )

    nSquares = 0
    while nSquares < 7:
        move()
        nSquares += 1

    info = String()
    twist.linear.x = 0
    twist.angular.z = 0
    pubMovement.publish( twist )
    rospy.loginfo("STOP")