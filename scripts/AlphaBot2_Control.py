#!/usr/bin/env python3
from tkinter import Y
import rospy
import math
import numpy as np
import time
from datetime import datetime

import tf

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState

from Augbot.msg import synchPoint

#moves alphabot2 in the simulation in squares ( each side = 10m) -> distance in def move()
#listens to /tf to obtain (x,y) and yaw of the robot

global yaw
global x, y
global prev_time

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

def synchPointPub ():
    global prev_time
    t = round(time.time() * 1000)
    pub.publish ( synchPoint( t, t - prev_time ) )
    prev_time = t


def move():
    global twist, yaw, x, y, prev_time
    i = 0
    wanted_yaw = [ -math.pi/2, math.pi, math.pi/2, 0.00000000]
    distance = 10
    speed = 0.7
    while i < 4:
        x[0] = x[1] #xinitial
        y[0] = y[1] #yinitial
        twist.linear.x = speed
        twist.angular.z = 0
        pubMovement.publish( twist )
        #turn right 90º degrees
        turn ( wanted_yaw[i], 0 )
        synchPointPub()
      #  twist.linear.x = 0
        twist.angular.z = 0
        pubMovement.publish( twist )
        rospy.loginfo("STOP")
        rospy.sleep ( 1 )

        rospy.loginfo("FORWARD")
        prev = -1
        while ( math.sqrt( ( x[1] - x[0] )**2 + ( y[1] - y[0] )**2 ) ) < distance:
            prev = moveForward ( wanted_yaw[i], speed, prev )
            rospy.sleep ( 0.1 )

        #twist.linear.x = 0
        #twist.angular.z = 0
        #pubMovement.publish( twist )


        #rospy.sleep ( 5 )

        i += 1

def moveForward ( wanted_yaw, x, prev):
    global yaw, twist
    speed = 0.003
    error = [ 0.0005, 0.001 ]
    twist.linear.x = x
    #rospy.loginfo ( "%f", yaw - wanted_yaw)
    if ( abs (yaw - wanted_yaw ) < error[0] ) or ( abs ( yaw - wanted_yaw ) > 2 * math.pi - error[0] ):
        if prev != 0:
            twist.angular.z = 0
            pubMovement.publish ( twist )
            rospy.loginfo ("0")
        return 0
    if ( yaw - wanted_yaw > 0 ): #turn right
        if prev != 2:
            twist.angular.z = +speed
            pubMovement.publish ( twist )
        rospy.loginfo("2 - right")
        return 2
    if ( yaw - wanted_yaw < 0 ) and (yaw - wanted_yaw > -6 ): #turn left
        if prev != 1:
            twist.angular.z = -speed
            pubMovement.publish ( twist )
        rospy.loginfo("1 - left")
        return 1
    elif prev != 2:
            twist.angular.z = +speed
            pubMovement.publish ( twist )
            rospy.loginfo("2 - right")
            return 2


def turn ( wanted_yaw, i):
    global twist, yaw, x, y
    #error = [ 0.0003, 0.001, 0.002,  0.02,  0.05, 0.5, 1.0, 2.0 ]
    #speed = [ 0.0001, 0.0005,  0.002,  0.004, 0.03, 0.1, 0.3, 1.0 ]
    error = [ 0.05, 0.1, 0.2, 0.5, 1.0, 2.0]
    speed = [ 0.02, 0.05, 0.1, 0.2, 0.5, 1]
    #twist.linear.x = 0
    rospy.loginfo("TURN start")
    while (not abs (yaw - wanted_yaw ) < error[i] ) and (not abs ( yaw - wanted_yaw ) > 2 * math.pi - error[i] ) :
        if i+1 != len( error ):
            if (not abs (yaw - wanted_yaw ) <= error[i+1] ) and (not abs ( yaw - wanted_yaw ) >= 2 * math.pi - error[i+1] ):
                turn ( wanted_yaw, i+1 )
        if twist.angular.z != speed[i]:
            twist.angular.z = speed[i]
            pubMovement.publish ( twist )
        rospy.sleep(0.001)
    if i == 0:
        twist.angular.z = 0
        pubMovement.publish ( twist )
        rospy.loginfo( "yaw: " + str( yaw ) )
        rospy.loginfo("TURN end")

def Reset ():
    global prev_time
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

    prev_time = round(time.time() * 1000)



if __name__ == '__main__':
    rospy.init_node('gazebo_move', anonymous=True)
  
    pubMovement = rospy.Publisher('/alphabot2/control', Twist, queue_size=1 )
    pub = rospy.Publisher('/synchPoints', synchPoint, queue_size=1)

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
    while nSquares < 5:
        rospy.loginfo ("VOLTA = " + str(nSquares))
        move()
        nSquares += 1

    info = String()
    twist.linear.x = 0
    twist.angular.z = 0
    pubMovement.publish( twist )
    rospy.loginfo("STOP")