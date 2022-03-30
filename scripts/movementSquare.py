#!/usr/bin/env python
from tkinter import Y
import rospy
from datetime import datetime

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

global z
global w
global x, y

def callback(data):
    global z, w, x, Y
    now = datetime.utcnow().timestamp()
    w = data.transforms[0].transform.rotation.w
    z = data.transforms[0].transform.rotation.z

    x[1] = data.transforms[0].transform.translation.x
    y[1] = data.transforms[0].transform.translation.y


def move():
    global twist, z, w, x, y
    i = 0
    wanted_z = [-0.7069, 0.9998, 0.7069, 0.000]
    wanted_w = [0.7069, 0.0000, 0.7069, 0.9997]
    distance = 10
    while i < 4:
        twist.linear.x = 0
        twist.angular.z = 0
        pubMovement.publish( twist )
        rospy.loginfo("STOP")
        rospy.sleep ( 5 )

        x[0] = x[1]
        y[0] = y[1]

        twist.linear.x = 1
        twist.angular.z = 0
        pubMovement.publish( twist )
        rospy.loginfo("FORWARD")
        while ( abs( x[1] - x[0] ) + abs( y[1] - y[0] ) ) < distance:
            continue

        twist.linear.x = 0
        twist.angular.z = 0
        pubMovement.publish( twist )
        rospy.sleep ( 10 )

        #turn right 90º degrees
        twist.linear.x = 0
        twist.angular.z = 0.1
        rospy.loginfo("TURN start")
        pubMovement.publish( twist )
        while round(z, 3) != round(wanted_z[i], 3) or round(w, 3) != round( wanted_w[i], 3):
            continue
        rospy.loginfo("TURN end")
        i += 1



if __name__ == '__main__':

    rospy.init_node('gazebo_move', anonymous=True)
  
    pubMovement = rospy.Publisher('/alphabot2/control', Twist, queue_size=1 )


    w,z = 0.000, 0.000
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
    twist.angular.z = 0.1
    rospy.sleep ( 1 )
    rospy.loginfo("turn start")
    pubMovement.publish( twist )
    while round(z, 3) != round(0.0000, 3) or round(w, 3) != round(0.9997, 3):
        continue
    rospy.loginfo("turn finish")

    twist.angular.z = 0
    pubMovement.publish( twist )

    nSquares = 0
    while nSquares<2: 
        move()
        nSquares += 1

    info = String()
    twist.linear.x = 0
    twist.angular.z = 0
    pubMovement.publish( twist )
    rospy.loginfo("STOP")