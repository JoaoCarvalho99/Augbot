#!/usr/bin/env python3
import rospy
import math
import numpy as np
import time
from datetime import datetime

import tf

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Int32MultiArray
from gazebo_msgs.msg import ModelState

from Augbot.msg import synchPoint

last_value = 0
last_proportional = 0
integral = 0
P = 0.6
I = 0.00005
D = 0

startingX = 0
startingY = 0

#max_turn 0.8, thresh 0.5, speed = 0.3 funciona
#maximum 35, maxturn 0.85, threshold 0.55, divisor 1, speed .3

maximum = 35 #0.5
max_turn = 0.85 #1.4
threshold = 0.55 #0.04 #1
divisor = 1 #10
speed = 0.3

def callback(data):
#multiplicar dados por 10
    #print ( data.data )
    global last_value, last_proportional, integral, threshold, maximum, max_turn, divisor, speed
    numSensors = 5
    sensor_values = data.data*10
    avg = 0
    sum = 0
    on_line = 0
    for i in range(0, numSensors):
        value = sensor_values[i]
        #if(white_line):
        value = 1000-value
        # keep track of whether we see the line at all
        if(value > 200):
            on_line = 1
            
        # only average in values that are above a noise threshold
        if(value > 50):
            avg += value * (i * 1000)  # this is for the weighted total,
            sum += value                  #this is for the denominator 

    if(on_line != 1):
        # If it last read to the left of center, return 0.
        if( last_value < (numSensors - 1)*1000/2):
            #print("left")
            last_value = 0

        # If it last read to the right of center, return the max.
        else:
            #print("right")
            last_value = (numSensors - 1)*1000
    else:
            last_value = avg/sum
		
    #print ( "position: " + str(last_value) )

    # The "proportional" term should be 0 when we are on the line.
    proportional = last_value - 2000
    
    # Compute the derivative (change) and integral (sum) of the position.
    derivative = proportional - last_proportional
    integral += proportional
    
    # Remember the last position.
    last_proportional = proportional

    '''
    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.  You can adjust the constants by which
    // the proportional, integral, and derivative terms are multiplied to
    // improve performance.
    '''
    power_difference = proportional*P + integral*I + derivative*D
    twist.linear.x = speed
    if ( last_value != 2000 ):
        print ("power_diff:" + str (power_difference ))
    if (power_difference > maximum):
        power_difference = maximum
    if (power_difference < - maximum):
        power_difference = - maximum
    power_difference = power_difference / divisor
    if ( last_value != 2000 ):
        print(last_value,power_difference)

    #if (power_difference < 0):
    #    Ab.setPWMA(maximum + power_difference)
    #    Ab.setPWMB(maximum);
    #else:
    #    Ab.setPWMA(maximum);
    #    Ab.setPWMB(maximum - power_difference)

    if ( power_difference > threshold ):
        power_difference = max_turn
        twist.linear.x = speed / 2
    if ( power_difference < -threshold ):
        power_difference = -max_turn
        twist.linear.x = speed / 2
    if ( last_value != 2000 ):
        print ( "final power diff: " + str(power_difference))
        print ( "_______")
    #twist.linear.x = speed #-(abs(power_difference)/maximum *0.3) #- abs(power_difference)
    twist.angular.z = power_difference
    pubMovement.publish( twist )
    rospy.sleep(0.01)


def synchPointPub ():
    global prev_time
    t = round(time.time() * 1000)
    pub.publish ( synchPoint( t, t - prev_time ) )
    prev_time = t

def Reset ():
    global prev_time
    msg = ModelState()
    msg.model_name = 'alphabot2'
    msg.pose.position.x = startingX
    msg.pose.position.y = startingY
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0.47
    msg.pose.orientation.w = 0.87
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


    rospy.Subscriber('/alphabot2/bottom_sensors', Int32MultiArray, callback)

    twist = Twist()

    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.linear.x = 0
    twist.angular.z = 0

    pubMovement.publish( twist )

    nSquares = 0
    #while nSquares < 5:
    #    rospy.loginfo ("VOLTA = " + str(nSquares))
        #move()
    #    nSquares += 1

    #info = String()
    #twist.linear.x = 0
    #twist.angular.z = 0
    #pubMovement.publish( twist )
    #rospy.loginfo("STOP")

    rospy.spin()