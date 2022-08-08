#!/usr/bin/env python3
import rospy
import scipy
from scipy.optimize import least_squares
from std_msgs.msg import String
from Augbot.msg import tagFull, position


#                                           reads tagFull from chatter 'UWB'
#                                           calculates leastSquares with data received
#                                           sends position calculated to chatter "leastSquares"



xi = []
yi = []
zi = []
ri = []
ranges = []
initial_guess = []

queue = []


def equations ( guess ):
    x, y , z = guess
    global ranges

    return [
        (x - a.position.x)**2 + (y - a.position.y)**2 + (z - a.position.z)**2 - (a.range )**2 for a in ranges
    ]

def movingAverage ( estimation ):
    global queue
    average = [ 0.0 ,0.0 ,0.0 ]
    if ( len ( queue ) < 10 ):
        queue.append ( estimation )
    else:
        queue.pop (0)
        queue.append ( estimation )
    for q in queue:
        average += q
    average = average / len ( queue )
    return average

#publish estimation into publish/subscribe chatter "leastSquares"
def publish ( estimation ):
    pos = position()
    pos.x = estimation[0]
    pos.y = estimation[1]
    pos.z = estimation[2]
    average = movingAverage ( estimation )
    pub.publish ( pos )
    pos.x = average[0]
    pos.y = average[1]
    pos.z = average[2]
    pubMA.publish( pos )

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "[%f,%f,%f]", data.estimate.position.x,data.estimate.position.y,data.estimate.position.z)
    global ranges, initial_guess

    ranges = [ data.anchors [ i ] for i in range ( data.nAnchors ) ] 

    results = least_squares(equations, initial_guess)
    initial_guess = [ results.x[0], results.x[1], results.x[2] ]
    #print( results )
    rospy.loginfo("LEAST_SQUARE [%f,%f,%f]", results.x[0], results.x[1], results.x[2] )
    rospy.loginfo("DECAWAVE [%f,%f,%f]", data.estimate.position.x, data.estimate.position.y, data.estimate.position.z )
    publish ( results.x )
    return results.x

    
def listener():

    rospy.Subscriber("UWB", tagFull, callback)

    rospy.spin()
  
if __name__ == '__main__':
    initial_guess = [1.0, 1.0, 0.0]

    pub = rospy.Publisher('leastSquares', position, queue_size=1)
    pubMA = rospy.Publisher('leastSquaresMA', position, queue_size=1)
    rospy.init_node('leastSquares', anonymous=True)
   
    listener()

