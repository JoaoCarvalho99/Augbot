#!/usr/bin/env python3
import rospy
import scipy
from scipy.optimize import least_squares
from std_msgs.msg import String
from Augbot.msg import tagFull, position


#                                           reads tagFull from chatter 'localization'
#                                           calculates least_squares with data received
#                                           sends position calculated to chatter "least_squares"



xi = []
yi = []
zi = []
ri = []
ranges = []
initial_guess = []


def equations ( guess ):
    x, y , z = guess
    global ranges

    return [
        (x - a.position.x)**2 + (y - a.position.y)**2 + (z - a.position.z)**2 - (a.range )**2 for a in ranges
    ]

#publish estimation into publish/subscribe chatter "least_squares"
def publish ( estimation ):
    pos = position()
    pos.x = estimation[0]
    pos.y = estimation[1]
    pos.z = estimation[2]
    print(pub.name)
    pub.publish( pos )

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

    rospy.Subscriber("localization", tagFull, callback)

    rospy.spin()
  
if __name__ == '__main__':
    initial_guess = [1.0, 1.0, 0.0]

    pub = rospy.Publisher('least_squares', position, queue_size=1)
    rospy.init_node('least_squares', anonymous=True)
   
    listener()

