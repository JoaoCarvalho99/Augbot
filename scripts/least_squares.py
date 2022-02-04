#!/usr/bin/env python
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


def equations2 ( guess ):
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

def callback2(data):
    rospy.loginfo(rospy.get_caller_id() + "[%f,%f,%f]", data.estimate.position.x,data.estimate.position.y,data.estimate.position.z)
    global ranges

    ranges = [ data.anchors [ i ] for i in range ( data.nAnchors ) ] 

    results = least_squares(equations2, initial_guess)
    #print( results )
    rospy.loginfo("LEAST_SQUARE [%f,%f,%f]", results.x[0], results.x[1], results.x[2] )
    rospy.loginfo("DECAWAVE [%f,%f,%f]", data.estimate.position.x, data.estimate.position.y, data.estimate.position.z )
    publish ( results.x )
    return results.x


#not used
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "[%f,%f,%f]", data.estimate.position.x,data.estimate.position.y,data.estimate.position.z)
    
    nAnchors = 0
    xi.clear()
    yi.clear()
    zi.clear()
    ri.clear()
    for anchor in data.anchors:
        xi.append( anchor.position.x )
        yi.append( anchor.position.y )
        zi.append( anchor.position.z )
        ri.append( anchor.range )
        rospy.loginfo(" range: \t%f", anchor.range)
        if data.nAnchors == nAnchors:
            if nAnchors == 2:
                results = least_squares(equations, initial_guess)
                #print( results )
                rospy.loginfo("LEAST_SQUARE [%f,%f,%f]", results.x[0], results.x[1], results.x[2] )
                rospy.loginfo("DECAWAVE [%f,%f,%f]", data.estimate.position.x, data.estimate.position.y, data.estimate.position.z )
                publish ( results.x )
                return results.x
        nAnchors += 1

    
def listener():

    rospy.Subscriber("localization", tagFull, callback2)

    rospy.spin()
  
if __name__ == '__main__':
    initial_guess = [1.0, 1.0, 0.0]

    pub = rospy.Publisher('least_squares', position, queue_size=1)
    rospy.init_node('least_squares', anonymous=True)
   
    listener()

