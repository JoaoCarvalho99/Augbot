#!/usr/bin/env python
import rospy
import scipy
from scipy.optimize import least_squares
from std_msgs.msg import String
from DWM1001Reader.msg import tagFull






xi = []
yi = []
zi = []
ri = []
initial_guess = []


def equations ( guess ):
    x, y , z = guess

    return (
        (x - xi[0])**2 + (y - yi[0])**2 + (z - zi[0])**2 - (ri[0] )**2,
        (x - xi[1])**2 + (y - yi[1])**2 + (z - zi[1])**2 - (ri[1] )**2,
        (x - xi[2])**2 + (y - yi[2])**2 + (z - zi[2])**2 - (ri[2] )**2,
    )



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "[%f,%f,%f]", data.estimate.position.x,data.estimate.position.y,data.estimate.position.z)
    nAnchors = 0
    for anchor in data.anchors:
        if anchor.ID == "":
            if nAnchors > 2:
                results = least_squares(equations, initial_guess)
                print( results )
                rospy.loginfo("LEAST_SQUARE [%f,%f,%f]", results.x[0], results.x[1], results.x[2] )
                rospy.loginfo("DECAWAVE [%f,%f,%f]", data.estimate.position.x,data.estimate.position.y,data.estimate.position.z )
                return results.x
            xi.clear()
            yi.clear()
            zi.clear()
            ri.clear()
        rospy.loginfo("%s[%f,%f,%f]=%f",anchor.ID,anchor.position.x,anchor.position.y,anchor.position.z,anchor.range)
        xi.append( anchor.position.x )
        yi.append( anchor.position.y )
        zi.append( anchor.position.z )
        ri.append( anchor.range )
        nAnchors += 1

    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("localization", tagFull, callback)

    rospy.spin()
  
if __name__ == '__main__':
    initial_guess = [1.0, 1.0, 0.0]
    listener()