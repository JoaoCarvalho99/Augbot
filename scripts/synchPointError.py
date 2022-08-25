#!/usr/bin/env python3
import rospy
import rosbag
from datetime import datetime
import time


from Augbot.msg import *
from sensor_msgs.msg import *
from tf2_msgs.msg import TFMessage

topics = [[ "leastSquares", position, 0 ],
["leastSquaresMA", position, 1 ],
[ "UWB", tagFull, -1 ],
[ "deadReckoningSP", position, 2 ],
[ "deadReckoning", position, 3 ],
[ "deadReckoningACC", position, 4 ],
[ "synchPoints", synchPoint, -1],
["tf", TFMessage, -1 ]]

lock = 0

estimations = [position(0,0,0), position(0,0,0), position(0,0,0), position(0,0,0), position(0,0,0)]

uwb = position(0,0,0)
real = [position (0, -1.5, 0), position (0, -3.50, 0), position (-4, -3.25, 0), position (-4, -1.75, 0)]
nPoint = 0



#writes every msg to rosbag
def callback( data, topic ):
    global estimations, uwb, real, nPoint, lock
    if ( topic [0] == "synchPoints" ):
        lock = 1
        synchPointErrorMsg = synchPointError()
        synchPointErrorMsg.deadReckoningSP = estimations[2]
        synchPointErrorMsg.deadReckoning = estimations[3]
        synchPointErrorMsg.deadReckoningACC = estimations[4]
        synchPointErrorMsg.synchPoint = data
        synchPointErrorMsg.real = real [nPoint]
        synchPointErrorMsg.leastSquares = estimations[0]
        synchPointErrorMsg.leastSquaresMA = estimations[1]
        synchPointErrorMsg.UWB= uwb
        pub.publish ( synchPointErrorMsg )
        rospy.loginfo ( synchPointErrorMsg )
        lock = 0
        nPoint += 1
        if nPoint == 4:
            nPoint = 0
        return
    if lock == 0:
        if topic[0] == "tf":
            real[0] = position (data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, 0)
            nPoint = 0
        elif topic[0] == "UWB":
            uwb = data.estimate.position
        else:
            estimations[ topic[2] ] = data



if __name__ == '__main__':

    rospy.init_node('synchPointError', anonymous=True)

    for topic in topics:
        rospy.Subscriber( topic[0], topic[1], callback, topic )
    pub = rospy.Publisher('synchPointError', synchPointError, queue_size=1)

    rospy.spin()
