#!/usr/bin/env python
import rospy
from Augbot.msg import tagFull, anchor, anchorConfig
import numpy as np

from nav_msgs.msg import Odometry

def dead_reck(self):
    


def callback ( data ):
    print ("-----------")
    print( data.pose )
    print("############")
    print( data.twist )
    print()



def listener():

    rospy.Subscriber("odom", Odometry , callback)

    rospy.spin()
  
if __name__ == '__main__':
    rospy.init_node('simulation', anonymous=True)

    #pub = rospy.Publisher('localization', tagFull, queue_size=1)


    listener()