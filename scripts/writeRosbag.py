#!/usr/bin/env python
import rospy
import rosbag
from datetime import datetime
import os


from Augbot.msg import *

topics = []


def load_yaml ():
    global topics

    params = rospy.get_param("~", {})

    print ( params )

    if len( params ) == 0:
        return

    params = params.get( "topics2rosbag" ).get( "topics" )

    for topic in params:
        topics.append ( topic )

    print ( topics )


def callback( data, topic ):
    for aux in topic[2]:
        exec ( aux[1] )

if __name__ == '__main__':

    rospy.init_node('writeRosbag', anonymous=True)
  
    load_yaml()

    name = "/home/augmanity1/catkin_ws/rosbag/simulation/positions/" + str( datetime.now() ) + ".bag"
    name = name.replace(' ', '_')
    name = name.replace(':', '-')
    print ( name )
    bag = rosbag.Bag( name, 'w' )

    for topic in topics:
        rospy.Subscriber( topic[0], eval( topic[1] ), callback, topic )

    rospy.spin()

    bag.close()

    print ( "wrote to bag: " + name )