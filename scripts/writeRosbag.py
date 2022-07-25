#!/usr/bin/env python3
import rospy
import rosbag
from datetime import datetime
import os
import time
from threading import Lock


from Augbot.msg import *
from sensor_msgs.msg import *

topics = []
type
lock = Lock()
lock1 = Lock()

#reads tuplet ( topic.name, topic.msg_type ) for every topic
def load_yaml ():
    global topics, type

    params = rospy.get_param("~", {})

    print ( params )

    if len( params ) == 0:
        return

    type = params.get( "topics2rosbag" ).get( "type" )
    params = params.get( "topics2rosbag" ).get( "topics" )


    for topic in params:
        topics.append ( topic )

    print ( topics )

#writes every msg to rosbag
def callback( data, topic ):
    global lock, lock1
    #rospy.loginfo ( topic[0] )
    if topic[0] == "imu" or topic[0] == "microbit" or topic[0] == "synchPoints" or topic[0] == "localization":
        lock1.acquire()
        bag1.write ( topic[0], data )
        lock1.release()

    lock.acquire()
    bag.write ( topic[0], data )
    lock.release()


if __name__ == '__main__':

    rospy.init_node('writeRosbag', anonymous=True)

    load_yaml()

    name = "/home/augmanity1/catkin_ws/rosbag/" + type + "/" + str( datetime.now() ) + ".bag"
    name = name.replace(' ', '_')
    name = name.replace(':', '-')
    print ( name )
    bag = rosbag.Bag( name, 'w' )
    name = name.replace('.bag', '_sensors.bag')
    bag1 = rosbag.Bag( name, 'w' )

    for topic in topics:
        rospy.Subscriber( topic[0], eval( topic[1] ), callback, topic )

    rospy.spin()

    bag.close()
    bag1.close()

    print ( "wrote to bag: " + name )

    time.sleep(1)