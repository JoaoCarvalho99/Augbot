#!/usr/bin/env python3
from pickle import TRUE
import rospy
from mock import MagicMock
import paho.mqtt.client as mqtt

import logging
import sys
import time
from logging import getLogger

import rosgraph

from Augbot.msg import *
from sensor_msgs.msg import *

#                   from mqttListener_params.yaml listens messages of type topic[2] from the mqtt channel topic[1]
#                   and publishes in publish/subscribe channel topic[3] according to the subset defined in the .launch file


logging.basicConfig(stream=sys.stderr)
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)

def get_publisher( topic_path, msg_type, **kwargs):
        pub = rospy.Publisher(topic_path, msg_type, **kwargs)
        num_subs = len( _get_subscribers( topic_path ) )
        for i in range(20):
            num_cons = pub.get_num_connections()
            if num_cons == num_subs:
                return pub
            time.sleep(0.1)
        print("failed to get publisher")

def _get_subscribers( topic_path ):
        ros_master = rosgraph.Master('/rostopic')
        topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
        state = ros_master.getSystemState()
        subs = []
        for sub in state[1]:
            if sub[0] == topic_path:
                subs.extend( sub[1] )
        return subs

def _wait_callback( callback_func ):
        for i in range(10):
            if callback_func.called:
                return
            time.sleep(0.1)
        print("fail")


topics = []

def load_yaml():
    global topics

    params = rospy.get_param("~", {})

    print ( params )

    if len( params ) == 0:
        return

    topics = params.get( "topics" )

    print ( topics )

    return topics

    #toDo = params.get( "toDo" )

    #topic[3] = params.get( toDo ).get( subset ).get( "topic[3]" )

    #topic[1] = params.get( toDo).get( subset ).get( "topic[1]" )

    #topic[2] = params.get( toDo ).get( subset ).get( "topic[2]" )


def listen(topic, ros_callback, pub):
    msg1 = None
    msg = None
    while TRUE:
        msg1 = ros_callback.call_args
        if msg1 != msg:
            print( msg1[0][0] )
            pub.publish( msg1[0][0] )
            print("published")
            msg = msg1
        time.sleep ( 0.1 )

def callback(data, args):
    ros_callback = args[0]
    publisher = args[1]
    publisher.publish( data )
    _wait_callback ( ros_callback )

def listener(topic, ros_callback, publisher):
    #publish/subscribe
    rospy.Subscriber( topic[1], eval( topic[2] ), callback, (ros_callback, publisher))



if __name__ == '__main__':
    rospy.init_node( "mqttBridge" )

    topics = load_yaml()

    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)

    rospy.sleep ( 1 )

    for topic in topics:

        if topic[0] == "mqttListen":
            mqtt_callback = MagicMock()
            ros_callback = MagicMock()
            mqttc.message_callback_add( topic[1], mqtt_callback)
            mqttc.subscribe( topic[1] )
            mqttc.loop_start()

            subscriber_output = rospy.Subscriber( topic[1], eval( topic[2] ), ros_callback)

            pub = rospy.Publisher( topic[3], eval( topic[2] ), queue_size=1)

            topic.append ( ros_callback )
            topic.append ( pub )

            listen(topic, ros_callback, pub)

    
        if topic[0] == "mqttPublish":
            print ( "publish from " + topic[1] + " to " + topic[3] )
            mqtt_callback = MagicMock()
            ros_callback = MagicMock()
            mqttc.message_callback_add( topic[3], mqtt_callback)
            mqttc.subscribe( topic[3] )
            mqttc.loop_start()

            subscriber = rospy.Subscriber( topic[3], eval( topic[2] ), ros_callback)

            publisher = get_publisher( topic[3], eval( topic[2] ), queue_size=1)

            listener(topic, ros_callback, publisher)
    
    rospy.spin()

