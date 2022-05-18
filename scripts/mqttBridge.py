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

#                   from mqttListener_params.yaml listens messages of type msgType from the mqtt channel listenTo
#                   and publishes in publish/subscribe channel pubTo according to the subset defined in the .launch file


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


listenTo = ""
pubTo = ""
msgType = ""
toDo = ""

def load_yaml():
    global pubTo,listenTo,msgType, toDo

    params = rospy.get_param("~", {})

    print ( params )

    if len( params ) == 0:
        return

    subset = params.get( "subset" )

    toDo = params.get( "toDo" )

    pubTo = params.get( toDo ).get( subset ).get( "pubTo" )

    listenTo = params.get( toDo).get( subset ).get( "listenTo" )

    msgType = params.get( toDo ).get( subset ).get( "msgType" )


def listen():
    msg1 = None
    msg = None
    while TRUE:
        msg1 = ros_callback.call_args
        if msg1 != msg:
            print( msg1[0][0] )
            pub.publish( msg1[0][0] )
            print("published")
            msg = msg1

def callback(data):
    publisher.publish( data )
    _wait_callback ( ros_callback )

def listener():
    #publish/subscribe
    rospy.Subscriber( listenTo, eval( msgType ), callback)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node( "mqttBridge" )

    load_yaml()

    mqtt_callback = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)

    ros_callback = MagicMock()

    if toDo == "mqttListen":
        mqttc.message_callback_add( listenTo, mqtt_callback)
        mqttc.subscribe( listenTo )
        mqttc.loop_start()

        subscriber_output = rospy.Subscriber( listenTo, eval( msgType ), ros_callback)

        pub = rospy.Publisher( pubTo, eval( msgType ), queue_size=1)

        listen()
    
    if toDo == "mqttPublish":
        mqttc.message_callback_add( pubTo, mqtt_callback)
        mqttc.subscribe("pubTo")
        mqttc.loop_start()
    
        subscriber = rospy.Subscriber( pubTo, eval( msgType ), ros_callback)
    
        publisher = get_publisher( pubTo, eval( msgType ), queue_size=1)
    
        listener()

