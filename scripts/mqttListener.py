#!/usr/bin/env python
from pickle import TRUE
import rospy
from ros2mqtt import get_publisher, _wait_callback
from mock import MagicMock
import paho.mqtt.client as mqtt

from Augbot.msg import *

#                   from mqttListener_params.yaml listens messages of type msgType from the mqtt channel listenTo
#                   and publishes in publish/subscribe channel pubTo according to the subset defined in the .launch file

listenTo = ""
pubTo = ""
msgType = ""

def load_yaml():
    global nodeName,pubTo,listenTo,msgType

    params = rospy.get_param("~", {})

    print ( params )

    if len( params ) == 0:
        return

    subset = params.get( "subset" )

    pubTo = params.get( "mqttListener" ).get( subset ).get( "pubTo" )

    listenTo = params.get( "mqttListener" ).get( subset ).get( "listenTo" )

    msgType = params.get( "mqttListener" ).get( subset ).get( "msgType" )




def listener():
    msg1 = None
    msg = None
    while TRUE:
        msg1 = ros_callback_output.call_args
        if msg1 != msg:
            print( msg1[0][0] )
            pub.publish( msg1[0][0] )
            print("published")
            msg = msg1


if __name__ == '__main__':
    rospy.init_node( "mqttListener" )

    load_yaml()

    mqtt_callback_output = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add( listenTo, mqtt_callback_output)
    mqttc.subscribe( listenTo )
    mqttc.loop_start()

    ros_callback_output = MagicMock()

    subscriber_output = rospy.Subscriber( listenTo, eval( msgType ), ros_callback_output)

    pub = rospy.Publisher( pubTo, eval( msgType ), queue_size=1)

    listener()

