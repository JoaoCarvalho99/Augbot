#!/usr/bin/env python
import rospy
from ros2mqtt import get_publisher, _wait_callback
from mock import MagicMock
import paho.mqtt.client as mqtt

from Augbot.msg import *

#                   from mqttListener_params.yaml listens messages of type msgType from the publish/subscribe listenTo
#                   and publishes in mqtt channel pubTo according to the subset defined in the .launch file

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

def callback(data):
    publisher.publish( )
    _wait_callback ( ros_callback )

def listener():
    #publish/subscribe
    rospy.Subscriber( listenTo, eval( msgType ), callback)

    rospy.spin()


if __name__ == '__main__':

    rospy.init_node('mqttPublisher')

    mqtt_callback = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add( pubTo, mqtt_callback)
    mqttc.subscribe("pubTo")
    mqttc.loop_start()

    ros_callback = MagicMock()

    subscriber = rospy.Subscriber( pubTo, eval( msgType ), ros_callback)

    publisher = get_publisher( pubTo, eval( msgType ), queue_size=1)
    print(publisher.name)

    listener()

