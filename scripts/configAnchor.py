#!/usr/bin/env python3
from turtle import listen
import rospy
from mqttBridge import get_publisher, _wait_callback
from mock import MagicMock
import paho.mqtt.client as mqtt

from Augbot.msg import anchorConfig, anchor

#to configure anchor configuration in the UWB simulation

def sendMsg ():
    params = rospy.get_param("~", {})

    print ( params )

    if len( params ) == 0:
        return

    anchorConfigMsg = anchorConfig()

    msg = params.get( "configAnchorMsg" )

    print( msg )

    a = anchor()

    anchorConfigMsg.nAnchors = msg.get ("nAnchors")
    anchors = msg.get ( "anchors" )

    for i in range ( anchorConfigMsg.nAnchors ):
        a.position.x = anchors[i][0]
        a.position.y = anchors[i][0]
        a.position.z = anchors[i][0]
        anchorConfigMsg.anchors.append ( a )

    print( anchorConfigMsg )

    publisher.publish( anchorConfigMsg )
    _wait_callback ( ros_callback_anchorConfig )


if __name__ == '__main__':

    rospy.init_node('anchorConfig')

    mqtt_callback_anchorConfig = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add("anchorConfig", mqtt_callback_anchorConfig)
    mqttc.subscribe("anchorConfig")
    mqttc.loop_start()

    ros_callback_anchorConfig = MagicMock()

    subscriber_anchorConfig = rospy.Subscriber("/mqtt_anchorConfig", anchorConfig, ros_callback_anchorConfig)

    publisher = get_publisher("/mqtt_anchorConfig", anchorConfig, queue_size=1)

    sendMsg()

    