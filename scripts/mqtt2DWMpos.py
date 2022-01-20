#!/usr/bin/env python
import rospy
from ros2mqtt import get_publisher, _wait_callback
from mock import MagicMock
import paho.mqtt.client as mqtt

from Augbot.msg import position


#                               reads position (data read in localization_improved_publisher.cpp) from mqtt "position"


def listener():
    msg1 = None
    while msg1 == None:
        msg1 = ros_callback_dwm1001_output.call_args
        if msg1:
            a = msg1[0][0]
            print ( a.x )
            print ( a.y )
            print ( a.z )
            msg1 = None

if __name__ == '__main__':

    rospy.init_node('mqtt2tagFull')

    mqtt_callback_dwm1001_output = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add("dwm1001_output", mqtt_callback_dwm1001_output)
    mqttc.subscribe("dwm1001_output")
    mqttc.loop_start()

    ros_callback_dwm1001_output = MagicMock()

    subscriber_dwm1001_output = rospy.Subscriber("/dwm1001_output", position, ros_callback_dwm1001_output)

    listener()

