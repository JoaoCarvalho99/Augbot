#!/usr/bin/env python
from pickle import TRUE
import rospy
from ros2mqtt import get_publisher, _wait_callback
from mock import MagicMock
import paho.mqtt.client as mqtt

from Augbot.msg import anchorConfig



def listener():
    msg1 = None
    msg = None
    while TRUE:
        msg1 = ros_callback_anchorConfig_output.call_args
        if msg1 != msg:
            print( msg1[0][0] )
            pub.publish( msg1[0][0] )
            print("published")
            msg = msg1


if __name__ == '__main__':

    rospy.init_node('mqtt2tagFull')

    mqtt_callback_anchorConfig_output = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add("anchorConfig_output", mqtt_callback_anchorConfig_output)
    mqttc.subscribe("anchorConfig_output")
    mqttc.loop_start()

    ros_callback_anchorConfig_output = MagicMock()

    subscriber_anchorConfig_output = rospy.Subscriber("/mqtt_anchorConfig_output", anchorConfig, ros_callback_anchorConfig_output)

    pub = rospy.Publisher('anchorConfig', anchorConfig, queue_size=1)

    listener()

