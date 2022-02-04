#!/usr/bin/env python
from turtle import listen
import rospy
from ros2mqtt import get_publisher, _wait_callback
from mock import MagicMock
import paho.mqtt.client as mqtt

from Augbot.msg import position

#                       reads positions calculated by least_squares algorithm by chatter "least_squares"
#                                      publishes with mqtt into "/least_squares"



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "[%f,%f,%f]", data.x,data.y,data.z)
    publisher.publish( data )
    _wait_callback ( ros_callback_least_squares )

def listener():
    
    #publish/subscribe
    rospy.Subscriber("least_squares", position, callback)

    rospy.spin()


if __name__ == '__main__':

    rospy.init_node('least_squares2mqtt')

    mqtt_callback_least_squares = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add("least_squares", mqtt_callback_least_squares)
    mqttc.subscribe("least_squares")
    mqttc.loop_start()

    ros_callback_least_squares = MagicMock()

    subscriber_least_squares = rospy.Subscriber("/mqtt_least_squares", position, ros_callback_least_squares)

    publisher = get_publisher("/mqtt_least_squares", position, queue_size=1)

    listener()
