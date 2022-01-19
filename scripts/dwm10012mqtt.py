#!/usr/bin/env python
import rospy
from ros2mqtt import get_publisher, _wait_callback
from mock import MagicMock
import paho.mqtt.client as mqtt

from Augbot.msg import tagFull, position


#                               reads tagFull (data read in localization_improved_publisher.cpp) from chatter "localization"
#                                                           publishes position estimated with mqtt into "/dwm1001"



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "[%f,%f,%f]", data.estimate.position.x,data.estimate.position.y,data.estimate.position.z)
    publisher.publish( data.estimate.position )
    _wait_callback ( ros_callback_dwm1001 )

def listener():
    #publish/subscribe
    rospy.Subscriber("localization", tagFull, callback)

    rospy.spin()


if __name__ == '__main__':

    rospy.init_node('dwm10012mqtt')

    mqtt_callback_dwm1001 = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add("dwm1001", mqtt_callback_dwm1001)
    mqttc.subscribe("dwm1001")
    mqttc.loop_start()

    ros_callback_dwm1001 = MagicMock()

    subscriber_dwm1001 = rospy.Subscriber("/dwm1001", position, ros_callback_dwm1001)

    publisher = get_publisher("/dwm1001", position, queue_size=1)
    print(publisher.name)

    listener()

