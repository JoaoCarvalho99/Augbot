#!/usr/bin/env python
import logging
import sys
import time
from logging import getLogger

import msgpack
from mock import MagicMock
from genpy import message

import rosgraph
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import Bool, String

from Augbot.msg import tagFull, anchor, estimate, position



#                               reads tagFull (data read in localization_improved_publisher.cpp) from chatter "localization"
#                                                           publishes with mqtt into "/tagFull"



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

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "[%f,%f,%f]", data.estimate.position.x,data.estimate.position.y,data.estimate.position.z)
    publisher.publish( data )
    _wait_callback ( ros_callback_tagFull )

def listener():
    #publish/subscribe
    rospy.Subscriber("localization", tagFull, callback)

    rospy.spin()


if __name__ == '__main__':

    rospy.init_node('tagFull2mqtt')

    mqtt_callback_tagFull = MagicMock()
    mqttc = mqtt.Client("client-id")
    mqttc.connect("localhost", 1883)
    mqttc.message_callback_add("tagFull", mqtt_callback_tagFull)
    mqttc.subscribe("tagFull")
    mqttc.loop_start()

    ros_callback_tagFull = MagicMock()

    subscriber_tagFull = rospy.Subscriber("/tagFull", tagFull, ros_callback_tagFull)

    publisher = get_publisher("/tagFull", tagFull, queue_size=1)
    print(publisher.name)

    listener()