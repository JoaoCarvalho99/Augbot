#!/usr/bin/env python3
import rospy

from mqtt_bridge.app import mqtt_bridge_node



#       MUST LAUNCH
# launch Augbot ROSmqtt.launch
#       TO ACTIVE MQTT
#       ALSO
# mosquitto -v

try:
    mqtt_bridge_node()
except rospy.ROSInterruptException:
    pass
