#!/usr/bin/env python
import rospy
import paho.mqtt.client as paho

broker = "192.168.1.249"
port = 1883

def on_publish ( client, userdata, result ):
    print("data published\n")
    pass

client1 = paho.Client("control1")
client1.on_publish = on_publish
client1.connect(broker, port)
ret = client1.publish("house/bulb1","on")

#Example code: MQTT
#import paho.mqtt.client as paho
#broker="192.168.1.184"
#port=1883
#def on_publish(client,userdata,result):             #create function for callback
#    print("data published \n")
#    pass
#client1= paho.Client("control1")                           #create client object
#client1.on_publish = on_publish                          #assign function to callback
#client1.connect(broker,port)                                 #establish connection
#ret= client1.publish("house/bulb1","on") 