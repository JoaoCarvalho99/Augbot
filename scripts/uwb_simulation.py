#!/usr/bin/env python3
from distutils.log import error
from time import time
import rospy
from Augbot.msg import tagFull, anchor, anchorConfig
import numpy as np
from datetime import datetime

from tf2_msgs.msg import TFMessage

#                                           reads position of simulated alphabot2 'tf'
#                                           calculates ranges from the anchors to the robot's position
#                                           sends tagFull to chatter "UWB"




#initial_guess = []
#ri = []
pos = [ 0.0, 0.0, 0.0 ]

nAnchors = 5
anchors = [
    [ -5.0, -5.0, 2.0 ],
    [ 5.0, 5.0, 2.0 ],
    [ 5.0, -5.0, 2.0 ],
    [ -5.0, 5.0, 2.0 ],
    [ 0.0, 0.0, 2.0 ]
]

prev = 0

mu, sigma = 0, 0.1 # mean and standard deviation


#reads mu, sigma, nAnchors and anchors from .yaml
def load_yaml():
    global nAnchors, anchors, mu, sigma


    params = rospy.get_param("~", {})

    print ( params )

    if len( params ) == 0:
        return

    n = params.get( "uwb_simulation" ).get("noise")

    mu, sigma = n.get( "mu" ), n.get( "sigma" )

    nAnchors = params.get( "uwb_simulation" ).get("nAnchors")

    anchors = params.get( "uwb_simulation" ).get("anchors")

#adds noise to range calculation
def noise ( ):
    global mu, sigma
    n = np.random.default_rng().normal(mu, sigma, 1)
    #rospy.loginfo ( "error: " + str(n[0]) )
    return n[0]

#calculates range
def distance ( anchor, pos ):
    return ( ( pos[0] - anchor[0] )**2 + ( pos[1] - anchor[1] )**2 + ( pos[2] - anchor[2] )**2 )**( 1/2 ) + noise ()


#callback to new anchor configuration
def ConfigCallback ( data ):
    global anchors, nAnchors

    nAnchors = data.nAnchors

    rospy.loginfo ( "anchors changed" )

    for i in range ( data.nAnchors ):
        anchors[i][0] = data.anchors[i].position.x
        anchors[i][1] = data.anchors[i].position.y
        anchors[i][2] = data.anchors[i].position.z



#callback to new UWB chatter position
def callback(data):
    global prev
    now = datetime.utcnow().timestamp()
    if  now - prev < 0.1:
        return
    prev = now 
    tagFullMsg = tagFull()
    rospy.loginfo( rospy.get_caller_id() + "[%f,%f,%f]", 
    data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z )

    tagFullMsg.estimate.position.x = data.transforms[0].transform.translation.x
    tagFullMsg.estimate.position.y = data.transforms[0].transform.translation.y
    tagFullMsg.estimate.position.z = data.transforms[0].transform.translation.z
    tagFullMsg.estimate.accuracy = 100
    tagFullMsg.estimate.le_us = 100
    tagFullMsg.estimate.timestamp = now
    tagFullMsg.estimate.valid = True


    pos[0] = tagFullMsg.estimate.position.x 
    pos[1] = tagFullMsg.estimate.position.y 
    pos[2] = tagFullMsg.estimate.position.z 

    #rospy.loginfo( "Simulated Ranges:" )

    i = 0
    while i < nAnchors:
        anchorMsg = anchor()
        anchorMsg.ID = str( i )
        anchorMsg.position.x = anchors[i][0]
        anchorMsg.position.y = anchors[i][1]
        anchorMsg.position.z = anchors[i][2]
        anchorMsg.range = distance( anchors[i], pos )
        anchorMsg.timestamp = now
        #rospy.loginfo( "\t%f", anchorMsg.range )
        #anchorMsg.range = results.x[i]
        tagFullMsg.anchors.append ( anchorMsg )
        i += 1
    tagFullMsg.nAnchors = nAnchors
    pub.publish ( tagFullMsg )


#setup of chatters to listen and callback
def listener():

    rospy.Subscriber("tf", TFMessage , callback)

    rospy.Subscriber('anchorConfig', anchorConfig , ConfigCallback)

    rospy.spin()
  
if __name__ == '__main__':
    rospy.init_node('simulation', anonymous=True)

    pub = rospy.Publisher('UWB', tagFull, queue_size=1)

    load_yaml()
   
    listener()
