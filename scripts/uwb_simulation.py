#!/usr/bin/env python
from distutils.log import error
from time import time
import rospy
from Augbot.msg import tagFull, anchor, anchorConfig
import numpy as np
from datetime import datetime

from tf2_msgs.msg import TFMessage

#                                           reads position of simulated alphabot2 'tf'
#                                           calculates least_squares with data received
#                                           sends tagFull to chatter "simulation"




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

#equation for 5 exactly anchors (least_squares)
#def equations ( guess ):
#    r1, r2, r3, r4, r5 = guess
#
#    return (
#        (pos[0] - anchors[0][0])**2 + (pos[1] - anchors[0][1])**2 + (pos[2] - anchors[0][2])**2 - (r1 )**2,
#        (pos[0] - anchors[1][0])**2 + (pos[1] - anchors[1][1])**2 + (pos[2] - anchors[1][2])**2 - (r2 )**2,
#        (pos[0] - anchors[2][0])**2 + (pos[1] - anchors[2][1])**2 + (pos[2] - anchors[2][2])**2 - (r3 )**2,
#        (pos[0] - anchors[3][0])**2 + (pos[1] - anchors[3][1])**2 + (pos[2] - anchors[3][2])**2 - (r4 )**2,
#        (pos[0] - anchors[4][0])**2 + (pos[1] - anchors[4][1])**2 + (pos[2] - anchors[4][2])**2 - (r5 )**2,
#    )


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


def noise ( ):
    global mu, sigma
    n = np.random.default_rng().normal(mu, sigma, 1)
    return n[0]

def distance ( anchor, pos ):
    return ( ( pos[0] - anchor[0] )**2 + ( pos[1] - anchor[1] )**2 + ( pos[2] - anchor[2] )**2 )**( 1/2 ) + noise ()


def ConfigCallback ( data ):
    global anchors, nAnchors

    nAnchors = data.nAnchors

    rospy.loginfo ( "anchors changed" )

    for i in range ( data.nAnchors ):
        anchors[i][0] = data.anchors[i].position.x
        anchors[i][1] = data.anchors[i].position.y
        anchors[i][2] = data.anchors[i].position.z




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

    pos[0] = tagFullMsg.estimate.position.x 
    pos[1] = tagFullMsg.estimate.position.y 
    pos[2] = tagFullMsg.estimate.position.z 

    #results = least_squares(equations, initial_guess)

    rospy.loginfo( "Simulated Ranges:" )

    i = 0
    while i < nAnchors:
        anchorMsg = anchor()
        anchorMsg.ID = str( i )
        anchorMsg.position.x = anchors[i][0]
        anchorMsg.position.y = anchors[i][1]
        anchorMsg.position.z = anchors[i][2]
        anchorMsg.range = distance( anchors[i], pos )
        rospy.loginfo( "\t%f", anchorMsg.range )
        #anchorMsg.range = results.x[i]
        tagFullMsg.anchors.append ( anchorMsg )
        i += 1
    tagFullMsg.nAnchors = nAnchors
    pub.publish ( tagFullMsg )


    
def listener():

    rospy.Subscriber("tf", TFMessage , callback)

    rospy.Subscriber('anchorConfig', anchorConfig , ConfigCallback)

    rospy.spin()
  
if __name__ == '__main__':
    rospy.init_node('simulation', anonymous=True)

    pub = rospy.Publisher('localization', tagFull, queue_size=1)

    #initial_guess = [ 1.0, 1.0, 1.0, 1.0, 1.0 ]

    load_yaml()
   
    listener()
