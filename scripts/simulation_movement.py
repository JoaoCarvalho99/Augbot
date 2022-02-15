#!/usr/bin/env python
import rospy
import rosbag
from datetime import datetime

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from Augbot.msg import position

prev = 0
global bag


def callback(data):
    global prev
    now = datetime.utcnow().timestamp()
    if  now - prev < 1:
        return
    prev = now 
    rospy.loginfo( rospy.get_caller_id() + "[%f,%f,%f]", 
    data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z )
    pos = position()
    pos = data.transforms[0].transform.translation
    bag.write( "position", pos )
    
def move():
    global twist
    info = String()
    twist.linear.x = 0
    pub.publish( twist )
    rospy.loginfo("STOP")
    info.data = "STOP"
    bag.write( 'info', info )
    rospy.sleep ( 5 )

    twist.linear.x = 1
    pub.publish ( twist )
    rospy.loginfo("moving forward")
    info.data = "forward"
    bag.write( 'info', info )
    rospy.sleep ( 10 )

    twist.linear.x = 0
    pub.publish( twist )
    rospy.loginfo("STOP")
    info.data = "STOP"
    bag.write( 'info', info )
    rospy.sleep ( 5 )

    twist.linear.x = -1
    pub.publish ( twist )
    rospy.loginfo("moving backwards")
    info.data = "backward"
    bag.write( 'info', info )
    rospy.sleep ( 10 )


if __name__ == '__main__':

    rospy.init_node('gazebo_move', anonymous=True)
  
    pub = rospy.Publisher('/alphabot2/control', Twist, queue_size=10 )

    twist = Twist()

    name = "rosbag/simulation/positions/" + str( datetime.now() ) + ".bag"
    name.replace(" ", "_")
    bag = rosbag.Bag( name, 'w' )

    rospy.Subscriber("tf", TFMessage , callback)

    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    i = 0
    while i<10:
        move()
        i += 1
    bag.close()