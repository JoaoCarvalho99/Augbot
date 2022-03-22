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

#callback to alphabot2 position (not used)
def callback(data):
    global prev
    now = datetime.utcnow().timestamp()
    if  now - prev < 1:
        return
    prev = now 
    rospy.loginfo( rospy.get_caller_id() + "[%f,%f,%f]", 
    data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z )
    #pos = position()
    #pos = data.transforms[0].transform.translation
    #bag.write( "position", pos )


def move():
    global twist
    twist.linear.x = 0
    pub.publish( twist )
    rospy.loginfo("STOP")
    rospy.sleep ( 5 )

    twist.linear.x = 0.2
    pub.publish ( twist )
    rospy.loginfo("moving forward")
    rospy.sleep ( 30 )

    twist.linear.x = 0
    pub.publish( twist )
    rospy.loginfo("STOP")
    rospy.sleep ( 5 )

    twist.linear.x = -0.2
    pub.publish ( twist )
    rospy.loginfo("moving backwards")
    rospy.sleep ( 30 )


if __name__ == '__main__':

    rospy.init_node('gazebo_move', anonymous=True)
  
    pub = rospy.Publisher('/alphabot2/control', Twist, queue_size=10 )

    twist = Twist()

    #rospy.Subscriber("tf", TFMessage , callback)

    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    i = 0
    while i<1000:
        move()
        i += 1

    info = String()
    twist.linear.x = 0
    pub.publish( twist )
    rospy.loginfo("STOP")