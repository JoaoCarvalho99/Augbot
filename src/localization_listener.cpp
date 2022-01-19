#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Augbot/tagFull.h"


/**
 * @brief 
 *                                                     Reads data from 'localization' in the form of tagFull
 */




/**
 * @brief callback for 'localization' chatter
 * 
 * @param msg tagFull message received
 */
void chatterCallback(const Augbot::tagFull::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f,%f,%f]", msg->estimate.position.x, msg->estimate.position.y, msg->estimate.position.z );
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "localization_listener");

  
  ros::NodeHandle n;

  //subscreve o topico localization (chama chatterCallback() smp que recebe nova mensagem)
  ros::Subscriber sub = n.subscribe("localization", 1000, chatterCallback);

  //entra no loop... chamando message callbacks o mais rapido possivel
  ros::spin();

  return 0;
}
