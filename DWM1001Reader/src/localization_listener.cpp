#include "ros/ros.h"
#include "std_msgs/String.h"

#include "DWM1001Reader/tagFull.h"

//chamado quando recebe uma nova mensagem no topico chatter
void chatterCallback(const DWM1001Reader::tagFull::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f,%f,%f]", msg->estimate.position.x, msg->estimate.position.y, msg->estimate.position.z );
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  
  ros::NodeHandle n;

  //subscreve o topico localization (chama chatterCallback() smp que recebe nova mensagem)
  ros::Subscriber sub = n.subscribe("localization", 1000, chatterCallback);

  //entra no loop... chamando message callbacks o mais rapido possivel
  ros::spin();

  return 0;
}
