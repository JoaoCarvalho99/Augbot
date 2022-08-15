#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Augbot/tagFull.h"

ros::Publisher chatter_pub;

void chatterCallback(const Augbot::tagFull::ConstPtr& msg)
{
    chatter_pub.publish ( msg );
}

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "localization2UWB");

    ros::NodeHandle n;

  //subscreve o topico UWB (chama chatterCallback() smp que recebe nova mensagem)
    
    chatter_pub = n.advertise<Augbot::tagFull>("UWB", 1000);
    ros::Subscriber sub = n.subscribe("localization", 1000, chatterCallback);


  //entra no loop... chamando message callbacks o mais rapido possivel
    ros::spin();

    return 0;
}
