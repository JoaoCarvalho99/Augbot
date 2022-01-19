#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <sstream>

#include "Augbot/anchor.h"
#include "Augbot/estimate.h"
#include "Augbot/tagFull.h"


/**
 * @brief 
 *                                                Previous version of localization_improved_publisher.cpp
 * 
 * 
 */


int parser( std::string input, std::string args[] ) //retorna 0 se anchor, 1 se for le_us, 2 se for est
{
  std::string const delims{ ",[]= \n" };
  int i = 0;
  size_t beg, pos = 0;
  while ((beg = input.find_first_not_of(delims, pos)) != std::string::npos)
  {
    pos = input.find_first_of(delims, beg + 1);
    args[i] = input.substr(beg, pos - beg);
    i++;
  }
  if ( args[0] == "le_us" )
    return 1;
  if ( args[0] == "est" )
    return 2;
  return 0;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  //mensagens do tipo std_msg/String no topico chatter
  //valor int é o numero de mensagens a guardar
  ros::Publisher chatter_pub = n.advertise<Augbot::tagFull>("localization", 1000);

  //corre a 10Hz, frequencia do loop
  ros::Rate loop_rate(10);

  //conta mensagens enviadas
  int count = 0;

  std::string str;
  std::string str_aux;
  std::fstream f;
  f.open("/dev/ttyACM0");

  Augbot::tagFull tagFullMsg;
  Augbot::estimate estimateMsg;
  int nAnchor = 0;

  while ( f )
  {
    f >> str;
   
    std_msgs::String msg;

    std::stringstream ss;
    //ss << str << " " << count;
    ss << str;
    msg.data = ss.str();

    std::string args[10];
    int final = parser (msg.data, args);

    if ( final == 1 ) //estimativa
    {
      estimateMsg = Augbot::estimate();
      estimateMsg.timestamp = std::time(nullptr);;
      estimateMsg.le_us = std::stoi(args[1]);
    } else if ( final == 2 )
    {
      estimateMsg.position.x = std::stod(args[1]);
      estimateMsg.position.y = std::stod(args[2]);
      estimateMsg.position.z = std::stod(args[3]);
      estimateMsg.accuracy = std::stoi(args[4]);
      tagFullMsg.timestamp = std::time(nullptr);;
      tagFullMsg.estimate = estimateMsg;
      chatter_pub.publish(tagFullMsg);
      tagFullMsg = Augbot::tagFull();
      nAnchor = 0;
    } else //anchor
    {
      Augbot::anchor anchorMsg;
      anchorMsg.timestamp = std::time(nullptr);
      anchorMsg.ID = args[0];
      anchorMsg.position.x = std::stod(args[1]);
      anchorMsg.position.y = std::stod(args[2]);
      anchorMsg.position.z = std::stod(args[3]);
      anchorMsg.range = std::stod(args[4]);
      tagFullMsg.anchors[nAnchor] = anchorMsg;
      nAnchor++;
    }
    
    //"printf"
    ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    //publica msg a 10Hz (10 mensagens por segundo) -> loop_rate = 10
    //loop_rate.sleep();
    //++count;
  }

  ROS_INFO_STREAM("USB PORT DISCONECTED");

  return 0;
}
