#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cstdint"

#include "serial/serial.h"
#include "Augbot/anchor.h"
#include "Augbot/estimate.h"
#include "Augbot/tagFull.h"


void make_estimateMsg ( Augbot::tagFull* tagFullMsg, std::string args[] )
{
    tagFullMsg->estimate.timestamp = std::time(nullptr);;
    tagFullMsg->estimate.le_us = std::stoi(args[1]);
}

void make_finalMsg ( Augbot::tagFull* tagFullMsg, std::string args[] )
{
    tagFullMsg->estimate.position.x = std::stod(args[1]);
    tagFullMsg->estimate.position.y = std::stod(args[2]);
    tagFullMsg->estimate.position.z = std::stod(args[3]);
    tagFullMsg->estimate.accuracy = std::stoi(args[4]);
    tagFullMsg->timestamp = std::time(nullptr);
    tagFullMsg->estimate.valid = true;
}

void make_AnchorMsg ( Augbot::tagFull* tagFullMsg, std::string args[], int nAnchor )
{
    Augbot::anchor anchorMsg;
    anchorMsg.timestamp = std::time(nullptr);   
    anchorMsg.ID = args[0];                     
    anchorMsg.position.x = std::stod(args[1]);  
    anchorMsg.position.y = std::stod(args[2]);  
    anchorMsg.position.z = std::stod(args[3]);  
    anchorMsg.range = std::stod(args[4]);       
    tagFullMsg->anchors[nAnchor] = anchorMsg;
    tagFullMsg->nAnchors = nAnchor;
}

int parser( std::string input, Augbot::tagFull* tagFullMsg, int nAnchor )
{
    std::string const delims{ ",[]= \n" };
    std::string args[10];
    int i = 0;
    size_t beg, pos = 0;
    while ((beg = input.find_first_not_of(delims, pos)) != std::string::npos)
    {
      pos = input.find_first_of(delims, beg + 1);
      args[i] = input.substr(beg, pos - beg);
      i++;
    }
    if ( args[0] == "es" ){
        ROS_WARN_STREAM ( " MISREAD FROM INPUT\n");
    }
    else if ( args[0] == "le_us" && i == 2  )
    {
        make_estimateMsg ( tagFullMsg, args );
    }
    else if ( args[0] == "est" && i == 6 )
    {
        make_finalMsg ( tagFullMsg, args );
        return 1;
    }
    else if ( i == 5 )
    {
        make_AnchorMsg ( tagFullMsg, args, nAnchor );
    }
    return 0;
}

Augbot::tagFull parser( std::string input )
{
    Augbot::tagFull tagFullMsg;
    std::string const delims{ " \n" };
    size_t beg, pos = 0;
    int i = 0;
    int flag = 0;
    while ((beg = input.find_first_not_of(delims, pos)) != std::string::npos)
    {
      pos = input.find_first_of(delims, beg + 1);
      flag = parser ( input.substr(beg, pos - beg), &tagFullMsg, i++ );
    }
    return tagFullMsg;
}


int main(int argc, char **argv)
{

    printf("%d\n",argc);

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<Augbot::tagFull>("localization", 1000);

    std::string stdIn = "/dev/ttyACM0";

    if ( argc > 1 )
        stdIn = argv[1];
    
    serial::Serial ser;
    std::string input = "";


    try
    {
        ser.setPort(stdIn);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch ( serial::IOException& e )
    {
        ROS_ERROR_STREAM("Unable to open port: " + stdIn );
        return -1;
    }

    if ( ser.isOpen() )
    {
        ROS_INFO_STREAM("Serial Port: " + stdIn + " initialized");
    } else
    {
        return -1;
    }


    while ( ros::ok() && ser.isOpen() )
    {

       while ( ser.available() )
       {

            input.append( ser.readline() );

        }

        if ( input != "" ){

            ROS_INFO("%s", input.c_str());

            Augbot::tagFull tagFullMsg = parser ( input );

            if ( tagFullMsg.estimate.valid )
                chatter_pub.publish ( tagFullMsg );

            input = "";

        }

        ros::spinOnce();

    }


    ser.close();
    ROS_INFO_STREAM("USB PORT DISCONECTED");

    return 0;
}
