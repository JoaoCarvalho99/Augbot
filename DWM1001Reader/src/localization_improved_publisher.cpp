#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cstdint"

#include "serial/serial.h"
#include "DWM1001Reader/anchor.h"
#include "DWM1001Reader/estimate.h"
#include "DWM1001Reader/tagFull.h"


void make_estimateMsg ( DWM1001Reader::tagFull* tagFullMsg, std::string args[] )
{
    tagFullMsg->estimative.timestamp = std::time(nullptr);;
    tagFullMsg->estimative.le_us = std::stoi(args[1]);
}

void make_finalMsg ( DWM1001Reader::tagFull* tagFullMsg, std::string args[] )
{
    tagFullMsg->estimative.position.x = std::stod(args[1]);
    tagFullMsg->estimative.position.y = std::stod(args[2]);
    tagFullMsg->estimative.position.z = std::stod(args[3]);
    tagFullMsg->estimative.accuracy = std::stoi(args[4]);
    tagFullMsg->timestamp = std::time(nullptr);
}

void make_AnchorMsg ( DWM1001Reader::tagFull* tagFullMsg, std::string args[], int nAnchor )
{
    DWM1001Reader::anchor anchorMsg;
    anchorMsg.timestamp = std::time(nullptr);   
    anchorMsg.ID = args[0];                     
    anchorMsg.position.x = std::stod(args[1]);  
    anchorMsg.position.y = std::stod(args[2]);  
    anchorMsg.position.z = std::stod(args[3]);  
    anchorMsg.range = std::stod(args[4]);       
    tagFullMsg->anchors[nAnchor] = anchorMsg;   
}

int parser( std::string input, DWM1001Reader::tagFull* tagFullMsg, int nAnchor )
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

DWM1001Reader::tagFull parser( std::string input )
{
    DWM1001Reader::tagFull tagFullMsg;
    std::string const delims{ " \n" };
    size_t beg, pos = 0;
    int i = 0;
    int flag = 0;
    while ((beg = input.find_first_not_of(delims, pos)) != std::string::npos)
    {
      pos = input.find_first_of(delims, beg + 1);
      flag = parser ( input.substr(beg, pos - beg), &tagFullMsg, i++ );
      if ( flag == 1 )
      {
        tagFullMsg.success = flag;
        return tagFullMsg;
      }
    }
    tagFullMsg.success = flag;
    return tagFullMsg;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<DWM1001Reader::tagFull>("localization", 1000);

    std::string str;
    std::string stdIn = "/dev/ttyACM0";
    serial::Serial ser;
    std::string input = "";

/*
    std::cout << "Enter port to be read:\nEnter to use [default]: /dev/ttyACM0\n";
    getline(std::cin, input);

    if ( input != "" )
    {
        stdIn = input;
    }
*/

    try
    {
        ser.setPort(stdIn);
        ser.setBaudrate(115200); //testar novo baudrate
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

    std::string test = "";

    while ( ros::ok() && ser.isOpen() )
    {

       while ( ser.available() )
       {

            test.append( ser.readline() );

        }

        if ( test != "" ){

            ROS_INFO("%s", test.c_str());

            DWM1001Reader::tagFull tagFullMsg = parser ( test );

            if ( tagFullMsg.success )
                chatter_pub.publish ( tagFullMsg );

            test = "";

        }

        ros::spinOnce();

    }


    ser.close();
    ROS_INFO_STREAM("USB PORT DISCONECTED");

    return 0;
}
