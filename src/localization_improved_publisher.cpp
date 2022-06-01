#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cstdint"

#include "serial/serial.h"
#include "Augbot/anchor.h"
#include "Augbot/estimate.h"
#include "Augbot/tagFull.h"


/**
 * @brief 
 * 
 *                                               Reads data from serial port connection with DWM1001, 
 *                                          transforms data into ROS custom messages (final one is tagFull) 
 *                                                      and publishes into 'localization'
 * 
 */


void load_yaml(std::string &stdIn){
   if (ros::param::has("/port")) {
        ros::param::get("/port", stdIn);
   }
}


/**
 * @brief 
 * 
 * @param tagFullMsg final message to publish
 * @param args args[0] ="le_us", args[1] = value of le_us
 */
void make_estimateMsg ( Augbot::tagFull* tagFullMsg, std::string args[] )
{
    tagFullMsg->estimate.timestamp = std::time(nullptr);;
    tagFullMsg->estimate.le_us = std::stoi(args[1]);
}



/**
 * @brief 
 * 
 * @param tagFullMsg final message to publish ( now finished )
 * @param args args[0] = "est"
 *             args[1] = x
 *             args[2] = y
 *             args[3] = z
 *             args[4] = df
 */
void make_finalMsg ( Augbot::tagFull* tagFullMsg, std::string args[] )
{
    tagFullMsg->estimate.position.x = std::stod(args[1]);
    tagFullMsg->estimate.position.y = std::stod(args[2]);
    tagFullMsg->estimate.position.z = std::stod(args[3]);
    tagFullMsg->estimate.accuracy = std::stoi(args[4]);
    tagFullMsg->timestamp = std::time(nullptr);
    tagFullMsg->estimate.valid = true;
}



/**
 * @brief 
 * 
 * @param tagFullMsg final message to publish
 * @param args all related to the anchor: args[0] = ID
 *                                        args[1] = x
 *                                        args[2] = y
 *                                        args[3] = z
 *                                        args[4] = range calculated between anchor and tag
 * @param nAnchor
 */
void make_AnchorMsg ( Augbot::tagFull* tagFullMsg, std::string args[], int nAnchor )
{
    Augbot::anchor anchorMsg;
    anchorMsg.timestamp = std::time(nullptr);   
    anchorMsg.ID = args[0];                     
    anchorMsg.position.x = std::stod(args[1]);  
    anchorMsg.position.y = std::stod(args[2]);  
    anchorMsg.position.z = std::stod(args[3]);  
    anchorMsg.range = std::stod(args[4]);       
    tagFullMsg->anchors.push_back ( anchorMsg );
    //tagFullMsg->anchors[nAnchor] = anchorMsg;
    tagFullMsg->nAnchors = nAnchor + 1;
}



/**
 * @brief 
 * 
 * @param input data read from the serial port
 * @param tagFullMsg 
 * @param nAnchor
 * @return int flag, 1 if estimation was successfull, 0 if not
 */
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


/**
 * @brief 
 * 
 * @param input data read from the serial port connection with dwm1001 and publishes tagFull to chatter 'localization"
 */
void parser( std::string input, ros::Publisher chatter_pub )
{
    Augbot::tagFull tagFullMsg;
    std::string const delims{ " \n" };
    size_t beg, pos = 0;
    int i = 0;
    int flag = 0;
    ROS_INFO ( "%s", input.c_str() );
    while ((beg = input.find_first_not_of(delims, pos)) != std::string::npos)
    {
        pos = input.find_first_of(delims, beg + 1);
        flag = parser ( input.substr(beg, pos - beg), &tagFullMsg, i++ );
        if ( flag == 1)
        {
            if ( tagFullMsg.estimate.valid )
            {
                chatter_pub.publish ( tagFullMsg );

            }

            tagFullMsg = Augbot::tagFull();
            i = 0;
        }
    }
}


int main(int argc, char **argv)
{

    printf("%d\n",argc);

    ros::init(argc, argv, "dwm1001Reader");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<Augbot::tagFull>("localization", 1000);

    std::string stdIn = "/dev/ttyACM0";

    load_yaml(stdIn);
    
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
        ROS_INFO_STREAM("DWM: Serial Port: " + stdIn + " initialized");
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

            parser ( input, chatter_pub );

            input = "";

        }

        ros::spinOnce();

    }


    ser.close();
    ROS_INFO_STREAM("USB PORT DISCONECTED");

    return 0;
}
