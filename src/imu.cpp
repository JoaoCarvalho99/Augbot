#include "ros/ros.h"
#include "ros/time.h"
#include <sensor_msgs/Imu.h>
#include "cstdint"
#include <nlohmann/json.hpp>

#include <Augbot/orientation.h>

#include "serial/serial.h"

/*
reads orientation and acceleration for $sensor, from $port with $offset and sends info to /imu
*/

struct Quaternion
{
    double w, x, y, z;
};

void load_yaml(ros::NodeHandle n, std::string &stdIn, std::string &sensor, int &offset){
    if ( n.hasParam("port")) {
        n.getParam( "port", stdIn);
    }
    if ( n.hasParam("sensor")) {
        n.getParam( "sensor", sensor);
    }
    if ( n.hasParam("offset")) {
        n.getParam( "offset", offset);
        ROS_INFO ( "offset: %d", offset);
    }
}


//bit shift for pipico - returns in millig
double AccToMillig (double value ) {
    float constant  = 1000.0/16384.0;
    return value * constant;
}


//converts acceleration from millig to m/s²
double milligToMSsquare (double value ) {
    double constant  = 0.00980665;
    ROS_INFO ( "value: %f", value);
    return value * constant;
}

//Degree to randians
double headingToYaw (double heading, int offset){
    heading += offset;
    if ( heading > 180 )
        heading -= 360;
    ROS_INFO ( "heading %f", heading);
    float constant = M_PI / 180;
    return heading * constant;
}

//roll pitch yaw to quaternion
geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q = geometry_msgs::Quaternion();
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

void parser( std::string input, ros::Publisher chatter_pub, std::string sensor, ros::Publisher chatter_pub1, int offset)
{
    sensor_msgs::Imu imuMsg = sensor_msgs::Imu();

    ROS_INFO ( "%s", input.c_str() );
    nlohmann::json data;

    if ( nlohmann::json::accept ( input ) ){
        data = nlohmann::json::parse(input);
    }
    else {
        ROS_INFO("Unable to parse: %s", input.c_str() );
        return;
    }
    geometry_msgs::Quaternion q;
    if ( sensor == "micro:bit" ){
        q = ToQuaternion ( headingToYaw(data["heading"], offset), 0, 0 );
        imuMsg.linear_acceleration.x = milligToMSsquare( data["accel_x"] );

        //orientation
        Augbot::orientation msg = Augbot::orientation();
        msg.yaw = data["heading"];
        msg.accel_x = milligToMSsquare ( data["accel_x"] );
        msg.accel_y = milligToMSsquare ( data["accel_y"] );
        msg.accel_z = milligToMSsquare ( data["accel_z"] );
        msg.timestamp = std::time(nullptr);
        chatter_pub1.publish( msg );
    }
    if ( sensor == "pi:pico" ) {
        q = ToQuaternion ( data["yaw"], data["pitch"], data["roll"] );
        imuMsg.linear_acceleration.x = milligToMSsquare( AccToMillig ( data["accel_y"] ) );
    }

    imuMsg.orientation = q;
    imuMsg.linear_acceleration.y = milligToMSsquare( data["accel_y"] ); //TROCAR X POR Y (ATENCAO)
    
    imuMsg.linear_acceleration.z = milligToMSsquare( data["accel_z"] );
    float timestamp = int(data["timestamp"])/1000 + (int(data["timestamp"])%1000)/1000.0 ;

    imuMsg.header.stamp = ros::Time ( timestamp );
    chatter_pub.publish( imuMsg );
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "Imu_publisher");
    ros::NodeHandle n("~");

    ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("/imu", 1000);
    ros::Publisher chatter_pub1 = n.advertise<Augbot::orientation>("/microbit", 1000);

    std::string stdIn = "/dev/ttyACM0";
    std::string sensor = "micro:bit";//"pi:pico";
    int offset = 10;

    load_yaml(n, stdIn, sensor, offset);

    std::cout << "param:" << std::endl;
    std::cout << stdIn << std::endl;
    std::cout << offset << std::endl;


    serial::Serial ser;
    std::string input = "";

    std::cout << "sensor: " + sensor << std::endl;


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
        ROS_INFO_STREAM("IMU: Serial Port: " + stdIn + " initialized");
    } else
    {
        return -1;
    }


    while ( ros::ok() && ser.isOpen() )
    {

       if ( ser.available() )
       {
            while ( ser.available() )
                input.append( ser.readline() );

            ROS_INFO("%s", input.c_str());

            parser ( input, chatter_pub, sensor, chatter_pub1, offset );

            input = "";

        }
        else ros::Duration(0.01).sleep();

    }


    ser.close();
    ROS_INFO_STREAM("USB PORT DISCONECTED");

    return 0;

    ros::spin();
}




