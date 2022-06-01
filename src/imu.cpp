#include "ros/ros.h"
#include "ros/time.h"
#include <sensor_msgs/Imu.h>
#include "cstdint"
#include <nlohmann/json.hpp>

#include "serial/serial.h"

struct Quaternion
{
    double w, x, y, z;
};

void load_yaml(std::string &stdIn){
   if (ros::param::has("port")) {
        ros::param::get("port", stdIn);
   }

}

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

void parser( std::string input, ros::Publisher chatter_pub )
{
    sensor_msgs::Imu imuMsg = sensor_msgs::Imu();

    ROS_INFO ( "%s", input.c_str() );
    nlohmann::json data = nlohmann::json::parse(input);
    geometry_msgs::Quaternion q = ToQuaternion ( data["yaw"], data["pitch"], data["roll"]);

    imuMsg.orientation = q;
    imuMsg.linear_acceleration.x = data["accel_x"];
    imuMsg.linear_acceleration.y = data["accel_y"];
    imuMsg.linear_acceleration.x = data["accel_z"];

    imuMsg.header.stamp = ros::Time ( data["timestamp"] );

    chatter_pub.publish( imuMsg );
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "Imu_publisher");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("/imu", 1);

    std::string stdIn = "/dev/ttyACM1";//ver isto

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
        ROS_INFO_STREAM("IMU: Serial Port: " + stdIn + " initialized");
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

    ros::spin();
}




