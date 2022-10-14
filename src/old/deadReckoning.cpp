// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cmath>

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

#include "Augbot/position.h"
#include <sensor_msgs/Imu.h>

/*
deadReckoning used to estimate position of alphabot2 in the simulation
Listens to /imu chat from gazebo imu plugin (orientation - quaternion, linear acceleration(x,y,z) m/s² )
*/


struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos; 
    Eigen::Matrix3d orien;
};

class ImuIntegrator
{
private:
    Pose pose;
    ros::Time time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;
    visualization_msgs::Marker path;

    double prev = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    Augbot::position pubMsg = Augbot::position();

    ros::NodeHandle n;
    ros::Publisher line_pub = n.advertise<Augbot::position>("deadReckoning", 1);

    double deltaT;
    bool firstT;
public:
    //! Constructor.
    ImuIntegrator();
    //! Destructor.
    ~ImuIntegrator();

    //! Callback function for dynamic reconfigure server.
    //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

    //! Publish the message.
    void publishMessage();

    //! Callback function for subscriber.
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg); //reads message from imu

    void setGravity(const geometry_msgs::Vector3 &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::Vector3 &vel, const geometry_msgs::Vector3 &acc);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
};

ImuIntegrator::ImuIntegrator() {
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  velocity = zero;
  firstT = true;

  // Line strip is blue
  path.color.b = 1.0;
  path.color.a = 1.0;
  path.type = visualization_msgs::Marker::LINE_STRIP;
  path.header.frame_id = "/global";
  path.ns = "points_and_lines";
  path.action = visualization_msgs::Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = 0.2;
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  path.points.push_back(p);
}


//function to calculate orientation from quaternion
Eigen::Matrix3d setOrientation (const geometry_msgs::Quaternion& msg){
  float q0 = msg.x;
  float q1 = msg.y;
  float q2 = msg.z;
  float q3 = msg.w;
  Eigen::Matrix3d orien = Eigen::Matrix3d();
   
  orien << 1 - 2*q1*q1 - 2*q2*q2,	2*q0*q1 - 2*q2*q3,	2*q0*q2 + 2*q1*q3,
            2*q0*q1 + 2*q2*q3,	1 - 2*q0*q0 - 2*q2*q2,	2*q1*q2 - 2*q0*q3,
            2*q0*q2 - 2*q1*q3,	2*q1*q2 + 2*q0*q3,	1 - 2*q0*q0 - 2*q1*q1;

  return orien;
}

void ImuIntegrator::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (firstT) {
      time = msg->header.stamp;
      deltaT = 0;
      setGravity( msg->linear_acceleration );//msg->twist.twist.linear;
      pose.pos = Eigen::Vector3d(0, 0, 0);
      firstT = false;
    } else {
      deltaT = (msg->header.stamp - time).toSec();
      //std::cout << deltaT << std::endl;
      time = msg->header.stamp;
      pose.orien = setOrientation( msg->orientation );
      calcPosition( msg->angular_velocity, msg->linear_acceleration );
      updatePath(pose.pos);

      auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

     if ( now - prev > 100 ) {
        prev = now;

        pubMsg.x = pose.pos[0];
        pubMsg.y = pose.pos[1];
        pubMsg.z = pose.pos[2];

        ROS_INFO ( "[%f,%f,%f]", pubMsg.x, pubMsg.y, pubMsg.z);
        
        publishMessage();
      }
    }
}


//MARTELADA - not used
void ImuIntegrator::setGravity( const geometry_msgs::Vector3 &msg)
{
  gravity[0] =  0;//msg.x;
  gravity[1] =  0;//msg.y;
  gravity[2] =  msg.z;
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  path.points.push_back(p);
}

//publishes position estimated each 100ms to /deadReckoning
void ImuIntegrator::publishMessage ()
{
  line_pub.publish(pubMsg); 
}


//not used
void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3 &msg) {
  Eigen::Matrix3d B;
  B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT, -msg.y * deltaT, msg.x * deltaT, 0;
  double sigma = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) * deltaT;
  pose.orien = pose.orien * (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B - ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
}


//estimates position
void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &vel, const geometry_msgs::Vector3 &acc) {
  Eigen::Vector3d acc_l(acc.x, acc.y, acc.z);
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  gravity = Eigen::Vector3d ( 0, 0, acc_g[2] ); //to negate acceleration in z (robot is always at constant z)
  velocity = velocity + deltaT * (acc_g - gravity);
  pose.pos = pose.pos + deltaT * velocity;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Imu_Integrator_node");
  ros::NodeHandle nh;
  ImuIntegrator *imu_integrator = new ImuIntegrator();//line);


  ros::Subscriber Imu_message = nh.subscribe("/imu", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  // nh.subscribe("/imu", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  ros::spin();
}
