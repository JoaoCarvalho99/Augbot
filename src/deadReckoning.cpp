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

    double g = 0;
    int nG = 0;

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


    void reduceError( Eigen::Vector3d &acc);
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


//my function to calculate orientation from quaternion
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
    nG += 1;
    g += msg->linear_acceleration.z;
    //ROS_INFO ( "%f", g/nG);
    if (firstT) {
      time = msg->header.stamp;
      deltaT = 0;
      setGravity( msg->linear_acceleration );//msg->twist.twist.linear;
      pose.pos = Eigen::Vector3d(0, 0, 0);
      firstT = false;
    } else {
      deltaT = (msg->header.stamp - time).toSec();
      std::cout << time << std::endl;
      time = msg->header.stamp;
      pose.orien = setOrientation( msg->orientation );
      calcPosition( msg->angular_velocity, msg->linear_acceleration );
      std::cout << time << std::endl;
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


//MARTELADA
void ImuIntegrator::setGravity( const geometry_msgs::Vector3 &msg)
{
  gravity[0] =  msg.x;
  gravity[1] =  msg.y;
  gravity[2] =  msg.z;


/*  gravity[0] = abs ( msg.x );
  gravity[1] = abs ( msg.y);
  gravity[2] = abs ( msg.z );

  if ( gravity[0] > gravity[1] )
    gravity[1] = gravity[0];
  else gravity[0] = gravity[1];*/
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  path.points.push_back(p);
}

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

void ImuIntegrator::reduceError( Eigen::Vector3d &acc) {
  int i, total = 0;
  for ( i = 0; i < 3; i++ ){
    if ( abs( acc[i] ) <= 1.5 * abs ( gravity[i] ) ){
      acc[i] = 0;
      total+=1;
    }
    else {
      if ( acc[i] > 0 )
        acc[i] -= gravity[i];
      else if ( acc[i] < 0 )
        acc[i] += gravity[i];
    }
  }
  //if ( total == 3 )
  //  velocity = Eigen::Vector3d(0, 0, 0);
}

void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &vel, const geometry_msgs::Vector3 &acc) {
  Eigen::Vector3d acc_l(acc.x - gravity[0], acc.y - gravity[1], acc.z - gravity[2]);
  //Eigen::Vector3d acc_l ( acc.x, acc.y, acc.z);
  //reduceError ( acc_l );
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  ROS_INFO ( "vel [%f,%f,%f] ### acc [%f,%f,%f] --- grav [%f,%f,%f] === acc_l [%f,%f,%f] -> acc_g [%f,%f,%f]"
, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z, gravity[0], gravity[1], gravity[2], acc_l[0], acc_l[1], acc_l[2], acc_g[0], acc_g[1], acc_g[2] );
  velocity = velocity + deltaT * (acc_g);
  pose.pos = pose.pos + deltaT * velocity;

  ROS_INFO ("velocity [%f,%f,%f]", velocity[0], velocity[1], velocity[2]);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Imu_Integrator_node");
  ros::NodeHandle nh;
  ImuIntegrator *imu_integrator = new ImuIntegrator();//line);


  ros::Subscriber Imu_message = nh.subscribe("/imu", 1, &ImuIntegrator::ImuCallback, imu_integrator);
  // nh.subscribe("/imu", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  ros::spin();
}
