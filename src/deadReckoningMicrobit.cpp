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
#include "Augbot/synchPoint.h"
#include <sensor_msgs/Imu.h>

/*
deadReckoning used to estimate position of alphabot2
Listens to /imu chat imu.cpp - received by microbit (orientation - quaternion,
 linear acceleration(x,y,z) m/s² -> since microbit gives wrong acceleration, constant speed was used ( 0.15m/s) )
*/

struct Pose
{
    Eigen::Vector3d pos; 
    Eigen::Matrix3d orien;
};

class ImuIntegrator
{
private:
    Pose pose;
    Pose pose1;
    Pose pose2;
    Eigen::Vector3d synchPose;
    ros::Time time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;
    visualization_msgs::Marker path;

    int synchPoint = 0;
    

    double prev = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    Augbot::position pubMsg = Augbot::position();
    Augbot::position pubMsg1 = Augbot::position();
    Augbot::position pubMsg2 = Augbot::position();

    ros::NodeHandle n;
    ros::Publisher line_pub = n.advertise<Augbot::position>("deadReckoning", 1);
    ros::Publisher line_pub1 = n.advertise<Augbot::position>("deadReckoning1", 1);
    ros::Publisher line_pub2 = n.advertise<Augbot::position>("deadReckoning2", 1);

    double deltaT;
    bool firstT;
public:
    //! Constructor.
    ImuIntegrator();
    //! Destructor.
    ~ImuIntegrator();

    //! Publish the message.
    void publishMessage();

    //! Callback function for subscriber.
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg); //reads message from imu

    void setGravity(const geometry_msgs::Vector3 &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition( const geometry_msgs::Vector3 &acc );
    void calcOrientation(const geometry_msgs::Vector3 &msg);
    
    void synchPointCallback ( const Augbot::synchPoint &msg ); 
    void load_yaml(ros::NodeHandle n);
};

ImuIntegrator::ImuIntegrator() {
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose1.pos = zero;
  pose2.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  pose1.orien = Eigen::Matrix3d::Identity();
  pose2.orien = Eigen::Matrix3d::Identity();
  velocity = zero;
  firstT = true;


  synchPoint = 0;
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

void ImuIntegrator::load_yaml(ros::NodeHandle nh){
    double x, y;
    if ( nh.hasParam("synchY") and nh.hasParam("synchX")) {
        nh.getParam( "synchY", y);
        nh.getParam( "synchX", x);
        synchPose = Eigen::Vector3d( x, y, 0);
    }
}

void ImuIntegrator::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (firstT) {
      time = msg->header.stamp;
      deltaT = 0;
      pose.pos = Eigen::Vector3d(0, 0, 0);
      pose1.pos = Eigen::Vector3d(0, 0, 0);
      pose2.pos = Eigen::Vector3d(0, 0, 0);
      setGravity( msg->linear_acceleration );
      firstT = false;
    } else {
      deltaT = (msg->header.stamp - time).toSec();
      //std::cout << time << std::endl;
      time = msg->header.stamp;
      pose.orien = setOrientation( msg->orientation );
      calcPosition( msg->linear_acceleration );
      //std::cout << time << std::endl;
      updatePath(pose.pos);

      auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

     if ( now - prev > 100 ) {
        prev = now;

        pubMsg.x = pose.pos[0];
        pubMsg.y = pose.pos[1];
        pubMsg.z = pose.pos[2];

        ROS_INFO ( "deadReckoning: [%f,%f,%f]", pubMsg.x, pubMsg.y, pubMsg.z);
        
        pubMsg1.x = pose1.pos[0];
        pubMsg1.y = pose1.pos[1];
        pubMsg1.z = pose1.pos[2];

        ROS_INFO ( "deadReckoning1: [%f,%f,%f]", pubMsg1.x, pubMsg1.y, pubMsg1.z);

        pubMsg2.x = pose2.pos[0];
        pubMsg2.y = pose2.pos[1];
        pubMsg2.z = pose2.pos[2];

        ROS_INFO ( "deadReckoning2: [%f,%f,%f]", pubMsg2.x, pubMsg2.y, pubMsg2.z);

        publishMessage();
      }
    }
}

void ImuIntegrator::synchPointCallback ( const Augbot::synchPoint &msg) {
  //if ( msg.diff > 9000 )
  synchPoint ++;
  if ( synchPoint == 5 ) {
    pose.pos = synchPose; 
    synchPoint = 1;
  }
}

//not used
void ImuIntegrator::setGravity( const geometry_msgs::Vector3 &msg)
{
  gravity[0] =  msg.x;
  gravity[1] =  msg.y;
  gravity[2] =  msg.z;

}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  path.points.push_back(p);
}


//publishes position estimated each 100ms to /deadReckoning (constant speed) and /deadReckoning (acceleration by microbit - wrong)
void ImuIntegrator::publishMessage ()
{
  line_pub.publish(pubMsg); 
  line_pub1.publish(pubMsg1); 
  line_pub2.publish(pubMsg2); 
}



void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &acc) {
    Eigen::Vector3d speed( 0.15, 0, 0 );
    Eigen::Vector3d velocity1 = pose.orien * speed;
    pose.pos = pose.pos + deltaT * velocity1;
    pose1.pos = pose1.pos + deltaT * velocity1;
    //position estimated with constant speed (m/s)
  //  ROS_INFO ("velocity [%f,%f,%f]", velocity1[0], velocity1[1], velocity1[2]);

    Eigen::Vector3d acc_l(acc.x, acc.y, acc.z);
    Eigen::Vector3d acc_g = pose.orien * acc_l;
    gravity = Eigen::Vector3d ( 0, 0, acc_g[2] );
    velocity = velocity + deltaT * (acc_g - gravity);
    pose2.pos = pose2.pos + deltaT * velocity;
    //position estimated by microbit's accelerations (m/s²)
  //  ROS_INFO ("velocity1 [%f,%f,%f]", velocity[0], velocity[1], velocity[2]);

/*    Eigen::Vector3d acc_l(acc.x, acc.y, acc.z);
    //Eigen::Vector3d acc_g = pose.orien * acc_l;
    //gravity = Eigen::Vector3d ( 0, 0, acc_l[2] );
    velocity = velocity + deltaT * acc_l;
    Eigen::Vector3d velocity_l =  pose.orien * velocity;
    velocity_l[2] = 0;
    pose1.pos = pose1.pos + deltaT * velocity_l;
    //position estimated by microbit's accelerations (m/s²)
    ROS_INFO ("velocity1 [%f,%f,%f]", velocity_l[0], velocity_l[1], velocity_l[2]);*/
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Imu_Integrator_node_Microbit");
  ros::NodeHandle nh("~");
  ImuIntegrator *imu_integrator = new ImuIntegrator();//line);

  imu_integrator->load_yaml(nh);

  ros::Subscriber Imu_message = nh.subscribe("/imu", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  ros::Subscriber synchPoint_message = nh.subscribe("/synchPoints", 10, &ImuIntegrator::synchPointCallback, imu_integrator);
  // nh.subscribe("/imu", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  ros::spin();
}
