// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cmath>

#include "Augbot/deadReckoning.h"
#include "Augbot/position.h"

struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos;
    Eigen::Vector3d teste;
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

    Augbot::deadReckoning pubMsg = Augbot::deadReckoning();

    ros::NodeHandle n;
    ros::Publisher line_pub = n.advertise<Augbot::deadReckoning>("deadReckoning", 1000);

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
    void ImuCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void setGravity(const geometry_msgs::Vector3 &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::Vector3 &msg);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
};

ImuIntegrator::ImuIntegrator() {
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.teste = zero;
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

Eigen::Matrix3d setOrientation (const geometry_msgs::Quaternion& msg){
  float q0 = msg.x;
  float q1 = msg.y;
  float q2 = msg.z;
  float q3 = msg.w;
  Eigen::Matrix3d orien = Eigen::Matrix3d();
   
  orien << 2 * (q0 * q0 + q1 * q1) - 1 , 2 * (q1 * q2 - q0 * q3),     2 * (q1 * q3 + q0 * q2),
                2 * (q1 * q2 + q0 * q3)     , 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1),
                2 * (q1 * q3 - q0 * q2)     , 2 * (q2 * q3 + q0 * q1),     2 * (q0 * q0 + q3 * q3) - 1;

  std::cout << "new orien: " << std::endl;
  std::cout << orien << std::endl;

  return orien;
}

void ImuIntegrator::ImuCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::cout << msg->twist.twist << std::endl;
    std::cout << msg->pose.pose << std::endl;
    Augbot::position posError = Augbot::position();
    if (firstT) {
      time = msg->header.stamp;
      deltaT = 0;
      //setGravity();//msg->twist.twist.linear;
      pose.pos = Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
      pose.teste = pose.pos;
      firstT = false;
    } else {
      deltaT = (msg->header.stamp - time).toSec();
      time = msg->header.stamp;
      setGravity ( msg->twist.twist.angular );
      calcOrientation(msg->twist.twist.angular);
      pose.orien = setOrientation( msg->pose.pose.orientation );
      calcPosition(msg->twist.twist.linear);
      ROS_INFO ( "deltaT = %f", deltaT);
      updatePath(pose.pos);
      ROS_INFO ( "ERROR [%f,%f,%f]", msg->pose.pose.position.x - pose.pos[0] ,msg->pose.pose.position.y - pose.pos[1] ,
      msg->pose.pose.position.z - pose.pos[2] );

      posError.x = msg->pose.pose.position.x - pose.pos[0];
      posError.y = msg->pose.pose.position.y - pose.pos[1];
      posError.z = msg->pose.pose.position.z - pose.pos[2];
      pubMsg.prev_error = posError;

      ROS_INFO("[%f,%f,%f]", pose.teste[0],pose.teste[1],pose.teste[2]);
      ROS_INFO ( "ERROR [%f,%f,%f]", msg->pose.pose.position.x - pose.teste[0] ,msg->pose.pose.position.y - pose.teste[1] ,
      msg->pose.pose.position.z - pose.teste[2] );

      posError.x = msg->pose.pose.position.x - pose.teste[0];
      posError.y = msg->pose.pose.position.y - pose.teste[1];
      posError.z = msg->pose.pose.position.z - pose.teste[2];
      pubMsg.justVel_error =  posError;

      publishMessage();
    }
}

void ImuIntegrator::setGravity( const geometry_msgs::Vector3 &msg) {
  gravity[0] = 0;//msg.x;
  gravity[1] = 0;//msg.y;
  gravity[2] = 0;//msg.z;
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  ROS_INFO("[%f,%f,%f]", msg[0],msg[1],msg[2]);
  path.points.push_back(p);
}

void ImuIntegrator::publishMessage() { line_pub.publish(pubMsg); }

void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3 &msg) {
  Eigen::Matrix3d B;
  B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT, -msg.y * deltaT, msg.x * deltaT, 0;
  double sigma = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) * deltaT;
  //std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
  // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;
  pose.orien = pose.orien * (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B - ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
  std::cout << pose.orien << std::endl;
}

void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
  // gravity[2]);
  velocity = velocity + deltaT * (acc_g - gravity);
  pose.pos = pose.pos + deltaT * velocity;

  pubMsg.prev.x = pose.pos[0];
  pubMsg.prev.y = pose.pos[1];
  pubMsg.prev.z = pose.pos[2];

  velocity = Eigen::Vector3d(msg.x,msg.y,msg.z);
  pose.teste = pose.teste + deltaT * velocity;

  pubMsg.justVel.x = pose.teste[0];
  pubMsg.justVel.y = pose.teste[1];
  pubMsg.justVel.z = pose.teste[2];

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Imu_Integrator_node");
  ros::NodeHandle nh;
  ImuIntegrator *imu_integrator = new ImuIntegrator();//line);

  ros::Subscriber Imu_message = nh.subscribe("/odom", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  ros::spin();
}