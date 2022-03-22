// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cmath>
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
    Eigen::Vector3d teste; //test purpose only
    Eigen::Vector3d teste1; //test purpose only
    Eigen::Matrix3d orien;
};

class ImuIntegrator
{
private:
    Pose pose;
    ros::Time time;
    Eigen::Vector3d gravity; //not used properly
    Eigen::Vector3d velocity;
    Eigen::Vector3d velocity2; //test purpose only
    visualization_msgs::Marker path;

    std::time_t prev = std::time(nullptr);

    Augbot::position pubMsg = Augbot::position();
    Augbot::position pubMsg1 = Augbot::position(); //test purpose only

    ros::NodeHandle n;
    ros::Publisher line_pub = n.advertise<Augbot::position>("deadReckoning", 1);
    ros::Publisher line_pub1 = n.advertise<Augbot::position>("deadReckoning1", 1000); //test purpose only

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

    void setGravity();//const geometry_msgs::Vector3 &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::Vector3 &vel, const geometry_msgs::Vector3 &acc);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
};

ImuIntegrator::ImuIntegrator() {
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.teste = zero;
  pose.teste1 = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  velocity = zero;
  firstT = true;

  const auto prev = std::chrono::system_clock::now();

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
   
  //orien << 2 * (q0 * q0 + q1 * q1) - 1 , 2 * (q1 * q2 - q0 * q3),     2 * (q1 * q3 + q0 * q2),
  //              2 * (q1 * q2 + q0 * q3)     , 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1),
  //              2 * (q1 * q3 - q0 * q2)     , 2 * (q2 * q3 + q0 * q1),     2 * (q0 * q0 + q3 * q3) - 1;
//

  orien << 1 - 2*q1*q1 - 2*q2*q2,	2*q0*q1 - 2*q2*q3,	2*q0*q2 + 2*q1*q3,
            2*q0*q1 + 2*q2*q3,	1 - 2*q0*q0 - 2*q2*q2,	2*q1*q2 - 2*q0*q3,
            2*q0*q2 - 2*q1*q3,	2*q1*q2 + 2*q0*q3,	1 - 2*q0*q0 - 2*q1*q1;

  return orien;
}

void ImuIntegrator::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (firstT) {
      time = msg->header.stamp;
      deltaT = 0;
      setGravity();//msg->twist.twist.linear;
      pose.pos = Eigen::Vector3d(0, 0, 0);
      pose.teste = pose.pos;
      firstT = false;
    } else {
      deltaT = (msg->header.stamp - time).toSec();
      time = msg->header.stamp;
      //std::cout << "data" << std::endl;
      //std::cout << msg->linear_acceleration << std::endl;
      //std::cout << msg->angular_velocity << std::endl;
      //calcOrientation(msg->twist.twist.angular);
      pose.orien = setOrientation( msg->orientation );
      calcPosition( msg->angular_velocity, msg->linear_acceleration );
      //ROS_INFO ( "deltaT = %f", deltaT);
      updatePath(pose.pos);

      std::time_t now = std::time(nullptr);

      if ( now - prev > 1 ) {
        prev = now;

        pubMsg.x = pose.pos[0];
        pubMsg.y = pose.pos[1];
        pubMsg.z = pose.pos[2];
        pubMsg1.x = pose.teste[0];
        pubMsg1.x = pose.teste[1];
        pubMsg1.x = pose.teste[2];

        ROS_INFO ( "[%f,%f,%f] ... [%f,%f,%f]", pubMsg.x, pubMsg.y, pubMsg.z, pubMsg1.x, pubMsg1.y, pubMsg1.z);
        
        publishMessage();
      }
    }
}


//MARTELADA
void ImuIntegrator::setGravity( )//const geometry_msgs::Vector3 &msg)
{
  gravity[0] = 0;//msg.x;
  gravity[1] = 0;//msg.y;
  gravity[2] = 10;//msg.z;
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  //ROS_INFO("[%f,%f,%f]", msg[0],msg[1],msg[2]);
  path.points.push_back(p);
}

void ImuIntegrator::publishMessage ()
{
  line_pub.publish(pubMsg); 
  //line_pub1.publish( pubMsg1 );
}


//not used
void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3 &msg) {
  Eigen::Matrix3d B;
  B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT, -msg.y * deltaT, msg.x * deltaT, 0;
  double sigma = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) * deltaT;
  //std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
  // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;
  pose.orien = pose.orien * (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B - ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
  //std::cout << pose.orien << std::endl;
}

void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &vel, const geometry_msgs::Vector3 &acc) {
  Eigen::Vector3d acc_l(acc.x, acc.y, acc.z);
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
  // gravity[2]);
  //std::cout << deltaT << std::endl;
  //ROS_INFO ( "vel [%f,%f,%f] --- acc [%f,%f,%f]", vel.x, vel.y, vel.z, acc.x, acc.y, acc.z );
  gravity = Eigen::Vector3d ( 0, 0, acc_g[2] );
  //ROS_INFO ( "grav [%f,%f,%f] ### acc [%f,%f,%f]", gravity[0], gravity[1], gravity[2], acc_g[0], acc_g[1], acc_g[2] );
  //ROS_INFO ( "acc_g - gravity = %f" , acc_g[2] - gravity[2]);
  velocity = velocity + deltaT * (acc_g - gravity);
  //ROS_INFO ( "vel [%f,%f,%f]", velocity[0], velocity[1], velocity[2] );
  pose.pos = pose.pos + deltaT * velocity;

  //std::cout << "first" << std::endl;
  //std::cout << velocity << std::endl;

  Eigen::Vector3d velocity1 (vel.x,vel.y,vel.z);
  pose.teste = pose.teste + deltaT * velocity1;
  
  //std::cout << "sec" << std::endl;
  //std::cout << velocity << std::endl;

  Eigen::Vector3d acc_l1(vel.x, vel.y, vel.z);
  Eigen::Vector3d acc_g1 = pose.orien * acc_l1;
  pose.teste1 = pose.teste1 + deltaT * acc_g1;

  //ROS_INFO ( "POS [%f,%f,%f] ... [%f,%f,%f] ... [%f,%f,%f]", 
  //pose.pos[0], pose.pos[1], pose.pos[2], pose.teste[0], pose.teste[1], pose.teste[2], pose.teste1[0], pose.teste1[1], pose.teste1[2] );

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Imu_Integrator_node");
  ros::NodeHandle nh;
  ImuIntegrator *imu_integrator = new ImuIntegrator();//line);


  ros::Subscriber Imu_message = nh.subscribe("/imu", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  ros::spin();
}