#include <map>
#include <math.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "path_planner/CheckpointDistance.h" //NAME & DISTANCE
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#define pi 3.1415

using std::vector;
using std::string;
using path_planner::CheckpointDistance;

class Checkpoint{
public:
  Checkpoint(string name, double x, double y) :
  name(name), x(x), y(y) {}
  string name;
  double x;
  double y;
};

class CheckpointMonitor{
public:
  CheckpointMonitor():checkpoints_() {
    InitCheckpoints();
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    sub = nh.subscribe("/odometry/filtered", 10, &CheckpointMonitor::OdomCallback, this);
    //quat_sub = nh.subscribe("/tf", 10, &CheckpointMonitor::quatCallback, this);
  }

  CheckpointDistance DIS[4];
  int curCP = 0;
  geometry_msgs::Twist cmd_vel_msg;
/*  void quatCallback(const geometry_msgs::Quaternion msg){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    loopOver(x, y, rpy.z);
  }*/

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double qx= msg->pose.pose.orientation.x;
    double qy= msg->pose.pose.orientation.y;
    double qz= msg->pose.pose.orientation.z;
    double qw= msg->pose.pose.orientation.w;
    tf::Quaternion bt = tf::Quaternion(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf::Matrix3x3(bt).getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    loopOver(x, y, yaw);
  }

  void loopOver(double x, double y, double yaw) {
    if (curCP < checkpoints_.size() ) {
      lockOnCheckpoint(x, y, yaw, curCP);
      cmdSender(curCP, yaw);
    }
  }

  void lockOnCheckpoint(double x, double y, double yaw, int i){
      const Checkpoint& checkpoint = checkpoints_[i];
      double xc = checkpoint.x - x;
      double yc = checkpoint.y - y;
      DIS[i].distance = sqrt(xc*xc + yc*yc);
      DIS[i].name = "Checkpoint " + i;
      double theta = atan2(yc, xc);
      // double siny = +2.0 * (alpha*phi);
      // double cosy = 1;
      // double alpha_rad = atan2( siny, cosy);
      DIS[i].orientation = theta - yaw;
      ROS_INFO("DIS[i]: %f, yaw: %f, theta: %f, DIS[i].orientation: %f",
      DIS[i].distance, yaw, theta, DIS[i].orientation);
  }

  void cmdSender(int i, double yaw){
    // for (size_t i=0; i < checkpoints_.size(); ++i){
      if(DIS[i].orientation >= 0.1 || DIS[i].orientation < -0.1){
        if(0.01 <= DIS[i].orientation < pi){
          cmd_vel_msg.angular.z = 0.4;
          cmd_vel_msg.linear.x = 0;
          pub.publish(cmd_vel_msg);
          ROS_INFO("rotating towards CH c-clockwise");
        }else{
          cmd_vel_msg.angular.z = -0.4;
          cmd_vel_msg.linear.x = 0;
          pub.publish(cmd_vel_msg);
          ROS_INFO("rotating towards CH clockwise");
        }
      } else {
        // DIS[i].orientation < 0.1, ROB is oriented towards CH, therefore now moves forward
        if(DIS[i].distance >= 0.1){
          ROS_INFO("Distance to CH: %f, orientation: %f", DIS[i].distance, DIS[i].orientation);
          cmd_vel_msg.angular.z = 0;
          cmd_vel_msg.linear.x = 0.5;
          pub.publish(cmd_vel_msg);
        } else {
          // reached new checkpoint
          curCP++;
          //cmd_vel_msg.linear.x = 0.5;
          //pub.publish(cmd_vel_msg);
        }
      }
     }
  //}

private:
  vector<Checkpoint> checkpoints_;

  void InitCheckpoints(){
    checkpoints_.push_back(Checkpoint("Checkpoint 1", 5, 0));
    checkpoints_.push_back(Checkpoint("Checkpoint 2", 5, 5));
    checkpoints_.push_back(Checkpoint("Checkpoint 3", 0, 5));
    checkpoints_.push_back(Checkpoint("Checkpoint 4", 0, 0));
  }

protected:
  ros::Subscriber sub;
  ros::Publisher pub;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "path_planner");
  CheckpointMonitor monitor;
  ros::spin();

  /*ros::Rate rate(10);
  ROS_INFO("following the checkpoints...");

  while(nh.ok()){
    pub.publish(cmd_vel_msg);
    ros::spinOnce();
    rate.sleep();
  }*/
  return 0;
}
