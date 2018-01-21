#include <map>
#include <math.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "path_planner/CheckpointDistance.h" //NAME & DISTANCE
#include "geometry_msgs/Twist.h"
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
}; //this is only a helper class, to state the name and position of each checkpoint

class CheckpointMonitor{
public:
  CheckpointMonitor():checkpoints_() {
    InitCheckpoints();
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    sub = nh.subscribe("/odometry/filtered", 10, &CheckpointMonitor::OdomCallback, this);
  }

  CheckpointDistance DIS[4];
  int curCP = 0;
  geometry_msgs::Twist cmd_vel_msg;

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double alpha = msg->pose.pose.orientation.z;
    double phi = msg->pose.pose.orientation.w;
    loopOver(x, y, alpha, phi);
  }

  void lockOnCheckpoint(double x, double y, double alpha, double phi, int i){
    // for (size_t i = 0; i < checkpoints_.size(); ++i){
      const Checkpoint& checkpoint = checkpoints_[i];
      double xc = checkpoint.x - x;
      double yc = checkpoint.y - y;
      double theta = atan(yc/xc);
      DIS[i].distance = sqrt(xc*xc + yc*yc);
      DIS[i].name = "Checkpoint " + i;
      //transformation of orientation from quaternion to radians:
      double siny = +2.0 * (alpha*phi);
      double cosy = 1;
      double alpha_rad = atan2(siny, cosy);
      DIS[i].orientation = alpha_rad - theta;
      ROS_INFO("Inside distance: %f, orientation: %f", DIS[i].distance, DIS[i].orientation);
    // } this is being printed out (this ros info thing), but no command sent to the robot
  }

  void loopOver(double x, double y, double alpha, double phi) {
    if (curCP < checkpoints_.size() ) {
      lockOnCheckpoint(x, y, alpha, phi, curCP);
      cmdSender(curCP);
    }
  }

  void cmdSender(int i){
    // for (size_t i=0; i < checkpoints_.size(); ++i){
      if(abs(DIS[i].orientation) >= 0.1){
        if(0 <= DIS[i].orientation < pi){
          cmd_vel_msg.angular.z = 0.5;
          cmd_vel_msg.linear.x = 0;
          pub.publish(cmd_vel_msg);
          ROS_INFO("rotating towards CH c-clockwise");
        }else{
          cmd_vel_msg.angular.z = -0.5; //CHECK THIS AND ALSO QUATERNION TRANSFORMATION
          cmd_vel_msg.linear.x = 0;
          pub.publish(cmd_vel_msg);
          ROS_INFO("rotating towards CH clockwise");
        }
      } else {
        // DIS[i].orientation < 0.1, ROB is oriented towards CH, therefore now moves forward
        if(DIS[i].distance >= 0.1){
          ROS_INFO("outside distance: %f, orientation: %f", DIS[i].distance, DIS[i].orientation);
          cmd_vel_msg.linear.x = 0.5;
          cmd_vel_msg.angular.z = 0;
          pub.publish(cmd_vel_msg);
        } else {
          // reached new checkpoint
          ++curCP;
          //cmd_vel_msg.linear.x = 0.5;
          //pub.publish(cmd_vel_msg);
        }
      }
    // }
  }
//http://wiki.ros.org/topic_tools/mux
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
