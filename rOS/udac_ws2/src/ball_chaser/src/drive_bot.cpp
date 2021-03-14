#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

using std::string;
using std::vector;

typedef ball_chaser::DriveToTarget::Request ballReq;
typedef ball_chaser::DriveToTarget::Response ballRes;

class DriveBot {
  private:
    ros::Publisher motor_command_publisher;
    ros::ServiceServer drive_service_;

  public:
    DriveBot(ros::NodeHandle *n_) {
      /// Publish a message of type geometry_msgs::Twist on the wheel actuation  topic 
      motor_command_publisher = n_->advertise<geometry_msgs::Twist>("/cmd_vel",10);
      /// Drive service 
      drive_service_ = n_->advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
    }

    bool handle_drive_request(ballReq& req, ballRes& res) {
      ROS_INFO("DriveToTarget request received - linear_x:%1.2f, angular_z:%1.2f", req.linear_x, req.angular_z);
      /// Create velocity object 
      geometry_msgs::Twist motor_command;
      motor_command.linear.x = req.linear_x;
      motor_command.angular.z = req.angular_z;
      /// Publish the velocity
      motor_command_publisher.publish(motor_command);
      ///Return a response message 
      res.msg_feedback = "Linear velocity - x: " + std::to_string(motor_command.linear.x) + " , angular velocity - z: " + std::to_string(motor_command.angular.z); 
      ROS_INFO_STREAM(res.msg_feedback);
      return true;
    }
};

int main(int argc, char **argv) {
  /// initialize drive bot node
  ros::init(argc,argv,"drive_bot");
  /// create a node handler
  ros::NodeHandle nh;

  DriveBot ballChaser = DriveBot(&nh);
  ros::spin();

  return 0;
}


