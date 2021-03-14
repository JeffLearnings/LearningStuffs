#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
using std::vector;
using std::string;

//Global variables for convenience
vector<double> joints_last_position{0.0, 0.0};
bool moving_state{false};
ros::ServiceClient client;

void joint_states_callback(const sensor_msgs::JointState js);
void look_away_callback(const sensor_msgs::Image img);
void move_arm_center();

int main(int argc, char **argv) {
  ros::init(argc,argv,"look_away");
  ros::NodeHandle nh;

  //Define a client service capable of requesting a service from safe_move
  client = nh.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

  //Subscribe to simple_arm/joint_states topic to read arm joints positions
  ros::Subscriber sub1 = nh.subscribe("/simple_arm/joint_states",10,joint_states_callback);

  //Subscribe to rgb_camera/image_raw topic to read image data inside the look_away_callback function 
  ros::Subscriber sub2 = nh.subscribe("rgb_camera/image_raw", 10, look_away_callback);

  //Handle ROS communication events 
  ros::spin();

  return 0;
}

//This callback funtion continously executes and reads the arm joint angle positions 
void joint_states_callback(const sensor_msgs::JointState js) {
  //Get joints current positions 
  vector<double> current_positions{js.position};
  //Define a tolerance threshold to compare double values
  double tolerance{0.0005};
  //Check if arm is moving by comparing current joint position to latest joint position 
  if(fabs(current_positions[0] - joints_last_position[0]) < tolerance
      && fabs(current_positions[1] - joints_last_position[1])) {
    moving_state = false;    
  }else {
    moving_state = true;
    joints_last_position = current_positions;
  }
}

//This callback function continously executes and reads image data 
void look_away_callback(const sensor_msgs::Image img) {
  bool boring_image{true};
  //Loop through each pixel in the pic and check if it's the same as the first one
  int size = img.height*img.step;
  for(int i=0; i<size; ++i) {
    if(img.data[i] != img.data[0]) {
      boring_image = false;
      break;
    }
  }
  if(boring_image && !moving_state) move_arm_center();
}

//This function callls the safe_move service to safely move the arm to the center 
void move_arm_center() {
  ROS_INFO_STREAM("Moving the arm to the center");
  // Request centered joint angles [1.57,1.57]
  simple_arm::GoToPosition srv;
  srv.request.joint_1 = 1.57;
  srv.request.joint_2 = 1.57;

  //Call the safe_move service and pass the requested joint angles
  if(!client.call(srv)) {
    ROS_ERROR("Failed to call the service safe_move");
  }
}
