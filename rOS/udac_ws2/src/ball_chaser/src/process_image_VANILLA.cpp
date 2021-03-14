#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <unordered_set>
#include "sensor_msgs/Image.h"

/**
 * This is a client node. 
 * 1. Subscribe to robot's camera images 
 *     --> determine position of the white ball
 * 2. Request a service from drive_bot server to drive robot 
 *     --> Avail movement: Left, right forward
 *     --> Service is 'ball_chaser/command_robot'
 */

using std::vector;
using std::string;
using std::unordered_set;

class StartBrain {
  private:
    ros::ServiceClient cli_;
    ros::Subscriber sub_;
    /// 63:BLUE 92:GREEN 225:RED 227:MAGENTA 252:YELLOW 255:WHITE 
    //unordered_set<int> colors{63,92,225,227,252,255};
    

  public:
    StartBrain(ros::NodeHandle *n_) {
      //Define a client service capable of requesting service from command_robot 
      cli_ = n_->serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
      //Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
      sub_ = n_->subscribe("/camera/rgb/image_raw",10,&StartBrain::process_image_callback,this);
    }

    //Calls the command_robot service to drive the robot in the specified direction
    void drive_bot(float lin_x, float ang_z) {
      //: Request a service and pass the velocities to it to drive the robot
      ball_chaser::DriveToTarget srv;
      srv.request.linear_x  = lin_x;
      srv.request.angular_z = ang_z; 
      //Call the service
      if(!cli_.call(srv)) {
        ROS_INFO("Failed to call the service :(");
      }
    }

    //Continuosly executes and reads the image data
    void process_image_callback(const sensor_msgs::Image& img) {
      int white_pixel = 255;
      //: Loop throught each pixel in the image and check if there's a bright white one
      //Then, identify if this pixel falls in the left, mid or right side of the image 
      //Depending on the white ball position, call the drive_bot function and pass velocities to it 
      //Request a stop when there's no white ball seen by the camera
      /*
       *             step
       * ----------------------------
       * -                          -
       * -                          - height
       * -                          -
       * ---------------------------- 
       */
      unsigned int height{img.height}, step{img.step};
      unsigned int imgSize = height*step;
      //bool foundBall{false};
      double goLeft{0.25*step}, goRight{0.75*step};

      for(int i=0;i<imgSize-1;i+=3) {
        if(img.data[i] == white_pixel 
            && img.data[i+1] == white_pixel
            && img.data[i+2] == white_pixel){
            //foundBall = true;
            if(i%step-5 <= goLeft) {
              drive_bot(0.0, 0.0);
              drive_bot(0.0, 0.710);
              ros::Duration(1).sleep();
            }else if(i%step-5 >= goRight) {
              drive_bot(0.0, 0.0);
              drive_bot(0.0, -0.710);
              ros::Duration(1).sleep();
            }else {
              drive_bot(0.0, 0.0);
              drive_bot(0.5, 0.0);
              ros::Duration(1).sleep();
            }
            return;
          }
      }
      //Stop the car if no white ball is found
      drive_bot(0.0, 0.0);
    }
    
    void work() {
      ros::spinOnce();
    }
};

int main(int argc, char** argv) {
  //Initialize the process_image node and create a handle to it
  ros::init(argc,argv,"process_image_VANILLA");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  StartBrain brain(&nh);

  //Handle ROS communication events 
  while(ros::ok()) {
    brain.work();
    loop_rate.sleep();
  }
  return 0;
}
