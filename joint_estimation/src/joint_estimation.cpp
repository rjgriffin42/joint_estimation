#include "ros/ros.h"
#include "std_msgs/Float32.msg"

// magnetic axes subscriber
static ros::Subscriber magnet_x;
static ros::Subscriber magnet_y;
static ros::Subscriber magnet_z;

float x_value;
float y_value;
float z_value;

// joint state publisher
//static ros::Publisher joint_angle;

void MagnetXCallback(const std_msgss::Float32& msg) {
  x_value = msg;
}
void MagnetYCallback(const std_msgss::Float32& msg) {
  y_value = msg;
}
void MagnetZCallback(const std_msgss::Float32& msg) {
  z_value = msg;
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, ROS_NAME);
  ros::NodeHandle node_handle;
  
  // Declare customizable variables
  int rate;
  node_handle.param("rate", rate);

  // magnetic axes subscriber
  magnet_x = node_handle.subscribe(MAGNET_X_TOPIC, 3, MagnetXCallback);
  magnet_y = node_handle.subscribe(MAGNET_Y_TOPIC, 3, MagnetYCallback);
  magnet_z = node_handle.subscribe(MAGNET_Z_TOPIC, 3, MagnetZCallback);

  // Tell ROS how fast to run this node
  ros::Rate r(rate);

  ros::spin();
}
