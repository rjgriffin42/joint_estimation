#include <joint_estimation/AxesValues.h>
#include <joint_estimation/joint_estimation.h>
#include "ros/ros.h"

// magnetic axes subscriber
static ros::Subscriber axes_values;

// joint state publisher
//static ros::Publisher joint_angle;

void MagnetCallback(const joint_estimation::AxesValues& msg) {
  float x_value = msg.x_axis;
  float y_value = msg.y_axis;
  float z_value = msg.z_axis;
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, ROS_NAME);
  ros::NodeHandle node_handle;
  
  // Declare customizable variables
  double rate = 10;
  if(node_handle.getParam("~rate", rate))
  {
    ROS_WARN("Rate parameter defined incorrectly!");
  }

  // magnetic axes subscriber
  axes_values = node_handle.subscribe("magnet_topic", 3, MagnetCallback);

  // Tell ROS how fast to run this node
  ros::Rate r(rate);

  ros::spin();
}
