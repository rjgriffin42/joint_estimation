#include <joint_estimation/AxesValues.h>
#include <joint_estimation/joint_estimation.h>
#include <joint_estimation/mechanics.h>
#include <joint_estimation/vector3d.h>
#include "ros/ros.h"

// magnetic axes subscriber
static ros::Subscriber axes_values;

// joint state publisher
//static ros::Publisher joint_state;

// velocity estimator

void MagnetCallback(const joint_estimation::AxesValues& msg) {
  // pull values from sensor message
  float x_value = msg.x_axis;
  float y_value = msg.y_axis;
  float z_value = msg.z_axis;

  // update sensor position TODO convert from magnetometer value to position
  vector3d sensor_position[NO_SENSORS] = {{x_value, y_value, z_value}};

  // update sensor velocity TODO calculate sensor velocity
  vector3d sensor_velocity[NO_SENSORS] = { {0.0f, 0.0f, 0.0f} };

  // find joint position
  float joint_position;
  float joint_velocity;
  mechanics_compute_inverse_kinematics(sensor_position, sensor_velocity,
    joint_position, joint_velocity);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, ROS_NAME);
  ros::NodeHandle node_handle;
  
  // Declare customizable variables
  double rate = 10;
  if(ros::param::get("rate", rate))
  {
    ROS_WARN("Rate parameter defined incorrectly!");
  }

  // magnetic axes subscriber
  axes_values = node_handle.subscribe("magnet_topic", 3, MagnetCallback);

  // Tell ROS how fast to run this node
  ros::Rate r(rate);

  ros::spin();
}
