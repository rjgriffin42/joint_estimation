#include <joint_estimation/AxesValues.h>
#include <joint_estimation/joint_estimation.h>
#include <joint_estimation/mechanics.h>
#include <joint_estimation/vector3d.h>
#include <joint_estimation/VelocityEstimator.h>
#include "ros/ros.h"

// magnetic axes subscriber
static ros::Subscriber axes_values;

// joint state publisher
//static ros::Publisher joint_state;

// velocity estimators
std::vector<VelocityEstimator> sensor_x_velocity_estimator(NO_SENSORS,
  VelocityEstimator(10, CONFIG_SENSOR_POSITION_RESOLUTION,
  CONFIG_SENSOR_VELOCITY_BREAK_FREQUENCY)
);
std::vector<VelocityEstimator> sensor_y_velocity_estimator(NO_SENSORS,
  VelocityEstimator(10, CONFIG_SENSOR_POSITION_RESOLUTION,
  CONFIG_SENSOR_VELOCITY_BREAK_FREQUENCY)
);
std::vector<VelocityEstimator> sensor_z_velocity_estimator(NO_SENSORS,
  VelocityEstimator(10, CONFIG_SENSOR_POSITION_RESOLUTION,
  CONFIG_SENSOR_VELOCITY_BREAK_FREQUENCY)
);
VelocityEstimator joint_velocity_estimator(10, CONFIG_SENSOR_POSITION_RESOLUTION, 
  CONFIG_JOINT_VELOCITY_BREAK_FREQUENCY);

void MagnetCallback(const joint_estimation::AxesValues& msg) {
  // pull values from sensor message
  float x_value = msg.x_axis;
  float y_value = msg.y_axis;
  float z_value = msg.z_axis;

  // update sensor position TODO convert from magnetometer value to position
  vector3d sensor_position[NO_SENSORS];
  vector3d sensor_velocity[NO_SENSORS];
  for(int i = 0; i < NO_SENSORS; i++)
  {
    sensor_position[i][0] = x_value;
    sensor_position[i][1] = y_value;
    sensor_position[i][2] = z_value;
    sensor_velocity[i][0] = 0.0f;
    sensor_velocity[i][1] = 0.0f;
    sensor_velocity[i][2] = 0.0f;
  }

  // update sensor velocity
  for(int i = 0; i < NO_SENSORS; i++)
  {
    sensor_x_velocity_estimator[i].update(sensor_position[i][0]);
    sensor_y_velocity_estimator[i].update(sensor_position[i][1]);
    sensor_z_velocity_estimator[i].update(sensor_position[i][2]);
    sensor_velocity[i][0] = sensor_x_velocity_estimator[i].get_filtered_velocity();
    sensor_velocity[i][1] = sensor_y_velocity_estimator[i].get_filtered_velocity();
    sensor_velocity[i][2] = sensor_z_velocity_estimator[i].get_filtered_velocity();
  }

  // find joint position
  float joint_position;
  float joint_velocity;
  // option for computing joint velocity using velocity of sensor
  //mechanics_compute_inverse_kinematics(sensor_position, sensor_velocity,
  //  joint_position, joint_velocity);
  // option for computing joint velocity using displacement of sensor
  mechanics_compute_inverse_kinematics(sensor_position, joint_position);
  joint_velocity = joint_velocity_estimator.update(joint_position);
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
  joint_velocity_estimator.set_sample_rate(rate);
  for(int i = 0; i < NO_SENSORS; i++)
  {
    sensor_x_velocity_estimator[i].set_sample_rate(rate);
    sensor_y_velocity_estimator[i].set_sample_rate(rate);
    sensor_z_velocity_estimator[i].set_sample_rate(rate);
  }

  // magnetic axes subscriber
  axes_values = node_handle.subscribe("magnet_topic", 3, MagnetCallback);

  // Tell ROS how fast to run this node
  ros::Rate r(rate);

  ros::spin();
}
