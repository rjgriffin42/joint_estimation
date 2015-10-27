#ifndef _JOINT_ESTIMATION_H
#define _JOINT_ESTIMATION_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <joint_estimation/VelocityEstimator.h>
#include <joint_estimation/mechanics.h>
#include <joint_estimation/vector3d.h>

#define ROS_NAME             "joint_estimation"
#define SAMPLE_RATE          100.0
#define SAMPLE_PERIOD        1.0/SAMPLE_RATE

// public values
vector3d sensor_position[NO_SENSORS];
vector3d sensor_velocity[NO_SENSORS];

// define messages
sensor_msgs::JointState joint_state_msg;
sensor_msgs::JointState sensor_state_msg;

// define subscribers
ros::Subscriber axes_values_sub;

// define publishers
ros::Publisher joint_state_pub;
ros::Publisher sensor_state_pub;

// define velocity estimators
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

#endif // _JOINT_ESTIMATION_H
