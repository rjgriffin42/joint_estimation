#ifndef _JOINT_ESTIMATION_H
#define _JOINT_ESTIMATION_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

#define ROS_NAME             "joint_estimation"
#define SAMPLE_RATE          100.0
#define SAMPLE_PERIOD        1.0/SAMPLE_RATE

sensor_msgs::JointState joint_state_msg;

ros::Subscriber axes_values_sub;
ros::Publisher joint_state_pub;

#endif // _JOINT_ESTIMATION_H
