#ifndef _MECHANICS_H_
#define _MECHANICS_H_

#include <joint_estimation/vector3d.h>

#define NO_SENSORS 1

void mechanics_compute_inverse_kinematics(vector3d sensor_position[NO_SENSORS], vector3d sensor_velocity[NO_SENSORS], float joint_position, float joint_velocity);

#endif
