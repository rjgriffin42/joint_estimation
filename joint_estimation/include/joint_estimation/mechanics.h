#ifndef _MECHANICS_H_
#define _MECHANICS_H_

#define NO_SENSORS 1

void mechanics_compute_inverse_kinematics(float sensor_position[NO_SENSORS], float sensor_velocity[NO_SENSORS], float joint_position, float joint_velocity);

#endif
