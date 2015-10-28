#include <joint_estimation/mechanics.h>

static const vector3d sensor_top[NO_SENSORS] = {
  {0.0f, 0.0f, 0.0f} // sensor one
};

static const vector3d sensor_bot[NO_SENSORS] = {
  {0.0f, 0.0f, 0.0f} // sensor one
};

static const vector3d joint_axis[NO_SENSORS] = {
  {0.0f, 1.0f, 0.0f} // sensor one
};

void mechanics_compute_inverse_kinematics(vector3d sensor_position[NO_SENSORS], vector3d sensor_velocity[NO_SENSORS], float joint_position, float joint_velocity)
{
}
