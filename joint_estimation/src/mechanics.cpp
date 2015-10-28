#include <joint_estimation/mechanics.h>
#include <joint_estimation/vector3d.h>

static const vector3d sensor_top[NO_SENSORS] = {
  {0.0f, 0.0f, 0.0f} // sensor one
};

static const vector3d sensor_bot[NO_SENSORS] = {
  {0.0f, 0.0f, 0.0f} // sensor one
};

static const vector3d joint_axis[NO_SENSORS] = {
  {0.0f, 1.0f, 0.0f} // sensor one
};

void mechanics_compute_inverse_kinematics(float sensor_position[NO_SENSORS], float sensor_velocity[NO_SENSORS], float joint_position, float joint_velocity)
{
}
