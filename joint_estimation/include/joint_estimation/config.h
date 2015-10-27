#ifndef _CONFIG_H_
#define _CONFIG_H_

static const float CONFIG_SENSOR_POSITION_RESOLUTION = 0.001f; // m
static const float CONFIG_SENSOR_VELOCITY_BREAK_FREQUENCY = 30; // Hz
static const float CONFIG_JOINT_POSITION_RESOLUTION = 0.01f; // m
static const float CONFIG_JOINT_VELOCITY_BREAK_FREQUENCY = 15; // Hz
static const int CONFIG_JOINT_LEVEL_VELOCITY_ESTIMATION = 1; // 1 = at joint, 0 = at sensor
static const float CONFIG_CONVERSION_VALUE = 1;

#endif
