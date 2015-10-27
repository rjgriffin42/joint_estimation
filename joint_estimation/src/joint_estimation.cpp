#include <joint_estimation/AxesValues.h>
#include <joint_estimation/joint_estimation.h>
#include <joint_estimation/vector3d.h>

float get_sensor_position(const float value)
{
  return CONFIG_CONVERSION_VALUE * value;
}

void MagnetCallback(const joint_estimation::AxesValues& msg) {
  for(int i = 0; i < NO_SENSORS; i++)
  {
    sensor_position[i][0] = get_sensor_position(msg.x_axis[i]);
    sensor_position[i][1] = get_sensor_position(msg.y_axis[i]);
    sensor_position[i][2] = get_sensor_position(msg.z_axis[i]);
  }

  // find joint position and velocity
  float joint_position;
  float joint_velocity;
  // update sensor velocity
  if (CONFIG_JOINT_LEVEL_VELOCITY_ESTIMATION == 0)
  {
    for(int i = 0; i < NO_SENSORS; i++)
    {
      // update sensor velocity estimates
      sensor_x_velocity_estimator[i].update(sensor_position[i][0]);
      sensor_y_velocity_estimator[i].update(sensor_position[i][1]);
      sensor_z_velocity_estimator[i].update(sensor_position[i][2]);
      sensor_velocity[i][0] = sensor_x_velocity_estimator[i].get_filtered_velocity();
      sensor_velocity[i][1] = sensor_y_velocity_estimator[i].get_filtered_velocity();
      sensor_velocity[i][2] = sensor_z_velocity_estimator[i].get_filtered_velocity();

      // solve inverse kinematics for joint velocity and position
      mechanics_compute_inverse_kinematics(sensor_position, sensor_velocity,
        joint_position, joint_velocity);
    }
  }
  else
  {
    for(int i = 0; i < NO_SENSORS; i++)
    {
      sensor_velocity[i][0] = 0.0f;
      sensor_velocity[i][1] = 0.0f;
      sensor_velocity[i][2] = 0.0f;
    }

    // solve inverse kinematics for joint position
    mechanics_compute_inverse_kinematics(sensor_position, joint_position);

    // compute joint velocity from joint position change
    joint_velocity = joint_velocity_estimator.update(joint_position);
  }
  
  // assemble sensor position state and publish
  for(int i = 0; i < NO_SENSORS; i++)
  {
    sensor_state_msg.name[i] = i; 
    sensor_state_msg.position[i] = vector3d_norm(sensor_position[i]);
    sensor_state_msg.velocity[i] = vector3d_norm(sensor_velocity[i]);
    sensor_state_msg.effort[i] = 0.0f;
  }
  sensor_state_pub.publish(sensor_state_msg);

  // assemble joint state and publish
  joint_state_msg.name[0] = "Elbow";
  joint_state_msg.position[0] = joint_position;
  joint_state_msg.velocity[0] = joint_velocity;
  joint_state_msg.effort[0] = 0.0f;
  joint_state_pub.publish(joint_state_msg);
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

  // Finish intializing velocity estimators
  joint_velocity_estimator.set_sample_rate(rate);
  for(int i = 0; i < NO_SENSORS; i++)
  {
    sensor_x_velocity_estimator[i].set_sample_rate(rate);
    sensor_y_velocity_estimator[i].set_sample_rate(rate);
    sensor_z_velocity_estimator[i].set_sample_rate(rate);
  }

  joint_state_msg.name.resize(1);
  joint_state_msg.position.resize(1);
  joint_state_msg.velocity.resize(1);
  joint_state_msg.effort.resize(1);

  // magnetic axes subscriber
  axes_values_sub = node_handle.subscribe("magnet_topic", 1, MagnetCallback);

  joint_state_pub = node_handle.advertise<sensor_msgs::JointState>("/elbow_joint_state", 1);
  sensor_state_pub = node_handle.advertise<sensor_msgs::JointState>("/sensor_position_state", 1);

  // Tell ROS how fast to run this node
  ros::Rate r(rate);

  ros::spin();
}
