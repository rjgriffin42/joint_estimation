#ifndef _VelocityEstimator_h_
#define _VelocityEstimator_h_

#include <joint_estimation/Filter.h>

class VelocityEstimator
{
public:
  explicit VelocityEstimator(const float& sample_rate = 100.0f, const float& position_resolution = 0.01f, const float& velocity_break_frequency = 0.0f);
  ~VelocityEstimator(); 
  float get_raw_velocity() const; 
  float get_filtered_velocity() const;
  float update(float position);
protected:
  float _sample_period;
  float _delta_time;
  float _position_resolution;
  float _position;
  float _velocity;
  float _filtered_velocity;
  float _velocity_break_frequency;
  Filter _velocity_filter;
  int _initialized;
  void velocity_estimator_reset();
private:
};

#endif
