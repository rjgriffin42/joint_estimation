#include <joint_estimation/VelocityEstimator.h>
#include <math.h>

/**
  * Default Constructor
  */
#define EPSILON 0.001f

VelocityEstimator::VelocityEstimator(const float& sample_rate, const float& position_resolution, const float& velocity_break_frequency) :
  _sample_period(1 / sample_rate),
  _delta_time(0.0f),
  _position_resolution(fabs(position_resolution)),
  _position(0.0f),
  _velocity(0.0f),
  _filtered_velocity(0.0f),
  _velocity_break_frequency(velocity_break_frequency),
  _initialized(0)
{
  _velocity_filter.new_low_pass(_sample_period, _velocity_break_frequency);
}

VelocityEstimator::~VelocityEstimator()
{
}

void VelocityEstimator::velocity_estimator_reset()
{
  _initialized = 0;
  _velocity_filter.reset();
}

float VelocityEstimator::get_raw_velocity() const
{
  return _velocity;
}

float VelocityEstimator::get_filtered_velocity() const
{
  return _filtered_velocity;
}

float VelocityEstimator::update(float position)
{
  // intialize velocity estimator
  if(!_initialized)
  {
    _position = position;
    _velocity = 0.0f;
    _delta_time = 0.0f;
    _initialized = 1;
  }

  // compute position and time deltas with respect to last reading
  float delta_position = position - _position;
  _delta_time += _sample_period; 

  // update velocity estimate using fixed time / fixed displacement
  if (fabs(delta_position) > EPSILON)
  {
    _position = position;
    _velocity = delta_position / _delta_time;
    _delta_time = 0.0f;
  }
  else
  {
    float max_velocity = _position_resolution / _delta_time;
    if (_velocity > max_velocity)
      _velocity = max_velocity;
    if (_velocity < -max_velocity)
      _velocity = -max_velocity;
  }

  if (_velocity_break_frequency > 0.0f)
  {
    // apply low pass filter
    return _velocity_filter.update(_velocity);
  }
  else
  {
    return _velocity;
  }
}
