#include <joint_estimation/Filter.h>
#include <float.h>

#define MAX(A, B) (((A) > (B)) ? (A) : (B))
#define MIN(A, B) (((A) < (B)) ? (A) : (B))
#define PI 3.14159265

// b : numerator coefficients of z-domain transfer function
// a : denominator coefficients of z-domain transfer function
Filter::Filter() :
  _initialized(0),
  _size_b(MAX_FILTER_SIZE),
  _size_a(MAX_FILTER_SIZE),
  _min_output(-FLT_MAX),
  _max_output(FLT_MAX)
{
  for(int i = 0; i < MAX_FILTER_SIZE; i++)
  {
    _input[i] = 0.0f;
    _output[i] = 0.0f;
    _b[i] = 0.0f;
    _a[i] = 0.0f;
  }
}

Filter::~Filter()
{
}

void Filter::new_filter(float b[], float a[], const int size_b, const int size_a)
{
  _initialized = 0;
  _size_b = MIN(size_b, MAX_FILTER_SIZE),
  _size_a = MIN(size_a, MAX_FILTER_SIZE),
  _min_output =-FLT_MAX;
  _max_output = FLT_MAX;
  for(int i = 0; i < MAX_FILTER_SIZE; i++)
  {
    _input[i] = 0.0f;
    _output[i] = 0.0f;
  }
  for(int i = 0; i < size_b; i++)
    _b[i] = b[i]/a[0];
  for(int i = 0; i < size_a; i++)
    _a[i] = a[i]/a[0];
}

void Filter::new_low_pass(const float& Ts, const float& freq)
{
  float w = 2 * PI * freq;
  float b[2] = {w * Ts, w * Ts};
  float a[2] = {w * Ts + 2, w * Ts - 2};
  new_filter(b, a, 2, 2);
}

void Filter::reset()
{
  _initialized = 0;
}

float Filter::update(float input)
{
  // initialize input / output history
  if (!_initialized)
  {
    for(int i = 0; i < _size_b; i++)
      _input[i] = input;
    for(int i = 0; i < _size_a; i++)
      _output[i] = 0.0f;
    _initialized = 1;
  }

  // update input / output history
  for(int i = _size_b-1; i > 0; i--)
    _input[i] = _input[i-1];
  _input[0] = input;
  for(int i = _size_b-1; i > 0; i--)
    _output[i] = _output[i-1];
  _output[0] = 0;

  // return linear filter output
  for(int i = 0; i < _size_b; i++)
    _output[0] += _b[i] * _input[i];
  for(int i = 1; i < _size_a; i++)
    _output[0] -= _a[i] * _output[i];
  _output[0] = MIN(_output[0], _max_output);
  _output[0] = MAX(_output[0], _min_output);

  return _output[0];
}
