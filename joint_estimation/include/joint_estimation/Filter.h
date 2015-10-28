#ifndef _FILTER_H_
#define _FILTER_H_

#define MAX_FILTER_SIZE 64

class Filter {
public:
  explicit Filter();
  ~Filter();
  void new_low_pass(const float& Ts, const float& freq);
  void reset();
  float update(float input);
protected:
  int _initialized;
  int _size_b;
  int _size_a;
  float _min_output;
  float _max_output;
  float _input[MAX_FILTER_SIZE];
  float _output[MAX_FILTER_SIZE];
  float _b[MAX_FILTER_SIZE];
  float _a[MAX_FILTER_SIZE];
private:
  void new_filter(float b[], float a[], const int size_b, const int size_a);
};

#endif
