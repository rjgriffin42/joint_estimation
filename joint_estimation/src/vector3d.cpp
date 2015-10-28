#include <joint_estimation/vector3d.h>
#include <math.h>
#include <stdio.h>

void vector3d_zeros(vector3d v)
{
  /* zero vector */
  v[0] = 0;
  v[1] = 0;
  v[2] = 0;
}

void vector3d_ones(vector3d v)
{
  /* one vector */
  v[0] = 1;
  v[1] = 1;
  v[2] = 1;
}

void vector3d_copy(const vector3d v1, vector3d v2)
{
  /* copy vector (v2 = v1) */
  v2[0] = v1[0];
  v2[1] = v1[1];
  v2[2] = v1[2];
}

void vector3d_scale(const vector3d v, float s, vector3d v_out)
{
  /* scale vector (v_out = s*v) */
  v_out[0] = v[0]*s;
  v_out[1] = v[1]*s;
  v_out[2] = v[2]*s;
}

void vector3d_add(const vector3d v1, const vector3d v2, vector3d v_out)
{
  /* add vectors (v_out = v1 + v2) */
  v_out[0] = v1[0] + v2[0];
  v_out[1] = v1[1] + v2[1];
  v_out[2] = v1[2] + v2[2];
}

void vector3d_subtract(const vector3d v1, const vector3d v2, vector3d v_out)
{
  /* subtract vectors (v_out = v1 - v2) */
  v_out[0] = v1[0] - v2[0];
  v_out[1] = v1[1] - v2[1];
  v_out[2] = v1[2] - v2[2];
}

void vector3d_cross_product(const vector3d v1, const vector3d v2, vector3d v_out)
{
  /* compute vector cross product (v_out = v1 x v2) */
  float x = v1[1]*v2[2] - v1[2]*v2[1];
  float y = v1[2]*v2[0] - v1[0]*v2[2];
  float z = v1[0]*v2[1] - v1[1]*v2[0];
  v_out[0] = x;
  v_out[1] = y;
  v_out[2] = z;
}

float vector3d_dot_product(const vector3d v1, const vector3d v2)
{
  /* compute vector dot product (result = v1 . v2) */
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

float vector3d_norm(const vector3d v)
{
  /* compute vector norm (result = || v ||) */
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

float vector3d_distance(const vector3d v1, const vector3d v2)
{
  /* compute euclidean distance between vectors (result = || v1 - v2||) */
  float d0 = v1[0] - v2[0];
  float d1 = v1[1] - v2[1];
  float d2 = v1[2] - v2[2];
  return sqrt(d0*d0 + d1*d1 + d2*d2);
}

void vector3d_to_string(const vector3d v, char *buffer, int nbytes)
{
  /* print vector to string */
  snprintf(buffer, nbytes, "%7.4f %7.4f %7.4f\n", v[0], v[1], v[2]);
}
