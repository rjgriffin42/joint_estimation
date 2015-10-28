#ifndef _VECTOR3D_H_
#define _VECTOR3D_H_

/*****************************************************************************
** vector3d.o : 3d vector module
******************************************************************************/

typedef float vector3d[3];

void vector3d_zeros(vector3d v);
void vector3d_ones(vector3d v);
void vector3d_copy(const vector3d v1, vector3d v2);
void vector3d_scale(const vector3d v, float s, vector3d v_out);
void vector3d_add(const vector3d v1, const vector3d v2, vector3d v_out);
void vector3d_subtract(const vector3d v1, const vector3d v2, vector3d v_out);
void vector3d_cross_product(const vector3d v1, const vector3d v2, vector3d v_out);
float vector3d_dot_product(const vector3d v1, const vector3d v2);
float vector3d_norm(const vector3d v);
float vector3d_distance(const vector3d v1, const vector3d v2);
void vector3d_to_string(const vector3d v, char *buffer, int nbytes);

#endif
