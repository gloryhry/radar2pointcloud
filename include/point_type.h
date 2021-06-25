#define PCL_NO_PRECOMPILE

#ifndef __RADAR2POINTCLOUD_H
#define __RADAR2POINTCLOUD_H

#include <pcl/point_types.h>

// Euclidean coordinate, including velocity in radial direction, variance of
// range, VrelRad, AzAng
struct pointXYZVvar {
  PCL_ADD_POINT4D;
  float vel_rad = 0.f;
  float oritation = 0.f;
  float range_var = 0.f;
  float vel_rad_var = 0.f;
  float ang_var = 0.f;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(pointXYZVvar,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, vel_rad, vel_rad)
                                  (float, oritation, oritation)
                                  (float, range_var, range_var)
                                  (float, vel_rad_var, vel_rad_var)
                                  (float, ang_var, ang_var))

#endif