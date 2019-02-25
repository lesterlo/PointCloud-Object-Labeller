#ifndef LABEL_DEFINE_H_
#define LABEL_DEFINE_H_

#include <string>

typedef struct
{
  std::string type;
  double truncated;
  int occluded;
  double alpha;
  double x_min;
  double y_min;
  double x_max;
  double y_max;
  double obj_height;
  double obj_width;
  double obj_length;
  double obj_x;
  double obj_y;
  double obj_z;
  double rotation_y;
} KITTI_Label;

#endif