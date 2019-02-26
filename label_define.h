#ifndef LABEL_DEFINE_H_
#define LABEL_DEFINE_H_

#include <string>

//KITTI Label Type
// typedef struct
// {
//   std::string type;
//   double truncated;
//   int occluded;
//   double alpha;
//   double x_min;
//   double y_min;
//   double x_max;
//   double y_max;
//   double obj_height;
//   double obj_width;
//   double obj_length;
//   double obj_x;
//   double obj_y;
//   double obj_z;
//   double rotation_y;
// } KITTI_Label;


//(.hst)Human Skeleton Tracking Model Label
typedef struct
{
  std::string name;
  double center_x;
  double center_y;
  double center_z;
  double x_size;
  double y_size;
  double z_size;
  double rotate_x;
  double rotate_y;
  double rotate_z;
  double skeleton_n1;
  double skeleton_n2;
  double skeleton_n3;
  double skeleton_n4;
  double skeleton_n5;
  double skeleton_n6;
  double skeleton_n7;
  double skeleton_n8;
  double skeleton_n9;
  double skeleton_n10;
  double skeleton_n11;
  double skeleton_n12;
  double skeleton_n13;
  double skeleton_n14;
  double skeleton_n15;
} HSTM_Label;

#endif