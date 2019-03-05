#ifndef LABEL_DEFINE_H_
#define LABEL_DEFINE_H_

#include <string>

#define SKELETON_NODE_COUNT 15

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

  double sk_n1_x;
  double sk_n1_y;
  double sk_n1_z;
  double sk_n2_x;
  double sk_n2_y;
  double sk_n2_z;
  double sk_n3_x;
  double sk_n3_y;
  double sk_n3_z;
  double sk_n4_x;
  double sk_n4_y;
  double sk_n4_z;
  double sk_n5_x;
  double sk_n5_y;
  double sk_n5_z;
  double sk_n6_x;
  double sk_n6_y;
  double sk_n6_z;
  double sk_n7_x;
  double sk_n7_y;
  double sk_n7_z;
  double sk_n8_x;
  double sk_n8_y;
  double sk_n8_z;
  double sk_n9_x;
  double sk_n9_y;
  double sk_n9_z;
  double sk_n10_x;
  double sk_n10_y;
  double sk_n10_z;
  double sk_n11_x;
  double sk_n11_y;
  double sk_n11_z;
  double sk_n12_x;
  double sk_n12_y;
  double sk_n12_z;
  double sk_n13_x;
  double sk_n13_y;
  double sk_n13_z;
  double sk_n14_x;
  double sk_n14_y;
  double sk_n14_z;
  double sk_n15_x;
  double sk_n15_y;
  double sk_n15_z;
} HSTM_Label;

#endif