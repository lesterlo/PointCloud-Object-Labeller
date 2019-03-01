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

typedef struct
{
  double x;
  double y;
  double z;
} SK_node;



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
  SK_node node[SKELETON_NODE_COUNT];
} HSTM_Label;

#endif