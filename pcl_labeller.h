// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 


#ifndef PCL_LABELLER_H
#define PCL_LABELLER_H

//Include
//std
#include <iostream>
#include <string>
#include <regex>
#include <fstream>
#include <sstream>

// Qt
#include <QMainWindow>
#include <QtWidgets>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum {LABEL_UI_CUROR, LABEL_UI};

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



//QT Class decalration
namespace Ui
{
  class PCL_Labeller;
}

class PCL_Labeller : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCL_Labeller (QWidget *parent = 0);
  ~PCL_Labeller ();


protected: 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr display_cloud;

private slots:
  void openFolder();
  void about();
  void onFileListItemClicked(QListWidgetItem *);
  void onLabelListItemClicked(QListWidgetItem *);
  void onPrevPCDButtonClicked();
  void onNextPCDButtonClicked();
  void onSaveButtonClicked();
  void onInsertLabelButtonClicked();
  void onDeleteLabelButtonClicked();
  void onLabelEditFinish();
  void onLabelValueChange(double);

private:
  //function
  int read_label();
  int write_label();
  void label_UI_enable(int , bool );
  void construct_labelWidget();
  void drawAllLabel(int );
  void clean_viewer();
  void clearLabelUI();
  void labelUI_Signal_enable(bool );

  //QT Area
  Ui::PCL_Labeller *ui;
  void createActions();
  void createMenus();
  void contextMenuEvent(QContextMenuEvent *event);
  
  //QMenu
  QMenu *fileMenu;
  QMenu *formatMenu;
  QMenu *helpMenu;
  //QAction
  QAction *openAct;
  QAction *aboutAct;

  //folder store path 
  QString cur_folder_path;
  QString cur_pcd_file;
  QString cur_label_file;

  //C std Area
  std::vector<KITTI_Label> label_holder;
  int prev_label_index; //For save label

};

#endif //PCL_LABELLER_H
