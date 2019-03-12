// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 


#include "pcl_labeller.h"
#include "config.h"
#include "../build/ui_pcl_labeller.h"

//###################################################
//
//          Core function
//
//###################################################

//Constructor
PCL_Labeller::PCL_Labeller (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCL_Labeller)
{
  prev_label_index = -1;

  // prepareUI();//Setup the lazy pointer array
  ui->setupUi (this);
  this->setWindowTitle (WINDOW_TITLE);
  label_UI_enable(LABEL_UI_CUROR, false);
  label_UI_enable(LABEL_UI, false);
  
  // Setup the cloud pointer
  display_cloud.reset (new PointCloudT);
  // The number of points in the cloud

  // Set up the QVTK window
  statusBar()->showMessage(tr("Setup PCL Visualizer"));
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false)); //Create PCL Viualizer
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();
  viewer->addCoordinateSystem (1.0);
  viewer->setCameraPosition(10, 0, 10, 0, 0, 1);
  ui->qvtkWidget->GetRenderWindow()->Render();
  statusBar()->showMessage(tr("Finish setup PCL Visualizer"));

  //QT Works
  createActions();
  createMenus();
  

  statusBar()->showMessage(tr("Ready"));
}
//Destructor
PCL_Labeller::~PCL_Labeller ()
{
  delete ui;
}

void 
PCL_Labeller::about()
{
   QMessageBox::about(this, tr(ABOUT_TITLE), tr(ABOUT_CONTENT));
}
