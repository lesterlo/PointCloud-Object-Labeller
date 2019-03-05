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

// void 
// PCL_Labeller::prepareUI()
// {
//   ui_sk_storage[0] = ui->sk_n1x_dsb;
//   ui_sk_storage[1] = ui->sk_n1y_dsb;
//   ui_sk_storage[2] = ui->sk_n1z_dsb;
//   ui_sk_storage[3] = ui->sk_n2x_dsb;
//   ui_sk_storage[4] = ui->sk_n2y_dsb;
//   ui_sk_storage[5] = ui->sk_n2z_dsb;
//   ui_sk_storage[6] = ui->sk_n3x_dsb;
//   ui_sk_storage[7] = ui->sk_n3y_dsb;
//   ui_sk_storage[8] = ui->sk_n3z_dsb;
//   ui_sk_storage[9] = ui->sk_n4x_dsb;
//   ui_sk_storage[10] = ui->sk_n4y_dsb;
//   ui_sk_storage[11] = ui->sk_n4z_dsb;
//   ui_sk_storage[12] = ui->sk_n5x_dsb;
//   ui_sk_storage[13] = ui->sk_n5y_dsb;
//   ui_sk_storage[14] = ui->sk_n5z_dsb;
//   ui_sk_storage[15] = ui->sk_n6x_dsb;
//   ui_sk_storage[16] = ui->sk_n6y_dsb;
//   ui_sk_storage[17] = ui->sk_n6z_dsb;
//   ui_sk_storage[18] = ui->sk_n7x_dsb;
//   ui_sk_storage[19] = ui->sk_n7y_dsb;
//   ui_sk_storage[20] = ui->sk_n7z_dsb;
//   ui_sk_storage[21] = ui->sk_n8x_dsb;
//   ui_sk_storage[22] = ui->sk_n8y_dsb;
//   ui_sk_storage[23] = ui->sk_n8z_dsb;
//   ui_sk_storage[24] = ui->sk_n9x_dsb;
//   ui_sk_storage[25] = ui->sk_n9y_dsb;
//   ui_sk_storage[26] = ui->sk_n9z_dsb;
//   ui_sk_storage[27] = ui->sk_n10x_dsb;
//   ui_sk_storage[28] = ui->sk_n10y_dsb;
//   ui_sk_storage[29] = ui->sk_n10z_dsb;
//   ui_sk_storage[30] = ui->sk_n11x_dsb;
//   ui_sk_storage[31] = ui->sk_n11y_dsb;
//   ui_sk_storage[32] = ui->sk_n11z_dsb;
//   ui_sk_storage[33] = ui->sk_n12x_dsb;
//   ui_sk_storage[34] = ui->sk_n12y_dsb;
//   ui_sk_storage[35] = ui->sk_n12z_dsb;
//   ui_sk_storage[36] = ui->sk_n13x_dsb;
//   ui_sk_storage[37] = ui->sk_n13y_dsb;
//   ui_sk_storage[38] = ui->sk_n13z_dsb;
//   ui_sk_storage[39] = ui->sk_n14x_dsb;
//   ui_sk_storage[40] = ui->sk_n14y_dsb;
//   ui_sk_storage[41] = ui->sk_n14z_dsb;
//   ui_sk_storage[42] = ui->sk_n15x_dsb;
//   ui_sk_storage[43] = ui->sk_n15y_dsb;
//   ui_sk_storage[44] = ui->sk_n15z_dsb;
// }


//###################################################
//
//          Initialize function
//
//###################################################

//QT initialzation 
void 
PCL_Labeller::createActions()
{
  //@File->Open action
  openAct = new QAction(tr("&Open..."), this);
  openAct->setShortcuts(QKeySequence::Open);
  openAct->setStatusTip(tr("Open an existing file"));
  connect(openAct, &QAction::triggered, this, &PCL_Labeller::openFolder);
  //@Help->About action
  aboutAct = new QAction(tr("&About"), this);
  aboutAct->setStatusTip(tr("Show the application's About box"));
  connect(aboutAct, &QAction::triggered, this, &PCL_Labeller::about);


  //Connect signal
  //currentItemChanged will emit signal rather readd the items on the listWidget
  connect(ui->file_listWidget, SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),  this, SLOT(onFileListItemClicked(QListWidgetItem*)));
  connect(ui->label_listWidget, SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),  this, SLOT(onLabelListItemClicked(QListWidgetItem*)));
  connect(ui->save_pb, SIGNAL(pressed()),  this, SLOT(onSaveButtonClicked()));
  connect(ui->newLabel_pb, SIGNAL(pressed()),  this, SLOT(onInsertLabelButtonClicked()));
  connect(ui->deleteLabel_pb, SIGNAL(pressed()),  this, SLOT(onDeleteLabelButtonClicked()));
  connect(ui->prevPCD_pb, SIGNAL(pressed()),  this, SLOT(onPrevPCDButtonClicked()));
  connect(ui->nextPCD_pb, SIGNAL(pressed()),  this, SLOT(onNextPCDButtonClicked()));
  labelUI_Signal_enable(true);
}

void 
PCL_Labeller::createMenus()
{
  menuBar()->setNativeMenuBar(false);//Must add to show the menu bar
  //File Menu tab
  fileMenu = menuBar()->addMenu(tr("&File"));
  fileMenu->addAction(openAct);

  //Format Menu tab
  formatMenu = menuBar()->addMenu(tr("&Format"));

  //Help Menu tab
  helpMenu = menuBar()->addMenu(tr("&Help"));
  helpMenu->addAction(aboutAct);
}

#ifndef QT_NO_CONTEXTMENU
void 
PCL_Labeller::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    menu.exec(event->globalPos());
}
#endif // QT_NO_CONTEXTMENU

//###################################################
//
//          QT Signal Related function
//
//###################################################

void 
PCL_Labeller::labelUI_Signal_enable(bool state)
{
  if(state == true)
  {
    //Normal
    connect(ui->label_le, SIGNAL(editingFinished()),  this, SLOT(onLabelEditFinish()));
    connect(ui->centerx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->centery_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->centerz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->rotatex_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->rotatey_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->rotatez_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->widthx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->heighty_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->depthz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));

    //Joint Tab
    // for(int i; i < SKELETON_NODE_COUNT*3; i++)
    //   connect(ui_sk_storage[i] , SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));

    connect(ui->sk_n1x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n1y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n1z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n2x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n2y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n2z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n3x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n3y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n3z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n4x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n4y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n4z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n5x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n5y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n5z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n6x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n6y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n6z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n7x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n7y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n7z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n8x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n8y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n8z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n9x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n9y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n9z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n10x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n10y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n10z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n11x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n11y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n11z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n12x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n12y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n12z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n13x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n13y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n13z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n14x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n14y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n14z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n15x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n15y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->sk_n15z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
  }
  else
  {
    disconnect(ui->label_le, SIGNAL(editingFinished()),  this, SLOT(onLabelEditFinish()));
    disconnect(ui->centerx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->centery_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->centerz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->rotatex_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->rotatey_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->rotatez_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->widthx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->heighty_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->depthz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));

    //Joint Tab
    // for(int i; i < SKELETON_NODE_COUNT*3; i++)
    //   disconnect(ui_sk_storage[i] , SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));


    disconnect(ui->sk_n1x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n1y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n1z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n2x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n2y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n2z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n3x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n3y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n3z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n4x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n4y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n4z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n5x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n5y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n5z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n6x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n6y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n6z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n7x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n7y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n7z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n8x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n8y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n8z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n9x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n9y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n9z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n10x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n10y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n10z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n11x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n11y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n11z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n12x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n12y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n12z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n13x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n13y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n13z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n14x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n14y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n14z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n15x_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n15y_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->sk_n15z_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
  }
}

//###################################################
//
//          QT Signal Slot function (handler)
//
//###################################################

//Function is triggered when the file listWidget item is changed
void 
PCL_Labeller::onFileListItemClicked(QListWidgetItem* item)
{
  if(item != NULL)//Prevent seg fault when clearing all items on QListWidget
  {
    //Pre open procedure
    label_holder.clear();
    cur_pcd_file = item->text();//Get the pcd file name
    this->setWindowTitle(cur_pcd_file+QString("   -   ")+cur_folder_path+QString("   -   ")+QString(WINDOW_TITLE));//Change title
    
    //Read PointCloud data
    display_cloud.reset (new PointCloudT);
    statusBar()->showMessage(tr(LOADING_PCD_FILE)+cur_pcd_file);
    //maybe use other thread to read, current setup will occupy the UI thread.
    clean_viewer();
    int reValue = pcl::io::loadPCDFile<PointT> ((cur_folder_path+QString("/")+cur_pcd_file).toStdString(), *display_cloud);
    if (reValue == -1) //cannot load the pcd file
    {
      //Set ui disable when the system cannot load the PCD file
      label_UI_enable(LABEL_UI, false);
      
      statusBar()->showMessage(tr(CANNOT_OPEN_PCD)+cur_pcd_file);
      QMessageBox::about(this, tr(ERROR_DIALOG_TITLE),tr(CANNOT_OPEN_PCD)+cur_pcd_file);    
    }
    else //successfully load the pcd file
    {
      statusBar()->showMessage(tr(CAN_OPEN_PCD)+cur_pcd_file);
      viewer->addPointCloud(display_cloud, "cloud");// Add the current pointcloud to the viewer
      ui->qvtkWidget->GetRenderWindow()->Render(); //Update the qvtk widget
      //Prepare the label file name
      cur_label_file = QString::fromStdString(std::regex_replace(cur_pcd_file.toStdString(), std::regex(".pcd"), ".hst"));//Replace .pcd to .hst
      //Then load the label file
      if(read_label() == 0)
      {//NO error
        label_UI_enable(LABEL_UI, true);
        //Add a label member to the label list ui
        prev_label_index = -1;//Important, clear the previous label_index since the file is change
        clearLabelUI();
        construct_labelWidget();
        drawAllLabel(-1);//Update the UI that the show elemnt is same with the label_holder vector
      }
      else
      {//Have error
        label_UI_enable(LABEL_UI, false);
      }
    }//END-successfully load the pcd file
  }
}

//Function is triggered when the label listWidget item is changed
void 
PCL_Labeller::onLabelListItemClicked(QListWidgetItem* item)
{
  if(item != NULL){//Prevent seg fault when clearing all items on QListWidget
    int cur_label_index = ui->label_listWidget->currentRow();//Load the current label index 

    labelUI_Signal_enable(false);//Important, disable signal when call setValue function which is not a user input

    //Save the previous Selected label from the UI to the label_holder vector
    if(prev_label_index != -1)//Prevert seg fault when no previous label are selected
    {
      label_holder[prev_label_index].name = ui->label_le->text().toStdString();
      ui->label_listWidget->item(prev_label_index)->setText(ui->label_le->text());//Change the listWidget item shown name
      label_holder.at(prev_label_index).center_x = ui->centerx_dsb->value();
      label_holder.at(prev_label_index).center_y = ui->centery_dsb->value();
      label_holder.at(prev_label_index).center_z = ui->centerz_dsb->value();
      label_holder.at(prev_label_index).x_size = ui->widthx_dsb->value();
      label_holder.at(prev_label_index).y_size = ui->heighty_dsb->value();
      label_holder.at(prev_label_index).z_size = ui->depthz_dsb->value();
      label_holder.at(prev_label_index).rotate_x = ui->rotatex_dsb->value();
      label_holder.at(prev_label_index).rotate_y = ui->rotatey_dsb->value();
      label_holder.at(prev_label_index).rotate_z = ui->rotatez_dsb->value();

      //Joint Tab
      label_holder.at(prev_label_index).node[0].x = ui->sk_n1x_dsb->value();
      label_holder.at(prev_label_index).node[0].y = ui->sk_n1y_dsb->value();
      label_holder.at(prev_label_index).node[0].z = ui->sk_n1z_dsb->value();
      label_holder.at(prev_label_index).node[1].x = ui->sk_n2x_dsb->value();
      label_holder.at(prev_label_index).node[1].y = ui->sk_n2y_dsb->value();
      label_holder.at(prev_label_index).node[1].z = ui->sk_n2z_dsb->value();
      label_holder.at(prev_label_index).node[2].x = ui->sk_n3x_dsb->value();
      label_holder.at(prev_label_index).node[2].y = ui->sk_n3y_dsb->value();
      label_holder.at(prev_label_index).node[2].z = ui->sk_n3z_dsb->value();
      label_holder.at(prev_label_index).node[3].x = ui->sk_n4x_dsb->value();
      label_holder.at(prev_label_index).node[3].y = ui->sk_n4y_dsb->value();
      label_holder.at(prev_label_index).node[3].z = ui->sk_n4z_dsb->value();
      label_holder.at(prev_label_index).node[4].x = ui->sk_n5x_dsb->value();
      label_holder.at(prev_label_index).node[4].y = ui->sk_n5y_dsb->value();
      label_holder.at(prev_label_index).node[4].z = ui->sk_n5z_dsb->value();
      label_holder.at(prev_label_index).node[5].x = ui->sk_n6x_dsb->value();
      label_holder.at(prev_label_index).node[5].y = ui->sk_n6y_dsb->value();
      label_holder.at(prev_label_index).node[5].z = ui->sk_n6z_dsb->value();
      label_holder.at(prev_label_index).node[6].x = ui->sk_n7x_dsb->value();
      label_holder.at(prev_label_index).node[6].y = ui->sk_n7y_dsb->value();
      label_holder.at(prev_label_index).node[6].z = ui->sk_n7z_dsb->value();
      label_holder.at(prev_label_index).node[7].x = ui->sk_n8x_dsb->value();
      label_holder.at(prev_label_index).node[7].y = ui->sk_n8y_dsb->value();
      label_holder.at(prev_label_index).node[7].z = ui->sk_n8z_dsb->value();
      label_holder.at(prev_label_index).node[8].x = ui->sk_n9x_dsb->value();
      label_holder.at(prev_label_index).node[8].y = ui->sk_n9y_dsb->value();
      label_holder.at(prev_label_index).node[8].z = ui->sk_n9z_dsb->value();
      label_holder.at(prev_label_index).node[9].x = ui->sk_n10x_dsb->value();
      label_holder.at(prev_label_index).node[9].y = ui->sk_n10y_dsb->value();
      label_holder.at(prev_label_index).node[9].z = ui->sk_n10z_dsb->value();
      label_holder.at(prev_label_index).node[10].x = ui->sk_n11x_dsb->value();
      label_holder.at(prev_label_index).node[10].y = ui->sk_n11y_dsb->value();
      label_holder.at(prev_label_index).node[10].z = ui->sk_n11z_dsb->value();
      label_holder.at(prev_label_index).node[11].x = ui->sk_n12x_dsb->value();
      label_holder.at(prev_label_index).node[11].y = ui->sk_n12y_dsb->value();
      label_holder.at(prev_label_index).node[11].z = ui->sk_n12z_dsb->value();
      label_holder.at(prev_label_index).node[12].x = ui->sk_n13x_dsb->value();
      label_holder.at(prev_label_index).node[12].y = ui->sk_n13y_dsb->value();
      label_holder.at(prev_label_index).node[12].z = ui->sk_n13z_dsb->value();
      label_holder.at(prev_label_index).node[13].x = ui->sk_n14x_dsb->value();
      label_holder.at(prev_label_index).node[13].y = ui->sk_n14y_dsb->value();
      label_holder.at(prev_label_index).node[13].z = ui->sk_n14z_dsb->value();
      label_holder.at(prev_label_index).node[14].x = ui->sk_n15x_dsb->value();
      label_holder.at(prev_label_index).node[14].y = ui->sk_n15y_dsb->value();
      label_holder.at(prev_label_index).node[14].z = ui->sk_n15z_dsb->value();
      

      // for(int i=0; i < SKELETON_NODE_COUNT; i++)
      // {
      //   label_holder.at(prev_label_index).node[i].x = ui_sk_storage[i*3]->value();
      //   label_holder.at(prev_label_index).node[i].y = ui_sk_storage[i*3+1]->value();
      //   label_holder.at(prev_label_index).node[i].z = ui_sk_storage[i*3+2]->value();
      // }
    }
    //Apply the new data from the label_holder vector to the UI
    ui->label_le->setText(QString::fromStdString(label_holder[cur_label_index].name));
    ui->centerx_dsb->setValue(label_holder.at(cur_label_index).center_x);
    ui->centery_dsb->setValue(label_holder.at(cur_label_index).center_y);
    ui->centerz_dsb->setValue(label_holder.at(cur_label_index).center_z);
    ui->widthx_dsb->setValue(label_holder.at(cur_label_index).x_size);
    ui->heighty_dsb->setValue(label_holder.at(cur_label_index).y_size);
    ui->depthz_dsb->setValue(label_holder.at(cur_label_index).z_size);
    ui->rotatex_dsb->setValue(label_holder.at(cur_label_index).rotate_x);
    ui->rotatey_dsb->setValue(label_holder.at(cur_label_index).rotate_y);
    ui->rotatez_dsb->setValue(label_holder.at(cur_label_index).rotate_z);

    //Joint Tab
    ui->sk_n1x_dsb->setValue(  label_holder.at(cur_label_index).node[0].x );
    ui->sk_n1y_dsb->setValue(  label_holder.at(cur_label_index).node[0].y );
    ui->sk_n1z_dsb->setValue(  label_holder.at(cur_label_index).node[0].z );
    ui->sk_n2x_dsb->setValue(  label_holder.at(cur_label_index).node[1].x );
    ui->sk_n2y_dsb->setValue(  label_holder.at(cur_label_index).node[1].y );
    ui->sk_n2z_dsb->setValue(  label_holder.at(cur_label_index).node[1].z );
    ui->sk_n3x_dsb->setValue(  label_holder.at(cur_label_index).node[2].x );
    ui->sk_n3y_dsb->setValue(  label_holder.at(cur_label_index).node[2].y );
    ui->sk_n3z_dsb->setValue(  label_holder.at(cur_label_index).node[2].z );
    ui->sk_n4x_dsb->setValue(  label_holder.at(cur_label_index).node[3].x );
    ui->sk_n4y_dsb->setValue(  label_holder.at(cur_label_index).node[3].y );
    ui->sk_n4z_dsb->setValue(  label_holder.at(cur_label_index).node[3].z );
    ui->sk_n5x_dsb->setValue(  label_holder.at(cur_label_index).node[4].x );
    ui->sk_n5y_dsb->setValue(  label_holder.at(cur_label_index).node[4].y );
    ui->sk_n5z_dsb->setValue(  label_holder.at(cur_label_index).node[4].z );
    ui->sk_n6x_dsb->setValue(  label_holder.at(cur_label_index).node[5].x );
    ui->sk_n6y_dsb->setValue(  label_holder.at(cur_label_index).node[5].y );
    ui->sk_n6z_dsb->setValue(  label_holder.at(cur_label_index).node[5].z );
    ui->sk_n7x_dsb->setValue(  label_holder.at(cur_label_index).node[6].x );
    ui->sk_n7y_dsb->setValue(  label_holder.at(cur_label_index).node[6].y );
    ui->sk_n7z_dsb->setValue(  label_holder.at(cur_label_index).node[6].z );
    ui->sk_n8x_dsb->setValue(  label_holder.at(cur_label_index).node[7].x );
    ui->sk_n8y_dsb->setValue(  label_holder.at(cur_label_index).node[7].y );
    ui->sk_n8z_dsb->setValue(  label_holder.at(cur_label_index).node[7].z );
    ui->sk_n9x_dsb->setValue(  label_holder.at(cur_label_index).node[8].x );
    ui->sk_n9y_dsb->setValue(  label_holder.at(cur_label_index).node[8].y );
    ui->sk_n9z_dsb->setValue(  label_holder.at(cur_label_index).node[8].z );
    ui->sk_n10x_dsb->setValue( label_holder.at(cur_label_index).node[9].x );
    ui->sk_n10y_dsb->setValue( label_holder.at(cur_label_index).node[9].y );
    ui->sk_n10z_dsb->setValue( label_holder.at(cur_label_index).node[9].z );
    ui->sk_n11x_dsb->setValue( label_holder.at(cur_label_index).node[10].x);
    ui->sk_n11y_dsb->setValue( label_holder.at(cur_label_index).node[10].y);
    ui->sk_n11z_dsb->setValue( label_holder.at(cur_label_index).node[10].z);
    ui->sk_n12x_dsb->setValue( label_holder.at(cur_label_index).node[11].x);
    ui->sk_n12y_dsb->setValue( label_holder.at(cur_label_index).node[11].y);
    ui->sk_n12z_dsb->setValue( label_holder.at(cur_label_index).node[11].z);
    ui->sk_n13x_dsb->setValue( label_holder.at(cur_label_index).node[12].x);
    ui->sk_n13y_dsb->setValue( label_holder.at(cur_label_index).node[12].y);
    ui->sk_n13z_dsb->setValue( label_holder.at(cur_label_index).node[12].z);
    ui->sk_n14x_dsb->setValue( label_holder.at(cur_label_index).node[13].x);
    ui->sk_n14y_dsb->setValue( label_holder.at(cur_label_index).node[13].y);
    ui->sk_n14z_dsb->setValue( label_holder.at(cur_label_index).node[13].z);
    ui->sk_n15x_dsb->setValue( label_holder.at(cur_label_index).node[14].x);
    ui->sk_n15z_dsb->setValue( label_holder.at(cur_label_index).node[14].y);
    ui->sk_n15y_dsb->setValue( label_holder.at(cur_label_index).node[14].z);
      
    
    // for(int i=0; i < SKELETON_NODE_COUNT; i++)
    // {
    //   ui_sk_storage[i*3]->setValue(label_holder.at(cur_label_index).node[i].x);
    //   ui_sk_storage[i*3+1]->setValue(label_holder.at(cur_label_index).node[i].y);
    //   ui_sk_storage[i*3+2]->setValue(label_holder.at(cur_label_index).node[i].z);
    // }

    prev_label_index = ui->label_listWidget->currentRow();//Update the previous index 
    //Change the color??, highted item -> wireframe, others, solid color
    drawAllLabel(ui->label_listWidget->currentRow());//Update the UI that the show elemnt is same with the label_holder vector

    labelUI_Signal_enable(true);//Important, enable back the signal
  }
}

//Function is triggered when the save button is pressed
void
PCL_Labeller::onSaveButtonClicked()
{
  write_label();
}

//Function is triggered when the insert button is pressed
void 
PCL_Labeller::onInsertLabelButtonClicked()
{
  //Prepare new label
  HSTM_Label newLabel;
  //Initialize the parameter
  newLabel.name = "NO_NAME";
  newLabel.center_x = 0.0;
  newLabel.center_y = 0;
  newLabel.center_z = 0.0;
  newLabel.x_size =0.0;
  newLabel.y_size = 0.0;
  newLabel.z_size = 0.0;
  newLabel.rotate_x = 0.0;
  newLabel.rotate_y = 0.0;
  newLabel.rotate_z = 0.0;
  for(int i=0; i < SKELETON_NODE_COUNT; i++)
  {
    newLabel.node[i].x = 0.0;
    newLabel.node[i].y = 0.0;
    newLabel.node[i].z = 0.0;
  }

  //Insert the new element to the tail of the list
  label_holder.push_back(newLabel);
  ui->label_listWidget->addItem(QString::fromStdString(newLabel.name));//Insert the new element to the tail of the UI list
  drawAllLabel(ui->label_listWidget->currentRow());//Update the UI that the show element is same with the label_holder vector
}

//Function is triggered when the Delete button is pressed
void 
PCL_Labeller::onDeleteLabelButtonClicked()
{

  //Delete the current index of item
  if(label_holder.size() > 0)
  {
    int selected_item_index = ui->label_listWidget->currentRow();

    if(selected_item_index != -1)
    {
      //Remove on std vector
      prev_label_index = -1;//Important, clear the previous label_index since the previos label has already deleted
      label_holder.erase(label_holder.begin()+selected_item_index);//Delete the item that in the std vector
      ui->label_listWidget->setCurrentRow(selected_item_index-1);//Move up the selected row   
      //Remove item on label listwidget UI
      delete ui->label_listWidget->takeItem(selected_item_index);//Delete the item that in the ui lisWidget
      drawAllLabel(ui->label_listWidget->currentRow());//Update the UI that the show elemnt is same with the label_holder vector
      statusBar()->showMessage(tr(CAN_DELETE_LABEL));//Show the status
    }
    else
    {
      statusBar()->showMessage(tr("Please select something to delete"));//Show the status
    }
  }
  else
  {
    statusBar()->showMessage(tr(NONEED_DELETE_LABEL));//Show the status
  }
}

//Function is triggered when the label is out of focus or pressed enter
void
PCL_Labeller::onLabelEditFinish()
{
  //Render current selection only
  int selected_item_index = ui->label_listWidget->currentRow();
  if(label_holder.size() > 0 && selected_item_index != -1)
  {
    //Save the update input to the label_holder Vector
    label_holder.at(selected_item_index).name = ui->label_le->text().toStdString();
    ui->label_listWidget->item(selected_item_index)->setText(ui->label_le->text());//Change the listWidget item shown name
    label_holder.at(selected_item_index).center_x = ui->centerx_dsb->value();
    label_holder.at(selected_item_index).center_y = ui->centery_dsb->value();
    label_holder.at(selected_item_index).center_z = ui->centerz_dsb->value();
    label_holder.at(selected_item_index).x_size = ui->widthx_dsb->value();
    label_holder.at(selected_item_index).y_size = ui->heighty_dsb->value();
    label_holder.at(selected_item_index).z_size = ui->depthz_dsb->value();
    label_holder.at(selected_item_index).rotate_x = ui->rotatex_dsb->value();
    label_holder.at(selected_item_index).rotate_y = ui->rotatey_dsb->value();
    label_holder.at(selected_item_index).rotate_z = ui->rotatez_dsb->value();

    //Joint Tab
    label_holder.at(selected_item_index).node[0].x = ui->sk_n1x_dsb->value();
    label_holder.at(selected_item_index).node[0].y = ui->sk_n1y_dsb->value();
    label_holder.at(selected_item_index).node[0].z = ui->sk_n1z_dsb->value();
    label_holder.at(selected_item_index).node[1].x = ui->sk_n2x_dsb->value();
    label_holder.at(selected_item_index).node[1].y = ui->sk_n2y_dsb->value();
    label_holder.at(selected_item_index).node[1].z = ui->sk_n2z_dsb->value();
    label_holder.at(selected_item_index).node[2].x = ui->sk_n3x_dsb->value();
    label_holder.at(selected_item_index).node[2].y = ui->sk_n3y_dsb->value();
    label_holder.at(selected_item_index).node[2].z = ui->sk_n3z_dsb->value();
    label_holder.at(selected_item_index).node[3].x = ui->sk_n4x_dsb->value();
    label_holder.at(selected_item_index).node[3].y = ui->sk_n4y_dsb->value();
    label_holder.at(selected_item_index).node[3].z = ui->sk_n4z_dsb->value();
    label_holder.at(selected_item_index).node[4].x = ui->sk_n5x_dsb->value();
    label_holder.at(selected_item_index).node[4].y = ui->sk_n5y_dsb->value();
    label_holder.at(selected_item_index).node[4].z = ui->sk_n5z_dsb->value();
    label_holder.at(selected_item_index).node[5].x = ui->sk_n6x_dsb->value();
    label_holder.at(selected_item_index).node[5].y = ui->sk_n6y_dsb->value();
    label_holder.at(selected_item_index).node[5].z = ui->sk_n6z_dsb->value();
    label_holder.at(selected_item_index).node[6].x = ui->sk_n7x_dsb->value();
    label_holder.at(selected_item_index).node[6].y = ui->sk_n7y_dsb->value();
    label_holder.at(selected_item_index).node[6].z = ui->sk_n7z_dsb->value();
    label_holder.at(selected_item_index).node[7].x = ui->sk_n8x_dsb->value();
    label_holder.at(selected_item_index).node[7].y = ui->sk_n8y_dsb->value();
    label_holder.at(selected_item_index).node[7].z = ui->sk_n8z_dsb->value();
    label_holder.at(selected_item_index).node[8].x = ui->sk_n9x_dsb->value();
    label_holder.at(selected_item_index).node[8].y = ui->sk_n9y_dsb->value();
    label_holder.at(selected_item_index).node[8].z = ui->sk_n9z_dsb->value();
    label_holder.at(selected_item_index).node[9].x = ui->sk_n10x_dsb->value();
    label_holder.at(selected_item_index).node[9].y = ui->sk_n10y_dsb->value();
    label_holder.at(selected_item_index).node[9].z = ui->sk_n10z_dsb->value();
    label_holder.at(selected_item_index).node[10].x = ui->sk_n11x_dsb->value();
    label_holder.at(selected_item_index).node[10].y = ui->sk_n11y_dsb->value();
    label_holder.at(selected_item_index).node[10].z = ui->sk_n11z_dsb->value();
    label_holder.at(selected_item_index).node[11].x = ui->sk_n12x_dsb->value();
    label_holder.at(selected_item_index).node[11].y = ui->sk_n12y_dsb->value();
    label_holder.at(selected_item_index).node[11].z = ui->sk_n12z_dsb->value();
    label_holder.at(selected_item_index).node[12].x = ui->sk_n13x_dsb->value();
    label_holder.at(selected_item_index).node[12].y = ui->sk_n13y_dsb->value();
    label_holder.at(selected_item_index).node[12].z = ui->sk_n13z_dsb->value();
    label_holder.at(selected_item_index).node[13].x = ui->sk_n14x_dsb->value();
    label_holder.at(selected_item_index).node[13].y = ui->sk_n14y_dsb->value();
    label_holder.at(selected_item_index).node[13].z = ui->sk_n14z_dsb->value();
    label_holder.at(selected_item_index).node[14].x = ui->sk_n15x_dsb->value();
    label_holder.at(selected_item_index).node[14].y = ui->sk_n15y_dsb->value();
    label_holder.at(selected_item_index).node[14].z = ui->sk_n15z_dsb->value();

    // for(int i=0; i < SKELETON_NODE_COUNT; i++)
    // {
    //   label_holder.at(selected_item_index).node[i].x = ui_sk_storage[i*3]->value();
    //   label_holder.at(selected_item_index).node[i].y = ui_sk_storage[i*3+1]->value();
    //   label_holder.at(selected_item_index).node[i].z = ui_sk_storage[i*3+2]->value();
    // }

    drawAllLabel(ui->label_listWidget->currentRow());//Update the UI that the shown element is same with the label_holder vector
  }
}

//Function is triggered when the label is out of focus or pressed enter
void 
PCL_Labeller::onLabelValueChange(double)
{
  onLabelEditFinish();
}

void
PCL_Labeller::onPrevPCDButtonClicked()
{
  int cur_row = ui->file_listWidget->currentRow();
  int total_row = ui->file_listWidget->count();
  if((cur_row-1) >= 0)
  {
    ui->prevPCD_pb->setEnabled(true);
    ui->nextPCD_pb->setEnabled(true);
    ui->file_listWidget->setCurrentRow(cur_row-1);
  }
  else
  {
    ui->prevPCD_pb->setEnabled(false);
  }
}

void
PCL_Labeller::onNextPCDButtonClicked()
{
  int cur_row = ui->file_listWidget->currentRow();
  int total_row = ui->file_listWidget->count();
  if((cur_row+1) < total_row)
  {
    ui->prevPCD_pb->setEnabled(true);
    ui->nextPCD_pb->setEnabled(true);
    ui->file_listWidget->setCurrentRow(cur_row+1);
    ui->label_listWidget->setCurrentRow(-1);
  }
  else
  {
    ui->nextPCD_pb->setEnabled(false);
  }
}

//###################################################
//
//          QT UI Helper function
//
//###################################################

void
PCL_Labeller::clearLabelUI()
{
  labelUI_Signal_enable(false);//Important, disable signal when call setValue function which is not a user input
  ui->label_le->setText(QString(""));
  ui->centerx_dsb->setValue(0.0);
  ui->centery_dsb->setValue(0.0);
  ui->centerz_dsb->setValue(0.0);
  ui->widthx_dsb->setValue(0.0);
  ui->heighty_dsb->setValue(0.0);
  ui->depthz_dsb->setValue(0.0);
  ui->rotatex_dsb->setValue(0.0);
  ui->rotatey_dsb->setValue(0.0);
  ui->rotatez_dsb->setValue(0.0);

  //Joint Tab
  ui->sk_n1x_dsb->setValue( 0.0);
  ui->sk_n1y_dsb->setValue( 0.0);
  ui->sk_n1z_dsb->setValue( 0.0);
  ui->sk_n2x_dsb->setValue( 0.0);
  ui->sk_n2y_dsb->setValue( 0.0);
  ui->sk_n2z_dsb->setValue( 0.0);
  ui->sk_n3x_dsb->setValue( 0.0);
  ui->sk_n3y_dsb->setValue( 0.0);
  ui->sk_n3z_dsb->setValue( 0.0);
  ui->sk_n4x_dsb->setValue( 0.0);
  ui->sk_n4y_dsb->setValue( 0.0);
  ui->sk_n4z_dsb->setValue( 0.0);
  ui->sk_n5x_dsb->setValue( 0.0);
  ui->sk_n5y_dsb->setValue( 0.0);
  ui->sk_n5z_dsb->setValue( 0.0);
  ui->sk_n6x_dsb->setValue( 0.0);
  ui->sk_n6y_dsb->setValue( 0.0);
  ui->sk_n6z_dsb->setValue( 0.0);
  ui->sk_n7x_dsb->setValue( 0.0);
  ui->sk_n7y_dsb->setValue( 0.0);
  ui->sk_n7z_dsb->setValue( 0.0);
  ui->sk_n8x_dsb->setValue( 0.0);
  ui->sk_n8y_dsb->setValue( 0.0);
  ui->sk_n8z_dsb->setValue( 0.0);
  ui->sk_n9x_dsb->setValue( 0.0);
  ui->sk_n9y_dsb->setValue( 0.0);
  ui->sk_n9z_dsb->setValue( 0.0);
  ui->sk_n10x_dsb->setValue(0.0);
  ui->sk_n10y_dsb->setValue(0.0);
  ui->sk_n10z_dsb->setValue(0.0);
  ui->sk_n11x_dsb->setValue(0.0);
  ui->sk_n11y_dsb->setValue(0.0);
  ui->sk_n11z_dsb->setValue(0.0);
  ui->sk_n12x_dsb->setValue(0.0);
  ui->sk_n12y_dsb->setValue(0.0);
  ui->sk_n12z_dsb->setValue(0.0);
  ui->sk_n13x_dsb->setValue(0.0);
  ui->sk_n13y_dsb->setValue(0.0);
  ui->sk_n13z_dsb->setValue(0.0);
  ui->sk_n14x_dsb->setValue(0.0);
  ui->sk_n14y_dsb->setValue(0.0);
  ui->sk_n14z_dsb->setValue(0.0);
  ui->sk_n15x_dsb->setValue(0.0);
  ui->sk_n15z_dsb->setValue(0.0);
  ui->sk_n15y_dsb->setValue(0.0);
  // for(int i; i < SKELETON_NODE_COUNT*3; i++)
  //     ui_sk_storage[i]->setValue(0.0);
  labelUI_Signal_enable(true);//Important, enable back the signal
}


void
PCL_Labeller::construct_labelWidget()
{
  QStringList pc_files;

  //Get the label name from the contain
  for(HSTM_Label item: label_holder)
  {
    pc_files << QString::fromStdString(item.name);
  }
    
  //Add items to the label tab
  ui->label_listWidget->clear();
  ui->label_listWidget->addItems(pc_files);
}

void 
PCL_Labeller::label_UI_enable(int level, bool state)
{
    //label related ui
    switch(level)
    {
      case LABEL_UI_CUROR:
        ui->prevPCD_pb->setEnabled(state);
        ui->nextPCD_pb->setEnabled(state);
        break;
      case LABEL_UI:
        ui->label_le->setEnabled(state);
        ui->centerx_dsb->setEnabled(state);
        ui->centery_dsb->setEnabled(state);
        ui->centerz_dsb->setEnabled(state);
        ui->rotatex_dsb->setEnabled(state);
        ui->rotatey_dsb->setEnabled(state);
        ui->rotatez_dsb->setEnabled(state);
        ui->widthx_dsb->setEnabled(state);
        ui->heighty_dsb->setEnabled(state);
        ui->depthz_dsb->setEnabled(state);
        ui->label_listWidget->setEnabled(state);
        ui->save_pb->setEnabled(state);
        ui->newLabel_pb->setEnabled(state);
        ui->deleteLabel_pb->setEnabled(state);

        //Joint Tab
        // for(int i; i < SKELETON_NODE_COUNT*3; i++)
        //   ui_sk_storage[i]->setEnabled(state);
        ui->sk_n1x_dsb->setEnabled(state);
        ui->sk_n1y_dsb->setEnabled(state);
        ui->sk_n1z_dsb->setEnabled(state);
        ui->sk_n2x_dsb->setEnabled(state);
        ui->sk_n2y_dsb->setEnabled(state);
        ui->sk_n2z_dsb->setEnabled(state);
        ui->sk_n3x_dsb->setEnabled(state);
        ui->sk_n3y_dsb->setEnabled(state);
        ui->sk_n3z_dsb->setEnabled(state);
        ui->sk_n4x_dsb->setEnabled(state);
        ui->sk_n4y_dsb->setEnabled(state);
        ui->sk_n4z_dsb->setEnabled(state);
        ui->sk_n5x_dsb->setEnabled(state);
        ui->sk_n5y_dsb->setEnabled(state);
        ui->sk_n5z_dsb->setEnabled(state);
        ui->sk_n6x_dsb->setEnabled(state);
        ui->sk_n6y_dsb->setEnabled(state);
        ui->sk_n6z_dsb->setEnabled(state);
        ui->sk_n7x_dsb->setEnabled(state);
        ui->sk_n7y_dsb->setEnabled(state);
        ui->sk_n7z_dsb->setEnabled(state);
        ui->sk_n8x_dsb->setEnabled(state);
        ui->sk_n8y_dsb->setEnabled(state);
        ui->sk_n8z_dsb->setEnabled(state);
        ui->sk_n9x_dsb->setEnabled(state);
        ui->sk_n9y_dsb->setEnabled(state);
        ui->sk_n9z_dsb->setEnabled(state);
        ui->sk_n10x_dsb->setEnabled(state);
        ui->sk_n10y_dsb->setEnabled(state);
        ui->sk_n10z_dsb->setEnabled(state);
        ui->sk_n11x_dsb->setEnabled(state);
        ui->sk_n11y_dsb->setEnabled(state);
        ui->sk_n11z_dsb->setEnabled(state);
        ui->sk_n11x_dsb->setEnabled(state);
        ui->sk_n11y_dsb->setEnabled(state);
        ui->sk_n11z_dsb->setEnabled(state);
        ui->sk_n12x_dsb->setEnabled(state);
        ui->sk_n12y_dsb->setEnabled(state);
        ui->sk_n12z_dsb->setEnabled(state);
        ui->sk_n13x_dsb->setEnabled(state);
        ui->sk_n13y_dsb->setEnabled(state);
        ui->sk_n13z_dsb->setEnabled(state);
        ui->sk_n14x_dsb->setEnabled(state);
        ui->sk_n14y_dsb->setEnabled(state);
        ui->sk_n14z_dsb->setEnabled(state);
        ui->sk_n15x_dsb->setEnabled(state);
        ui->sk_n15y_dsb->setEnabled(state);
        ui->sk_n15z_dsb->setEnabled(state);
        break;
    }
}


void 
PCL_Labeller::drawAllLabel(int highlisted_index)
{
  int render_id = 0;

  //Remove all shape and Coordinate System before Drawing
  viewer->removeAllShapes();//Clear all bounding cube first
  viewer->removeAllCoordinateSystems(); //Clear all CoordinateSystem

  //Add the element
  viewer->addCoordinateSystem(); //Add the Center Axis
  for(HSTM_Label item : label_holder)
  {
    //Draw the Reference 3D axis
    if(render_id == highlisted_index)
      viewer->addCoordinateSystem(
        REF_AXIS_WIDTH, //Scale size
        Eigen::Affine3f( //Add Pose
          //Add translation
          Eigen::Translation3f( 
            item.center_x, 
            item.center_y, 
            item.center_z
          ) *
          //Add X, Y, Z rotation
          Eigen::AngleAxisf(TO_RAD(item.rotate_x), Eigen::Vector3f::UnitX()) * 
          Eigen::AngleAxisf(TO_RAD(item.rotate_y), Eigen::Vector3f::UnitY()) * 
          Eigen::AngleAxisf(TO_RAD(item.rotate_z), Eigen::Vector3f::UnitZ())
        ),
        std::to_string(render_id)//Specific ID
      );
    //Draw the bounding cube
    viewer->addCube(
      Eigen::Vector3f( //Translation of center
        item.center_x, 
        item.center_y, 
        item.center_z
      ), 
      Eigen::Quaternionf( //Rotation of the Cube
        Eigen::AngleAxisf(TO_RAD(item.rotate_x), Eigen::Vector3f::UnitX()) * 
        Eigen::AngleAxisf(TO_RAD(item.rotate_y), Eigen::Vector3f::UnitY()) * 
        Eigen::AngleAxisf(TO_RAD(item.rotate_z), Eigen::Vector3f::UnitZ())  
      ),
      item.x_size, //width
      item.y_size, //Height
      item.z_size,   //Depth
      std::to_string(render_id)//Specific ID
    );
    //Make the cube to wireframe
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      std::to_string(render_id));
    //Make the cube to red/green depends on the selected item
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
      render_id == highlisted_index ? 0.0:1.0, //R
      render_id == highlisted_index ? 1.0:0.0, //G
      0.0, //B
      std::to_string(render_id)
    );
    //Set the Wireframe line width
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
      BCUBE_LINEWIDTH, //Line Width
      std::to_string(render_id)
    );
    // viewer->addText3D(
    //   item.name, 
    //   pcl::PointXYZ(
    //     item.center_x + 15.0, 
    //     item.center_y + 15.0, 
    //     item.center_z + 15.0), 
    //   1.0, //textScale
    //   1.0, //red color
    //   1.0, //green color
    //   1.0, //blue color
    //   std::to_string(render_id)
    // );//Add anotation Text
    
    //Render the Skeleton
    
    render_id++;//Inrement the id counter
  }
  ui->qvtkWidget->GetRenderWindow()->Render();//Update the qvtkWidget to show the updated render view
}

//QT work action
void 
PCL_Labeller::openFolder()
{
  cur_folder_path = QFileDialog::getExistingDirectory(this, tr("Select one or more files to open"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if(cur_folder_path.isEmpty()) //Prevent seg fault
  {
    return;
  }
  else
  {
    this->setWindowTitle(cur_folder_path+QString("   -   ")+QString(WINDOW_TITLE));//Set the path to the title bar
    //Start read all files under the dir
    statusBar()->showMessage(tr(READING_FOLDER)+cur_folder_path);
    QDir folder(cur_folder_path);
    QStringList pc_files = folder.entryList(QStringList() << "*.pcd",QDir::Files);//Read only .pcd file
    
    //Add items to file tab
    ui->file_listWidget->clear();
    ui->file_listWidget->addItems(pc_files);
    label_UI_enable(LABEL_UI_CUROR, true);

    statusBar()->showMessage(tr(FINISH_READING_FOLDER)+cur_folder_path);
  }
}

void
PCL_Labeller::clean_viewer()
{
  // Clear and empty the viewer
  viewer->removeAllPointClouds(); //Clear all Point Cloud
  viewer->removeAllCoordinateSystems(); //Clear all CoordinateSystem
  viewer->removeAllShapes(); //Clear all Shape such as Cube
}


//###################################################
//
//          Application Helper Function 
//
//###################################################


int 
PCL_Labeller::read_label() //Read the label of the current pointcloud
{
  std::fstream label_file;
  label_file.open((cur_folder_path+QString("/")+cur_label_file).toStdString(), ios::in);//read label file
  //Can find the label file
  if(label_file.is_open())//read/dump the all labels data to the store
  {
    statusBar()->showMessage(tr(CAN_OPEN_LABEL)+cur_label_file);
    statusBar()->showMessage(tr(LOADING_LABEL_FILE)+cur_label_file);
    std::string line;
    std::istringstream iss;
    for(int i=0;std::getline(label_file, line);i++)
    {
        iss.str(line);//Convert each line in the file to a string stream
        HSTM_Label label_in_file;

        //Get the normal information
        if(!(iss //Parse the stream
              >> label_in_file.name 
              >> label_in_file.center_x
              >> label_in_file.center_y
              >> label_in_file.center_z
              >> label_in_file.x_size
              >> label_in_file.y_size
              >> label_in_file.z_size
              >> label_in_file.rotate_x
              >> label_in_file.rotate_y
              >> label_in_file.rotate_z
              // >> label_in_file.node[0].x
              // >> label_in_file.node[0].y
              // >> label_in_file.node[0].z
              // >> label_in_file.node[1].x
              // >> label_in_file.node[1].y
              // >> label_in_file.node[1].z
              // >> label_in_file.node[2].x
              // >> label_in_file.node[2].y
              // >> label_in_file.node[2].z
              // >> label_in_file.node[3].x
              // >> label_in_file.node[3].y
              // >> label_in_file.node[3].z
              // >> label_in_file.node[4].x
              // >> label_in_file.node[4].y
              // >> label_in_file.node[4].z
              // >> label_in_file.node[5].x
              // >> label_in_file.node[5].y
              // >> label_in_file.node[5].z
              // >> label_in_file.node[6].x
              // >> label_in_file.node[6].y
              // >> label_in_file.node[6].z
              // >> label_in_file.node[7].x
              // >> label_in_file.node[7].y
              // >> label_in_file.node[7].z
              // >> label_in_file.node[8].x
              // >> label_in_file.node[8].y
              // >> label_in_file.node[8].z
              // >> label_in_file.node[9].x
              // >> label_in_file.node[9].y
              // >> label_in_file.node[9].z
              // >> label_in_file.node[10].x
              // >> label_in_file.node[10].y
              // >> label_in_file.node[10].z
              // >> label_in_file.node[11].x
              // >> label_in_file.node[11].y
              // >> label_in_file.node[11].z
              // >> label_in_file.node[12].x
              // >> label_in_file.node[12].y
              // >> label_in_file.node[12].z
              // >> label_in_file.node[13].x
              // >> label_in_file.node[13].y
              // >> label_in_file.node[13].z
              // >> label_in_file.node[14].x
              // >> label_in_file.node[14].y
              // >> label_in_file.node[14].z
            )   
          )
        { // error
          statusBar()->showMessage(tr(PARSING_LABEL_FILE_ERROR_P1)+cur_label_file);
          QMessageBox::StandardButton reply;
          reply = QMessageBox::question(this, tr(ERROR_DIALOG_TITLE),tr(PARSING_LABEL_FILE_ERROR_P1)+cur_label_file+tr(PARSING_LABEL_FILE_ERROR_P2)+QString::number(i)+tr(PARSING_LABEL_FILE_ERROR_P3), QMessageBox::Ignore | QMessageBox::Abort); 

          //Skip the error line of the label
          if(reply == QMessageBox::Ignore)
          {
            continue;
          }
          //Abort the reading procedure
          else
          {
            statusBar()->showMessage(tr(ABORT_LABEL_PARSING));
            return 2; 
          }        
        }
        //Insert the readed label data to the label holder
        label_in_file.node[0].x  = 0.0;
        label_in_file.node[0].y  = 0.0;
        label_in_file.node[0].z  = 0.0;
        label_in_file.node[1].x  = 0.0;
        label_in_file.node[1].y  = 0.0;
        label_in_file.node[1].z  = 0.0;
        label_in_file.node[2].x  = 0.0;
        label_in_file.node[2].y  = 0.0;
        label_in_file.node[2].z  = 0.0;
        label_in_file.node[3].x  = 0.0;
        label_in_file.node[3].y  = 0.0;
        label_in_file.node[3].z  = 0.0;
        label_in_file.node[4].x  = 0.0;
        label_in_file.node[4].y  = 0.0;
        label_in_file.node[4].z  = 0.0;
        label_in_file.node[5].x  = 0.0;
        label_in_file.node[5].y  = 0.0;
        label_in_file.node[5].z  = 0.0;
        label_in_file.node[6].x  = 0.0;
        label_in_file.node[6].y  = 0.0;
        label_in_file.node[6].z  = 0.0;
        label_in_file.node[7].x  = 0.0;
        label_in_file.node[7].y  = 0.0;
        label_in_file.node[7].z  = 0.0;
        label_in_file.node[8].x  = 0.0;
        label_in_file.node[8].y  = 0.0;
        label_in_file.node[8].z  = 0.0;
        label_in_file.node[9].x  = 0.0;
        label_in_file.node[9].y  = 0.0;
        label_in_file.node[9].z  = 0.0;
        label_in_file.node[10].x = 0.0;
        label_in_file.node[10].y = 0.0;
        label_in_file.node[10].z = 0.0;
        label_in_file.node[11].x = 0.0;
        label_in_file.node[11].y = 0.0;
        label_in_file.node[11].z = 0.0;
        label_in_file.node[12].x = 0.0;
        label_in_file.node[12].y = 0.0;
        label_in_file.node[12].z = 0.0;
        label_in_file.node[13].x = 0.0;
        label_in_file.node[13].y = 0.0;
        label_in_file.node[13].z = 0.0;
        label_in_file.node[14].x = 0.0;
        label_in_file.node[14].y = 0.0;
        label_in_file.node[14].z = 0.0;
        label_holder.push_back(label_in_file);//Save the data
        
    }//END-while() getline
    statusBar()->showMessage(tr(LOADED_LABEL_FILE)+cur_label_file);
    //Set ui enable when the system can load the pcd and label file
    label_file.close();//Close the file after read all of the data of the label txt file
    return 0;
  }
  //CANNOT find the label file
  else
  {
    statusBar()->showMessage(tr(CANNOT_OPEN_LABEL_P1)+cur_label_file);
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, tr(ERROR_DIALOG_TITLE),tr(CANNOT_OPEN_LABEL_P1)+cur_label_file+tr(CANNOT_OPEN_LABEL_P2), QMessageBox::Yes | QMessageBox::No); 

    //Skip the error line of the label
    if(reply == QMessageBox::Yes)
    {
      label_file.open((cur_folder_path+QString("/")+cur_label_file).toStdString(), ios::out | ios::trunc);//create label file
      label_file.close();//Close the file after read all of the data of the label txt file
      return 0;
    }
    else{
      //No need to close the file since the file cannot open
      // label_file.close();//Close the file after read all of the data of the label txt file
      return 1;
    }
  }
}

//Read the label of the current pointcloud
int 
PCL_Labeller::write_label()
{
  std::fstream label_file;
  //Open
  label_file.open((cur_folder_path+QString("/")+cur_label_file).toStdString(), ios::out | ios::trunc);//read label file
  statusBar()->showMessage(tr(SAVING_LABEL)+cur_label_file);
  if(label_file.is_open())
  {
    for(HSTM_Label item: label_holder)
    {
      label_file
      << item.name << ' '
      << item.center_x << ' '
      << item.center_y << ' '
      << item.center_z << ' '
      << item.x_size << ' '
      << item.y_size << ' '
      << item.z_size << ' '
      << item.rotate_x << ' '
      << item.rotate_y << ' '
      << item.rotate_z << '\n';
      // << item.node[0].x << ' '
      // << item.node[0].y << ' '
      // << item.node[0].z << ' '
      // << item.node[1].x << ' '
      // << item.node[1].y << ' '
      // << item.node[1].z << ' '
      // << item.node[2].x << ' '
      // << item.node[2].y << ' '
      // << item.node[2].z << ' '
      // << item.node[3].x << ' '
      // << item.node[3].y << ' '
      // << item.node[3].z << ' '
      // << item.node[4].x << ' '
      // << item.node[4].y << ' '
      // << item.node[4].z << ' '
      // << item.node[5].x << ' '
      // << item.node[5].y << ' '
      // << item.node[5].z << ' '
      // << item.node[6].x << ' '
      // << item.node[6].y << ' '
      // << item.node[6].z << ' '
      // << item.node[7].x << ' '
      // << item.node[7].y << ' '
      // << item.node[7].z << ' '
      // << item.node[8].x << ' '
      // << item.node[8].y << ' '
      // << item.node[8].z << ' '
      // << item.node[9].x << ' '
      // << item.node[9].y << ' '
      // << item.node[9].z << ' '
      // << item.node[10].x << ' '
      // << item.node[10].y << ' '
      // << item.node[10].z << ' '
      // << item.node[11].x << ' '
      // << item.node[11].y << ' '
      // << item.node[11].z << ' '
      // << item.node[12].x << ' '
      // << item.node[12].y << ' '
      // << item.node[12].z << ' '
      // << item.node[13].x << ' '
      // << item.node[13].y << ' '
      // << item.node[13].z << ' '
      // << item.node[14].x << ' '
      // << item.node[14].y << ' '
      // << item.node[14].z <<  std::endl;

      // for(int i=0; i < SKELETON_NODE_COUNT; i++)
      // {
      //   label_file 
      //   << ' ' << item.node[i].x 
      //   << ' ' << item.node[i].y 
      //   << ' ' << item.node[i].z;
      // }
      // label_file << '\n';//Add new line at the end of the line
    }
  
    label_file.close();
    statusBar()->showMessage(tr(CAN_SAVE_LABEL)+cur_label_file);
    return 0;
  }
  else
  {
    statusBar()->showMessage(tr(CANNOT_SAVE_LABEL)+cur_label_file);
    return 1;
  }
}


