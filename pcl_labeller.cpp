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

    //Joint tab
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
      label_holder.at(prev_label_index).sk_n1_x  = ui->sk_n1x_dsb->value();
      label_holder.at(prev_label_index).sk_n1_y  = ui->sk_n1y_dsb->value();
      label_holder.at(prev_label_index).sk_n1_z  = ui->sk_n1z_dsb->value();
      label_holder.at(prev_label_index).sk_n2_x  = ui->sk_n2x_dsb->value();
      label_holder.at(prev_label_index).sk_n2_y  = ui->sk_n2y_dsb->value();
      label_holder.at(prev_label_index).sk_n2_z  = ui->sk_n2z_dsb->value();
      label_holder.at(prev_label_index).sk_n3_x  = ui->sk_n3x_dsb->value();
      label_holder.at(prev_label_index).sk_n3_y  = ui->sk_n3y_dsb->value();
      label_holder.at(prev_label_index).sk_n3_z  = ui->sk_n3z_dsb->value();
      label_holder.at(prev_label_index).sk_n4_x  = ui->sk_n4x_dsb->value();
      label_holder.at(prev_label_index).sk_n4_y  = ui->sk_n4y_dsb->value();
      label_holder.at(prev_label_index).sk_n4_z  = ui->sk_n4z_dsb->value();
      label_holder.at(prev_label_index).sk_n5_x  = ui->sk_n5x_dsb->value();
      label_holder.at(prev_label_index).sk_n5_y  = ui->sk_n5y_dsb->value();
      label_holder.at(prev_label_index).sk_n5_z  = ui->sk_n5z_dsb->value();
      label_holder.at(prev_label_index).sk_n6_x  = ui->sk_n6x_dsb->value();
      label_holder.at(prev_label_index).sk_n6_y  = ui->sk_n6y_dsb->value();
      label_holder.at(prev_label_index).sk_n6_z  = ui->sk_n6z_dsb->value();
      label_holder.at(prev_label_index).sk_n7_x  = ui->sk_n7x_dsb->value();
      label_holder.at(prev_label_index).sk_n7_y  = ui->sk_n7y_dsb->value();
      label_holder.at(prev_label_index).sk_n7_z  = ui->sk_n7z_dsb->value();
      label_holder.at(prev_label_index).sk_n8_x  = ui->sk_n8x_dsb->value();
      label_holder.at(prev_label_index).sk_n8_y  = ui->sk_n8y_dsb->value();
      label_holder.at(prev_label_index).sk_n8_z  = ui->sk_n8z_dsb->value();
      label_holder.at(prev_label_index).sk_n9_x  = ui->sk_n9x_dsb->value();
      label_holder.at(prev_label_index).sk_n9_y  = ui->sk_n9y_dsb->value();
      label_holder.at(prev_label_index).sk_n9_z  = ui->sk_n9z_dsb->value();
      label_holder.at(prev_label_index).sk_n10_x = ui->sk_n10x_dsb->value();
      label_holder.at(prev_label_index).sk_n10_y = ui->sk_n10y_dsb->value();
      label_holder.at(prev_label_index).sk_n10_z = ui->sk_n10z_dsb->value();
      label_holder.at(prev_label_index).sk_n11_x = ui->sk_n11x_dsb->value();
      label_holder.at(prev_label_index).sk_n11_y = ui->sk_n11y_dsb->value();
      label_holder.at(prev_label_index).sk_n11_z = ui->sk_n11z_dsb->value();
      label_holder.at(prev_label_index).sk_n12_x = ui->sk_n12x_dsb->value();
      label_holder.at(prev_label_index).sk_n12_y = ui->sk_n12y_dsb->value();
      label_holder.at(prev_label_index).sk_n12_z = ui->sk_n12z_dsb->value();
      label_holder.at(prev_label_index).sk_n13_x = ui->sk_n13x_dsb->value();
      label_holder.at(prev_label_index).sk_n13_y = ui->sk_n13y_dsb->value();
      label_holder.at(prev_label_index).sk_n13_z = ui->sk_n13z_dsb->value();
      label_holder.at(prev_label_index).sk_n14_x = ui->sk_n14x_dsb->value();
      label_holder.at(prev_label_index).sk_n14_y = ui->sk_n14y_dsb->value();
      label_holder.at(prev_label_index).sk_n14_z = ui->sk_n14z_dsb->value();
      label_holder.at(prev_label_index).sk_n15_x = ui->sk_n15x_dsb->value();
      label_holder.at(prev_label_index).sk_n15_y = ui->sk_n15y_dsb->value();
      label_holder.at(prev_label_index).sk_n15_z = ui->sk_n15z_dsb->value();
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
    ui->sk_n1x_dsb->setValue(  label_holder.at(cur_label_index).sk_n1_x );
    ui->sk_n1y_dsb->setValue(  label_holder.at(cur_label_index).sk_n1_y );
    ui->sk_n1z_dsb->setValue(  label_holder.at(cur_label_index).sk_n1_z );
    ui->sk_n2x_dsb->setValue(  label_holder.at(cur_label_index).sk_n2_x );
    ui->sk_n2y_dsb->setValue(  label_holder.at(cur_label_index).sk_n2_y );
    ui->sk_n2z_dsb->setValue(  label_holder.at(cur_label_index).sk_n2_z );
    ui->sk_n3x_dsb->setValue(  label_holder.at(cur_label_index).sk_n3_x );
    ui->sk_n3y_dsb->setValue(  label_holder.at(cur_label_index).sk_n3_y );
    ui->sk_n3z_dsb->setValue(  label_holder.at(cur_label_index).sk_n3_z );
    ui->sk_n4x_dsb->setValue(  label_holder.at(cur_label_index).sk_n4_x );
    ui->sk_n4y_dsb->setValue(  label_holder.at(cur_label_index).sk_n4_y );
    ui->sk_n4z_dsb->setValue(  label_holder.at(cur_label_index).sk_n4_z );
    ui->sk_n5x_dsb->setValue(  label_holder.at(cur_label_index).sk_n5_x );
    ui->sk_n5y_dsb->setValue(  label_holder.at(cur_label_index).sk_n5_y );
    ui->sk_n5z_dsb->setValue(  label_holder.at(cur_label_index).sk_n5_z );
    ui->sk_n6x_dsb->setValue(  label_holder.at(cur_label_index).sk_n6_x );
    ui->sk_n6y_dsb->setValue(  label_holder.at(cur_label_index).sk_n6_y );
    ui->sk_n6z_dsb->setValue(  label_holder.at(cur_label_index).sk_n6_z );
    ui->sk_n7x_dsb->setValue(  label_holder.at(cur_label_index).sk_n7_x );
    ui->sk_n7y_dsb->setValue(  label_holder.at(cur_label_index).sk_n7_y );
    ui->sk_n7z_dsb->setValue(  label_holder.at(cur_label_index).sk_n7_z );
    ui->sk_n8x_dsb->setValue(  label_holder.at(cur_label_index).sk_n8_x );
    ui->sk_n8y_dsb->setValue(  label_holder.at(cur_label_index).sk_n8_y );
    ui->sk_n8z_dsb->setValue(  label_holder.at(cur_label_index).sk_n8_z );
    ui->sk_n9x_dsb->setValue(  label_holder.at(cur_label_index).sk_n9_x );
    ui->sk_n9y_dsb->setValue(  label_holder.at(cur_label_index).sk_n9_y );
    ui->sk_n9z_dsb->setValue(  label_holder.at(cur_label_index).sk_n9_z );
    ui->sk_n10x_dsb->setValue( label_holder.at(cur_label_index).sk_n10_x);
    ui->sk_n10y_dsb->setValue( label_holder.at(cur_label_index).sk_n10_y);
    ui->sk_n10z_dsb->setValue( label_holder.at(cur_label_index).sk_n10_z);
    ui->sk_n11x_dsb->setValue( label_holder.at(cur_label_index).sk_n11_x);
    ui->sk_n11y_dsb->setValue( label_holder.at(cur_label_index).sk_n11_y);
    ui->sk_n11z_dsb->setValue( label_holder.at(cur_label_index).sk_n11_z);
    ui->sk_n12x_dsb->setValue( label_holder.at(cur_label_index).sk_n12_x);
    ui->sk_n12y_dsb->setValue( label_holder.at(cur_label_index).sk_n12_y);
    ui->sk_n12z_dsb->setValue( label_holder.at(cur_label_index).sk_n12_z);
    ui->sk_n13x_dsb->setValue( label_holder.at(cur_label_index).sk_n13_x);
    ui->sk_n13y_dsb->setValue( label_holder.at(cur_label_index).sk_n13_y);
    ui->sk_n13z_dsb->setValue( label_holder.at(cur_label_index).sk_n13_z);
    ui->sk_n14x_dsb->setValue( label_holder.at(cur_label_index).sk_n14_x);
    ui->sk_n14y_dsb->setValue( label_holder.at(cur_label_index).sk_n14_y);
    ui->sk_n14z_dsb->setValue( label_holder.at(cur_label_index).sk_n14_z);
    ui->sk_n15x_dsb->setValue( label_holder.at(cur_label_index).sk_n15_x);
    ui->sk_n15y_dsb->setValue( label_holder.at(cur_label_index).sk_n15_y);
    ui->sk_n15z_dsb->setValue( label_holder.at(cur_label_index).sk_n15_z);
      
    
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
  newLabel.sk_n1_x  = 0.0;
  newLabel.sk_n1_y  = 0.0;
  newLabel.sk_n1_z  = 0.0;
  newLabel.sk_n2_x  = 0.0;
  newLabel.sk_n2_y  = 0.0;
  newLabel.sk_n2_z  = 0.0;
  newLabel.sk_n3_x  = 0.0;
  newLabel.sk_n3_y  = 0.0;
  newLabel.sk_n3_z  = 0.0;
  newLabel.sk_n4_x  = 0.0;
  newLabel.sk_n4_y  = 0.0;
  newLabel.sk_n4_z  = 0.0;
  newLabel.sk_n5_x  = 0.0;
  newLabel.sk_n5_y  = 0.0;
  newLabel.sk_n5_z  = 0.0;
  newLabel.sk_n6_x  = 0.0;
  newLabel.sk_n6_y  = 0.0;
  newLabel.sk_n6_z  = 0.0;
  newLabel.sk_n7_x  = 0.0;
  newLabel.sk_n7_y  = 0.0;
  newLabel.sk_n7_z  = 0.0;
  newLabel.sk_n8_x  = 0.0;
  newLabel.sk_n8_y  = 0.0;
  newLabel.sk_n8_z  = 0.0;
  newLabel.sk_n9_x  = 0.0;
  newLabel.sk_n9_y  = 0.0;
  newLabel.sk_n9_z  = 0.0;
  newLabel.sk_n10_x = 0.0;
  newLabel.sk_n10_y = 0.0;
  newLabel.sk_n10_z = 0.0;
  newLabel.sk_n11_x = 0.0;
  newLabel.sk_n11_y = 0.0;
  newLabel.sk_n11_z = 0.0;
  newLabel.sk_n12_x = 0.0;
  newLabel.sk_n12_y = 0.0;
  newLabel.sk_n12_z = 0.0;
  newLabel.sk_n13_x = 0.0;
  newLabel.sk_n13_y = 0.0;
  newLabel.sk_n13_z = 0.0;
  newLabel.sk_n14_x = 0.0;
  newLabel.sk_n14_y = 0.0;
  newLabel.sk_n14_z = 0.0;
  newLabel.sk_n15_x = 0.0;
  newLabel.sk_n15_y = 0.0;
  newLabel.sk_n15_z = 0.0;

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
    label_holder.at(selected_item_index).sk_n1_x  = ui->sk_n1x_dsb->value();
    label_holder.at(selected_item_index).sk_n1_y  = ui->sk_n1y_dsb->value();
    label_holder.at(selected_item_index).sk_n1_z  = ui->sk_n1z_dsb->value();
    label_holder.at(selected_item_index).sk_n2_x  = ui->sk_n2x_dsb->value();
    label_holder.at(selected_item_index).sk_n2_y  = ui->sk_n2y_dsb->value();
    label_holder.at(selected_item_index).sk_n2_z  = ui->sk_n2z_dsb->value();
    label_holder.at(selected_item_index).sk_n3_x  = ui->sk_n3x_dsb->value();
    label_holder.at(selected_item_index).sk_n3_y  = ui->sk_n3y_dsb->value();
    label_holder.at(selected_item_index).sk_n3_z  = ui->sk_n3z_dsb->value();
    label_holder.at(selected_item_index).sk_n4_x  = ui->sk_n4x_dsb->value();
    label_holder.at(selected_item_index).sk_n4_y  = ui->sk_n4y_dsb->value();
    label_holder.at(selected_item_index).sk_n4_z  = ui->sk_n4z_dsb->value();
    label_holder.at(selected_item_index).sk_n5_x  = ui->sk_n5x_dsb->value();
    label_holder.at(selected_item_index).sk_n5_y  = ui->sk_n5y_dsb->value();
    label_holder.at(selected_item_index).sk_n5_z  = ui->sk_n5z_dsb->value();
    label_holder.at(selected_item_index).sk_n6_x  = ui->sk_n6x_dsb->value();
    label_holder.at(selected_item_index).sk_n6_y  = ui->sk_n6y_dsb->value();
    label_holder.at(selected_item_index).sk_n6_z  = ui->sk_n6z_dsb->value();
    label_holder.at(selected_item_index).sk_n7_x  = ui->sk_n7x_dsb->value();
    label_holder.at(selected_item_index).sk_n7_y  = ui->sk_n7y_dsb->value();
    label_holder.at(selected_item_index).sk_n7_z  = ui->sk_n7z_dsb->value();
    label_holder.at(selected_item_index).sk_n8_x  = ui->sk_n8x_dsb->value();
    label_holder.at(selected_item_index).sk_n8_y  = ui->sk_n8y_dsb->value();
    label_holder.at(selected_item_index).sk_n8_z  = ui->sk_n8z_dsb->value();
    label_holder.at(selected_item_index).sk_n9_x  = ui->sk_n9x_dsb->value();
    label_holder.at(selected_item_index).sk_n9_y  = ui->sk_n9y_dsb->value();
    label_holder.at(selected_item_index).sk_n9_z  = ui->sk_n9z_dsb->value();
    label_holder.at(selected_item_index).sk_n10_x = ui->sk_n10x_dsb->value();
    label_holder.at(selected_item_index).sk_n10_y = ui->sk_n10y_dsb->value();
    label_holder.at(selected_item_index).sk_n10_z = ui->sk_n10z_dsb->value();
    label_holder.at(selected_item_index).sk_n11_x = ui->sk_n11x_dsb->value();
    label_holder.at(selected_item_index).sk_n11_y = ui->sk_n11y_dsb->value();
    label_holder.at(selected_item_index).sk_n11_z = ui->sk_n11z_dsb->value();
    label_holder.at(selected_item_index).sk_n12_x = ui->sk_n12x_dsb->value();
    label_holder.at(selected_item_index).sk_n12_y = ui->sk_n12y_dsb->value();
    label_holder.at(selected_item_index).sk_n12_z = ui->sk_n12z_dsb->value();
    label_holder.at(selected_item_index).sk_n13_x = ui->sk_n13x_dsb->value();
    label_holder.at(selected_item_index).sk_n13_y = ui->sk_n13y_dsb->value();
    label_holder.at(selected_item_index).sk_n13_z = ui->sk_n13z_dsb->value();
    label_holder.at(selected_item_index).sk_n14_x = ui->sk_n14x_dsb->value();
    label_holder.at(selected_item_index).sk_n14_y = ui->sk_n14y_dsb->value();
    label_holder.at(selected_item_index).sk_n14_z = ui->sk_n14z_dsb->value();
    label_holder.at(selected_item_index).sk_n15_x = ui->sk_n15x_dsb->value();
    label_holder.at(selected_item_index).sk_n15_y = ui->sk_n15y_dsb->value();
    label_holder.at(selected_item_index).sk_n15_z = ui->sk_n15z_dsb->value();


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
  ui->sk_n15y_dsb->setValue(0.0);
  ui->sk_n15z_dsb->setValue(0.0);
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
    //Get common element
    //Rotation
    Eigen::Quaternionf box_rotate = Eigen::Quaternionf( //Rotation of the Cube
        Eigen::AngleAxisf(TO_RAD(item.rotate_x), Eigen::Vector3f::UnitX()) * 
        Eigen::AngleAxisf(TO_RAD(item.rotate_y), Eigen::Vector3f::UnitY()) * 
        Eigen::AngleAxisf(TO_RAD(item.rotate_z), Eigen::Vector3f::UnitZ())  
      );
    //Translation
    Eigen::Vector3f box_vector = Eigen::Vector3f( 
        //Translation of center of the bounding box
          item.center_x, 
          item.center_y, 
          item.center_z
        );
    //Draw the Reference 3D axis
    if(render_id == highlisted_index)
      viewer->addCoordinateSystem(
        REF_AXIS_WIDTH, //Scale size
        Eigen::Affine3f( //Add Pose
          Eigen::Translation3f(box_vector) * //Add translation
          box_rotate//Add X, Y, Z rotation
        ),
        "cs_"+std::to_string(render_id)//Specific ID, for coordinateSystem (cs_)
      );
    //Draw the bounding cube
    viewer->addCube(
      box_vector,  //Ceter of the Box
      box_rotate,  //Rotation of the Box
      item.x_size, //width
      item.y_size, //Height
      item.z_size, //Depth
      "bb_"+std::to_string(render_id)//Specific ID, for bounding box (bb_)
    );

    //Make the cube to wireframe
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      "bb_"+std::to_string(render_id)
    );

    //Make the cube to red/green depends on the selected item
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 
      render_id == highlisted_index ? 0.0:1.0, //Red Color
      render_id == highlisted_index ? 1.0:0.0, //Green Color
      0.0, //Blue Color
      "bb_"+std::to_string(render_id)//Use the spcific ID, for bounding box (bb_)
    );
    //Set the Wireframe line width
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
      BCUBE_LINEWIDTH, //Line Width
      "bb_"+std::to_string(render_id)//Use the spcific ID, for bounding box (bb_)
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
    bool drawSkeleton = true;
    if (drawSkeleton)
    {
      //Sk1
      Eigen::Vector3f sk1_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n1_x, 
          item.sk_n1_y, 
          item.sk_n1_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk1_shift(0), 
          sk1_shift(1), 
          sk1_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk1_"+std::to_string(render_id)
      );

      //Sk2
      Eigen::Vector3f sk2_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n2_x, 
          item.sk_n2_y, 
          item.sk_n2_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk2_shift(0), 
          sk2_shift(1), 
          sk2_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk2_"+std::to_string(render_id)
      );

      //Sk3
      Eigen::Vector3f sk3_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n3_x, 
          item.sk_n3_y, 
          item.sk_n3_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk3_shift(0), 
          sk3_shift(1), 
          sk3_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk3_"+std::to_string(render_id)
      );

      //Sk4
      Eigen::Vector3f sk4_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n4_x, 
          item.sk_n4_y, 
          item.sk_n4_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk4_shift(0), 
          sk4_shift(1), 
          sk4_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk4_"+std::to_string(render_id)
      );

      //Sk5
      Eigen::Vector3f sk5_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n5_x, 
          item.sk_n5_y, 
          item.sk_n5_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk5_shift(0), 
          sk5_shift(1), 
          sk5_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk5_"+std::to_string(render_id)
      );

      //Sk6
      Eigen::Vector3f sk6_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n6_x, 
          item.sk_n6_y, 
          item.sk_n6_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk6_shift(0), 
          sk6_shift(1), 
          sk6_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk6_"+std::to_string(render_id)
      );

      //Sk7
      Eigen::Vector3f sk7_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n7_x, 
          item.sk_n7_y, 
          item.sk_n7_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk7_shift(0), 
          sk7_shift(1), 
          sk7_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk7_"+std::to_string(render_id)
      );

      //Sk8
      Eigen::Vector3f sk8_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n8_x, 
          item.sk_n8_y, 
          item.sk_n8_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk8_shift(0), 
          sk8_shift(1), 
          sk8_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk8_"+std::to_string(render_id)
      );

      //Sk9
      Eigen::Vector3f sk9_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n9_x, 
          item.sk_n9_y, 
          item.sk_n9_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk9_shift(0), 
          sk9_shift(1), 
          sk9_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk9_"+std::to_string(render_id)
      );

      //Sk10
      Eigen::Vector3f sk10_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n10_x, 
          item.sk_n10_y, 
          item.sk_n10_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk10_shift(0), 
          sk10_shift(1), 
          sk10_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk10_"+std::to_string(render_id)
      );

      //Sk11
      Eigen::Vector3f sk11_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n11_x, 
          item.sk_n11_y, 
          item.sk_n11_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk11_shift(0), 
          sk11_shift(1), 
          sk11_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk11_"+std::to_string(render_id)
      );

      //Sk12
      Eigen::Vector3f sk12_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n12_x, 
          item.sk_n12_y, 
          item.sk_n12_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk12_shift(0), 
          sk12_shift(1), 
          sk12_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk12_"+std::to_string(render_id)
      );

      //Sk13
      Eigen::Vector3f sk13_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n13_x, 
          item.sk_n13_y, 
          item.sk_n13_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk13_shift(0), 
          sk13_shift(1), 
          sk13_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk13_"+std::to_string(render_id)
      );

      //Sk14
      Eigen::Vector3f sk14_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n14_x, 
          item.sk_n14_y, 
          item.sk_n14_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk14_shift(0), 
          sk14_shift(1), 
          sk14_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk14_"+std::to_string(render_id)
      );

      //Sk15
      Eigen::Vector3f sk15_shift = 
        box_rotate * 
        Eigen::Vector3f( //Translation from the local coordinate of skeleton node
          item.sk_n15_x, 
          item.sk_n15_y, 
          item.sk_n15_z
        ) + 
        box_vector; //Translation of center of the bounding box

      viewer->addSphere(
        pcl::PointXYZ(
          //Apply The Vector to the pointxyz
          sk15_shift(0), 
          sk15_shift(1), 
          sk15_shift(2)
        ),
        SK_NODE_SIZE, //Radius of the sphere
        render_id == highlisted_index ? 0.0:1.0, //Red Color
        render_id == highlisted_index ? 1.0:0.0, //Green Color
        0.0, //Blue color
        "sk15_"+std::to_string(render_id)
      );


    }
    
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
  
    for(int i=0;std::getline(label_file, line);i++)
    {
        std::istringstream iss(line);//Convert each line in the file to a string stream
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
              >> label_in_file.sk_n1_x 
              >> label_in_file.sk_n1_y 
              >> label_in_file.sk_n1_z 
              >> label_in_file.sk_n2_x 
              >> label_in_file.sk_n2_y 
              >> label_in_file.sk_n2_z 
              >> label_in_file.sk_n3_x 
              >> label_in_file.sk_n3_y 
              >> label_in_file.sk_n3_z 
              >> label_in_file.sk_n4_x 
              >> label_in_file.sk_n4_y 
              >> label_in_file.sk_n4_z 
              >> label_in_file.sk_n5_x 
              >> label_in_file.sk_n5_y 
              >> label_in_file.sk_n5_z 
              >> label_in_file.sk_n6_x 
              >> label_in_file.sk_n6_y 
              >> label_in_file.sk_n6_z 
              >> label_in_file.sk_n7_x 
              >> label_in_file.sk_n7_y 
              >> label_in_file.sk_n7_z 
              >> label_in_file.sk_n8_x 
              >> label_in_file.sk_n8_y 
              >> label_in_file.sk_n8_z 
              >> label_in_file.sk_n9_x 
              >> label_in_file.sk_n9_y 
              >> label_in_file.sk_n9_z 
              >> label_in_file.sk_n10_x
              >> label_in_file.sk_n10_y
              >> label_in_file.sk_n10_z
              >> label_in_file.sk_n11_x
              >> label_in_file.sk_n11_y
              >> label_in_file.sk_n11_z
              >> label_in_file.sk_n12_x
              >> label_in_file.sk_n12_y
              >> label_in_file.sk_n12_z
              >> label_in_file.sk_n13_x
              >> label_in_file.sk_n13_y
              >> label_in_file.sk_n13_z
              >> label_in_file.sk_n14_x
              >> label_in_file.sk_n14_y
              >> label_in_file.sk_n14_z
              >> label_in_file.sk_n15_x
              >> label_in_file.sk_n15_y
              >> label_in_file.sk_n15_z
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
      << item.rotate_z << ' '
      << item.sk_n1_x  << ' '
      << item.sk_n1_y  << ' '
      << item.sk_n1_z  << ' '
      << item.sk_n2_x  << ' '
      << item.sk_n2_y  << ' '
      << item.sk_n2_z  << ' '
      << item.sk_n3_x  << ' '
      << item.sk_n3_y  << ' '
      << item.sk_n3_z  << ' '
      << item.sk_n4_x  << ' '
      << item.sk_n4_y  << ' '
      << item.sk_n4_z  << ' '
      << item.sk_n5_x  << ' '
      << item.sk_n5_y  << ' '
      << item.sk_n5_z  << ' '
      << item.sk_n6_x  << ' '
      << item.sk_n6_y  << ' '
      << item.sk_n6_z  << ' '
      << item.sk_n7_x  << ' '
      << item.sk_n7_y  << ' '
      << item.sk_n7_z  << ' '
      << item.sk_n8_x  << ' '
      << item.sk_n8_y  << ' '
      << item.sk_n8_z  << ' '
      << item.sk_n9_x  << ' '
      << item.sk_n9_y  << ' '
      << item.sk_n9_z  << ' '
      << item.sk_n10_x << ' '
      << item.sk_n10_y << ' '
      << item.sk_n10_z << ' '
      << item.sk_n11_x << ' '
      << item.sk_n11_y << ' '
      << item.sk_n11_z << ' '
      << item.sk_n12_x << ' '
      << item.sk_n12_y << ' '
      << item.sk_n12_z << ' '
      << item.sk_n13_x << ' '
      << item.sk_n13_y << ' '
      << item.sk_n13_z << ' '
      << item.sk_n14_x << ' '
      << item.sk_n14_y << ' '
      << item.sk_n14_z << ' '
      << item.sk_n15_x << ' '
      << item.sk_n15_y << ' '
      << item.sk_n15_z << '\n';

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


