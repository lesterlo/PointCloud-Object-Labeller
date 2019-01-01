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
    connect(ui->label_le, SIGNAL(editingFinished()),  this, SLOT(onLabelEditFinish()));
    connect(ui->centerx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->centery_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->centerz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->widthx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->centerx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->heighty_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    connect(ui->depthz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
  }
  else
  {
    disconnect(ui->label_le, SIGNAL(editingFinished()),  this, SLOT(onLabelEditFinish()));
    disconnect(ui->centerx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->centery_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->centerz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->widthx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->centerx_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->heighty_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
    disconnect(ui->depthz_dsb, SIGNAL(valueChanged(double )),  this, SLOT(onLabelValueChange(double)));
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
      cur_label_file = QString::fromStdString(std::regex_replace(cur_pcd_file.toStdString(), std::regex(".pcd"), ".txt"));//Replace .pcd to .txt
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
    int cur_label_index = ui->label_listWidget->currentRow();

    labelUI_Signal_enable(false);//Important, disable signal when call setValue function which is not a user input

    //Save the previous label from the UI to the label_holder vector
    if(prev_label_index != -1)
    {
      label_holder[prev_label_index].type = ui->label_le->text().toStdString();
      ui->label_listWidget->item(prev_label_index)->setText(ui->label_le->text());//Change the listWidget item shown name
      label_holder.at(prev_label_index).obj_x = ui->centerx_dsb->value();
      label_holder.at(prev_label_index).obj_y = ui->centery_dsb->value();
      label_holder.at(prev_label_index).obj_z = ui->centerz_dsb->value();
      label_holder.at(prev_label_index).obj_width = ui->widthx_dsb->value();
      label_holder.at(prev_label_index).obj_height = ui->heighty_dsb->value();
      label_holder.at(prev_label_index).obj_length = ui->depthz_dsb->value();
    }
    //Apply the new data from the label_holder vector to the UI
    ui->label_le->setText(QString::fromStdString(label_holder[cur_label_index].type));
    ui->centerx_dsb->setValue(label_holder.at(cur_label_index).obj_x);
    ui->centery_dsb->setValue(label_holder.at(cur_label_index).obj_y);
    ui->centerz_dsb->setValue(label_holder.at(cur_label_index).obj_z);
    ui->widthx_dsb->setValue(label_holder.at(cur_label_index).obj_width);
    ui->heighty_dsb->setValue(label_holder.at(cur_label_index).obj_height);
    ui->depthz_dsb->setValue(label_holder.at(cur_label_index).obj_length);

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
  KITTI_Label newLabel;
  //Initialize the parameter
  newLabel.type = "NO_NAME";
  newLabel.truncated = 0.0;
  newLabel.occluded = 0;
  newLabel.alpha = 0.0;
  newLabel.x_min =0.0;
  newLabel.y_min = 0.0;
  newLabel.x_max = 0.0;
  newLabel.y_max = 0.0;
  newLabel.obj_height = 0.0;
  newLabel.obj_width = 0.0;
  newLabel.obj_length = 0.0;
  newLabel.obj_x = 0.0;
  newLabel.obj_y = 0.0;
  newLabel.obj_z = 0.0;
  newLabel.rotation_y = 0.0;

  //Insert the new element to the tail of the list
  label_holder.push_back(newLabel);
  ui->label_listWidget->addItem(QString::fromStdString(newLabel.type));//Insert the new element to the tail of the UI list
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
    label_holder.at(selected_item_index).type = ui->label_le->text().toStdString();
    ui->label_listWidget->item(selected_item_index)->setText(ui->label_le->text());//Change the listWidget item shown name
    label_holder.at(selected_item_index).obj_x = ui->centerx_dsb->value();
    label_holder.at(selected_item_index).obj_y = ui->centery_dsb->value();
    label_holder.at(selected_item_index).obj_z = ui->centerz_dsb->value();
    label_holder.at(selected_item_index).obj_width = ui->widthx_dsb->value();
    label_holder.at(selected_item_index).obj_height = ui->heighty_dsb->value();
    label_holder.at(selected_item_index).obj_length = ui->depthz_dsb->value();

    drawAllLabel(ui->label_listWidget->currentRow());//Update the UI that the show elemnt is same with the label_holder vector
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
  labelUI_Signal_enable(true);//Important, enable back the signal
}


void
PCL_Labeller::construct_labelWidget()
{
  QStringList pc_files;
  for(KITTI_Label item: label_holder)
  {
    pc_files << QString::fromStdString(item.type);
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
        ui->widthx_dsb->setEnabled(state);
        ui->heighty_dsb->setEnabled(state);
        ui->depthz_dsb->setEnabled(state);
        ui->label_listWidget->setEnabled(state);
        ui->save_pb->setEnabled(state);
        ui->newLabel_pb->setEnabled(state);
        ui->deleteLabel_pb->setEnabled(state);
        break;
    }
}


void 
PCL_Labeller::drawAllLabel(int highlisted_index)
{
  QStringList label_name;
  int render_id = 0;

  viewer->removeAllShapes();//Clear all bounding cube first
  for(KITTI_Label item : label_holder)
  {
    //render the bounding cube
    viewer->addCube(
      Eigen::Vector3f(item.obj_x, item.obj_y, item.obj_z), //Translation
      Eigen::Quaternionf(Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0*M_PI,  Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ())), //Rotation, default is 0
      item.obj_width, //width
      item.obj_height, //Height
      item.obj_length,   //Depth
      std::to_string(render_id)//ID
    );
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
      std::to_string(render_id));//Make the cube to wireframe
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
      render_id == highlisted_index ? 0.0:1.0, //R
      render_id == highlisted_index ? 1.0:0.0, //G
      0.0, //B
      std::to_string(render_id)
    );//Change color
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 
      BCUBE_LINEWIDTH, //Line Width
      std::to_string(render_id)
    );
    // viewer->addText3D (const std::string &text, const PointT &position, double textScale=1.0, double r=1.0, double g=1.0, double b=1.0, const std::string &id="");//Add anotation Text
    
    //Add label name to the element
    label_name << QString::fromStdString(item.type);
    
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
  viewer->removeAllPointClouds();// Clear and empty the viewer
  viewer->removeAllShapes();
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
        KITTI_Label label_in_file;

        if (!(iss //Parse the stream
        >> label_in_file.type 
        >> label_in_file.truncated
        >> label_in_file.occluded
        >> label_in_file.alpha
        >> label_in_file.x_min
        >> label_in_file.y_min
        >> label_in_file.x_max
        >> label_in_file.y_max
        >> label_in_file.obj_height
        >> label_in_file.obj_width
        >> label_in_file.obj_length
        >> label_in_file.obj_x
        >> label_in_file.obj_y
        >> label_in_file.obj_z
        >> label_in_file.rotation_y
        ))
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
    for(KITTI_Label item: label_holder)
    {
      label_file
      << item.type << ' '
      << item.truncated << ' '
      << item.occluded << ' '
      << item.alpha << ' '
      << item.x_min << ' '
      << item.y_min << ' '
      << item.x_max << ' '
      << item.y_max << ' '
      << item.obj_height << ' '
      << item.obj_width << ' '
      << item.obj_length << ' '
      << item.obj_x << ' '
      << item.obj_y << ' '
      << item.obj_z << ' '
      << item.rotation_y << '\n';
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


