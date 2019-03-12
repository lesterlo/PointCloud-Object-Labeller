// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 


//###################################################
//
//          QT Signal Slot function (handler)
//
//###################################################

#include "pcl_labeller.h"
#include "config.h"
#include "../build/ui_pcl_labeller.h"

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