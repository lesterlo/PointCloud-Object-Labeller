//###################################################
//
//          File I/O Function 
//
//###################################################
#include "pcl_labeller.h"
#include "config.h"
#include "../build/ui_pcl_labeller.h"

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