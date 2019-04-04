// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 


//###################################################
//
//          QT UI Helper function
//
//###################################################

#include "pcl_labeller.h"
#include "config.h"
#include "../build/ui_pcl_labeller.h"

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