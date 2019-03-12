// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 

//###################################################
//
//          QT Signal Related function
//
//###################################################
#include "pcl_labeller.h"
#include "config.h"
#include "../build/ui_pcl_labeller.h"


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