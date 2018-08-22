// License: MIT. Please Check the license file in root directory
// Copyright (c) (2018) Lester Lo 

#include "pcl_labeller.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCL_Labeller w;


  w.show ();

  return a.exec ();
}
