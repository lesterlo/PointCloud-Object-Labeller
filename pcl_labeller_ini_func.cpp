//###################################################
//
//          Initialize function
//
//###################################################

#include "pcl_labeller.h"
#include "config.h"
#include "../build/ui_pcl_labeller.h"
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
