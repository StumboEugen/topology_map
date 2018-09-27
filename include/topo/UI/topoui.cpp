#include "topoui.h"
#include "ui_topoui.h"

TopoUI::TopoUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TopoUI)
{
    ui->setupUi(this);
}

TopoUI::~TopoUI()
{
    delete ui;
}
