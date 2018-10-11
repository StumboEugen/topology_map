#include "topoui.h"
#include "ui_topoui.h"

#include "topo/Topo.h"

#include <QPainter>

TopoUI::TopoUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TopoUI)
{
    ui->setupUi(this);
    connect(ui->btnInputMap, SIGNAL(clicked()), this, SLOT(loadMapFromFile()));
}

TopoUI::~TopoUI()
{
    delete ui;
}

void TopoUI::paintEvent(QPaintEvent *event) {
    QWidget::paintEvent(event);
    QPainter painter(this);
    painter.drawLine(0, 0, 100, 100);
}

void TopoUI::loadMapFromFile() {
    if (mapGroup.reloadFromFile(ui->etInputMap->text().toStdString())) {
        cout << "UI load map successful" << endl;
    } else {
        cerr << "UI load map FAIL" << endl;
    }
}
