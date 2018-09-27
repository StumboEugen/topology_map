#ifndef TOPOUI_H
#define TOPOUI_H

#include <QMainWindow>

namespace Ui {
class TopoUI;
}

class TopoUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit TopoUI(QWidget *parent = 0);
    ~TopoUI();

private:
    Ui::TopoUI *ui;
};

#endif // TOPOUI_H
