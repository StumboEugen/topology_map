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
    ~TopoUI() override;

private:
    Ui::TopoUI *ui;
    void paintEvent(QPaintEvent * event) override;
};

#endif // TOPOUI_H
