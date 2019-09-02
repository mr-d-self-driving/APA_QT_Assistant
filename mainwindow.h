#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QCustomPlot\qcustomplot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QCustomPlot *mPlot;
    QTimer mDataTimer50ms;

private slots:
    void ProcessItemActiveState(QListWidgetItem *current, QListWidgetItem *previous);
    void ProcessItemActiveState(QListWidgetItem *current);
};

#endif // MAINWINDOW_H
