#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QCustomPlot\qcustomplot.h"
#include "WinZlgCan/win_zlg_can.h"


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
    // plot variable
    QCustomPlot *mControlPlot;
    QCustomPlot *mDetectPlot;
    QCustomPlot *mPathPlot;

    QTimer mDataTimer50ms;

    QListWidget *list_function;

    /* CAN Configure */
    QPushButton *button_CanConnect;
    QPushButton *button_CanOpen;
    QPushButton *button_CanClose;
    WinZlgCan mWinZlgCan;

private slots:
    // 功能选择激活函数图片选择的槽函数
    void ProcessItemActiveState(QListWidgetItem *current, QListWidgetItem *previous);

    // CAN 配置相关的槽函数
    void CAN_Connect(void);
    void CAN_Open(void);
    void CAN_Close(void);

};

#endif // MAINWINDOW_H
