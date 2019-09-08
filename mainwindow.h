#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QCustomPlot/qcustomplot.h"
#include "QCustomPlot/axistag.h"
#include "WinZlgCan/win_zlg_can.h"
#include "Interaction/HMI/Terminal.h"
#include  "WinZlgCan/can_rev_work_thread.h"

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
    /* Control UI*/
    QCustomPlot *mControlPlot;
    QPointer<QCPGraph> mControlGraph1;
    QPointer<QCPGraph> mControlGraph2;
    AxisTag *mControlTag1;
    AxisTag *mControlTag2;

    /* Detect UI*/
    QCustomPlot *mDetectPlot;
    QPointer<QCPGraph> mDetectGraph1;
    QPointer<QCPGraph> mDetectGraph2;
    AxisTag *mDetectTag1;
    AxisTag *mDetectTag2;

    QLabel *label_FrontObstacle_Text;
    QLabel *label_FrontObstacleDistance_Value;
    QLabel *label_FrontObstacleRegion_Value;
    QLabel *label_FrontObstacleStatus_Value;
    QLabel *label_RearObstacle_Text;
    QLabel *label_RearObstacleDistance_Value;
    QLabel *label_RearObstacleRegion_Value;
    QLabel *label_RearObstacleStatus_Value;

    QString obstacle_region[5] = {"左侧", "左中","中间","右中", "右侧"};
    QString obstacle_status[5] = {"正常", "盲区", "超探", "噪声", "无效" };

    /* Path UI*/
    QLineEdit *text_VehicleInitPointX;
    QLineEdit *text_VehicleInitPointY;
    QLineEdit *text_VehicleInitPointYaw;

    QLineEdit *text_ParkingLength;
    QLineEdit *text_ParkingWidth;

    QCustomPlot *mPathPlot;



    QTimer mDataTimer20ms;
    QPushButton *button_timer_control;

    QListWidget *list_function;

    /* CAN Configure */
    QPushButton *button_CanConnect;
    QPushButton *button_CanOpen;
    QPushButton *button_CanClose;
    WinZlgCan mWinZlgCan;
    CanRevWorkThread mCanRevWorkThread;

    //
    Terminal mTerminal;

    Percaption mPercaption;
    GeometricTrack mGeometricTrack;


private slots:
    // 功能选择激活函数图片选择的槽函数
    void sProcessItemActiveState(QListWidgetItem *current, QListWidgetItem *previous);

    // 50ms Task
    void sTimer20msTask(void);
    void sTimer20ms_Control(void);
    // CAN 配置相关的槽函数
    void sCAN_Connect(void);
    void sCAN_Open(void);
    void sCAN_Close(void);

    void DisplayPercaption(Percaption *p);
};

#endif // MAINWINDOW_H
