#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QCustomPlot/qcustomplot.h"
#include "QCustomPlot/axistag.h"
#include "WinZlgCan/win_zlg_can.h"
#include "Interaction/HMI/Terminal.h"
#include "Interaction/HMI/simulation.h"
#include  "WinZlgCan/can_rev_work_thread.h"
#include "Planning/ParallelParking/parallel_planning.h"
#include "Planning/VerticalParking/vertical_planning.h"

#ifdef BORUI
#include "Interaction/CANBUS/BoRui/bo_rui_message.h"
#include "Interaction/CANBUS/BoRui/bo_rui_controller.h"
#endif

#define BOUNDARY_LEFT  (-14.0)
#define BOUNDARY_RIGHT ( 16.0)
#define BOUNDARY_TOP   ( 12.0)
#define BOUNDARY_DOWN  (-12.0)

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void Init(void);
    void VehicleModule(Vector2d p,float yaw);

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

    QLabel *label_VehiceTrackX_Value;
    QLabel *label_VehiceTrackY_Value;
    QLabel *label_VehiceTrackYaw_Value;

    QGridLayout *gPathLayout;

    QCustomPlot *mPathPlot;
    // 用于车辆模型的绘制
    QCPCurve *mPathVehicleModuleCurve;
    QCPCurve *mPathParkingCurve;
    QCPCurve *mPathVehicleCenterCurve;
    QCPItemEllipse *mPathLeftCircle;
    QCPItemEllipse *mPathRightCircle;

    QVector<double> ParkingPointX,ParkingPointY;

    QPointer<QCPGraph> mPathVehicleGraph;
    QPointer<QCPGraph> mPathVehicleModuleDownGraph;

    QTimer mDataTimer20ms;
    QPushButton *button_timer_control;

    QListWidget *list_function;

    /* CAN Configure */
    QPushButton *button_CanConnect;
    QPushButton *button_CanOpen;
    QPushButton *button_CanClose;
//    WinZlgCan mWinZlgCan;
//    CanRevWorkThread mCanRevWorkThread;

    //HMI
    Terminal *mTerminal;
    Simulation *mSimulation;

    Percaption mPercaption;
    GeometricTrack mGeometricTrack;
    VehilceConfig mVehilceConfig;

    BoRuiMessage *mBoRuiMessage;
    BoRuiController *mBoRuiController;

    Vector2d FrontLeftPoint,FrontRightPoint,RearLeftPoint,RearRightPoint;

    // Path
    ParallelPlanning *mParallelPlanning;
    VerticalPlanning *mVerticalPlanning;

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

    //障碍物感知信息
    void sDisplayPercaption(Percaption *p);

    //规划
    void sParkingConfirm();

    void sPathCirclePoint(uint8_t id,Circle *c);

    void SortSmallToBig(float *array,uint8_t *index_array,uint8_t cnt);
};

#endif // MAINWINDOW_H
