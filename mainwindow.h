#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QCustomPlot/qcustomplot.h"
#include "QCustomPlot/axistag.h"
#include "WinZlgCan/win_zlg_can.h"
#include "Interaction/HMI/Terminal.h"
#include "Interaction/HMI/simulation.h"
#include "Interaction/Ultrasonic/Ultrasonic.h"
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
    void DetectVehicleModule(Vector2d p,float yaw);
    void PathVehicleModule(Vector2d p,float yaw);

    //文件解析
    void FileDataInit(void);
    void AnalyzeOneLine(const QByteArray &baLine);
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
    QCPCurve *mDetectVehicleModuleCurve;
    QCPCurve *mDetectVehicleCenterCurve;
    QCPCurve *mDetectEdgePoint;
    QCPCurve *mDetectEdgeGroundPositionPoint;
    QCPCurve *mDetectSensorPosition;

    QCPCurve *mDetectLeftEdgeGroundPosition;
    QCPCurve *mDetectRightEdgeGroundPosition;

    QCPCurve *mDetectLeftEdgeFitLine;
    QCPCurve *mDetectRightEdgeFitLine;

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

    QPushButton *button_file_select;
    QFile *detect_file;
    QPushButton *button_start_calculate;

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
    QStackedWidget *stack_Widget;
    /* CAN Configure */
    QPushButton *button_CanConnect;
    QPushButton *button_CanOpen;
    QPushButton *button_CanClose;
//    WinZlgCan mWinZlgCan;
//    CanRevWorkThread mCanRevWorkThread;

    // 跟踪
    GeometricTrack mGeometricTrack;
    // 配置
    VehilceConfig mVehilceConfig;
    //HMI
    Terminal *mTerminal;
    Simulation *mSimulation;
    BoRuiMessage *mBoRuiMessage;
    BoRuiController *mBoRuiController;
    // 感知
    Percaption mPercaption;
    UltrasonicObstaclePercption mUltrasonicObstaclePercption;
    // Detect Module
    Ultrasonic mUltrasonic;
    QList<Ultrasonic_Packet> LRU_List[12];
    QList<ObstacleLocationPacket> LRU_PositionList[12];
    QList<ObstacleLocationPacket> ObstacleBody_List[4];
    ObstacleLocationPacket temp_obstacle_body;

    Vector2d vehicle_last_position[4];
    ParkingEdgeBufferLocationPacket _ultrasonic_data_buffer[4];

    QList<GeometricTrack> VehicleTrackList;
    uint8_t NewFileUpdateFlag;
    int32_t time_step_cnt;




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

    // 障碍物感知信息
    void sDisplayPercaption(Percaption *p);

    // 感知区域的文件数据载入
    void sPercaptionDataFileSelect(void);

    // 感知检测相关计算
    void sCalculateDetect(void);

    //规划
    void sParkingConfirm();

    void sPathCirclePoint(uint8_t id,Circle *c);

    void SortSmallToBig(float *array,uint8_t *index_array,uint8_t cnt);
};

#endif // MAINWINDOW_H
