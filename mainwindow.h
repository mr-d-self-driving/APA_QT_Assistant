#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>
/**
 * @brief 绘图库
 */
#include "QCustomPlot/qcustomplot.h"
#include "QCustomPlot/axistag.h"

/**
 * @brief Eigen矩阵库
 */
#include "Eigen/Dense"

/**
 * @brief ZLG CAN 驱动
 */
#include "WinZlgCan/win_zlg_can.h"
#include "WinZlgCan/can_rev_work_thread.h"

/**
 * @brief 交互
 */
#include "Interaction/HMI/Terminal.h"
#include "Interaction/HMI/simulation.h"
#include "Interaction/Ultrasonic/Ultrasonic.h"

/**
 * @brief 规划
 */
#include "Planning/ParallelParking/parallel_planning.h"
#include "Planning/VerticalParking/vertical_planning.h"
#include "Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.h"

/**
 * @brief 控制
 */
#include "Control/LatControl/lat_control.h"
#include "Control/LatControl/lat_control_lqr.h"
#include "Control/Common/trajectory_analyzer.h"

/**
 * @brief 车辆配置
 */
#ifdef BORUI
#include "Interaction/CANBUS/BoRui/bo_rui_message.h"
#include "Interaction/CANBUS/BoRui/bo_rui_controller.h"
#endif

/**
 * @brief 车位边界配置
 */
#define BOUNDARY_LEFT  (-16.0)
#define BOUNDARY_RIGHT ( 18.0)
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
private:
    /********************************************************************************/
    /********************************* UI function **********************************/
    /********************************************************************************/
    void ControlUI(void);
    void DetectUI(void);
    void PlanUI(void);
    void G2_PlanUI(void);
    void TrackUI(void);

    // init function
    void Init(void);

    void DetectVehicleModule(Vector2d p,float yaw);
    void PathVehicleModule(Vector2d p,float yaw);

    void VehicleModuleShow(Vector2d p,float yaw,QCPCurve *vehicle_center,QCPCurve *vehicle_modle,QCustomPlot *plot);
    /**
     * @brief 显示矢量箭头
     * @param x :the x axis location
     * @param y :the y axis location
     * @param yaw :the yaw angle od the vectoe arrow
     * @param arrow :the plot object
     */
    void VectorArrowShow(State base, State *head, QCPItemLine *arrow);

    /**
     * @brief HC_CC_PathShow
     * @param p
     */
    void HC_CC_PathShow(vector<State> &p);

    void TargetPathShow(TrackLinkList *list);
    void TargetPathShow(std::vector<TargetTrack> *vec);
    void SteeringAngleShow(float angle);

    //文件解析
    void FileDataInit(void);
    void AnalyzeOneLine(const QByteArray &baLine);

    void DetectTask(void);
    void PlanTask(void);
    void TrackTask(void);
    Ui::MainWindow *ui;
    // plot variable
    /* Control UI*/
    QGridLayout *gControlLayout;
    QCustomPlot *mControlPlot;
    QPointer<QCPGraph> mControlGraph1;
    QPointer<QCPGraph> mControlGraph2;
    AxisTag *mControlTag1;
    AxisTag *mControlTag2;

    /* Detect UI*/
    QGridLayout *gDetectLayout;
    QCustomPlot *mDetectPlot;
    QCPCurve *mDetectVehicleModuleCurve;
    QCPCurve *mDetectVehicleCenterCurve;
    QCPCurve *mDetectEdgePoint;
    QCPCurve *mDetectValidEdgePoint;
    QCPCurve *mDetectRearEdgeTrianglePosition;

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

    QRadioButton *radio_right_enter_location;
    QRadioButton *radio_left_enter_location;
    QRadioButton *radio_center_enter_location;

    QPushButton *button_file_select;
    QFile *detect_file;
    QPushButton *button_start_calculate;

    /* Planning UI*/
    /**
     * @brief G1 Ui elements
     */
    QLineEdit *text_VehicleInitPointX;
    QLineEdit *text_VehicleInitPointY;
    QLineEdit *text_VehicleInitPointYaw;

    QLineEdit *text_ParkingLength;
    QLineEdit *text_ParkingWidth;

    QLabel *label_VehiceTrackX_Value;
    QLabel *label_VehiceTrackY_Value;
    QLabel *label_VehiceTrackYaw_Value;

    QPushButton *gParkingInformationConfirm;

    QGridLayout *gPlanLayout;

    QCustomPlot *mPathPlot;

    // 用于车辆模型的绘制
    QCPCurve *mPathVehicleModuleCurve;
    QCPCurve *mPathParkingCurve;
    QCPCurve *mPathVehicleCenterCurve;

    QCPItemEllipse *mPathLeftCircle;
    QCPItemEllipse *mPathRightCircle;

    QVector<double> ParkingPointX,ParkingPointY;

//    QPointer<QCPGraph> mPathVehicleGraph;
//    QPointer<QCPGraph> mPathVehicleModuleDownGraph;

    /**
     * @brief G2 UI elements
     */
    QLineEdit *text_VehicleStartPointX;
    QLineEdit *text_VehicleStartPointY;
    QLineEdit *text_VehicleStartPointYaw;
    QLineEdit *text_VehicleStartPointKappa;

    QLineEdit *text_VehicleEndPointX;
    QLineEdit *text_VehicleEndPointY;
    QLineEdit *text_VehicleEndPointYaw;
    QLineEdit *text_VehicleEndPointKappa;

    QCPCurve *mPathPlanningStraightLine;
    QCPCurve *mPathPlanningClothoid;
    QCPCurve *mPathPlanningCircle;

    QCPItemLine *mStartArrow;
    QCPItemLine *mEndArrow;


    QCPItemEllipse *mStartCircle;
    QCPItemEllipse *mEndCircle;
    QCPItemEllipse *mMiddleCircle1;
    QCPItemEllipse *mMiddleCircle2;

    QCPCurve *mTangentCirclePoint;
    /**
     * @brief mTrackPlot: 跟踪模块
     */
    QCustomPlot *mTrackPlot;
    QCustomPlot *mTrackSinglePlot;

    QCPCurve *mTrackVehicleModuleCurve;
    QCPCurve *mTrackVehicleCenterCurve;
    QCPCurve *mTrackParkingCurve;
    QCPCurve *mTrackTargetCurve;

    QPointer<QCPGraph> mGraph_SteeringAngle;
    QPointer<QCPGraph> mGraph_Track;
    AxisTag *mTag_SteeringAngle;
    AxisTag *mTag_Track;

    QGridLayout *gPlotLayout;
    QGridLayout *gTrackLayout;


    QRadioButton *radio_sin_curvature;
    QRadioButton *radio_double_line;
    QRadioButton *radio_circle_curvature;

    QLabel *label_TrackUI_VehiceTrackX_Value;
    QLabel *label_TrackUI_VehiceTrackY_Value;
    QLabel *label_TrackUI_VehiceTrackYaw_Value;

    QPushButton *button_patn_generate;
    QPushButton *button_track_start;
    // timer 20ms Task
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
    // 规划
    ParallelPlanning *mParallelPlanning;
    VerticalPlanning *mVerticalPlanning;
    Curvature        *mCurvature;
    HC_ReedsSheppStateSpace *mHC_ReedsSheppStateSpace;

    State mBaseState[2];
    State mHeadState[2];

    int m_base_state_index, m_head_state_index;
    // 控制
    LatControl *mLatControl;
    LatControl_LQR *m_LatControl_LQR;

    TrackLinkList *_target_curvature_data_sets;
    std::vector<TargetTrack> *_target_curvature_vectors;
    TrajectoryAnalyzer *m_TrajectoryAnalyzer;

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
//    void sDisplayPercaption(Percaption *p);

    // 感知区域的文件数据载入
    void sPercaptionDataFileSelect(void);

    // 感知检测相关计算
    void sCalculateDetect(void);

    //规划
//    void sParkingConfirm();
    void sParkingConfirmG2();

    void sPathCirclePoint(uint8_t id,Circle *c);

    void sPathGenarate(void);

    // 鼠标拖拽功能
    /**
     * @brief 鼠标按下事件处理
     * @param event
     */
    void sMousePressEvent(QMouseEvent *event);

    /**
     * @brief 鼠标释放事件处理
     * @param event
     */
    void sMouseReleaseEvent(QMouseEvent *event);

    /**
     * @brief 鼠标移动事件处理
     * @param event
     */
    void sMouseMoveEvent(QMouseEvent *event);
    // 跟踪模块
    void sTrackStart(void);
    void SortSmallToBig(float *array,uint8_t *index_array,uint8_t cnt);
};

#endif // MAINWINDOW_H
