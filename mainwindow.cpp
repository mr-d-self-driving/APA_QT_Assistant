#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    Init();
    ui->setupUi(this);
    // 重新配置窗体大小
//    this->resize(1366,768);
    this->resize(1500,845);

    /**************************************************/
    list_function = new QListWidget();
    /*车辆控制*/
    QListWidgetItem *control_item = new QListWidgetItem(QIcon(":/Icon/unactive_car.png"),tr("控制"));
    list_function->addItem(control_item);
    /*检测模块*/
    QListWidgetItem *detect_item = new QListWidgetItem(QIcon(":/Icon/unactive_detect.png"),tr("检测"));
    list_function->addItem(detect_item);
    /*路径规划模块*/
    QListWidgetItem *path_item = new QListWidgetItem(QIcon(":/Icon/unactive_path.png"),tr("路径"));
    list_function->addItem(path_item);

    /*整体列表配置*/
    list_function->setViewMode(QListWidget::IconMode);//设置显示模式为图片模式
    list_function->setDragEnabled(false);//控件不允许拖动
    list_function->setFlow(QListWidget::TopToBottom);//设置元素排列方式从上往下
    list_function->setResizeMode(QListWidget::Adjust);//
    list_function->setMovement(QListWidget::Static);

    list_function->setIconSize(QSize(60,60));
    list_function->setMinimumHeight(100);
    list_function->setFixedWidth(70);

    list_function->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);//关闭水平滚动条
    list_function->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);//关闭垂直滚动条

//    list_function->setStyleSheet(
//                "QListWidget{border:1px solid gray; color:black; }"
//                );

    /**************************************************/

    /*** Control UI ***/
    /* CAN 配置 Group */
    button_CanConnect = new QPushButton(tr("连接"));
    button_CanOpen    = new QPushButton(tr("打开"));
    button_CanClose   = new QPushButton(tr("复位"));

    QVBoxLayout *gCanButtonLayout = new QVBoxLayout();
    gCanButtonLayout->addWidget(button_CanConnect);
    gCanButtonLayout->addWidget(button_CanOpen);
    gCanButtonLayout->addWidget(button_CanClose);
    gCanButtonLayout->setMargin(3);

    QGroupBox *gCanGroup = new QGroupBox();
    gCanGroup->setTitle("CAN Configure");
    gCanGroup->setFixedHeight(120);
    gCanGroup->setLayout(gCanButtonLayout);

    /* 车辆状态显示 Group */
    QLabel *label_VCU_Text = new QLabel();
    label_VCU_Text->setText("VCU:");
    QLabel *label_VCU_Value = new QLabel();
    label_VCU_Value->setText("0");

    QLabel *label_ESC_Text = new QLabel();
    label_ESC_Text->setText("ESC:");
    QLabel *label_ESC_Value = new QLabel();
    label_ESC_Value->setText("0");

    QLabel *label_EPS_Text = new QLabel();
    label_EPS_Text->setText("EPS:");
    QLabel *label_EPS_Value = new QLabel();
    label_EPS_Value->setText("0");

    QGridLayout *gVehicleStatusLayout = new QGridLayout();
    gVehicleStatusLayout->addWidget(label_VCU_Text,0,0);
    gVehicleStatusLayout->addWidget(label_VCU_Value,0,1);
    gVehicleStatusLayout->addWidget(label_ESC_Text,1,0);
    gVehicleStatusLayout->addWidget(label_ESC_Value,1,1);
    gVehicleStatusLayout->addWidget(label_EPS_Text,2,0);
    gVehicleStatusLayout->addWidget(label_EPS_Value,2,1);
    gVehicleStatusLayout->setColumnMinimumWidth(0,30);

    QGroupBox *gVehicleStatusGroup = new QGroupBox();
    gVehicleStatusGroup->setTitle("Status");
    gVehicleStatusGroup->setFixedHeight(100);
    gVehicleStatusGroup->setLayout(gVehicleStatusLayout);

    gVehicleStatusGroup->setStyleSheet("QGroupBox{color:green;\
                                                  font-weight:20px;\
                                                  font:bold;\
                                                 }");

//    gVehicleStatusGroup->setStyleSheet(QString("QGroupBox{subcontrol-origin: margin;subcontrol-position: top center;padding: 0 3px;background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,stop: 0 #FF0ECE, stop: 1 #FFFFFF);}"));
    /* 车辆转向显示 Group */
    QLabel *label_Angle_Text = new QLabel();
    label_Angle_Text->setText("角度:");
    QLabel *label_Angle_Value = new QLabel();
    label_Angle_Value->setText("0");

    QLabel *label_AngularVelocity_Text = new QLabel();
    label_AngularVelocity_Text->setText("角速度:");
    QLabel *label_AngularVelocity_Value = new QLabel();
    label_AngularVelocity_Value->setText("0");

    QGridLayout *gVehicleEPSLayout = new QGridLayout();
    gVehicleEPSLayout->addWidget(label_Angle_Text,0,0);
    gVehicleEPSLayout->addWidget(label_Angle_Value,0,1);
    gVehicleEPSLayout->addWidget(label_AngularVelocity_Text,1,0);
    gVehicleEPSLayout->addWidget(label_AngularVelocity_Value,1,1);

    QGroupBox *gVehicleEPSGroup = new QGroupBox();
    gVehicleEPSGroup->setTitle("EPS");
    gVehicleEPSGroup->setFixedHeight(90);
    gVehicleEPSGroup->setLayout(gVehicleEPSLayout);

    /* 车辆速度显示 Group */
    QLabel *label_VehicleVelocity_Text = new QLabel();
    label_VehicleVelocity_Text->setText("车速:");
    QLabel *label_VehicleVelocity_Value = new QLabel();
    label_VehicleVelocity_Value->setText("0");

    QLabel *label_RearLeftWheelVelocity_Text = new QLabel();
    label_RearLeftWheelVelocity_Text->setText("左轮速:");
    QLabel *label_RearLeftWheelVelocity_Value = new QLabel();
    label_RearLeftWheelVelocity_Value->setText("0");

    QLabel *label_RearRightWheelVelocity_Text = new QLabel();
    label_RearRightWheelVelocity_Text->setText("右轮速:");
    QLabel *label_RearRightWheelVelocity_Value = new QLabel();
    label_RearRightWheelVelocity_Value->setText("0");

    QLabel *label_RearLeftWheelPulse_Text = new QLabel();
    label_RearLeftWheelPulse_Text->setText("左脉冲:");
    QLabel *label_RearLeftWheelPulse_Value = new QLabel();
    label_RearLeftWheelPulse_Value->setText("0");

    QLabel *label_RearRightWheelPulse_Text = new QLabel();
    label_RearRightWheelPulse_Text->setText("右脉冲:");
    QLabel *label_RearRightWheelPulse_Value = new QLabel();
    label_RearRightWheelPulse_Value->setText("0");

    QGridLayout *gVehicleVelocityLayout = new QGridLayout();
    gVehicleVelocityLayout->addWidget(label_VehicleVelocity_Text,0,0);
    gVehicleVelocityLayout->addWidget(label_VehicleVelocity_Value,0,1);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelVelocity_Text,1,0);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelVelocity_Value,1,1);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelVelocity_Text,2,0);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelVelocity_Value,2,1);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelPulse_Text,3,0);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelPulse_Value,3,1);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelPulse_Text,4,0);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelPulse_Value,4,1);

    QGroupBox *gVehicleVelocityGroup = new QGroupBox();
    gVehicleVelocityGroup->setTitle("Velocity");
    gVehicleVelocityGroup->setFixedHeight(160);
    gVehicleVelocityGroup->setLayout(gVehicleVelocityLayout);

    /* vehilce motion group */
    QLabel *label_Lon_Text = new QLabel();
    label_Lon_Text->setText("Lon:");
    QLabel *label_Lon_Value = new QLabel();
    label_Lon_Value->setText("0");

    QLabel *label_Lat_Text = new QLabel();
    label_Lat_Text->setText("Lat:");
    QLabel *label_Lat_Value = new QLabel();
    label_Lat_Value->setText("0");

    QLabel *label_Yaw_Text = new QLabel();
    label_Yaw_Text->setText("Yaw:");
    QLabel *label_Yaw_Value = new QLabel();
    label_Yaw_Value->setText("0");

    QGridLayout *gVehicleMotionLayout = new QGridLayout();
    gVehicleMotionLayout->addWidget(label_Lon_Text,0,0);
    gVehicleMotionLayout->addWidget(label_Lon_Value,0,1);
    gVehicleMotionLayout->addWidget(label_Lat_Text,1,0);
    gVehicleMotionLayout->addWidget(label_Lat_Value,1,1);
    gVehicleMotionLayout->addWidget(label_Yaw_Text,2,0);
    gVehicleMotionLayout->addWidget(label_Yaw_Value,2,1);

    QGroupBox *gVehicleMotionGroup = new QGroupBox();
    gVehicleMotionGroup->setTitle("Motion");
    gVehicleMotionGroup->setFixedHeight(100);
    gVehicleMotionGroup->setLayout(gVehicleMotionLayout);

    /* Control Input Group */
    QLabel *label_TargetSteering_Text = new QLabel();
    label_TargetSteering_Text->setText("方向盘:");
    QLabel *label_TargetSteering_Value = new QLabel();
    label_TargetSteering_Value->setText("0");

    QLabel *label_TargetVelocity_Text = new QLabel();
    label_TargetVelocity_Text->setText("速度:");
    QLabel *label_TargetVelocity_Value = new QLabel();
    label_TargetVelocity_Value->setText("0");

    QLabel *label_TargetGear_Text = new QLabel();
    label_TargetGear_Text->setText("挡位:");
    QLabel *label_TargetGear_Value = new QLabel();
    label_TargetGear_Value->setText("0");

    QLabel *label_TargetTorque_Text = new QLabel();
    label_TargetTorque_Text->setText("扭矩:");
    QLabel *label_TargetTorque_Value = new QLabel();
    label_TargetTorque_Value->setText("0");

    QLabel *label_TargetAcc_Text = new QLabel();
    label_TargetAcc_Text->setText("加速度:");
    QLabel *label_TargetAcc_Value = new QLabel();
    label_TargetAcc_Value->setText("0");

    QGridLayout *gVehicleControlInputLayout = new QGridLayout();
    gVehicleControlInputLayout->addWidget(label_TargetSteering_Text,0,0);
    gVehicleControlInputLayout->addWidget(label_TargetSteering_Value,0,1);
    gVehicleControlInputLayout->addWidget(label_TargetVelocity_Text,1,0);
    gVehicleControlInputLayout->addWidget(label_TargetVelocity_Value,1,1);
    gVehicleControlInputLayout->addWidget(label_TargetGear_Text,2,0);
    gVehicleControlInputLayout->addWidget(label_TargetGear_Value,2,1);
    gVehicleControlInputLayout->addWidget(label_TargetTorque_Text,3,0);
    gVehicleControlInputLayout->addWidget(label_TargetTorque_Value,3,1);
    gVehicleControlInputLayout->addWidget(label_TargetAcc_Text,4,0);
    gVehicleControlInputLayout->addWidget(label_TargetAcc_Value,4,1);

    QGroupBox *gVehicleControlInputGroup = new QGroupBox();
    gVehicleControlInputGroup->setTitle("Control");
    gVehicleControlInputGroup->setFixedHeight(160);
    gVehicleControlInputGroup->setLayout(gVehicleControlInputLayout);

    button_timer_control = new QPushButton();
    button_timer_control->setText("开始");

    /********************* Control IO Layout *********************/
    QGridLayout *gControl_IO_Layout = new QGridLayout();
    gControl_IO_Layout->addWidget(gCanGroup,0,0);
    gControl_IO_Layout->addWidget(gVehicleStatusGroup,1,0);
    gControl_IO_Layout->addWidget(gVehicleEPSGroup,2,0);
    gControl_IO_Layout->addWidget(gVehicleVelocityGroup,3,0);
    gControl_IO_Layout->addWidget(gVehicleMotionGroup,4,0);
    gControl_IO_Layout->addWidget(gVehicleControlInputGroup,5,0);
    gControl_IO_Layout->addWidget(button_timer_control,6,0);
    gControl_IO_Layout->setRowStretch(0,1);
    gControl_IO_Layout->setRowStretch(1,1);
    gControl_IO_Layout->setRowStretch(2,1);
    gControl_IO_Layout->setRowStretch(3,1);
    gControl_IO_Layout->setRowStretch(4,1);
    gControl_IO_Layout->setRowStretch(5,1);
    gControl_IO_Layout->setRowStretch(6,1);
    gControl_IO_Layout->setRowStretch(7,1);
    // Control Plot 元素
    mControlPlot = new QCustomPlot();

    // configure plot to have two right axes:
    mControlPlot->yAxis->setTickLabels(false);
    connect(mControlPlot->yAxis2, SIGNAL(rangeChanged(QCPRange)), mControlPlot->yAxis, SLOT(setRange(QCPRange))); // left axis only mirrors inner right axis
    mControlPlot->yAxis2->setVisible(true);
    mControlPlot->axisRect()->addAxis(QCPAxis::atRight);
    mControlPlot->axisRect()->axis(QCPAxis::atRight, 0)->setPadding(30); // add some padding to have space for tags
    mControlPlot->axisRect()->axis(QCPAxis::atRight, 1)->setPadding(30); // add some padding to have space for tags

    mControlPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    // create graphs:
    mControlGraph1 = mControlPlot->addGraph(mControlPlot->xAxis, mControlPlot->axisRect()->axis(QCPAxis::atRight, 0));
    mControlGraph2 = mControlPlot->addGraph(mControlPlot->xAxis, mControlPlot->axisRect()->axis(QCPAxis::atRight, 1));
    mControlGraph1->setPen(QPen(QColor(250, 120, 0)));
    mControlGraph2->setPen(QPen(QColor(0, 180, 60)));

    // create tags with newly introduced AxisTag class (see axistag.h/.cpp):
    mControlTag1 = new AxisTag(mControlGraph1->valueAxis());
    mControlTag1->setPen(mControlGraph1->pen());
    mControlTag2 = new AxisTag(mControlGraph2->valueAxis());
    mControlTag2->setPen(mControlGraph2->pen());

    QGridLayout *gControlLayout = new QGridLayout();
    gControlLayout->addLayout(gControl_IO_Layout, 0, 0);
    gControlLayout->addWidget(mControlPlot, 0, 1);
    gControlLayout->setColumnStretch(0,1);
    gControlLayout->setColumnStretch(1,10);
    gControlLayout->setColumnMinimumWidth(0,100);

    /****************** Detect UI ******************/
    label_FrontObstacle_Text = new QLabel();
    label_FrontObstacle_Text->setText("前:");
    label_FrontObstacleDistance_Value = new QLabel();
    label_FrontObstacleDistance_Value->setText("0");
    label_FrontObstacleRegion_Value = new QLabel();
    label_FrontObstacleRegion_Value->setText("中间");
    label_FrontObstacleStatus_Value = new QLabel();
    label_FrontObstacleStatus_Value->setText("正常");

    label_RearObstacle_Text = new QLabel();
    label_RearObstacle_Text->setText("后:");
    label_RearObstacleDistance_Value = new QLabel();
    label_RearObstacleDistance_Value->setText("0");
    label_RearObstacleRegion_Value = new QLabel();
    label_RearObstacleRegion_Value->setText("中间");
    label_RearObstacleStatus_Value = new QLabel();
    label_RearObstacleStatus_Value->setText("正常");

    QGridLayout *gObstacleDistanceLayout = new QGridLayout();
    gObstacleDistanceLayout->addWidget(label_FrontObstacle_Text,0,0);
    gObstacleDistanceLayout->addWidget(label_FrontObstacleDistance_Value,0,1);
    gObstacleDistanceLayout->addWidget(label_FrontObstacleRegion_Value,0,2);
    gObstacleDistanceLayout->addWidget(label_FrontObstacleStatus_Value,0,3);
    gObstacleDistanceLayout->addWidget(label_RearObstacle_Text,1,0);
    gObstacleDistanceLayout->addWidget(label_RearObstacleDistance_Value,1,1);
    gObstacleDistanceLayout->addWidget(label_RearObstacleRegion_Value,1,2);
    gObstacleDistanceLayout->addWidget(label_RearObstacleStatus_Value,1,3);

    gObstacleDistanceLayout->setColumnStretch(0,1);
    gObstacleDistanceLayout->setColumnStretch(1,3);
    gObstacleDistanceLayout->setColumnStretch(2,2);
    gObstacleDistanceLayout->setColumnStretch(3,2);

    QGroupBox *gObstacleDistance = new QGroupBox();
    gObstacleDistance->setTitle("避障距离");
    gObstacleDistance->setFixedHeight(90);
    gObstacleDistance->setLayout(gObstacleDistanceLayout);

    QGridLayout *gDetect_Show_Layout = new QGridLayout();
    gDetect_Show_Layout->addWidget(gObstacleDistance,0,0);
    gDetect_Show_Layout->setRowStretch(0,1);
    gDetect_Show_Layout->setRowStretch(1,1);

    mDetectPlot = new QCustomPlot();
    // configure plot to have two right axes:
    mDetectPlot->yAxis->setTickLabels(false);
    connect(mDetectPlot->yAxis2, SIGNAL(rangeChanged(QCPRange)), mDetectPlot->yAxis, SLOT(setRange(QCPRange))); // left axis only mirrors inner right axis
    mDetectPlot->yAxis2->setVisible(true);
    mDetectPlot->axisRect()->addAxis(QCPAxis::atRight);
    mDetectPlot->axisRect()->axis(QCPAxis::atRight, 0)->setPadding(30); // add some padding to have space for tags
    mDetectPlot->axisRect()->axis(QCPAxis::atRight, 1)->setPadding(30); // add some padding to have space for tags

    mDetectPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    // create graphs:
    mDetectGraph1 = mDetectPlot->addGraph(mDetectPlot->xAxis, mDetectPlot->axisRect()->axis(QCPAxis::atRight, 0));
    mDetectGraph2 = mDetectPlot->addGraph(mDetectPlot->xAxis, mDetectPlot->axisRect()->axis(QCPAxis::atRight, 1));
    mDetectGraph1->setPen(QPen(QColor(250, 120, 0)));
    mDetectGraph2->setPen(QPen(QColor(0, 180, 60)));

    // create tags with newly introduced AxisTag class (see axistag.h/.cpp):
    mDetectTag1 = new AxisTag(mDetectGraph1->valueAxis());
    mDetectTag1->setPen(mDetectGraph1->pen());
    mDetectTag2 = new AxisTag(mDetectGraph2->valueAxis());
    mDetectTag2->setPen(mDetectGraph2->pen());


    QGridLayout *gDetectLayout = new QGridLayout();
    gDetectLayout->addLayout(gDetect_Show_Layout, 0,0);
    gDetectLayout->addWidget(mDetectPlot, 0, 1);
    gDetectLayout->setColumnStretch(0,1);
    gDetectLayout->setColumnStretch(1,9);

    /************************ Path UI ***************************/
    // 车辆初始位置 begin
    QLabel *label_VehicleInitPointX_Text = new QLabel();
    label_VehicleInitPointX_Text->setText("X:");
    QLabel *label_VehicleInitPointY_Text = new QLabel();
    label_VehicleInitPointY_Text->setText("Y:");
    QLabel *label_VehicleInitPointYaw_Text = new QLabel();
    label_VehicleInitPointYaw_Text->setText("Yaw:");

    text_VehicleInitPointX = new QLineEdit();
    text_VehicleInitPointX->setText("8");
    text_VehicleInitPointY = new QLineEdit();
    text_VehicleInitPointY->setText("3");
    text_VehicleInitPointYaw = new QLineEdit();
    text_VehicleInitPointYaw->setText("0");

    QGridLayout *gVehicleInitPosition_Layout = new QGridLayout();
    gVehicleInitPosition_Layout->addWidget(label_VehicleInitPointX_Text,0,0);
    gVehicleInitPosition_Layout->addWidget(label_VehicleInitPointY_Text,1,0);
    gVehicleInitPosition_Layout->addWidget(label_VehicleInitPointYaw_Text,2,0);

    gVehicleInitPosition_Layout->addWidget(text_VehicleInitPointX,0,1);
    gVehicleInitPosition_Layout->addWidget(text_VehicleInitPointY,1,1);
    gVehicleInitPosition_Layout->addWidget(text_VehicleInitPointYaw,2,1);

    QGroupBox *gVehicleInitPosition_Group = new QGroupBox();
    gVehicleInitPosition_Group->setTitle("车辆初始位置");
    gVehicleInitPosition_Group->setFixedHeight(120);
    gVehicleInitPosition_Group->setLayout(gVehicleInitPosition_Layout);
    // 车辆初始位置 end

    // 车位信息 begin
    QLabel *label_ParkingLength_Text = new QLabel();
    label_ParkingLength_Text->setText("长:");
    QLabel *label_ParkingWidth_Text = new QLabel();
    label_ParkingWidth_Text->setText("宽:");

    text_ParkingLength = new QLineEdit();
    text_ParkingLength->setText("6");
    text_ParkingWidth = new QLineEdit();
    text_ParkingWidth->setText("2.4");

    QGridLayout *gVehicleParking_Layout = new QGridLayout();
    gVehicleParking_Layout->addWidget(label_ParkingLength_Text,0,0);
    gVehicleParking_Layout->addWidget(label_ParkingWidth_Text,1,0);
    gVehicleParking_Layout->addWidget(text_ParkingLength,0,1);
    gVehicleParking_Layout->addWidget(text_ParkingWidth,1,1);
    gVehicleParking_Layout->setColumnStretch(0,1);
    gVehicleParking_Layout->setColumnStretch(1,3);

    QGroupBox *gVehicleParking_Group = new QGroupBox();
    gVehicleParking_Group->setTitle("车位信息");
    gVehicleParking_Group->setFixedHeight(100);
    gVehicleParking_Group->setLayout(gVehicleParking_Layout);
    // 车位信息 end

    QPushButton *gParkingInformationConfirm = new QPushButton();
    gParkingInformationConfirm->setText("库位信息确认");

    // 实时车辆位置跟踪 begin
    QLabel *label_VehiceTrackX_Text = new QLabel();
    label_VehiceTrackX_Text->setText("X:");
    QLabel *label_VehiceTrackY_Text = new QLabel();
    label_VehiceTrackY_Text->setText("Y:");
    QLabel *label_VehiceTrackYaw_Text = new QLabel();
    label_VehiceTrackYaw_Text->setText("Yaw:");

    label_VehiceTrackX_Value = new QLabel();
    label_VehiceTrackX_Value->setText("0");
    label_VehiceTrackY_Value = new QLabel();
    label_VehiceTrackY_Value->setText("0");
    label_VehiceTrackYaw_Value = new QLabel();
    label_VehiceTrackYaw_Value->setText("0");//单位 度

    QLabel *label_VehiceTrackX_Unit = new QLabel();
    label_VehiceTrackX_Unit->setText("m");
    QLabel *label_VehiceTrackY_Unit = new QLabel();
    label_VehiceTrackY_Unit->setText("m");
    QLabel *label_VehiceTrackYaw_Unit = new QLabel();
    label_VehiceTrackYaw_Unit->setText("°");

    QGridLayout *gVehicleTrack_Layout = new QGridLayout();
    gVehicleTrack_Layout->addWidget(label_VehiceTrackX_Text,0,0);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackY_Text,1,0);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackYaw_Text,2,0);

    gVehicleTrack_Layout->addWidget(label_VehiceTrackX_Value,0,1);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackY_Value,1,1);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackYaw_Value,2,1);

    gVehicleTrack_Layout->addWidget(label_VehiceTrackX_Unit,0,2);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackY_Unit,1,2);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackYaw_Unit,2,2);

    gVehicleTrack_Layout->setColumnStretch(0,2);
    gVehicleTrack_Layout->setColumnStretch(1,4);
    gVehicleTrack_Layout->setColumnStretch(1,1);

    QGroupBox *gVehicleTrack_Group = new QGroupBox();
    gVehicleTrack_Group->setTitle("实时跟踪");
    gVehicleTrack_Group->setFixedHeight(120);
    gVehicleTrack_Group->setLayout(gVehicleTrack_Layout);
    // 实时车辆位置跟踪 end

    QGridLayout *gPath_IO_Layout = new QGridLayout();
    gPath_IO_Layout->addWidget(gVehicleInitPosition_Group,0,0);
    gPath_IO_Layout->addWidget(gVehicleParking_Group,1,0);
    gPath_IO_Layout->addWidget(gParkingInformationConfirm,2,0);
    gPath_IO_Layout->addWidget(gVehicleTrack_Group,3,0);
    gPath_IO_Layout->setColumnMinimumWidth(0,70);
    gPath_IO_Layout->setRowStretch(0,1);
    gPath_IO_Layout->setRowStretch(1,1);
    gPath_IO_Layout->setRowStretch(2,1);
    gPath_IO_Layout->setRowStretch(3,1);
    gPath_IO_Layout->setRowStretch(4,1);


    mPathPlot = new QCustomPlot();
    mPathPlot->legend->setVisible(true);
    mPathPlot->legend->setFont(QFont("Helvetica", 9));
    mPathPlot->legend->setRowSpacing(-3);
    mPathPlot->xAxis->setLabel("x");
    mPathPlot->yAxis->setLabel("y");
    mPathPlot->xAxis->setRange(BOUNDARY_LEFT,BOUNDARY_RIGHT);
    mPathPlot->yAxis->setRange(BOUNDARY_DOWN,BOUNDARY_TOP);

    mPathVehicleModuleCurve = new QCPCurve(mPathPlot->xAxis,mPathPlot->yAxis);
    mPathVehicleModuleCurve->setName("车辆模型");
    mPathVehicleModuleCurve->setPen(QPen(Qt::red,3));
    mPathVehicleModuleCurve->setBrush(QBrush(QColor(90,35,255,20)));

    mPathParkingCurve = new QCPCurve(mPathPlot->xAxis,mPathPlot->yAxis);
    mPathParkingCurve->setName("库位");
    mPathParkingCurve->setPen(QPen(Qt::green,4));
    mPathParkingCurve->setBrush(QBrush(QColor(90,255,240,80)));

    mPathVehicleCenterCurve = new QCPCurve(mPathPlot->xAxis,mPathPlot->yAxis);
    mPathVehicleCenterCurve->setName("后轴中心");

    mPathLeftCircle = new QCPItemEllipse(mPathPlot);
    mPathLeftCircle->setPen(QPen(Qt::yellow,1));
    mPathLeftCircle->setBrush(QBrush(QColor(90,125,140,20)));

    mPathRightCircle = new QCPItemEllipse(mPathPlot);
    mPathRightCircle->setPen(QPen(Qt::black,1));
    mPathRightCircle->setBrush(QBrush(QColor(20,25,140,20)));

    ParkingPointX.resize(9);
    ParkingPointX[0] = BOUNDARY_LEFT;
    ParkingPointX[1] = 0;
    ParkingPointX[2] = 0;
    ParkingPointX[3] = 5;
    ParkingPointX[4] = 5;
    ParkingPointX[5] = BOUNDARY_RIGHT;
    ParkingPointX[6] = BOUNDARY_RIGHT;
    ParkingPointX[7] = BOUNDARY_LEFT;
    ParkingPointX[8] = BOUNDARY_LEFT;

    ParkingPointY.resize(9);

    ParkingPointY[0] = 0;
    ParkingPointY[1] = 0;
    ParkingPointY[2] = -2.3;
    ParkingPointY[3] = -2.3;
    ParkingPointY[4] = 0;
    ParkingPointY[5] = 0;
    ParkingPointY[6] = BOUNDARY_DOWN;
    ParkingPointY[7] = BOUNDARY_DOWN;
    ParkingPointY[8] = 0;

    mPathParkingCurve->setData(ParkingPointX,ParkingPointY);

    gPathLayout = new QGridLayout();

    gPathLayout->addLayout(gPath_IO_Layout, 0, 0);
    gPathLayout->addWidget(mPathPlot, 0, 1);
    gPathLayout->setColumnStretch(0,1);
    gPathLayout->setColumnStretch(1,9);

    QWidget *pControlWidget = new QWidget();
    QWidget *pDetectWidget = new QWidget();
    QWidget *pPathWidget = new QWidget();
    pControlWidget->setLayout(gControlLayout);
    pDetectWidget->setLayout(gDetectLayout);
    pPathWidget->setLayout(gPathLayout);

    QStackedWidget *stack_Widget = new QStackedWidget();
    stack_Widget->addWidget(pControlWidget);
    stack_Widget->addWidget(pDetectWidget);
    stack_Widget->addWidget(pPathWidget);

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(list_function,0,0);
    pMainLayout->addWidget(stack_Widget,0,1);
    // 设置第0列与第1列的比例为1：9
    pMainLayout->setColumnStretch(0,1);
    pMainLayout->setColumnStretch(1,9);
    pMainLayout->setColumnMinimumWidth(0,70);

    QWidget *PMainWidget = new QWidget();
    PMainWidget->setLayout(pMainLayout);

    setCentralWidget(PMainWidget);
    /****************** single connect slot *********************/
    /*** List Widget ***/
    // list 选择变动与 stack widget 界面更新的连接
    connect(list_function,SIGNAL(currentRowChanged(int)),stack_Widget,SLOT(setCurrentIndex(int)));
    // list 的元素变化时相应图片激活状态切换
    connect(list_function,SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)),this,SLOT(sProcessItemActiveState(QListWidgetItem*,QListWidgetItem*)));

    connect(&mDataTimer20ms,SIGNAL(timeout()),this,SLOT(sTimer20msTask()));
    connect(button_timer_control,SIGNAL(clicked()),this,SLOT(sTimer20ms_Control()));

    /*** CAN Configure single connect ***/
    // CAN Connect single and the Slot
    connect(button_CanConnect,SIGNAL(clicked()),this,SLOT(sCAN_Connect()));
    // CAN Open single and the Slot
    connect(button_CanOpen,SIGNAL(clicked()),this,SLOT(sCAN_Open()));
    // CAN Close single and the Slot
    connect(button_CanClose,SIGNAL(clicked()),this,SLOT(sCAN_Close()));

//    connect(&mCanRevWorkThread,SIGNAL(SendPercaptionMessage(Percaption*)),this,SLOT(sDisplayPercaption(Percaption*)));

    connect(gParkingInformationConfirm,SIGNAL(clicked()),this,SLOT(sParkingConfirm()));

    connect(mParallelPlanning,SIGNAL(sCircleCenterPoint(uint8_t,Circle*)),this,SLOT(sPathCirclePoint(uint8_t,Circle*)));
    connect(mVerticalPlanning,SIGNAL(sCircleCenterPoint(uint8_t,Circle*)),this,SLOT(sPathCirclePoint(uint8_t,Circle*)));

    // 定时器20ms
    mDataTimer20ms.start(20);
}

MainWindow::~MainWindow()
{
//    mCanRevWorkThread.quit();
//    mWinZlgCan.CanClose();
    delete ui;
}

void MainWindow::Init()
{
     mTerminal = new Terminal();
     mSimulation = new Simulation();

//    Percaption mPercaption;
//    GeometricTrack mGeometricTrack;
//    VehilceConfig mVehilceConfig;

     mBoRuiMessage = new BoRuiMessage();
     mBoRuiController= new BoRuiController();



//    // Path
//    ParallelPlanning mParallelPlanning;
    mParallelPlanning = new ParallelPlanning();
    mVerticalPlanning = new VerticalPlanning();

    mParallelPlanning->Init();
    mVerticalPlanning->Init();
}

/****** Function ******/

void MainWindow::VehicleModule(Vector2d p,float yaw)
{
    FrontLeftPoint  = p + Vector2d(static_cast<float>(mVehilceConfig.getFrontLeftDiagonal().Length),0.0f).rotate(static_cast<float>(mVehilceConfig.getFrontLeftDiagonal().Angle + yaw));
    FrontRightPoint = p + Vector2d(static_cast<float>(mVehilceConfig.getFrontRightDiagonal().Length),0.0f).rotate(static_cast<float>(mVehilceConfig.getFrontRightDiagonal().Angle + yaw));
    RearLeftPoint   = p + Vector2d(static_cast<float>(-mVehilceConfig.getRearLeftDiagonal().Length),0.0f).rotate(static_cast<float>(mVehilceConfig.getRearLeftDiagonal().Angle + yaw));
    RearRightPoint  = p + Vector2d(static_cast<float>(-mVehilceConfig.getRearRightDiagonal().Length),0.0f).rotate(static_cast<float>(mVehilceConfig.getRearRightDiagonal().Angle + yaw));

    mPathVehicleCenterCurve->addData(p.getX(),p.getY());

    QVector<double> VehiclePointX(5),VehiclePointY(5);
    VehiclePointX[0] = static_cast<double>(RearRightPoint.getX());
    VehiclePointX[1] = static_cast<double>(RearLeftPoint.getX());
    VehiclePointX[2] = static_cast<double>(FrontLeftPoint.getX());
    VehiclePointX[3] = static_cast<double>(FrontRightPoint.getX());
    VehiclePointX[4] = static_cast<double>(RearRightPoint.getX());

    VehiclePointY[0] = static_cast<double>(RearRightPoint.getY());
    VehiclePointY[1] = static_cast<double>(RearLeftPoint.getY());
    VehiclePointY[2] = static_cast<double>(FrontLeftPoint.getY());
    VehiclePointY[3] = static_cast<double>(FrontRightPoint.getY());
    VehiclePointY[4] = static_cast<double>(RearRightPoint.getY());
    mPathVehicleModuleCurve->setData(VehiclePointX,VehiclePointY);
//    mPathVehicleGraph->setData(VehiclePointX,VehiclePointY,true);

//    QVector<double> VehiclePointX(3),VehiclePointY(3);
//    VehiclePointX[0] = static_cast<double>(RearRightPoint.getX());
//    VehiclePointX[1] = static_cast<double>(RearLeftPoint.getX());
//    VehiclePointX[2] = static_cast<double>(FrontLeftPoint.getX());
//    VehiclePointY[0] = static_cast<double>(RearRightPoint.getY());
//    VehiclePointY[1] = static_cast<double>(RearLeftPoint.getY());
//    VehiclePointY[2] = static_cast<double>(FrontLeftPoint.getY());
//    mPathVehicleGraph->setData(VehiclePointX,VehiclePointY,true);

//    QVector<double> VehicleDownPointX(3),VehicleDownPointY(3);
//    VehicleDownPointX[0] = static_cast<double>(RearRightPoint.getX());
//    VehicleDownPointX[1] = static_cast<double>(FrontRightPoint.getX());
//    VehicleDownPointX[2] = static_cast<double>(FrontLeftPoint.getX());
//    VehicleDownPointY[0] = static_cast<double>(RearRightPoint.getY());
//    VehicleDownPointY[1] = static_cast<double>(FrontRightPoint.getY());
//    VehicleDownPointY[2] = static_cast<double>(FrontLeftPoint.getY());
//    mPathVehicleModuleDownGraph->setData(VehicleDownPointX,VehicleDownPointY,true);

    mPathPlot->replot();
}

/****** SLOT ******/

// 定时任务
void MainWindow::sTimer20msTask(void)
{
    float distance[6]={3.2f,4.2f,6.15f,2.3f,3.4f,1.3f};
    uint8_t min_index[6];
    SortSmallToBig(distance,min_index,4);
//    mBoRuiController.APAEnable = 1;
//    mBoRuiController.Velocity = 0.3f;
//    mBoRuiController.Gear = Drive;
//    mBoRuiController.SteeringAngle = 500;
//    mBoRuiController.SteeringAngleRate = 300;

//    mParallelPlanning->Work(&mPercaption);
//    mParallelPlanning->Control(mBoRuiController,mBoRuiMessage,&mGeometricTrack,&mPercaption);

    mVerticalPlanning->Work(&mPercaption,&mGeometricTrack);
    mVerticalPlanning->Control(mBoRuiController,mBoRuiMessage,&mGeometricTrack,&mPercaption);

    mSimulation->Update(mBoRuiController,mBoRuiMessage);

    mGeometricTrack.VelocityUpdate(mBoRuiMessage,0.02f);

    label_VehiceTrackX_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getPosition().X),'f',3));
    label_VehiceTrackY_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getPosition().Y),'f',3));
    label_VehiceTrackYaw_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getYaw()*57.3f),'f',3));

    VehicleModule(mGeometricTrack.getPosition(),mGeometricTrack.getYaw());


//    // calculate and add a new data point to each graph:
//    mControlGraph1->addData(mControlGraph1->dataCount(), qSin(mControlGraph1->dataCount()/50.0)+qSin(mControlGraph1->dataCount()/50.0/0.3843)*0.25);
//    mControlGraph2->addData(mControlGraph2->dataCount(), qCos(mControlGraph2->dataCount()/50.0)+qSin(mControlGraph2->dataCount()/50.0/0.4364)*0.15);

//    // make key axis range scroll with the data:
//    mControlPlot->xAxis->rescale();
//    mControlGraph1->rescaleValueAxis(false, true);
//    mControlGraph2->rescaleValueAxis(false, true);
//    mControlPlot->xAxis->setRange(mControlPlot->xAxis->range().upper, 100, Qt::AlignRight);

//    // update the vertical axis tag positions and texts to match the rightmost data point of the graphs:
//    double graph1Value = mControlGraph1->dataMainValue(mControlGraph1->dataCount()-1);
//    double graph2Value = mControlGraph2->dataMainValue(mControlGraph2->dataCount()-1);
//    mControlTag1->updatePosition(graph1Value);
//    mControlTag2->updatePosition(graph2Value);
//    mControlTag1->setText(QString::number(graph1Value, 'f', 2));
//    mControlTag2->setText(QString::number(graph2Value, 'f', 2));

//    mControlPlot->replot();
}

// 点击按钮
void MainWindow::sTimer20ms_Control(void)
{
    if(mDataTimer20ms.isActive())
    {
        mDataTimer20ms.stop();
        button_timer_control->setText("开始");
    }
    else
    {
        mDataTimer20ms.start(20);
        button_timer_control->setText("结束");
    }
}

void MainWindow::sProcessItemActiveState(QListWidgetItem *current, QListWidgetItem *previous)
{
    if(!previous)
    {
        current->setIcon(QIcon(":/Icon/active_car.png"));
    }
    else
    {
        switch (this->list_function->row(current)) {
            case 0:
                current->setIcon(QIcon(":/Icon/active_car.png"));
            break;

            case 1:
                current->setIcon(QIcon(":/Icon/active_detect.png"));
                break;

            case 2:
                current->setIcon(QIcon(":/Icon/active_path.png"));
                break;

            default:
            break;
        }
        switch (this->list_function->row(previous)) {
            case 0:
                previous->setIcon(QIcon(":/Icon/unactive_car.png"));
            break;

            case 1:
                previous->setIcon(QIcon(":/Icon/unactive_detect.png"));
                break;

            case 2:
                previous->setIcon(QIcon(":/Icon/unactive_path.png"));
                break;

            default:
            break;
        }
    }
}

void MainWindow::sCAN_Connect(void)
{
//    if(1 == mWinZlgCan.getConnectStatus())
//    {
//        mWinZlgCan.CanClose();
//        mCanRevWorkThread.quit();
//        this->button_CanOpen->setEnabled(false);
//        this->button_CanClose->setEnabled(false);
//        this->button_CanConnect->setText("连接");
//    }
//    else
//    {
//        mWinZlgCan.CanConnect();
//        if(1 == mWinZlgCan.getConnectStatus())
//        {
//            this->button_CanOpen->setEnabled(true);
//            this->button_CanClose->setEnabled(false);
//            this->button_CanConnect->setText("断开");
//        }
//    }
}

void MainWindow::sCAN_Open(void)
{
//    if((0 == mWinZlgCan.CanOpen(0)) && (0 == mWinZlgCan.CanOpen(1)) && (0 == mWinZlgCan.CanOpen(2)))
//    {
//        mCanRevWorkThread.start();
//        mWinZlgCan.setOpenStatus(1);
//        this->button_CanOpen->setEnabled(false);
//        this->button_CanClose->setEnabled(true);
//    }
//    else
//    {
//        QMessageBox::information(this, "错误", "CAN设备启动失败");
//        mWinZlgCan.setOpenStatus(0);
//        this->button_CanClose->setEnabled(false);
//        this->button_CanOpen->setEnabled(false);
//    }
}

void MainWindow::sCAN_Close(void)
{
//    mWinZlgCan.CanReset(0);
//    mWinZlgCan.CanReset(1);
//    mWinZlgCan.CanReset(2);
//    mWinZlgCan.setOpenStatus(0);
//    mCanRevWorkThread.quit();
//    this->button_CanOpen->setEnabled(true);
//    this->button_CanClose->setEnabled(false);
}

void MainWindow::sDisplayPercaption(Percaption *p)
{

    label_FrontObstacleDistance_Value->setText(QString::number(static_cast<double>(p->getFrontObstacleDistance().distance)));
    label_FrontObstacleRegion_Value->setText(obstacle_region[p->getFrontObstacleDistance().region]);
    label_FrontObstacleStatus_Value->setText(obstacle_status[p->getRearObstacleDistance().status]);

    label_RearObstacleDistance_Value->setText(QString::number(static_cast<double>(p->getRearObstacleDistance().distance)));
    label_RearObstacleRegion_Value->setText(obstacle_region[p->getRearObstacleDistance().region]);
    label_RearObstacleStatus_Value->setText(obstacle_status[p->getRearObstacleDistance().status]);

    // calculate and add a new data point to each graph:
//    mDetectGraph1->addData(mDetectGraph1->dataCount(), qSin(mDetectGraph1->dataCount()/50.0)+qSin(mDetectGraph1->dataCount()/50.0/0.3843)*0.25);
//    mDetectGraph2->addData(mDetectGraph2->dataCount(), qCos(mDetectGraph2->dataCount()/50.0)+qSin(mDetectGraph2->dataCount()/50.0/0.4364)*0.15);
    mDetectGraph1->addData(mDetectGraph1->dataCount(), static_cast<double>(p->getFrontObstacleDistance().distance));
    mDetectGraph2->addData(mDetectGraph2->dataCount(), static_cast<double>(p->getRearObstacleDistance().distance));

    // make key axis range scroll with the data:
    mDetectPlot->xAxis->rescale();
    mDetectGraph1->rescaleValueAxis(false, true);
    mDetectGraph2->rescaleValueAxis(false, true);
    mDetectPlot->xAxis->setRange(mDetectPlot->xAxis->range().upper, 100, Qt::AlignRight);

    // update the vertical axis tag positions and texts to match the rightmost data point of the graphs:
    double graph1Value = mDetectGraph1->dataMainValue(mDetectGraph1->dataCount()-1);
    double graph2Value = mDetectGraph2->dataMainValue(mDetectGraph2->dataCount()-1);
    mDetectTag1->updatePosition(graph1Value);
    mDetectTag2->updatePosition(graph2Value);
    mDetectTag1->setText(QString::number(graph1Value, 'f', 2));
    mDetectTag2->setText(QString::number(graph2Value, 'f', 2));

    mDetectPlot->replot();
}

// 库位确认按钮
void MainWindow::sParkingConfirm()
{
    mPercaption.PositionX = text_VehicleInitPointX->text().toFloat();
    mPercaption.PositionY = text_VehicleInitPointY->text().toFloat();
    mPercaption.AttitudeYaw = text_VehicleInitPointYaw->text().toFloat();

    mPercaption.ParkingLength = text_ParkingLength->text().toFloat();
    mPercaption.ParkingWidth  = text_ParkingWidth->text().toFloat();

    mGeometricTrack.Init(mPercaption.PositionX,mPercaption.PositionY,mPercaption.AttitudeYaw);

    ParkingPointX[1] = 0;
    ParkingPointX[2] = 0;
    ParkingPointX[3] = mPercaption.ParkingLength;
    ParkingPointX[4] = mPercaption.ParkingLength;

    ParkingPointY[1] = 0;
    ParkingPointY[2] = -mPercaption.ParkingWidth;
    ParkingPointY[3] = -mPercaption.ParkingWidth;
    ParkingPointY[4] = 0;

    mPathParkingCurve->setData(ParkingPointX,ParkingPointY);

    QVector<double> ParkingTrackPointX(1),ParkingTrackPointY(1);
    ParkingTrackPointX[0] = mGeometricTrack.getPosition().getX();
    ParkingTrackPointY[0] = mGeometricTrack.getPosition().getY();
    mPathVehicleCenterCurve->setData(ParkingTrackPointX,ParkingTrackPointY);

//    mParallelPlanning->Command = 0x60;
    mVerticalPlanning->Init();
    mVerticalPlanning->Command = 0x60;
}

//id:
// 0 -> right circle
// 1 -> left  circle
void MainWindow::sPathCirclePoint(uint8_t id,Circle *c)
{
    switch(id)
    {
        case 0:
        mPathRightCircle->topLeft->setCoords(c->Center.getX() - c->Radius,c->Center.getY() + c->Radius);
        mPathRightCircle->bottomRight->setCoords(c->Center.getX() + c->Radius,c->Center.getY() - c->Radius);
            break;

        case 1:
        mPathLeftCircle->topLeft->setCoords(c->Center.getX() - c->Radius,c->Center.getY() + c->Radius);
        mPathLeftCircle->bottomRight->setCoords(c->Center.getX() + c->Radius,c->Center.getY() - c->Radius);
            break;

        default:
            break;
    }
}

void MainWindow::SortSmallToBig(float *array,uint8_t *index_array,uint8_t cnt)
{
    float distance_temp = 0;
    uint8_t index_temp = 0;
    uint8_t i,j;

    for(i = 0; i < cnt;i++)
    {
        index_array[i] = i;
    }

    for(i=0;i < (cnt - 1);i++)
    {
        for(j=i;j < cnt;j++)
        {
            if(array[j] < array[i])
            {
                distance_temp = array[i];
                array[i] = array[j];
                array[j] = distance_temp;

                index_temp = index_array[i];
                index_array[i] = index_array[j];
                index_array[j] = index_temp;
            }
        }
    }
}
