#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // 重新配置窗体大小
    this->resize(1080,768);


    /**************************************************/
    QListWidget *list_function = new QListWidget();
    /*车辆控制*/
    QListWidgetItem *control_item = new QListWidgetItem(QIcon(":/Icon/unactive_car.png"),tr("Control"));
    list_function->addItem(control_item);
    /*检测模块*/
    QListWidgetItem *detect_item = new QListWidgetItem(QIcon(":/Icon/unactive_car.png"),tr("Control"));
    list_function->addItem(detect_item);
    /*整体列表配置*/
    list_function->setViewMode(QListWidget::IconMode);//设置显示模式为图片模式
    list_function->setDragEnabled(false);//控件不允许拖动
    list_function->setFlow(QListWidget::TopToBottom);//设置元素排列方式从上往下
    list_function->setResizeMode(QListWidget::Adjust);//
    list_function->setMovement(QListWidget::Static);

    list_function->setIconSize(QSize(list_function->width(),100));
    list_function->setMinimumHeight(100);
    /**************************************************/
    QStackedWidget *stack_Widget = new QStackedWidget();

    QPushButton *button_G1 = new QPushButton(tr("QGridLayout第一个按钮"));
    QPushButton *button_G2 = new QPushButton(tr("QGridLayout第二个按钮"));


    QGridLayout *gControlLayout = new QGridLayout();
    gControlLayout->addWidget(button_G1, 0, 0);        // 第0行0列，第一个参数为控，第二个参数为第几行，第三个参数为第几列
    gControlLayout->addWidget(button_G2, 1, 1);        // 第1行1列

    mPlot = new QCustomPlot();
    QGridLayout *gDetectLayout = new QGridLayout();
    gDetectLayout->addWidget(mPlot, 0, 1);        // 第2行0列
    gDetectLayout->setColumnStretch(0,1);
    gDetectLayout->setColumnStretch(1,9);

    QWidget *pControlWidget = new QWidget();
    QWidget *pDetectWidget = new QWidget();
    pControlWidget->setLayout(gControlLayout);
    pDetectWidget->setLayout(gDetectLayout);

    stack_Widget->addWidget(pControlWidget);
    stack_Widget->addWidget(pDetectWidget);

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(list_function,0,0);
    pMainLayout->addWidget(stack_Widget,0,1);
    // 设置第0列与第1列的比例为1：9
    pMainLayout->setColumnStretch(0,1);
    pMainLayout->setColumnStretch(1,9);
    pMainLayout->setColumnMinimumWidth(0,100);

    QWidget *PMainWidget = new QWidget();
    PMainWidget->setLayout(pMainLayout);

    setCentralWidget(PMainWidget);

    /****************** single connect slot *********************/
    connect(list_function,SIGNAL(currentRowChanged(int)),stack_Widget,SLOT(setCurrentIndex(int)));

    connect(list_function,SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)),this,SLOT(ProcessItemActiveState(QListWidgetItem*,QListWidgetItem*)));
//    connect(list_function,SIGNAL(itemActivated(QListWidgetItem*)),this,SLOT(ProcessItemActiveState(QListWidgetItem*)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::ProcessItemActiveState(QListWidgetItem *current, QListWidgetItem *previous)
{
    if(NULL == previous)
    {
        current->setIcon(QIcon(":/Icon/active_car.png"));
    }
    else
    {
        current->setIcon(QIcon(":/Icon/active_car.png"));
        previous->setIcon(QIcon(":/Icon/unactive_car.png"));
    }
}
void MainWindow::ProcessItemActiveState(QListWidgetItem *current)
{
    current->setIcon(QIcon(":/Icon/active_car.png"));
}
