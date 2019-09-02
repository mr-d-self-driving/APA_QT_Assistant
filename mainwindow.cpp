#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->resize(1080,768);

    QGridLayout *main_layout = new QGridLayout();
    /**************************************************/
    QListWidget *list_function = new QListWidget();
    /*车辆控制*/
    QListWidgetItem *control_item = new QListWidgetItem(QIcon(":/Icon/unactive_car.png"),tr("Control"));
//    control_item->setSizeHint(w->sizeHint());
//    control_item->setSizeHint(QSize(200,100));

    list_function->addItem(control_item);
//    list_function->setItemWidget(control_item,w);
    /*检测模块*/
    QListWidgetItem *detect_item = new QListWidgetItem(QIcon(":/Icon/active_car.png"),tr("Control"));
    list_function->addItem(detect_item);
    /*整体列表配置*/
    list_function->setViewMode(QListWidget::IconMode);//设置显示模式为图片模式
    list_function->setDragEnabled(false);//控件不允许拖动
    list_function->setFlow(QListWidget::TopToBottom);//设置元素排列方式从上往下
    list_function->setResizeMode(QListWidget::Adjust);//
    list_function->setMovement(QListWidget::Static);

    list_function->setIconSize(QSize(list_function->width(),100));
//    list_function->resize(QSize(100,-1));
    /**************************************************/
    QStackedWidget *stack_Widget = new QStackedWidget();

    QPushButton *button_G1 = new QPushButton(tr("QGridLayout第一个按钮"));
    QPushButton *button_G2 = new QPushButton(tr("QGridLayout第二个按钮"));
    QPushButton *button_G3 = new QPushButton(tr("QGridLayout第三个按钮"));
    QPushButton *button_G4 = new QPushButton(tr("QGridLayout第四个按钮"));

    QGridLayout *glayout = new QGridLayout();
    glayout->addWidget(button_G1, 0, 0);        // 第0行0列，第一个参数为控，第二个参数为第几行，第三个参数为第几列
    glayout->addWidget(button_G2, 1, 1);        // 第1行1列

    QGridLayout *glayout1 = new QGridLayout();
    glayout1->addWidget(button_G3, 2, 0);        // 第2行0列
    glayout1->addWidget(button_G4, 3, 1);        // 第2行1列

    QWidget *pWidget = new QWidget();
    pWidget->setLayout(glayout);

    QWidget *pWidget1 = new QWidget();
    pWidget1->setLayout(glayout1);

    stack_Widget->addWidget(pWidget);
    stack_Widget->addWidget(pWidget1);

    main_layout->addWidget(list_function,0,0);
    main_layout->addWidget(stack_Widget,0,1);
    // 设置第0列与第1列的比例为1：9
    main_layout->setColumnStretch(0,1);
    main_layout->setColumnStretch(1,9);
    main_layout->setColumnMinimumWidth(0,100);

    QWidget *mian_widget = new QWidget();

    mian_widget->setLayout(main_layout);

    setCentralWidget(mian_widget);

    connect(list_function,SIGNAL(currentRowChanged(int)),stack_Widget,SLOT(setCurrentIndex(int)));
//    this->MainWindow.geometry();
//    mPlot = new QCustomPlot(this);
    /******** 第一个TAB 相关内容 车辆控制界面 ********/
//    // 格栅布局(控件大小随着窗口变化自动拉伸)
//    QPushButton *button_G1 = new QPushButton(tr("QGridLayout第一个按钮"));
//    QPushButton *button_G2 = new QPushButton(tr("QGridLayout第二个按钮"));
//    QPushButton *button_G3 = new QPushButton(tr("QGridLayout第三个按钮"));
//    QPushButton *button_G4 = new QPushButton(tr("QGridLayout第四个按钮"));

//    QGridLayout *glayout = new QGridLayout();
//    glayout->addWidget(button_G1, 0, 0);        // 第0行0列，第一个参数为控，第二个参数为第几行，第三个参数为第几列
//    glayout->addWidget(button_G2, 1, 1);        // 第1行1列
////    glayout->addWidget(button_G3, 2, 0);        // 第2行0列
////    glayout->addWidget(button_G4, 2, 1);        // 第2行1列

//    QGridLayout *glayout1 = new QGridLayout();

////    glayout1->addWidget(button_G1, 0, 0);        // 第0行0列，第一个参数为控，第二个参数为第几行，第三个参数为第几列
////    glayout1->addWidget(button_G2, 1, 1);        // 第1行1列
//    glayout1->addWidget(button_G3, 2, 0);        // 第2行0列
//    glayout1->addWidget(button_G4, 3, 1);        // 第2行1列

//    QWidget *pWidget = new QWidget();
//    pWidget->setLayout(glayout);

//    QWidget *pWidget1 = new QWidget();
//    pWidget1->setLayout(glayout1);

//    QTabWidget *m_table_widget = new QTabWidget();


//    m_table_widget->addTab(pWidget,"test1");

//    m_table_widget->addTab(pWidget1,"test2");

//    setCentralWidget(m_table_widget);

}

MainWindow::~MainWindow()
{
    delete ui;
}
