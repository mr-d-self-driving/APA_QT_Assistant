#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mPlot = new QCustomPlot(this);
    setCentralWidget(mPlot);
}

MainWindow::~MainWindow()
{
    delete ui;
}
