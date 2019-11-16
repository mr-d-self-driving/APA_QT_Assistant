#-------------------------------------------------
#
# Project created by QtCreator 2019-08-30T14:39:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = APA_Assistant
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
CONFIG += resources_big

SOURCES += \
        Common/Configure/Configs/vehilce_config.cpp \
        Common/Math/algebraic_geometry.cpp \
        Common/Math/crc_compute.cpp \
        Common/Math/curve_fitting.cpp \
        Common/Math/huogh.cpp \
        Common/Math/interpolation.cpp \
        Common/Math/solve_equation.cpp \
        Common/Math/vector_2d.cpp \
        Common/Math/vehicle_body.cpp \
        Common/Utils/Src/link_list.cpp \
        Common/Utils/Src/node.cpp \
        Common/VehicleState/GeometricTrack/geometric_track.cpp \
        Common/VehicleState/Interface/vehicle_state.cpp \
        Interaction/CANBUS/BoRui/bo_rui_controller.cpp \
        Interaction/CANBUS/BoRui/bo_rui_message.cpp \
        Interaction/CANBUS/ChangAn/chang_an_controller.cpp \
        Interaction/CANBUS/ChangAn/chang_an_message.cpp \
        Interaction/CANBUS/DongFengE70/dong_feng_e70_message.cpp \
        Interaction/CANBUS/Interface/message_manager.cpp \
        Interaction/CANBUS/Interface/vehicle_controller.cpp \
        Interaction/HMI/Terminal.cpp \
        Interaction/HMI/simulation.cpp \
        Interaction/Ultrasonic/Ultrasonic.cpp \
        Percaption/Interface/percaption.cpp \
        Percaption/UltrasonicPercaption/ultrasonic_obstacle_percption.cpp \
        Planning/Interface/planning.cpp \
        Planning/ParallelParking/parallel_planning.cpp \
        Planning/VerticalParking/vertical_planning.cpp \
        QCustomPlot/axistag.cpp \
        main.cpp \
        mainwindow.cpp \
        QCustomPlot/qcustomplot.cpp

HEADERS += \
        Common/Configure/Configs/system_config.h \
        Common/Configure/Configs/vehilce_config.h \
        Common/Configure/Data/bo_rui_configure.h \
        Common/Configure/Data/chang_an_configure.h \
        Common/Configure/Data/common_configure.h \
        Common/Configure/Data/dong_feng_configure.h \
        Common/Math/algebraic_geometry.h \
        Common/Math/crc_compute.h \
        Common/Math/curve_fitting.h \
        Common/Math/huogh.h \
        Common/Math/interpolation.h \
        Common/Math/solve_equation.h \
        Common/Math/vector_2d.h \
        Common/Math/vehicle_body.h \
        Common/Utils/Inc/link_list.h \
        Common/Utils/Inc/node.h \
        Common/Utils/Inc/property.h \
        Common/VehicleState/GeometricTrack/geometric_track.h \
        Common/VehicleState/Interface/vehicle_state.h \
        Interaction/CANBUS/BoRui/bo_rui_controller.h \
        Interaction/CANBUS/BoRui/bo_rui_message.h \
        Interaction/CANBUS/ChangAn/chang_an_controller.h \
        Interaction/CANBUS/ChangAn/chang_an_message.h \
        Interaction/CANBUS/DongFengE70/dong_feng_e70_controller.h \
        Interaction/CANBUS/DongFengE70/dong_feng_e70_message.h \
        Interaction/CANBUS/Interface/message_manager.h \
        Interaction/CANBUS/Interface/vehicle_controller.h \
        Interaction/HMI/Terminal.h \
        Interaction/HMI/simulation.h \
        Interaction/Ultrasonic/Ultrasonic.h \
        Percaption/Interface/percaption.h \
        Percaption/UltrasonicPercaption/ultrasonic_obstacle_percption.h \
        Planning/Interface/planning.h \
        Planning/ParallelParking/parallel_planning.h \
        Planning/VerticalParking/vertical_planning.h \
        QCustomPlot/axistag.h \
        mainwindow.h \
        QCustomPlot/qcustomplot.h

LIBS += -L$$PWD/WinZlgCan/ -lControlCAN

FORMS += \
        mainwindow.ui

RESOURCES += \
    icon.qrc

DISTFILES +=
