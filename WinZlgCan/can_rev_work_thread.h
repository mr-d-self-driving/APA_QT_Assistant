#ifndef CAN_REV_WORK_THREAD_H
#define CAN_REV_WORK_THREAD_H

#include <QMainWindow>
#include <QThread>
#include <QMutex>
#include "./WinZlgCan/win_zlg_can.h"
#include "Percaption/Interface/percaption.h"
#include "Interaction/HMI/Terminal.h"

class CanRevWorkThread : public QThread
{
    Q_OBJECT
public:
    CanRevWorkThread();

    void CAN0Parse(uint32_t num,VCI_CAN_OBJ* packet);
    void CAN1Parse(uint32_t num,VCI_CAN_OBJ* packet);
    void CAN2Parse(uint32_t num,VCI_CAN_OBJ* packet);

    void ChangAnVehicleParse(VCI_CAN_OBJ packet);
    void BoRuiVehicleParse(VCI_CAN_OBJ packet);
    void DongFengE70VehicleParse(VCI_CAN_OBJ packet);

    //前视解码
    void FrontViewParse(VCI_CAN_OBJ packet);
    // 嵌入式平台解码
    void NXP_PlatformParse(VCI_CAN_OBJ packet);
protected:
    virtual void run();

private:
    WinZlgCan _m_zlg_can;
    VCI_CAN_OBJ *_rev_can_data;
    QMutex lockMute;

    Terminal t_Terminal;
    Percaption t_Percaption;

signals:
    void SendPercaptionMessage(Percaption *p);
//    void TestSignal(VehInf,int);
//    void NoData(int);
//    void SendPInfo(int, int, int, MvParkSlot, int);
//    void SendPInfoTime(unsigned int,unsigned int,int);
//    void SendPInfoTimeFisrt(unsigned int,unsigned int,int);
//    void SendCAInfo(VehInfCA);
//    void SendSlotInfo(MvParkingInfo);
//    void SendCurbInfo(MvCurbInfo);
//    void SendFreePInfo(int, MvWorldPoint);

//    //lin
//    void SendSignal318(int,int,LIN_STP318_ReadData);
//    void SendSignal313(int,int,LIN_STP313_ReadData);
//    void SendSignalD(float,int);

//    void SendACK(int,int);
//    void SendNxpPosition(int, int, int, int );
//    void SendNxpUPosition(int, float, float, int );
//    void SendLaneInfo(LaneInfo);
//    void SendWarningFlag(bool ,unsigned int);
//    void SendObst(unsigned int, Obs);

//    void SendNxpACK(int);
//    void SendNxpCtlStauts(NXPCTRL);
};

#endif // CAN_REV_WORK_THREAD_H
