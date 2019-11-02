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
};

#endif // CAN_REV_WORK_THREAD_H
