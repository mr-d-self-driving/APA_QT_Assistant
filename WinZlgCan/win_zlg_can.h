#ifndef WIN_ZLG_CAN_H
#define WIN_ZLG_CAN_H

#include <QMainWindow>
#include <QLibrary>
#include <QMessageBox>
#include <stdlib.h>
#include <stdio.h>
#include "ControlCAN.h"

class WinZlgCan
{
public:
    WinZlgCan();

    void CanConnect();
    int16_t CanOpen(DWORD id);
    void CanReset(DWORD id);
    void CanClose();

    uint32_t CAN_ReceiveLength(uint32_t ind);
    uint32_t CAN_Receive(uint32_t ind,VCI_CAN_OBJ* packet);
    VCI_CAN_OBJ* CAN_Receive(uint32_t ind,uint32_t* num);

    //ind:CAN通道号
    //id:帧ID
    //length:帧长度
    //data:数据
    void CAN_Send(uint32_t ind,uint32_t id, uint8_t length, uint8_t* data);

    uint32_t getConnectStatus();
    uint32_t getOpenStatus();
    void setOpenStatus(uint32_t s);
private:
    QLibrary zlg_lib;   //声明所用到的dll文件

    DWORD m_devtype = VCI_USBCAN_4E_U;
    //usb-e-u 波特率
    DWORD GCanBrTab[10]= {
        1000000,800000,500000,
        250000, 125000, 100000,
        50000, 20000, 10000,
        5000
    };

//    uint32_t STATUS_OK = 1;
    uint32_t m_bOpen = 0;
    uint32_t m_bConnect = 0;
    uint32_t m_devind = 0;
    uint32_t m_canind = 0;

    VCI_CAN_OBJ m_recobj[50];

    uint32_t m_arrdevtype[20];
};

#endif // WIN_ZLG_CAN_H
