#include "win_zlg_can.h"

WinZlgCan::WinZlgCan()
{
    m_bOpen = 0;
    m_devtype = VCI_USBCAN_4E_U;
    m_devind = 0;

}

void WinZlgCan::CanConnect()
{
    if(m_bConnect == 0)
    {
        if (VCI_OpenDevice(m_devtype, m_devind, 0) == 0)
        {
            QMessageBox::information(NULL, "错误", "打开设备失败,请检查设备类型和设备索引号是否正确");
            m_bConnect = 0;
        }
        else
        {
            m_bConnect = 1;
        }
    }
}

int16_t WinZlgCan::CanOpen(DWORD id)
{
    if (m_bConnect == 0 || 1 == m_bOpen)
        return -1;

    //USB-E-U 代码
    DWORD baud = GCanBrTab[2];
    if (VCI_SetReference(m_devtype, m_devind, id, 0, (DWORD*)&baud) != STATUS_OK)
    {
        QMessageBox::information(NULL, "错误", "设置波特率错误，打开设备0失败!");
        VCI_CloseDevice(m_devtype, m_devind);
        return -1;
    }
    //滤波设置
    //////////////////////////////////////////////////////////////////////////
    VCI_INIT_CONFIG config;
    config.AccCode = 00000000;// System.Convert.ToUInt32("0x" + textBox_AccCode.Text, 16);
    config.AccMask = 0xFFFFFFFF;// System.Convert.ToUInt32("0x" + textBox_AccMask.Text, 16);
    config.Timing0 = 0;// System.Convert.ToByte("0x" + textBox_Time0.Text, 16);
    config.Timing1 = 14;// System.Convert.ToByte("0x" + textBox_Time1.Text, 16);
    config.Filter = 1;// 单滤波 (Byte)comboBox_Filter.SelectedIndex;
    config.Mode = 0;//正常模式 (Byte)comboBox_Mode.SelectedIndex;
    VCI_InitCAN(m_devtype, m_devind, id,&config);
    //////////////////////////////////////////////////////////////////////////
    UINT filterMode = 2;// comboBox_e_u_Filter.SelectedIndex;
    if (2 != filterMode)//不是禁用
    {
        VCI_FILTER_RECORD filterRecord;
        filterRecord.ExtFrame = (UINT)filterMode;
        filterRecord.Start = 1;// System.Convert.ToUInt32("0x" + textBox_e_u_startid.Text, 16);
        filterRecord.End = 0xff;// System.Convert.ToUInt32("0x" + textBox_e_u_endid.Text, 16);
                                //填充滤波表格

        VCI_SetReference(m_devtype, m_devind, id, 1, (BYTE*)&filterRecord);
        //使滤波表格生效
        uint8_t tm = 0;
        if (VCI_SetReference(m_devtype, m_devind, id, 2, &tm) != STATUS_OK)
        {
            QMessageBox::information(NULL, "错误", "设置滤波失败");
            VCI_CloseDevice(m_devtype, m_devind);
            return -1;
        }
    }
    VCI_StartCAN(m_devtype, m_devind, id);
    return 0;
}

void WinZlgCan::CanReset(DWORD id)
{
    if (m_bOpen == 0)
        return;
    VCI_ResetCAN(m_devtype, m_devind, id);
}

void WinZlgCan::CanClose()
{
    VCI_CloseDevice(m_devtype, m_devind);
    m_bOpen = 0;
    m_bConnect = 0;
}

uint32_t WinZlgCan::CAN_ReceiveLength(uint32_t ind)
{
    uint32_t res;
    VCI_ERR_INFO errinfo;
    res = VCI_GetReceiveNum(m_devtype, m_devind, ind);
    if (res <= 0)
    {
        VCI_ReadErrInfo(m_devtype, m_devind, ind,&errinfo);
        return 0;
    }
    else
    {
        return res;
    }
}
uint32_t WinZlgCan::CAN_Receive(uint32_t ind,VCI_CAN_OBJ* packet)
{
    VCI_ERR_INFO errinfo;
    uint32_t res;
    res = VCI_GetReceiveNum(m_devtype, m_devind, ind);
    if (res <= 0)
    {
        VCI_ReadErrInfo(m_devtype, m_devind, ind,&errinfo);
        return res;
    }
    else
    {
        PVCI_CAN_OBJ zlg_rec_buffer = (PVCI_CAN_OBJ)malloc(res*sizeof(VCI_CAN_OBJ));
        res = VCI_Receive(m_devtype, m_devind, ind, zlg_rec_buffer, res, 100);
        packet = (PVCI_CAN_OBJ)malloc(res*sizeof(VCI_CAN_OBJ));
        memcpy(packet,zlg_rec_buffer,res*sizeof(VCI_CAN_OBJ));
        return res;
    }
}

VCI_CAN_OBJ* WinZlgCan::CAN_Receive(uint32_t ind,uint32_t* num)
{
    VCI_ERR_INFO errinfo;
    uint32_t res;
    res = VCI_GetReceiveNum(m_devtype, m_devind, ind);
    if (res <= 0)
    {
        VCI_ReadErrInfo(m_devtype, m_devind, ind,&errinfo);
        *num = 0;
        return NULL;
    }
    else
    {
        PVCI_CAN_OBJ zlg_rec_buffer = (PVCI_CAN_OBJ)malloc(res*sizeof(VCI_CAN_OBJ));
        res = VCI_Receive(m_devtype, m_devind, ind, zlg_rec_buffer, res, 100);
        *num = res;
        return zlg_rec_buffer;
    }
}

void WinZlgCan::CAN_Send(uint32_t ind,uint32_t id, uint8_t length, uint8_t* data)
{
    if (m_bOpen == 0)return;

    VCI_CAN_OBJ sendobj;
    sendobj.SendType = 0;//0 -> 正常发送 ;2 -> 自发自收(byte)comboBox_SendType.SelectedIndex;
    sendobj.RemoteFlag = 0;//标准帧 (byte)comboBox_FrameFormat.SelectedIndex;
    sendobj.ExternFlag = 0;// 标准帧数(byte)comboBox_FrameType.SelectedIndex;
    sendobj.ID = id;// System.Convert.ToUInt32("0x" + textBox_ID.Text, 16);
    sendobj.DataLen = length;

    for (int i = 0; i < 8; i++)
    {
        sendobj.Data[i] = data[i];
    }
    int nTimeOut = 3000;
    VCI_SetReference(m_devtype, m_devind, ind, 4, (BYTE*)&nTimeOut);
    if (VCI_Transmit(m_devtype, m_devind, ind, &sendobj, 1) == 0)
    {
//        QMessageBox::information(NULL, "错误", "发送失败");
        m_bOpen = 0;
        m_bConnect = 0;
    }
}

uint32_t WinZlgCan::getConnectStatus()
{
    return m_bConnect;
}

uint32_t WinZlgCan::getOpenStatus()
{
    return m_bOpen;
}

void WinZlgCan::setOpenStatus(uint32_t s)
{
    m_bOpen = s;
}
