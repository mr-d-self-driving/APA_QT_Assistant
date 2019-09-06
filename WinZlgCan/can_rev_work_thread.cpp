#include "can_rev_work_thread.h"

CanRevWorkThread::CanRevWorkThread()
{
//    tFlagCA = false;
//   _rev_can_data = (VCI_CAN_OBJ*)malloc(sizeof(VCI_CAN_OBJ));
}

void CanRevWorkThread::run()
{
    uint32_t len;
    while(1)
    {
        lockMute.lock();
        // CAN0 接收解码
        _rev_can_data = _m_zlg_can.CAN_Receive(0,&len);
        this->CAN0Parse(len,_rev_can_data);
        free(_rev_can_data);

        // CAN1 接收解码
        _rev_can_data = _m_zlg_can.CAN_Receive(1,&len);
        this->CAN1Parse(len,_rev_can_data);
        free(_rev_can_data);

        // CAN2 接收解码
        _rev_can_data = _m_zlg_can.CAN_Receive(2,&len);
        this->CAN2Parse(len,_rev_can_data);
        free(_rev_can_data);

        lockMute.unlock();
        msleep(10);
    }

}

void CanRevWorkThread::CAN0Parse(uint32_t num,VCI_CAN_OBJ* packet)
{
    uint32_t i;
    for(i=0;i<num;i++)
    {
        BoRuiVehicleParse(packet[i]);
    }
}

/****************************************************************/
/*  各类车辆的解码函数 */
/****************************************************************/
void CanRevWorkThread::ChangAnVehicleParse(VCI_CAN_OBJ packet)
{
//    switch(packet.ID)
//    {
//                    //长安车
//                case 0x2A3://eps status
//                    mVehInfCA.EPS2_FAILED_Status = (uint8_t)((packet.Data[2] >> 7) & 0x01);
//                    mVehInfCA.EPS_FAILED_Status = (uint8_t)((packet.Data[2] >> 1) & 0x01);//EPS  !!!!!!!
//                    mVehInfCA.torque_sensor_status = (uint8_t)( packet.Data[2] & 0x01 );
//                    mVehInfCA.apa_control_feedback = (uint8_t)((packet.Data[3] >> 5) & 0x01);
//                    mVehInfCA.SteeringTorque = (float)(packet.Data[4] * 0.1794 - 22.78);

//                    mVehInfCA.abort_feedback_status = (uint8_t)((packet.Data[2] >> 4) & 0x07);//ABORT FEEDBACK!!!!!!!
//                    tFlagCA = true;
//                    break;


//                case 0x20B:// wheel speed
//                    mVehInfCA.RearRight.direction = (uint8_t)(packet.Data[0] >> 5) & 0x03;
//                    mVehInfCA.RearRight.vaild = (uint8_t)(  packet.Data[0] >> 7) & 0x01;
//                    mVehInfCA.RearRight.data = ((uint16_t)(((packet.Data[0] & 0x1F) << 8) | packet.Data[1]))*0.05625;

//                    mVehInfCA.RearLeft.direction = (uint8_t)(packet.Data[2] >> 5) & 0x03;
//                    mVehInfCA.RearLeft.vaild = (uint8_t)(  packet.Data[2] >> 7) & 0x01;
//                    mVehInfCA.RearLeft.data = ((uint16_t)(((packet.Data[2] & 0x1F) << 8) | packet.Data[3]))*0.05625;

//                    mVehInfCA.FrontRight.direction = (uint8_t)(packet.Data[4] >> 5) & 0x03;
//                    mVehInfCA.FrontRight.vaild = (uint8_t)(  packet.Data[4] >> 7) & 0x01;
//                    mVehInfCA.FrontRight.data = ((uint16_t)(((packet.Data[4] & 0x1F) << 8) | packet.Data[5]))*0.05625;

//                    mVehInfCA.FrontLeft.direction = (uint8_t)(packet.Data[6] >> 5) & 0x03;
//                    mVehInfCA.FrontLeft.vaild = (uint8_t)(  packet.Data[6] >> 7) & 0x01;
//                    mVehInfCA.FrontLeft.data = ((uint16_t)(((packet.Data[6] & 0x1F) << 8) | packet.Data[7]))*0.05625;
//                    tFlagCA = true;
//                    break;

//                case 0x21B://vehicle speed
//                    mVehInfCA.vehicle_speed_valid = (uint8_t)( packet.Data[4] >> 5 ) & 0x01;
//                    mVehInfCA.Speed = (float)(((uint16_t)(((packet.Data[4] & 0x1F) << 8) | packet.Data[5])) * 0.05625);
//                    tFlagCA = true;
//                    break;

//                case 0x25B://Wheel speed pulse
//                    mVehInfCA.wheel_speed_direction = (uint8_t)(packet.Data[2] & 0x03);//0+ ; 1- ; 2STOP
//    //                mVehInfCA.wheel_speed_direction = (uint8_t)((packet.Data[1]>>1) & 0x03);//1+ ; 2- ; 0STOP
//                    if (mVehInfCA.wheel_speed_direction == 0)
//                        mVehInfCA.wheel_speed_direction = 1;
//                    else if (mVehInfCA.wheel_speed_direction == 1)
//                        mVehInfCA.wheel_speed_direction = 2;
//                    else if (mVehInfCA.wheel_speed_direction == 2)
//                        mVehInfCA.wheel_speed_direction = 0;

//                    //                mVehInfCA.wheel_speed_rear_right_pulse  = packet.Data[4];
//                    //                mVehInfCA.wheel_speed_rear_left_pulse   = packet.Data[5];
//                    mVehInfCA.wheel_speed_rear_right_pulse  = packet.Data[7];
//                    mVehInfCA.wheel_speed_rear_left_pulse   = packet.Data[4];

//                    mVehInfCA.wheel_speed_front_right_pulse = packet.Data[5];
//                    mVehInfCA.wheel_speed_front_left_pulse  = packet.Data[6];
//                    mVehInfCA.wheel_speed_count = (uint8_t)(packet.Data[3] & 0x0f);
//                    tFlagCA = true;
//                    break;

//                case 0x27A://ESP
//                    mVehInfCA.ESP_FAILED_Status = (uint8_t)( (packet.Data[5] >> 1) & 0x03);//ESP !
//                    tFlagCA = true;
//                    break;

//                case 0x27B://LAT ACC
//                    mVehInfCA.lat_acc = (uint8_t)( packet.Data[2] );//LAT ACC 0.1
//                    mVehInfCA.long_acc = (uint16_t)((uint16_t)((packet.Data[3] << 2) | packet.Data[4]>>6));//longACC 0.03125
//                    mVehInfCA.yaw_acc = (uint16_t)((uint16_t)(((packet.Data[4]&0x3F) << 8) | packet.Data[5]));//yaw ACC 0.01
//                    tFlagCA = true;
//                    break;

//                case 0x26B://GEAR
//                    switch((uint8_t)(packet.Data[1]) & 0x0F)
//                    {
//                    case 10:
//                        mVehInfCA.uGear = 1;//Parking;
//                        break;

//                    case 9:
//                        mVehInfCA.uGear = 2;//Reverse;
//                        break;

//                    case 0:
//                        mVehInfCA.uGear = 3;//Neutral;
//                        break;

//                    case 1:
//                    case 2:
//                    case 3:
//                    case 4:
//                    case 5:
//                    case 6:
//                        mVehInfCA.uGear = 4;//Drive;
//                        break;

//                    default:
//                        mVehInfCA.uGear = 0;
//                        break;
//                    }
//                    tFlagCA = true;
//                    break;

//                case 0x26D://EMS
//                    mVehInfCA.EMS_FAILED_Status = (uint8_t)( (packet.Data[3] >> 1) & 0x03);//EMS !
//                    tFlagCA = true;
//                    break;

//                case 0x183://actual steering angle
//                    mVehInfCA.steering_angle_valid = (uint8_t)( packet.Data[3] >> 7 ) & 0x01;
//                    //                if (!mVehInfCA.steering_angle_valid)
//                {
//                    mVehInfCA.ActualSteeringWheelAngle = (int16_t)(((int16_t)((packet.Data[3] << 8) | packet.Data[4])) * 0.1);
//                    mVehInfCA.SteeringWheelAgularVelocity = (uint16_t)(packet.Data[0] * 4);
//                    //                    mVehInfCA.steering_angle_count = (uint8_t)( packet.Data[6] & 0x0F);
//                    mVehInfCA.sas_failure_status = (uint8_t)( packet.Data[5] >> 6 ) & 0x01;
//                    tFlagCA = true;
//                }
//                    break;
//    }
}

void CanRevWorkThread::BoRuiVehicleParse(VCI_CAN_OBJ packet)
{
    switch(packet.ID)
    {
        case 0x2A0://eps status

            break;

//        case 0x130:
//        {
//            mVehInfCA.YRS_YawRateSensorState = (uint8_t)((packet.Data[1]) & 0x03);//0x0: Valid;  0x1: Invalid;  0x2: Reserved;  0x3: Initializing
//            mVehInfCA.YRS_YawRate  = ((uint16_t)( (packet.Data[4] << 8) | (packet.Data[5] ))) * 0.01 - 180.;
//            tFlagCA = true;
//        }break;


//        case 0x131:
//        {
//            mVehInfCA.YRS_LongitSensorState = (uint8_t)(((packet.Data[1])>>6) & 0x03);//0x0: Valid;  0x1: Invalid;  0x2: Reserved;  0x3: Initializing
//            mVehInfCA.YRS_LongitAcce  = ((uint16_t)( (packet.Data[2] << 8) | (packet.Data[3] ))) * 0.001 - 2.;
//            tFlagCA = true;
//        }break;

//        case 0x122:// wheel speed
//        {
//            mVehInfCA.FLWheelDirection = (uint8_t)((packet.Data[1]>>1) & 0x03);//1+ ; 2- ; 0STOP
//            mVehInfCA.FLWheelSpeedKPH  = ((uint16_t)( (packet.Data[0] << 5) | (packet.Data[1] >> 3))) * V_M_S;
//            mVehInfCA.FRWheelSpeedKPH   = ((uint16_t)( (packet.Data[2] << 5) | (packet.Data[3] >> 3))) * V_M_S;
//            tFlagCA = true;
//        }break;

//        // wheel speed
//        case 0x123:
//        {
//            mVehInfCA.wheel_speed_direction = (uint8_t)((packet.Data[1]>>1) & 0x03);//1+ ; 2- ; 0STOP
//            float WheelSpeedRearLeft  = ((uint16_t)( (packet.Data[0] << 5) | (packet.Data[1] >> 3))) * V_M_S;
//            float WheelSpeedRearRight   = ((uint16_t)( (packet.Data[2] << 5) | (packet.Data[3] >> 3))) * V_M_S;
//            mVehInfCA.RLWheelSpeedKPH = WheelSpeedRearLeft;
//            mVehInfCA.RRWheelSpeedKPH = WheelSpeedRearRight;
//            mVehInfCA.Speed = (WheelSpeedRearRight+WheelSpeedRearLeft)/2.0;
//            tFlagCA = true;
//        }break;

//        case 0x124://Wheel speed pulse
//        {
//            mVehInfCA.FrontLeft.data  = (uint16_t)(( (packet.Data[0] << 4) | (packet.Data[1] >> 4)) & 0x0fff);
//            mVehInfCA.FrontRight.data = (uint16_t)(( (packet.Data[1] << 8) |  packet.Data[2]      ) & 0x0fff);
//            mVehInfCA.RearLeft.data   = (uint16_t)(( (packet.Data[3] << 4) | (packet.Data[4] >> 4)) & 0x0fff);
//            mVehInfCA.RearRight.data  = (uint16_t)(( (packet.Data[4] << 8) |  packet.Data[5]      ) & 0x0fff);

//            mVehInfCA.wheel_speed_rear_right_pulse  = mVehInfCA.RearRight.data;
//            mVehInfCA.wheel_speed_rear_left_pulse   = mVehInfCA.RearLeft.data;
//            mVehInfCA.wheel_speed_front_right_pulse = mVehInfCA.FrontRight.data;
//            mVehInfCA.wheel_speed_front_left_pulse  = mVehInfCA.FrontLeft.data;
//            tFlagCA = true;
//        }break;

//        case 0x165:// TCU GEAR
//        {
//            mVehInfCA.uGear = (uint8_t)(packet.Data[1] & 0xf);
//            tFlagCA = true;
//        }break;

//        case 0x0E0://SAS
//        {
//            mVehInfCA.ActualSteeringWheelAngleF = 2 + ((int16_t)((packet.Data[0] << 8) | packet.Data[1])) * 0.1f;
//            mVehInfCA.SteeringAngleRateF = packet.Data[2]*4.0f;
//            mVehInfCA.ActualSteeringWheelAngle = mVehInfCA.ActualSteeringWheelAngleF;
//            mVehInfCA.SteeringWheelAgularVelocity = mVehInfCA.SteeringAngleRateF;
//            mVehInfCA.EPS_FAILED_Status = (uint8_t)( packet.Data[3] >> 7 ) & 0x01;
//            tFlagCA = true;
//        }break;

        default:
            break;
    }
}

void CanRevWorkThread::DongFengE70VehicleParse(VCI_CAN_OBJ packet)
{
//    case 0x122:
//        mVehInfCA.YRS_YawRate   = (((packet.Data[2] & 0x07) << 8 ) | packet.Data[3]) * 0.03 - 15.36;
//        mVehInfCA.YRS_YawRateSensorState = (uint8_t)((packet.Data[1]) & 0x80)>>7 ;//0x1: Valid;  0x0: Invalid;
//        mVehInfCA.YRS_LongitSensorState = (uint8_t)((packet.Data[1]) & 0x40)>>6 ;//0x1: Valid;  0x0: Invalid;
//        mVehInfCA.YRS_LongitAcce = (uint16_t)(((packet.Data[0] & 0x0f) << 8) | packet.Data[1] ) * 0.1 - 204.8;
//        tFlagCA = true;
//        break;

//    case 0x165:
//        mVehInfCA.EPS_FAILED_Status = (packet.Data[0] >> 6) & 0x03;
//        tFlagCA = true;
//        break;

//    case 0x279://GEAR  1:P  2:B  3:N  4:D
//        switch((uint8_t)(packet.Data[0]) & 0x0F)
//        {
//        case 5:
//            mVehInfCA.uGear = 1;//Parking;
//            break;

//        case 7:
//            mVehInfCA.uGear = 2;//Reverse;
//            break;

//        case 0:
//            mVehInfCA.uGear = 3;//Neutral;
//            break;

//        case 4:
//            mVehInfCA.uGear = 4;//Drive;
//            break;

//        default:
//            mVehInfCA.uGear = 0;
//            break;
//        }
//        tFlagCA = true;
//        break;

////            case 0x176://VCU 10 //GEAR  1:P  2:B  3:N  4:D
////                mVehInfCA.uGear = (uint8_t)( packet.Data[0] & 0x03);
////                break;

//    case 0xA3://ESC
//        if( 0 == ((packet.Data[0] >> 7) & 0x01))
//        {
//            mVehInfCA.FrontLeft.data  = ((uint16_t)(((packet.Data[0] & 0x7F) << 8) | packet.Data[1])) * V_M_S;
//        }
//        if( 0 == ((packet.Data[2] >> 7) & 0x01))
//        {
//            mVehInfCA.FrontRight.data  = ((uint16_t)(((packet.Data[2] & 0x7F) << 8) | packet.Data[3])) * V_M_S;
//        }
//        if( 0 == ((packet.Data[4] >> 7) & 0x01))
//        {
//            mVehInfCA.RearLeft.data  = ((uint16_t)(((packet.Data[4] & 0x7F) << 8) | packet.Data[5])) * V_M_S;
//        }
//        if( 0 == ((packet.Data[6] >> 7) & 0x01))
//        {
//            mVehInfCA.RearRight.data  = ((uint16_t)(((packet.Data[6] & 0x7F) << 8) | packet.Data[7])) * V_M_S;
//        }
//        tFlagCA = true;
//        break;

//    case 0xA6://ESC
//        if( 0 == ((packet.Data[5] >> 7) & 0x01))
//        {
//            mVehInfCA.wheel_speed_front_left_pulse    = (uint16_t)(((packet.Data[0] & 0xff) << 2) | ((packet.Data[1] >> 6) & 0x03));
//        }
//        if( 0 == ((packet.Data[5] >> 6) & 0x01))
//        {
//            mVehInfCA.wheel_speed_front_right_pulse   = (uint16_t)(((packet.Data[1] & 0x3f) << 4) | ((packet.Data[2] >> 4) & 0x0f));
//        }
//        if( 0 == ((packet.Data[5] >> 5) & 0x01))
//        {
//            mVehInfCA.wheel_speed_rear_left_pulse = (uint16_t)(((packet.Data[2] & 0x0f) << 6) | ((packet.Data[3] >> 2) & 0x3f));
//        }
//        if( 0 == ((packet.Data[5] >> 4) & 0x01))
//        {
//            mVehInfCA.wheel_speed_rear_right_pulse = (uint16_t)(((packet.Data[3] & 0x03) << 8) |   packet.Data[4]              );
//        }
//        if( 0 == ((packet.Data[5] >> 3) & 0x01))
//        {
//            mVehInfCA.long_acc = (uint16_t)(((packet.Data[5] & 0x07) << 8) | packet.Data[6]) * 0.03 - 15.36;
//        }
//        tFlagCA = true;
//        break;

//    case 0xA5:
//        mVehInfCA.ActualSteeringWheelAngle = (float)(((int16_t)((packet.Data[0] << 8) | packet.Data[1])) * 0.1);
//        mVehInfCA.SteeringAngleRateF = (uint16_t)(packet.Data[2] * 4);
//        tFlagCA = true;
//        //				SAS_Failure = (uint8_t)( data[3] >> 7 ) & 0x01;
//        break;

//    case 0x124://ESC7
//        if( 1 == ((packet.Data[3] >> 7) & 0x01))
//        {
//            mVehInfCA.wheel_speed_direction = (uint16_t)(packet.Data[2] & 0x3);//1+ ; 2- ; 0STOP
//            if (mVehInfCA.wheel_speed_direction == 3)
//                mVehInfCA.wheel_speed_direction = 0;
//            tFlagCA = true;
//        }



//        break;
}

/****************************************************************/

/****************************************************************/
void CanRevWorkThread::CAN1Parse(uint32_t num,VCI_CAN_OBJ* packet)
{
    uint32_t i;
    for(i=0;i<num;i++)
    {
        FrontViewParse(packet[i]);
    }
}


/******************************************************************/
void CanRevWorkThread::FrontViewParse(VCI_CAN_OBJ packet)
{
    switch(packet.ID)
    {
//        case 0x605:
//        {

//             tLaneInfo.uLaneWidth = 0xff & packet.Data[4];
//             tLaneInfo.flag_0x605 = true;

//             if ((tLaneInfo.flag_0x610 == true)&&(tLaneInfo.flag_0x611 == true)
//               &&(tLaneInfo.flag_0x612 == true)&&(tLaneInfo.flag_0x613 == true))
//             {
//                 emit SendLaneInfo(tLaneInfo);
//                 tLaneInfo.flag_0x605 = false;
//                 tLaneInfo.flag_0x610 = false;
//                 tLaneInfo.flag_0x611 = false;
//                 tLaneInfo.flag_0x612 = false;
//                 tLaneInfo.flag_0x613 = false;
//             }
//         }break;

//        case 0x610:
//        {
//            tLaneInfo.nLeftA0 = 0xffff & ((packet.Data[7] << 8) | packet.Data[6]);
//            tLaneInfo.nLeftA1 = 0xffff & ((packet.Data[5] << 8) | packet.Data[4]);
//            tLaneInfo.nLeftA2 = 0xffff & ((packet.Data[3] << 8) | packet.Data[2]);
//            tLaneInfo.flag_0x610 = true;

//            if ((tLaneInfo.flag_0x605 == true)&&(tLaneInfo.flag_0x611 == true)
//                    &&(tLaneInfo.flag_0x612 == true)&&(tLaneInfo.flag_0x613 == true))
//            {
//                emit SendLaneInfo(tLaneInfo);
//                tLaneInfo.flag_0x605 = false;
//                tLaneInfo.flag_0x610 = false;
//                tLaneInfo.flag_0x611 = false;
//                tLaneInfo.flag_0x612 = false;
//                tLaneInfo.flag_0x613 = false;
//            }
//        }
//        break;

//        case 0x611:
//        {
//            tLaneInfo.uLeftConfidence = 0x3 & ((packet.Data[1] & 0xc0)>>6);
//            tLaneInfo.uLeftRange = 0xfff & (((packet.Data[3] & 0xf)<<8) | packet.Data[2]);
//            tLaneInfo.nLeftTire2LaneDis = 0xfff & (((packet.Data[4] & 0x3f)<<4) | ((packet.Data[3] & 0xf0)>>4));
//            tLaneInfo.flag_0x611 = true;

//            if ((tLaneInfo.flag_0x605 == true)&&(tLaneInfo.flag_0x610 == true)
//                    &&(tLaneInfo.flag_0x612 == true)&&(tLaneInfo.flag_0x613 == true))
//            {
//                emit SendLaneInfo(tLaneInfo);
//                tLaneInfo.flag_0x605 = false;
//                tLaneInfo.flag_0x610 = false;
//                tLaneInfo.flag_0x611 = false;
//                tLaneInfo.flag_0x612 = false;
//                tLaneInfo.flag_0x613 = false;
//            }
//        }
//            break;

//        case 0x612:
//        {
//            tLaneInfo.nRightA0 = 0xffff & ((packet.Data[7] << 8) | packet.Data[6]);
//            tLaneInfo.nRightA1 = 0xffff & ((packet.Data[5] << 8) | packet.Data[4]);
//            tLaneInfo.nRightA2 = 0xffff & ((packet.Data[3] << 8) | packet.Data[2]);
//            tLaneInfo.flag_0x612 = true;

//            if ((tLaneInfo.flag_0x605 == true)&&(tLaneInfo.flag_0x610 == true)
//                    &&(tLaneInfo.flag_0x611 == true)&&(tLaneInfo.flag_0x613 == true))
//            {
//                emit SendLaneInfo(tLaneInfo);
//                tLaneInfo.flag_0x605 = false;
//                tLaneInfo.flag_0x610 = false;
//                tLaneInfo.flag_0x611 = false;
//                tLaneInfo.flag_0x612 = false;
//                tLaneInfo.flag_0x613 = false;
//            }
//        }
//            break;

//        case 0x613:
//        {
//            tLaneInfo.uRightConfidence = 0x3 & ((packet.Data[1] & 0xc0)>>6);
//            tLaneInfo.uRightRange = 0xfff & (((packet.Data[3] & 0xf)<<8) | packet.Data[2]);
//            tLaneInfo.nRightTire2LaneDis = 0xfff & (((packet.Data[4] & 0x3f)<<4) | ((packet.Data[3] & 0xf0)>>4));
//            tLaneInfo.flag_0x613 = true;

//            if ((tLaneInfo.flag_0x605 == true)&&(tLaneInfo.flag_0x610 == true)
//                    &&(tLaneInfo.flag_0x611 == true)&&(tLaneInfo.flag_0x612 == true))
//            {
//                emit SendLaneInfo(tLaneInfo);
//                tLaneInfo.flag_0x605 = false;
//                tLaneInfo.flag_0x610 = false;
//                tLaneInfo.flag_0x611 = false;
//                tLaneInfo.flag_0x612 = false;
//                tLaneInfo.flag_0x613 = false;
//            }
//        }
//            break;

//        case 0x650:
//        {
//            bool bFcwFalg;
//            unsigned int uLdwFalg;

//            bFcwFalg = 0x1 & ((packet.Data[2] & 0x2)>>1);
//            uLdwFalg = 0x3 & ((packet.Data[2] & 0x18)>>3);

//            emit SendWarningFlag(bFcwFalg, uLdwFalg);
//        }
//            break;

//        case 0x660:
//        case 0x664:
//        case 0x668:
//        case 0x66c:
//        case 0x670:
//        case 0x674:
//        case 0x678:
//        case 0x67c:
//        case 0x680:
//        case 0x684:
//        case 0x688:
//        case 0x68c:
//        case 0x690:
//        case 0x694:
//        case 0x698:
//        {
//            unsigned int index = (packet.ID - 0x660)/4;

//            tObs.uObstID = packet.Data[0] & 0x3f;
//            tObs.bCpivFlag = 0x1 & ((packet.Data[0] & 0x40)>>6);
//            tObs.uObstPosX = 0xffff & (((packet.Data[2] & 0xff) << 8) | (packet.Data[1] & 0xff));
//            tObs.nObstPosY = 0x3ff & (((packet.Data[4] & 0x3) << 8) | (packet.Data[3] & 0xff));
//            tObs.nObstVelX = 0xfff & (((packet.Data[5] & 0x3f) << 6) | (packet.Data[4] & 0xfc));
//            tObs.uflag[index]++;

//            if (tObs.uflag[index] == 2)
//            {
//                emit SendObst(index, tObs);
//                tObs.uflag[index] = 0;
//            }
//        }
//            break;

//        case 0x661:
//        case 0x665:
//        case 0x669:
//        case 0x66d:
//        case 0x671:
//        case 0x675:
//        case 0x679:
//        case 0x67d:
//        case 0x681:
//        case 0x685:
//        case 0x689:
//        case 0x68d:
//        case 0x691:
//        case 0x695:
//        case 0x699:
//        {
//            unsigned int index = (packet.ID - 0x661)/4;

//            tObs.uObstWidth = packet.Data[1] & 0xff;
//            tObs.uObstHeight = 0xff & packet.Data[2];
//            tObs.uObstStatus = 0x7 & ((packet.Data[4] & 0x38) >> 3) ;

//            tObs.uflag[index]++;

//            if (tObs.uflag[index] == 2)
//            {
//                emit SendObst(index, tObs);
//                tObs.uflag[index] = 0;
//            }
//        }
//            break;


    }

}

/******************************************************************/

void CanRevWorkThread::CAN2Parse(uint32_t num,VCI_CAN_OBJ* packet)
{
    uint32_t i;
    for(i=0;i<num;i++)
    {
        NXP_PlatformParse(packet[i]);
    }
}

void CanRevWorkThread::NXP_PlatformParse(VCI_CAN_OBJ packet)
{
    t_Terminal.Parse(packet,&t_Percaption);
    emit SendPercaptionMessage(&t_Percaption);
}
