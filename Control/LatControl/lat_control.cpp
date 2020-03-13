/*
 * lat_control.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#include "Control/LatControl/lat_control.h"

LatControl::LatControl() {
	// TODO Auto-generated constructor stub
    Init();
}

LatControl::~LatControl() {
	// TODO Auto-generated destructor stub
}

void LatControl::Init()
{
    last_cross_err = 0.0f;
    track_list = new TrackLinkList();
	_lat_control_status = init_status;
}

void LatControl::Proc(MessageManager *msg,VehicleController *ctl,PID *pid)
{
	if(ctl->SteeringEnable)
	{
		ctl->SteeringAngle = 0;
	}
}

float LatControl::TargetLine(float x)
{
	return cosf(COEFFICIENT_TLS*x)-1;
}

float LatControl::TargetLineFirstDerivative(float x)
{
	return -COEFFICIENT_TLS*sinf(COEFFICIENT_TLS*x);
}

float LatControl::TargetLineSecondDerivative(float x)
{
	return -COEFFICIENT_TLS*COEFFICIENT_TLS*cosf(COEFFICIENT_TLS*x);
}
/**
 * @brief LatControl::TrackingCurve 生成目标曲线
 * @param x：输入x轴数据
 * @return 返回目标跟踪路径数据集
 */
TargetTrack LatControl::TrackingCurve(float x)
{
    TargetTrack _temp_track;
    float _temp_cos_psi_r;

    _temp_track.point.setX(x);
    _temp_track.point.setY(TargetLine(x));
    _temp_track.yaw = atanf(TargetLineFirstDerivative(x)); // (-90,90)°
    _temp_cos_psi_r = cosf(_temp_track.yaw);
    _temp_track.curvature = _temp_cos_psi_r*_temp_cos_psi_r*_temp_cos_psi_r*TargetLineSecondDerivative(x);
    return _temp_track;
}

/**
 * @brief LatControl::GenerateCurvatureSets 产生曲线数据集
 * @param list：存储数据的链表
 */
void LatControl::GenerateCurvatureSets(TrackLinkList *list,uint16_t type)
{
    TargetTrack temp_track;
    float index_x,rho,gain;
    Node<TargetTrack>* last_track_node;
    Node<TargetTrack>* current_track_node;
    Node<TargetTrack>* next_track_node;
    float first_derivative_x,first_derivative_y;
    float second_derivative_x,second_derivative_y;
    /// 变量初始化
    list->Init();
    temp_track.point.setX(0.0f);
    temp_track.point.setY(0.0f);
    temp_track.yaw =0.0f;
    temp_track.curvature = 0.0f;
    index_x = 0.0f;

    switch(type)
    {
    case 1:
        // 三角函数曲线
        while(index_x < (M_PI/COEFFICIENT_TLS))
        {
            temp_track.point.setX(index_x);
            temp_track.point.setY(0.05f*TargetLine(index_x));
            list->Add(temp_track);
            index_x = index_x + 0.1f;
        }
        break;

    case 2:
        // 8形曲线
        index_x = -M_PI;
        while(index_x < M_PI)
        {
            rho = sqrtf(cosf(2*index_x));
            gain = rho*10;
            temp_track.point.setX(gain*cosf(index_x));
            temp_track.point.setY(gain*sinf(index_x));
            list->Add(temp_track);
            index_x = index_x + 0.01f;
        }
        break;

    case 3:
        // 8字圆
        // 后退
//        index_x = M_PI2;
//        while(index_x < M_PI)
//        {
//            gain = 5;
//            temp_track.point.setX(gain*cosf(index_x));
//            temp_track.point.setY(gain*sinf(index_x)-gain);
//            list->Add(temp_track);
//            index_x = index_x + 0.01f;
//        }
//        index_x = 0;
//        while(index_x > -(M_PI+M_PI2))
//        {
//            gain = 5;
//            temp_track.point.setX(gain*cosf(index_x)-2*gain);
//            temp_track.point.setY(gain*sinf(index_x)-gain);
//            list->Add(temp_track);
//            index_x = index_x - 0.01f;
//        }

        // 前进
        index_x = M_PI2;
        while(index_x > -M_PI)
        {
            gain = 5;
            temp_track.point.setX(gain*cosf(index_x));
            temp_track.point.setY(gain*sinf(index_x)-gain);
            list->Add(temp_track);
            index_x = index_x - 0.01f;
        }
        index_x = 0;
        while(index_x < (M_PI + M_PI2))
        {
            gain = 5;
            temp_track.point.setX(gain*cosf(index_x)-2*gain);
            temp_track.point.setY(gain*sinf(index_x)-gain);
            list->Add(temp_track);
            index_x = index_x + 0.01f;
        }
        break;

    default:
        break;
    }

    last_track_node = list->getHeadNode();
    current_track_node = last_track_node->next;
    next_track_node = current_track_node->next;
    do
    {
        first_derivative_x = next_track_node->data.point.getX() - current_track_node->data.point.getX();
        first_derivative_y = next_track_node->data.point.getY() - current_track_node->data.point.getY();
        second_derivative_x = next_track_node->data.point.getX() + last_track_node->data.point.getX() - 2*current_track_node->data.point.getX();
        second_derivative_y = next_track_node->data.point.getY() + last_track_node->data.point.getY() - 2*current_track_node->data.point.getY();
        current_track_node->data.yaw = atan2f(first_derivative_y,first_derivative_x);

        current_track_node->data.curvature = (second_derivative_y*first_derivative_x - second_derivative_x*first_derivative_y)
                                           / powf(powf(first_derivative_x,2)+powf(first_derivative_y,2),1.5);

        last_track_node = last_track_node->next;
        current_track_node = last_track_node->next;
        next_track_node = current_track_node->next;
    }while(next_track_node->next != NULL);
}

/**
 * @brief CalculateNearestPoint：计算最近的目标点
 * @param list：目标曲线数据集
 * @param a_track：当前车辆跟踪位置
 * @return 返回离车辆后轴最近的目标曲线点
 */
TargetTrack LatControl::CalculateNearestPoint(TrackLinkList *list,GeometricTrack *a_track)
{
    Node<TargetTrack>* track_index_node;
    TargetTrack index_node;
    float min_value;
    index_node.point.setX(0.0);
    index_node.point.setY(0.0);
    index_node.yaw = 0.0f;
    index_node.curvature = 0.0f;

    if(list->Length() < 1)
    {
        return index_node;
    }
    if(track_list->Length() == 0)
    {
        track_list->setHeadNode(list->getHeadNode());
    }
    else
    {
        track_list->setHeadNode(track_node);
    }
    track_index_node = track_list->getHeadNode();
    min_value = (track_index_node->data.point - a_track->getPosition()).Length();
    index_node = track_index_node->data;
    while(track_index_node->next != NULL)
    {
        if((track_index_node->data.point - a_track->getPosition()).Length() < min_value)
        {
            min_value = (track_index_node->data.point - a_track->getPosition()).Length();
            track_node = track_index_node;
            index_node = track_index_node->data;
        }
        track_index_node = track_index_node->next;
    }
    return index_node;
}

float LatControl::pi2pi(float angle)
{
    while(angle > M_PI)
    {
        angle = angle - 2*M_PI;
    }
    while(angle < -M_PI)
    {
        angle = angle + 2*M_PI;
    }
    return angle;
}

/**
 * @brief LatControl::SatFunction Sat函数
 * @param x：输入
 * @return 返回Sat数值
 */
float LatControl::SatFunction(float x)
{
	if(x > COEFFICIENT_DELTA)
	{
		return  1.0f;
	}
	else if(x < -COEFFICIENT_DELTA)
	{
		return -1.0f;
	}
	else
	{
		return x/COEFFICIENT_DELTA;
	}
}

/**
 * @brief LatControl::ProcV1_0 基于非时间参考的滑模控制算法
 * @param msg：车辆信号反馈，主要获取车辆速度信号
 * @param ctl：车辆控制信号量，控制转向角和角速度信号
 * @param a_track       :实际跟踪变量
 * @param a_track.point :实时跟踪坐标
 * @param a_track.yaw   :实时航向角@(-pi,pi)
 * @param t_track           :目标路径参数变量
 * @param t_track.point     :目标路径与车辆后轴最近的点
 * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
 * @param t_track.curvature :目标路径处的曲率 左正右负
 */
void LatControl::ProcV1_0(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
	float cos_psi_a,cos_psi_t;
	float tan_psi_a,tan_psi_t;
	float delta_t,rote_angle;
	float temp_a,temp_b;
	float delta_ctl;

	float yaw_a,yaw_t;
	Vector2d point_a,point_t;

	if((a_track->getYaw() >= -M_PI) && (a_track->getYaw() < -M_3PI4))
	{
		rote_angle = M_3PI4;
	}
	else if((a_track->getYaw() >= -M_3PI4) && (a_track->getYaw() < -M_PI2))
	{
		rote_angle = M_PI2;
	}
	else if((a_track->getYaw() >= -M_PI2) && (a_track->getYaw() < -M_PI4))
	{
		rote_angle = M_PI4;
	}
	else if((a_track->getYaw() >= -M_PI4) && (a_track->getYaw() < 0))
	{
		rote_angle = 0;
	}
	else if((a_track->getYaw() >= 0) && (a_track->getYaw() < M_PI4))
	{
		rote_angle = 0;
	}
	else if((a_track->getYaw() >= M_PI4) && (a_track->getYaw() < M_PI2))
	{
		rote_angle = -M_PI4;
	}
	else if((a_track->getYaw() >= M_PI2) && (a_track->getYaw() < M_3PI4))
	{
		rote_angle = -M_PI2;
	}
    else if((a_track->getYaw() >= M_3PI4) && (a_track->getYaw() <= M_PI))
	{
		rote_angle = -M_3PI4;
	}
	else
	{
        rote_angle = 0;
        APA_DEBUG("Warn:angle over the range!");
        return;
	}

	point_a = a_track->getPosition().rotate(rote_angle);
	point_t = t_track.point.rotate(rote_angle);

	yaw_a = a_track->getYaw() + rote_angle;
	yaw_t = t_track.yaw       + rote_angle;

	cos_psi_t = cosf(yaw_t);
	tan_psi_t = tanf(yaw_t);

	cos_psi_a = cosf(yaw_a);
	tan_psi_a = tanf(yaw_a);

	delta_t = atanf(WHEEL_BASE*t_track.curvature);

	_x1 = point_t.getY() - point_a.getY();

	if(Drive == msg->getGear())
	{
		_x2 = tan_psi_t - tan_psi_a;
	}
	else if(Reverse == msg->getGear())
	{
		_x2 = tan_psi_a - tan_psi_t;
	}
	else
	{

	}
	_sliding_variable = COEFFICIENT_SMV * _x1 + _x2;

	temp_a = WHEEL_BASE * cos_psi_a * cos_psi_a * cos_psi_a;
	temp_b = tanf(delta_t)/(WHEEL_BASE * cos_psi_t * cos_psi_t * cos_psi_t) +
			COEFFICIENT_SMV * _x2 +
			COEFFICIENT_RHO * SatFunction(_sliding_variable) +
			COEFFICIENT_K   * _sliding_variable;

	delta_ctl = atanf(temp_a * temp_b);

    delta_ctl = delta_ctl > 0.54f ? 0.54f : delta_ctl < -0.54f ? -0.54f:delta_ctl;
	ctl->SteeringAngle 		= delta_ctl * 16 * 57.3f;
	ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
}

/**
 * @brief LatControl::RearWheelFeedback 基于车辆后轴位置反馈的路径跟踪算法
 * @param msg：车辆信号反馈，主要获取车辆速度信号
 * @param ctl：车辆控制信号量，控制转向角和角速度信号
 * @param a_track       :实际跟踪变量
 * @param a_track.point :实时跟踪坐标
 * @param a_track.yaw   :实时航向角@(-pi,pi)
 * @param t_track           :目标路径参数变量
 * @param t_track.point     :目标路径与车辆后轴最近的点
 * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
 * @param t_track.curvature :目标路径处的曲率 左正右负
 */
void LatControl::RearWheelFeedback(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
    float err_yaw=0.0f,err_cro=0.0f;
	Vector2d vec_d,vec_t;
	float psi_omega;
    float v_x = 0.0f,k = 0.0f;
	float delta_ctl;

    vec_d = a_track->getPosition() - t_track.point;

    if(Drive == msg->getGear())
    {
        vec_t = Vector2d(cosf(t_track.yaw),sinf(t_track.yaw));
        err_yaw = pi2pi(a_track->getYaw() - t_track.yaw);
        v_x = msg->getVehicleMiddleSpeed();
        k =  t_track.curvature;
    }
    else if(Reverse == msg->getGear())
    {
        vec_t = Vector2d(cosf(t_track.yaw + M_PI),sinf(t_track.yaw + M_PI));
        err_yaw = pi2pi(a_track->getYaw() - t_track.yaw - M_PI);
        v_x = msg->getVehicleMiddleSpeed();
        k = -t_track.curvature;
    }
    else
    {
        vec_t = Vector2d(0.0f,0.0f);
        err_yaw = 0.0f;
        v_x = 0.0f;
    }
    err_cro = vec_t.CrossProduct(vec_d);

    psi_omega = v_x * k * cosf(err_yaw)/(1.0f - k * err_cro)
              - COEFFICIENT_KE   * v_x  * sinf(err_yaw) * err_cro / err_yaw
			  - COEFFICIENT_KPSI * fabs(v_x) * err_yaw ;

	if(fabs(psi_omega) < 1.0e-6f || fabs(err_yaw) < 1.0e-6f)
	{
		ctl->SteeringAngle 		= 0.0f;
		ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
	}
	else
	{
		delta_ctl = atanf(psi_omega * WHEEL_BASE / v_x);
        delta_ctl = delta_ctl > 0.54f ? 0.54f : delta_ctl < -0.54f ? -0.54f:delta_ctl;
		ctl->SteeringAngle 		= delta_ctl * 16 * 57.3f;
		ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
	}
}


/**
 * @brief LatControl::Work 横向控制状态切换控制
 * @param msg：车辆信息->挡位、车速
 * @param ctl：车辆控制->转向角、转向角速度、车速、挡位
 * @param a_track：当前跟踪位置信息
 * @param t_track：目标曲线信息
 * @param last_track：车辆终点信息
 */
void LatControl::Work(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track,TargetTrack last_track)
{
    float nerest_distance;
    float cross_err;
	switch(_lat_control_status)
	{
		case init_status:
			if(ctl->getAPAEnable() && (msg->getSteeringAngle() < 5.0f) && (msg->getSteeringAngle() > -5.0f))
			{
                a_track->Init(); // 跟踪位置初始化，从坐标零点开始控制
				_lat_control_status = process_status;
			}
			else
			{
				ctl->SteeringAngle 		= 0;
				ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
			}
			break;

		case process_status:
			if(ctl->getAPAEnable())
			{
//                ProcV1_0(msg,ctl,a_track,t_track);
                RearWheelFeedback(msg,ctl,a_track,t_track);
                nerest_distance = (a_track->getPosition() - last_track.point).Length();
                cross_err = last_track.point.CrossProduct(a_track->getPosition());
                if( nerest_distance < 0.1f)
                {
                    ctl->setDistance(0);
                    ctl->setVelocity(0.0);
                    ctl->setAPAEnable(0);
                    ctl->setGear(Parking);
                    _lat_control_status = init_status;
                }
                else
                {

                }
                last_cross_err = cross_err;
			}
			else
			{
				_lat_control_status = init_status;
			}
			break;

		default:
            a_track->Init(); // 跟踪位置初始化，从坐标零点开始控制
            ctl->SteeringAngle 		= 0;
            ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
            _lat_control_status = init_status;
			break;
	}
}

TargetTrack LatControl::getTargetTrack()                  { return  _target_track;}
void        LatControl::setTargetTrack(TargetTrack value) { _target_track = value;}

float LatControl::getX1()            { return  _x1;}
void  LatControl::setX1(float value) { _x1 = value;}

float LatControl::getX2()            { return  _x2;}
void  LatControl::setX2(float value) { _x2 = value;}

float LatControl::getSlidingVariable()            { return  _sliding_variable;}
void  LatControl::setSlidingVariable(float value) { _sliding_variable = value;}
