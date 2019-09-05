/*
 * ultrasonic_abstacle_percption.cpp
 *
 *  Created on: 2019年1月29日
 *      Author: zhuguohua
 */

#include "ultrasonic_obstacle_percption.h"

UltrasonicObstaclePercption::UltrasonicObstaclePercption() {
	UltrasonicLocationStatus.setContainer(this);
	UltrasonicLocationStatus.getter(&UltrasonicObstaclePercption::getUltrasonicLocationStatus);
	UltrasonicLocationStatus.setter(&UltrasonicObstaclePercption::setUltrasonicLocationStatus);
	Init();
}
UltrasonicObstaclePercption::~UltrasonicObstaclePercption() {
}

void UltrasonicObstaclePercption::Init()
{
	_ultrasonic_location_sts = LocationReady;

	_data_push_state = WaitPushStart;
	_parking_calculate_state = WaitCommandForCalculate;

	_edge_finding_state = ObstacleWaitEdge;

	_parking_position.First_Position  = Vector2d(0,0);
	_parking_position.Second_Position = Vector2d(0,0);
	_parking_position.Length = 0.0f;

	_vehicle_position.First_Position  = Vector2d(0,0);
	_vehicle_position.Second_Position = Vector2d(0,0);
	_vehicle_position.Length = 0.0f;
	// 最终输出的库位信息
	_valid_parking_edge_position.First_Position  = Vector2d(0,0);
	_valid_parking_edge_position.Second_Position = Vector2d(0,0);
	_valid_parking_edge_position.Length = 0.0f;
	// 进库后的库位中心调整
	_valid_parking_center_position.position = Vector2d(0,0);
	_valid_parking_center_position.angle    = 0;
	_push_cnt = 0;
	/******************************************/
	_ultrasonic_position_list          = new LinkList;
	_ultrasonic_triangle_location_list = new LinkList;

	_left_edge_position_list  = new LinkList;
	_right_edge_position_list = new LinkList;

	_left_fit_edge_list   = new LinkList;
	_right_fit_edge_list  = new LinkList;

	_front_obstacle_process_state = WaitObstacleState;
	_rear_obstacle_process_state = WaitObstacleState;

	front_obstacle_cnt = 0;
	rear_obstacle_cnt = 0;
}

void UltrasonicObstaclePercption::Push(LinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	// 时间序列的数据
	if(list->HeadNode == NULL)
	{
		if (u_dat.Distance2 > u_dat.Distance1)
		{
			if( (u_dat.Distance2 - u_dat.Distance1) < DISTANCE_ERR_THRESHOLD )
			{
				list->Add(p_dat);
			}
		}
		else
		{
			if((u_dat.Level > LEVEL_THRESHOLD) && (u_dat.Width > WIDTH_THRESHOLD))
			{
				list->Add(p_dat);
			}
		}
	}
	else
	{
		if(p_dat.Position.getX() < list->getEndNode()->data.Position.getX())
		{
			if (u_dat.Distance2 > u_dat.Distance1)
			{
				if( (u_dat.Distance2 - u_dat.Distance1) < DISTANCE_ERR_THRESHOLD )
				{
					list->Add(p_dat);
				}
			}
			else
			{
				if((u_dat.Level > LEVEL_THRESHOLD) && (u_dat.Width > WIDTH_THRESHOLD))
				{
					list->Add(p_dat);
				}
			}
		}
	}
}

void UltrasonicObstaclePercption::OrderedPush(LinkList *list,ObstacleLocationPacket p_dat)
{
	// 时间序列的数据
	if(list->HeadNode == NULL)
	{
		if(p_dat.Status == 0)
		{
			list->Add(p_dat);
		}
	}
	else
	{
		if( (p_dat.Status == 0) && (p_dat.Position.getX() < list->getEndNode()->data.Position.getX()))
		{
			list->Add(p_dat);
		}
	}
}

// 无序列的数据
void UltrasonicObstaclePercption::DisorderPush(LinkList *list,ObstacleLocationPacket p_dat)
{
	// 无序数据
	if(list->getHeadNode() == NULL)
	{
		if(p_dat.Status == 0)
		{
			list->Add(p_dat);
		}
	}
	else
	{
		if( (p_dat.Status == 0) && (p_dat.Position.LengthSquare() != list->getEndNode()->data.Position.LengthSquare()) )
		{
			list->Add(p_dat);
		}
	}
}

void UltrasonicObstaclePercption::ParkingCenterPush(LinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	ObstacleLocationPacket temp_packet;
	// 有序数据集
	if(list->getEndNode() == NULL)
	{
		if( u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD )
		{
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
			list->Add(p_dat);
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
			temp_packet.Position = Vector2d(p_dat.Position.getY(),-p_dat.Position.getX());
			temp_packet.Status   = p_dat.Status;
			list->Add(temp_packet);
#endif
		}
	}
	else
	{
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
		if((u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD) && (p_dat.Position.getX() < list->getEndNode()->data.Position.getX()))
		{
			list->Add(p_dat);
		}
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
		if((u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD) && (p_dat.Position.getY() < list->getEndNode()->data.Position.getY()))
		{
			temp_packet.Position = Vector2d(p_dat.Position.getY(),-p_dat.Position.getX());
			temp_packet.Status   = p_dat.Status;
			list->Add(temp_packet);
		}
#endif
	}
}
/******************************************************************************************************************************/
float UltrasonicObstaclePercption::WeightCalculation(float d)
{
	if(d < 0.5)
	{
		return 4*(d - 0.5)*(d - 0.5);
	}
	else
	{
		return 0;
	}
}

void UltrasonicObstaclePercption::DataCredibilityCalculate(LinkList *list)
{
//	float weigth_array_buffer[4];
//	uint8_t i;
//	uint16_t index_cnt;
//	Node* _current_node;//当前节点
//
//	if(list->HeadNode == NULL)
//	{
//		return;
//	}
//
//	for(i=0;i<4;i++){weigth_array_buffer[i] = 0;}
//	_current_node = list->HeadNode;//当前节点
//	float *weight_array = new float[list->Length()];
//	index_cnt = 0;
//	while(_current_node->next != NULL)
//	{
//		if(_current_node)
//	}

}
/******************************************************************************************************************************/
void UltrasonicObstaclePercption::EdgeFinding(LinkList *list)
{
	Node* _current_node;//当前节点
	Node* _last_node;//上一节点
	ObstacleInformationPacket parking_position_temp;
	ObstacleInformationPacket vehicle_position_temp;
	float max_y_axis_value;
	float _err_distance;

	if(list->HeadNode == NULL)
	{
		return;
	}
	_current_node = list->HeadNode;//当前节点
	_last_node    = list->HeadNode;//上一节点

	while(_current_node->next != NULL)
	{
		_err_distance = (_current_node->data.Position - _last_node->data.Position).Length();
		switch(_edge_finding_state)
		{
			case ObstacleWaitEdge:
				if(_err_distance < DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.First_Position = _last_node->data.Position;
					max_y_axis_value = _current_node->data.Position.getY();
					_edge_finding_state = VehicleEdgeWaitstate;
				}
				break;

			case VehicleEdgeWaitstate:
				if(_err_distance > DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.Second_Position = _last_node->data.Position;
					parking_position_temp.First_Position  = _current_node->data.Position;

					_vehicle_position.Length          = fabs(vehicle_position_temp.First_Position.getX() - vehicle_position_temp.Second_Position.getX());
					_vehicle_position.First_Position  = vehicle_position_temp.First_Position;
					_vehicle_position.Second_Position = vehicle_position_temp.Second_Position;
					_edge_finding_state = WaitEnterDenseArea;
				}
				else
				{
					max_y_axis_value = _current_node->data.Position.getY() > max_y_axis_value ? _current_node->data.Position.getY() : max_y_axis_value;
				}
				break;

			case WaitEnterDenseArea:
				if( (_err_distance < DISTANCE_THRESHOLD) && (_err_distance != 0 ))
				{
					parking_position_temp.Second_Position = _last_node->data.Position;
					vehicle_position_temp.First_Position  = _last_node->data.Position;
					_edge_finding_state = JudgeParkingValid;
				}
				break;

			case JudgeParkingValid:
				if(_err_distance < DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.Second_Position = _current_node->data.Position;
					if((vehicle_position_temp.First_Position - vehicle_position_temp.Second_Position).Length() > DISTANCE_THRESHOLD)
					{
						_parking_position.Length          = fabs(parking_position_temp.First_Position.getX() - parking_position_temp.Second_Position.getX());
						_parking_position.First_Position  = parking_position_temp.First_Position;
						_parking_position.Second_Position = parking_position_temp.Second_Position;
						_edge_finding_state = VehicleEdgeWaitstate;
					}
				}
				else//突然出现稀疏点
				{
					_edge_finding_state = WaitEnterDenseArea;
				}
				break;

			default:

				break;
		}
		_last_node = _current_node;
		_current_node = _current_node->next;
	}
	if(VehicleEdgeWaitstate == _edge_finding_state)
	{
		_vehicle_position.First_Position  = vehicle_position_temp.First_Position;
		_vehicle_position.Second_Position = _current_node->data.Position;
		_vehicle_position.Length          = fabs(_vehicle_position.First_Position.getX() - _vehicle_position.Second_Position.getX());
	}
	_valid_parking_edge_position.First_Position.setX( _vehicle_position.Second_Position.getX() );
	_valid_parking_edge_position.First_Position.setY( max_y_axis_value );
	list->Delete();
}

void UltrasonicObstaclePercption::EdgeFinding_V1_0(LinkList *list)
{
	Node* _current_node;//当前节点
	Node* _last_node;//上一节点
	ObstacleInformationPacket vehicle_position_temp;
	float max_y_axis_value;
	float _err_distance;

	if(list->HeadNode == NULL)
	{
		return;
	}

	_last_node    = list->HeadNode;//上一节点
	_current_node = _last_node->next;//当前节点

	while(_current_node->next != NULL)
	{
		_err_distance = (_current_node->data.Position - _last_node->data.Position).Length();
		switch(_edge_finding_state_v1_0)
		{
			case EdgeJudge:
				if(_err_distance > DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.First_Position = _current_node->data.Position;
				}
				else
				{
					vehicle_position_temp.First_Position = _last_node->data.Position;
				}
				max_y_axis_value = vehicle_position_temp.First_Position.getY();
				_edge_finding_state_v1_0 = EdgeLooking;
				break;

			case EdgeLooking:
				if(_err_distance > DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.Second_Position = _last_node->data.Position;

					_valid_parking_edge_position.First_Position.setX( vehicle_position_temp.Second_Position.getX() );
					_valid_parking_edge_position.First_Position.setY( max_y_axis_value );

					vehicle_position_temp.First_Position = _current_node->data.Position;
					max_y_axis_value = vehicle_position_temp.First_Position.getY();
				}
				else
				{
					vehicle_position_temp.Second_Position = _current_node->data.Position;
					max_y_axis_value = _current_node->data.Position.getY() > max_y_axis_value ? _current_node->data.Position.getY() : max_y_axis_value;
				}
				break;

			default:
				break;
		}
		if(_err_distance < DISTANCE_THRESHOLD)
		{
			_valid_parking_edge_position.First_Position.setX( vehicle_position_temp.Second_Position.getX() );
			_valid_parking_edge_position.First_Position.setY( max_y_axis_value );
		}
		_last_node = _current_node;
		_current_node = _current_node->next;
	}
}

float UltrasonicObstaclePercption::HighestDistribution(uint8_t group_number,uint16_t* group_value_array,float min_value)
{
	uint8_t i;
	uint8_t max_distribute_number_id;
	uint16_t max_distribute_number_value;

	uint16_t sum_value;
	float master_ratio,slave_ratio;

	for(i = 0; i < group_number; i++)
	{
		if(i == 0)
		{
			max_distribute_number_id = i;
			max_distribute_number_value = group_value_array[i];
		}
		else
		{
			if(group_value_array[i] > max_distribute_number_value)
			{
				max_distribute_number_value = group_value_array[i];
				max_distribute_number_id    = i;
			}
		}
	}
	//
	if((0 == max_distribute_number_id) && (group_number > 1))
	{
		if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id + 1])
		{
			sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id + 1];
			master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
			slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id + 1 ] / sum_value;
			return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id + 1.5f) * slave_ratio);
		}
		else
		{
			return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
		}
	}
	else if( (group_number - 1) == max_distribute_number_id )
	{
		if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id - 1])
		{
			sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id - 1];
			master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
			slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id - 1 ] / sum_value;
			return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id - 0.5f) * slave_ratio);
		}
		else
		{
			return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
		}
	}
	else
	{
		if(group_value_array[max_distribute_number_id - 1] > group_value_array[max_distribute_number_id + 1])
		{
			if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id - 1])
			{
				sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id - 1];
				master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
				slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id - 1 ] / sum_value;
				return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id - 0.5f) * slave_ratio);
			}
			else
			{
				return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
			}
		}
		else
		{
			if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id + 1])
			{
				sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id + 1];
				master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
				slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id + 1 ] / sum_value;
				return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id + 1.5f) * slave_ratio);
			}
			else
			{
				return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
			}
		}
	}
}

uint8_t UltrasonicObstaclePercption::HighestDistributionBase(uint8_t group_number,uint16_t* group_value_array)
{
	uint8_t i;
	uint8_t max_distribute_number_id;
	uint16_t max_distribute_number_value;

	for(i = 0; i < group_number; i++)
	{
		if(i == 0)
		{
			max_distribute_number_id = i;
			max_distribute_number_value = group_value_array[i];
		}
		else
		{
			if(group_value_array[i] > max_distribute_number_value)
			{
				max_distribute_number_value = group_value_array[i];
				max_distribute_number_id    = i;
			}
		}
	}
	return max_distribute_number_id;
}

void  UltrasonicObstaclePercption::ValueDistributed(LinkList *valid_list)
{
	uint8_t i;
	float min_x,max_x;
	float min_y,max_y;
	uint8_t  x_group_number,y_group_number;
	Node* _current_node_triangle;//当前节点零时变量

	if(valid_list->HeadNode == NULL)
	{
		return;
	}
	_current_node_triangle = valid_list->HeadNode;
	min_x = _current_node_triangle->data.Position.getX();
	max_x = _current_node_triangle->data.Position.getX();
	min_y = _current_node_triangle->data.Position.getY();
	max_y = _current_node_triangle->data.Position.getY();

	while(_current_node_triangle->next != NULL)
	{
		min_x = _current_node_triangle->data.Position.getX() < min_x ? _current_node_triangle->data.Position.getX() : min_x;
		max_x = _current_node_triangle->data.Position.getX() > max_x ? _current_node_triangle->data.Position.getX() : max_x;
		min_y = _current_node_triangle->data.Position.getY() < min_y ? _current_node_triangle->data.Position.getY() : min_y;
		max_y = _current_node_triangle->data.Position.getY() > max_y ? _current_node_triangle->data.Position.getY() : max_y;

		_current_node_triangle = _current_node_triangle->next;
	}

	x_group_number = (uint8_t)((max_x - min_x)/STEP_DISTANCE) + 1;
	y_group_number = (uint8_t)((max_y - min_y)/STEP_DISTANCE) + 1;

	_current_node_triangle = valid_list->HeadNode;

	uint16_t *distribute_number_x = new uint16_t[x_group_number];
	uint16_t *distribute_number_y = new uint16_t[y_group_number];

	for(i = 0;i<x_group_number;i++)
	{
		distribute_number_x[i] = 0;
	}
	for(i = 0;i<y_group_number;i++)
	{
		distribute_number_y[i] = 0;
	}

	while(_current_node_triangle->next != NULL)
	{
		for(i = 0;i<x_group_number;i++)
		{
			if( (_current_node_triangle->data.Position.getX() >= (min_x + STEP_DISTANCE*i)) && (_current_node_triangle->data.Position.getX() < (min_x + STEP_DISTANCE*(i + 1))))
			{
				distribute_number_x[i]++;
				break;
			}
		}
		for(i = 0;i<y_group_number;i++)
		{
			if( (_current_node_triangle->data.Position.getY() >= (min_y + STEP_DISTANCE*i)) && (_current_node_triangle->data.Position.getY() < (min_y + STEP_DISTANCE*(i + 1))))
			{
				distribute_number_y[i]++;
				break;
			}
		}
		_current_node_triangle = _current_node_triangle->next;
	}

	_valid_parking_edge_position.Second_Position.setX(HighestDistribution(x_group_number,distribute_number_x,min_x));
	_valid_parking_edge_position.Second_Position.setY(HighestDistribution(y_group_number,distribute_number_y,min_y));

	delete []distribute_number_x;
	delete []distribute_number_y;
	valid_list->Delete();
}

void UltrasonicObstaclePercption::ValueDistributedFilter(LinkList *valid_list,LinkList *fit_list)
{
	uint8_t i,max_distribute_number_id;
//	uint8_t x_group_number;
	uint8_t y_group_number;
//	float min_x,max_x;
	float min_y,max_y;
	float treshold_down,treshold_up;
	Node* current_node;

	if(valid_list->HeadNode == NULL){return;}
	current_node = valid_list->HeadNode;
//	min_x = current_node->data.Position.getX();
//	max_x = current_node->data.Position.getX();
	min_y = current_node->data.Position.getY();
	max_y = current_node->data.Position.getY();

	while(current_node->next != NULL)
	{
//		min_x = current_node->data.Position.getX() < min_x ? current_node->data.Position.getX() : min_x;
//		max_x = current_node->data.Position.getX() > max_x ? current_node->data.Position.getX() : max_x;
		min_y = current_node->data.Position.getY() < min_y ? current_node->data.Position.getY() : min_y;
		max_y = current_node->data.Position.getY() > max_y ? current_node->data.Position.getY() : max_y;
		current_node = current_node->next;
	}
//	x_group_number = (uint8_t)((max_x - min_x)/FIT_LINE_STEP_DISTANCE) + 1;
	y_group_number = (uint8_t)((max_y - min_y)/FIT_LINE_STEP_DISTANCE) + 1;

	current_node = valid_list->HeadNode;
	// create the array
//	uint16_t *distribute_number_x = new uint16_t[x_group_number];
	uint16_t *distribute_number_y = new uint16_t[y_group_number];
	// initialize the array
//	for(i = 0;i<x_group_number;i++)
//	{
//		distribute_number_x[i] = 0;
//	}
	for(i = 0;i<y_group_number;i++)
	{
		distribute_number_y[i] = 0;
	}
	// distribute calculate
	while(current_node->next != NULL)
	{
//		for(i = 0;i<x_group_number;i++)
//		{
//			if( (current_node->data.Position.getX() >= (min_x + FIT_LINE_STEP_DISTANCE*i)) && (current_node->data.Position.getX() < (min_x + FIT_LINE_STEP_DISTANCE*(i + 1))))
//			{
//				distribute_number_x[i]++;
//				break;
//			}
//		}
		for(i = 0;i<y_group_number;i++)
		{
			if( (current_node->data.Position.getY() >= (min_y + FIT_LINE_STEP_DISTANCE*i)) && (current_node->data.Position.getY() < (min_y + FIT_LINE_STEP_DISTANCE*(i + 1))))
			{
				distribute_number_y[i]++;
				break;
			}
		}
		current_node = current_node->next;
	}
//#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
	max_distribute_number_id = HighestDistributionBase(y_group_number,distribute_number_y);
	treshold_down = min_y + FIT_LINE_STEP_DISTANCE * max_distribute_number_id     ;
	treshold_up   = min_y + FIT_LINE_STEP_DISTANCE *(max_distribute_number_id + 1);
//#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
//	max_distribute_number_id = HighestDistributionBase(x_group_number,distribute_number_x);
//	treshold_down = min_x + FIT_LINE_STEP_DISTANCE * max_distribute_number_id     ;
//	treshold_up   = min_x + FIT_LINE_STEP_DISTANCE *(max_distribute_number_id + 1);
//#endif

	current_node = valid_list->HeadNode;
	if(fit_list->Length() != 0)
	{
		fit_list->Delete();
	}
	while(current_node->next != NULL)
	{
//#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
		if((current_node->data.Position.getY() >= treshold_down) && (current_node->data.Position.getY() < treshold_up))
//#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
//		if((current_node->data.Position.getX() >= treshold_down) && (current_node->data.Position.getX() < treshold_up))
//#endif
		{
			fit_list->Add(current_node->data);
		}
		current_node = current_node->next;
	}
//	delete []distribute_number_x;
	delete []distribute_number_y;
	valid_list->Delete();
}

// 车辆进库，通过库位边界，测定库位的中心位置，准确的前提是车辆基本垂直进入
void UltrasonicObstaclePercption::EdgeLineFitParkingCenterCalculate()
{
	float left,right;
	if(_left_fit_edge_list->Length() != 0){_left_fit_edge_list->Delete();}
	if(_right_fit_edge_list->Length() != 0){_right_fit_edge_list->Delete();}

	ValueDistributedFilter(_left_edge_position_list,_left_fit_edge_list);
	ValueDistributedFilter(_right_edge_position_list,_right_fit_edge_list);

	_line_fit.LineFitting(_left_fit_edge_list,&_left_fit_line_packet.angle,&_left_fit_line_packet.offset);
	_line_fit.LineFitting(_right_fit_edge_list,&_right_fit_line_packet.angle,&_right_fit_line_packet.offset);

	_center_fit_line_packet.angle  = 0.5 * (_left_fit_line_packet.angle + _right_fit_line_packet.angle);
	_center_fit_line_packet.offset = 0.5 * (_left_fit_line_packet.offset + _right_fit_line_packet.offset);

	left = _left_fit_edge_list->getHeadNode()->data.Position.getX();
	right = _right_fit_edge_list->getHeadNode()->data.Position.getX();
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
	_valid_parking_center_position.angle        = _center_fit_line_packet.angle;
	_valid_parking_center_position.position.setX((left > right) ? left : right);
	_valid_parking_center_position.position.setY(tanf(_center_fit_line_packet.angle) * _valid_parking_center_position.position.getX() + _center_fit_line_packet.offset);
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
	_valid_parking_center_position.angle        = _center_fit_line_packet.angle + PI_2;
	_valid_parking_center_position.position.setY((left > right) ? left : right);
	_valid_parking_center_position.position.setX(-tanf(_center_fit_line_packet.angle) * _valid_parking_center_position.position.getX() - _center_fit_line_packet.offset);
#endif
	_left_fit_edge_list->Delete();
	_right_fit_edge_list->Delete();
}
/*
 * 运行于定时器中，5ms运行一次
 * */
void  UltrasonicObstaclePercption::DataPushStateMachine(Ultrasonic* u_dat)
{
	switch(_data_push_state)
	{
		case WaitPushStart:
			if(0x50 == Command)
			{
				if(0 != _ultrasonic_position_list->Length()){_ultrasonic_position_list->Delete();}
				if(0 != _ultrasonic_triangle_location_list->Length()){_ultrasonic_triangle_location_list->Delete();}
				_ultrasonic_location_sts = LocationDataPush;
				_data_push_state = ParkingEdgeUltrasonicDataPush;
			}
			else if(0x55 == Command)
			{
				if(0 != _left_edge_position_list->Length()){_left_edge_position_list->Delete();}
				if(0 != _right_edge_position_list->Length()){_right_edge_position_list->Delete();}
				_ultrasonic_location_sts = LocationDataPush;
				_data_push_state = ParkingCenterUltrasonicDataPush;
			}
			else{}
			break;

		case ParkingEdgeUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
//			if( (0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
//			{
				Push(_ultrasonic_position_list,u_dat->UltrasonicPacket[11], u_dat->AbstacleGroundPositionTriangle[11]);
//			}
//			if(23 == u_dat->ScheduleTimeCnt)
//			{
				DisorderPush(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[5]);
				DisorderPush(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[6]);
//			}
#elif RUNNING_PLATFORM == PC_PLATFORM
			_push_cnt = (_push_cnt + 1)%3;
			Push(_ultrasonic_position_list,u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
			if(0 == _push_cnt)
			{
				Push(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[5]);
				Push(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[6]);
			}
#endif
			if(getLocationListLength() > MIN_LOCATION_NUM)
			{
				Command = 0x70;//库位计算命令
				_data_push_state = WaitPushStart;
			}
			break;

		case ParkingCenterUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
//			if((26 == u_dat->ScheduleTimeCnt) || (12 == u_dat->ScheduleTimeCnt))
//			{
				ParkingCenterPush(_left_edge_position_list,u_dat->UltrasonicPacket[10],u_dat->AbstacleGroundPositionTriangle[10]);
//			}
//			if((0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
//			{
				ParkingCenterPush(_right_edge_position_list,u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
//			}
#elif RUNNING_PLATFORM == PC_PLATFORM
			ParkingCenterPush(_left_edge_position_list,u_dat->UltrasonicPacket[10],u_dat->AbstacleGroundPositionTriangle[10]);
			ParkingCenterPush(_right_edge_position_list,u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
#endif
			if((_left_edge_position_list->EndNode != NULL) && (_right_edge_position_list->EndNode != NULL))
			{
				if(((_left_edge_position_list->getHeadNode()->data.Position - _left_edge_position_list->getEndNode()->data.Position).Length() > MIN_FIT_DISTANCE)  &&
				   ((_right_edge_position_list->getHeadNode()->data.Position - _right_edge_position_list->getEndNode()->data.Position).Length() > MIN_FIT_DISTANCE) &&
				   (getLeftEdgeListLength() > MIN_FIT_NUM) &&
				   (getRightEdgeListLength() > MIN_FIT_NUM))
				{
					Command = 0x75;
					_data_push_state = WaitPushStart;
				}
			}
			break;
		default:

			break;
	}
}

int8_t UltrasonicObstaclePercption::ParkingCalculateStateMachine(void)
{
	switch(_parking_calculate_state)
	{
		case WaitCommandForCalculate:
			if(0x70 == Command)
			{
				_ultrasonic_location_sts = LocationCalculate;
				_parking_calculate_state = FrontEdgeCalculate;
				_edge_finding_state      = ObstacleWaitEdge;
			}
			else if(0x75 == Command)
			{
				_ultrasonic_location_sts = LocationCalculate;
				_parking_calculate_state = ParkingCenterCalculate;
			}
			else
			{

			}
			break;

		case FrontEdgeCalculate:
//			EdgeFinding(_ultrasonic_position_list);
			EdgeFinding_V1_0(_ultrasonic_position_list);
			_parking_calculate_state = RearEdgeCalculate;
			break;

		case RearEdgeCalculate:
			ValueDistributed(_ultrasonic_triangle_location_list);
			Command = 0;
			_ultrasonic_location_sts = LocationFinish;
			_parking_calculate_state = WaitCommandForCalculate;
			return SUCCESS;
			break;

		case ParkingCenterCalculate:
			EdgeLineFitParkingCenterCalculate();
			Command = 0;
			_ultrasonic_location_sts = LocationFinish;
			_parking_calculate_state = WaitCommandForCalculate;
			return SUCCESS;
			break;

		default:
			break;
	}
	return FAIL;
}
/***********************************************************************************************/
/************************************* 超声波避障功能  ***********************************************/
/***********************************************************************************************/
int8_t UltrasonicObstaclePercption::UltrasonicCollisionStatus(Ultrasonic *u,MessageManager *msg)
{
	uint8_t i;
	if(Reverse == msg->Gear)
	{
		for(i = 0; i < 4; i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16) )
				{
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else if(Drive == msg->Gear)
	{
		for(i = 4;i < 8;i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16))
				{
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else
	{
		return FAIL;
	}
}

void UltrasonicObstaclePercption::UltrasonicCollisionDiatance(Ultrasonic *u,MessageManager *msg)
{
	uint8_t i;
	float distance,current_distance;
//	float direct_distance,direct_current_distance;
	uint8_t over_detection_cnt;
	Vector2d LF_P,RF_P,LR_P,RR_P;

	if((Drive == msg->Gear) || (Neutral == msg->Gear))
	{
		distance = 6;
		over_detection_cnt = 0;
		LF_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[1].Point.getX(), LEFT_EDGE_TO_CENTER);
		RF_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[1].Point.getX(),-RIGHT_EDGE_TO_CENTER);
		for(i = 0; i < 4; i++)
		{
			if(Normal == u->AbstacleBodyPositionTriangle[i].Status)
			{
				if(u->AbstacleBodyPositionTriangle[i].Position.getY() > 0.93)
				{
					current_distance = (u->AbstacleBodyPositionTriangle[i].Position - LF_P).Length();
				}
				else if(u->AbstacleBodyPositionTriangle[i].Position.getY() < -0.93)
				{
					current_distance = (u->AbstacleBodyPositionTriangle[i].Position - RF_P).Length();
				}
				else
				{
					current_distance = fabs(u->AbstacleBodyPositionTriangle[i].Position.getX() - _ultrasonic_obstacle_config.UltrasonicLocationArray[1].Point.getX());
				}
				distance = current_distance < distance ? current_distance : distance;
				_obstacle_distance.status = Normal;
			}
			else if(BlindZone == u->AbstacleBodyPositionTriangle[i].Status)
			{
				distance = 0;
				_obstacle_distance.status = BlindZone;
				break;
			}
			else if(OverDetection == u->AbstacleBodyPositionTriangle[i].Status)
			{
				over_detection_cnt++;
			}
			else if(Noise == u->AbstacleBodyPositionTriangle[i].Status)
			{
				distance                  = 0;
				_obstacle_distance.status = Noise;
			}
//			direct_distance = 6;
//			if(Normal == u->UltrasonicPacket[i].status)
//			{
//				direct_current_distance = u->UltrasonicPacket[i].Distance1;
//				direct_distance = direct_current_distance < direct_distance ? direct_current_distance : direct_distance;
//			}
		}
	}
	else if(Reverse == msg->Gear)
	{
		distance = 6;
		over_detection_cnt = 0;
		LR_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[5].Point.getX(), LEFT_EDGE_TO_CENTER);
		RR_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[5].Point.getX(),-RIGHT_EDGE_TO_CENTER);
		for(i = 4;i < 8;i++)
		{

			if(Normal == u->AbstacleBodyPositionTriangle[i].Status)
			{
				if(u->AbstacleBodyPositionTriangle[i].Position.getY() > 0.93)
				{
					current_distance = (u->AbstacleBodyPositionTriangle[i].Position - LR_P).Length();
				}
				else if(u->AbstacleBodyPositionTriangle[i].Position.getY() < -0.93)
				{
					current_distance = (u->AbstacleBodyPositionTriangle[i].Position - RR_P).Length();
				}
				else
				{
					current_distance = fabs(u->AbstacleBodyPositionTriangle[i].Position.getX() - _ultrasonic_obstacle_config.UltrasonicLocationArray[5].Point.getX());
				}
				distance = current_distance < distance ? current_distance : distance;
				_obstacle_distance.status = Normal;
			}
			else if(BlindZone == u->AbstacleBodyPositionTriangle[i].Status)
			{
				distance = 0;
				_obstacle_distance.status = BlindZone;
				break;
			}
			else if(OverDetection == u->AbstacleBodyPositionTriangle[i].Status)
			{
				over_detection_cnt++;
			}
			else if(Noise == u->AbstacleBodyPositionTriangle[i].Status)
			{
				distance                  = 0;
				_obstacle_distance.status = Noise;
			}
		}
	}
	if(4 == over_detection_cnt)
	{
		distance                  = 3;
		_obstacle_distance.status = OverDetection;
	}
	_obstacle_distance.distance = distance;
}


void UltrasonicObstaclePercption::CollisionDiatanceCalculate(Ultrasonic *u,uint8_t start,uint8_t end,uint8_t p_id,ObstacleDistancePacket *odp)
{
	uint8_t i;
	float distance,current_distance;
	UltrasonicStatus status;
	ObstacleRegion region;
	uint8_t over_detection_cnt;
	Vector2d L_P,R_P;
/////////////////////////////////////////////////////////////////////////////////////////////
	distance = MAX_DETECT_RANGE;
	status   = OverDetection;
	region   = CenterRegion;
	over_detection_cnt = 0;
	L_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[p_id].Point.getX(), LEFT_EDGE_TO_CENTER);
	R_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[p_id].Point.getX(),-RIGHT_EDGE_TO_CENTER);

	for(i = start; i < end; i++)
	{
		if(Normal == u->AbstacleBodyPositionTriangle[i].Status)
		{
			if(u->AbstacleBodyPositionTriangle[i].Position.getY() > 0.93)
			{
				current_distance = (u->AbstacleBodyPositionTriangle[i].Position - L_P).Length();
				region = current_distance < distance ? LeftRegion : region;
			}
			else if(u->AbstacleBodyPositionTriangle[i].Position.getY() < -0.93)
			{
				current_distance = (u->AbstacleBodyPositionTriangle[i].Position - R_P).Length();
				region = current_distance < distance ? RightRegion : region;
			}
			else
			{
				current_distance = fabs(u->AbstacleBodyPositionTriangle[i].Position.getX() - _ultrasonic_obstacle_config.UltrasonicLocationArray[p_id].Point.getX());
				if(u->AbstacleBodyPositionTriangle[i].Position.getY() < 0)
				{
					region = current_distance < distance ? RightCenterRegion : region;
				}
				else
				{
					region = current_distance < distance ? LeftCenterRegion : region;
				}
			}
			distance = current_distance < distance ? current_distance : distance;
			status = Normal;
		}
		else if(BlindZone == u->AbstacleBodyPositionTriangle[i].Status)
		{
			distance = MIN_DETECT_RANGE;
			status   = BlindZone;
			switch(i)
			{
				case 0:
				case 4:
					region   = LeftRegion;
					break;

				case 1:
				case 5:
					region   = LeftCenterRegion;
					break;

				case 2:
				case 6:
					region   = RightCenterRegion;
					break;

				case 3:
				case 7:
					region   = RightRegion;
					break;

				default:

					break;
			}
			break;
		}
		else if(OverDetection == u->AbstacleBodyPositionTriangle[i].Status)
		{
			over_detection_cnt++;
		}
		else if(Noise == u->AbstacleBodyPositionTriangle[i].Status)
		{
			distance = MAX_DETECT_RANGE;
			region   = CenterRegion;
			status   = Noise;
		}
		else if(InvalidPoint == u->AbstacleBodyPositionTriangle[i].Status)
		{
			if(0 == u->UltrasonicPacket[i].status)
			{
				if(u->UltrasonicPacket[i].Distance1 < 0.35f)
				{
					distance = MAX_DETECT_RANGE;
					region   = CenterRegion;
					status   = OverDetection;
				}
				else
				{
					current_distance = u->UltrasonicPacket[i].Distance1;
					if(current_distance < distance)
					{
						switch(i)
						{
							case 0:
							case 4:
								region   = LeftRegion;
							break;

							case 1:
							case 5:
								region   = LeftCenterRegion;
								break;

							case 2:
							case 6:
								region   = RightCenterRegion;
							break;

							case 3:
							case 7:
								region   = RightRegion;
							break;

							default:
								break;
						}
					}
					distance = current_distance < distance ? current_distance : distance;
					status = Normal;
				}
			}
			else if(16 == u->UltrasonicPacket[i].status)
			{
				distance = MIN_DETECT_RANGE;
				status   = BlindZone;
				switch(i)
				{
					case 0:
					case 4:
						region   = LeftRegion;
					break;

					case 1:
					case 5:
						region   = LeftCenterRegion;
						break;

					case 2:
					case 6:
						region   = RightCenterRegion;
					break;

					case 3:
					case 7:
						region   = RightRegion;
					break;

					default:
						break;
				}
				break;
			}
			else if(2 == u->UltrasonicPacket[i].status)
			{
				distance = MAX_DETECT_RANGE;
				region   = CenterRegion;
				status   = Noise;
			}
		}
	}
	if(4 == over_detection_cnt)
	{
		distance = MAX_DETECT_RANGE;
		region   = CenterRegion;
		status   = OverDetection;
		for(i=start;i<end;i++)
		{
			if(0 == u->UltrasonicPacket[i].status)
			{
				if(u->UltrasonicPacket[i].Distance1 > 0.35f)
				{
					current_distance = u->UltrasonicPacket[i].Distance1;
					if(current_distance < distance)
					{
						switch(i)
						{
							case 0:
							case 4:
								region   = LeftRegion;
							break;

							case 1:
							case 5:
								region   = LeftCenterRegion;
								break;

							case 2:
							case 6:
								region   = RightCenterRegion;
							break;

							case 3:
							case 7:
								region   = RightRegion;
							break;

							default:
								break;
						}
					}
					distance = current_distance < distance ? current_distance : distance;
					status = Normal;
				}
			}
			else if(16 == u->UltrasonicPacket[i].status)
			{
				distance = MIN_DETECT_RANGE;
				status   = BlindZone;
				switch(i)
				{
					case 0:
					case 4:
						region   = LeftRegion;
					break;

					case 1:
					case 5:
						region   = LeftCenterRegion;
						break;

					case 2:
					case 6:
						region   = RightCenterRegion;
					break;

					case 3:
					case 7:
						region   = RightRegion;
					break;

					default:
						break;
				}
				break;
			}
			else if(2 == u->UltrasonicPacket[i].status)
			{
				distance = MAX_DETECT_RANGE;
				region   = CenterRegion;
				status   = Noise;
				break;
			}
		}
	}
	odp->distance = distance;
	odp->region   = region;
	odp->status   = status ;
}

void UltrasonicObstaclePercption::ObstacleDistanceProcess(ObstacleDistancePacket *before,ObstacleDistancePacket *after,ObstacleDistanceProcessState *state,uint8_t *cnt)
{
	switch(*state)
	{
		case WaitObstacleState:
			if(Normal == before->status)
			{
				if(before->distance > 1.0f)
				{
					*after = *before;
					*cnt = 0;
					*state = ObstacleUpdateState;
				}
			}
			else if(OverDetection == before->status)
			{
				*after = *before;
			}
			break;

		case ObstacleUpdateState:
			if(OverDetection == before->status)
			{
				*cnt = *cnt + 1;
				if((*cnt > 10) && (after->distance > 1.0f))
				{
					*state = WaitObstacleState;
				}
			}
			else
			{
				*cnt = 0;
				*after = *before;
			}
			break;

		default:
			break;
	}
}

void UltrasonicObstaclePercption::UltrasonicCollisionDiatanceV1_0(Ultrasonic *u,MessageManager *msg)
{
	uint8_t i;
	float distance,current_distance;
	UltrasonicStatus status;
	ObstacleRegion region;
	uint8_t over_detection_cnt;
	Vector2d LF_P,RF_P,LR_P,RR_P;
/////////////////////////////////////////////////////////////////////////////////////////////
	distance = MAX_DETECT_RANGE;
	status   = Normal;
	region   = CenterRegion;
	over_detection_cnt = 0;
	LF_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[1].Point.getX(), LEFT_EDGE_TO_CENTER);
	RF_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[1].Point.getX(),-RIGHT_EDGE_TO_CENTER);

	for(i = 0; i < 4; i++)
	{
		if(Normal == u->AbstacleBodyPositionTriangle[i].Status)
		{
			if(u->AbstacleBodyPositionTriangle[i].Position.getY() > 0.93)
			{
				current_distance = (u->AbstacleBodyPositionTriangle[i].Position - LF_P).Length();
				region = current_distance < distance ? LeftRegion : region;
			}
			else if(u->AbstacleBodyPositionTriangle[i].Position.getY() < -0.93)
			{
				current_distance = (u->AbstacleBodyPositionTriangle[i].Position - RF_P).Length();
				region = current_distance < distance ? RightRegion : region;
			}
			else
			{
				current_distance = fabs(u->AbstacleBodyPositionTriangle[i].Position.getX() - _ultrasonic_obstacle_config.UltrasonicLocationArray[1].Point.getX());
				region = current_distance < distance ? CenterRegion : region;
			}
			distance = current_distance < distance ? current_distance : distance;
			status = Normal;
		}
		else if(BlindZone == u->AbstacleBodyPositionTriangle[i].Status)
		{
			distance = MIN_DETECT_RANGE;
			region   = CenterRegion;
			status   = BlindZone;
			break;
		}
		else if(OverDetection == u->AbstacleBodyPositionTriangle[i].Status)
		{
			over_detection_cnt++;
		}
		else if(Noise == u->AbstacleBodyPositionTriangle[i].Status)
		{
			distance = MAX_DETECT_RANGE;
			region   = CenterRegion;
			status   = Noise;
		}
		else if(InvalidPoint == u->AbstacleBodyPositionTriangle[i].Status)
		{
			if(0 == u->UltrasonicPacket[i].status)
			{
				if(0 == u->UltrasonicPacket[i].Distance1)
				{
					distance = MAX_DETECT_RANGE;
					region   = CenterRegion;
					status   = OverDetection;
				}
				else
				{
					current_distance = u->UltrasonicPacket[i].Distance1;
					if(current_distance < distance)
					{
						switch(i)
						{
							case 0:
								region   = LeftRegion;
							break;

							case 1:
							case 2:
								region   = CenterRegion;
							break;

							case 3:
								region   = RightRegion;
							break;

							default:
								break;
						}
					}
					distance = current_distance < distance ? current_distance : distance;
					status = Normal;
				}
			}
			else if(16 == u->UltrasonicPacket[i].status)
			{
				distance = MIN_DETECT_RANGE;
				region   = CenterRegion;
				status   = BlindZone;
				break;
			}
			else if(2 == u->UltrasonicPacket[i].status)
			{
				distance = MAX_DETECT_RANGE;
				region   = CenterRegion;
				status   = Noise;
			}
		}
	}
	if(4 == over_detection_cnt)
	{
		distance = MAX_DETECT_RANGE;
		region   = CenterRegion;
		status   = OverDetection;
		for(i=0;i<4;i++)
		{
			if(0 == u->UltrasonicPacket[i].status)
			{
				if(0 != u->UltrasonicPacket[i].Distance1)
				{
					current_distance = u->UltrasonicPacket[i].Distance1;
					if(current_distance < distance)
					{
						switch(i)
						{
							case 0:
								region   = LeftRegion;
							break;

							case 1:
							case 2:
								region   = CenterRegion;
							break;

							case 3:
								region   = RightRegion;
							break;

							default:
								break;
						}
					}
					distance = current_distance < distance ? current_distance : distance;
					status = Normal;
				}
			}
			else if(16 == u->UltrasonicPacket[i].status)
			{
				distance = MIN_DETECT_RANGE;
				region   = CenterRegion;
				status   = BlindZone;
				break;
			}
			else if(2 == u->UltrasonicPacket[i].status)
			{
				distance = MAX_DETECT_RANGE;
				region   = CenterRegion;
				status   = Noise;
				break;
			}
		}
	}
	_front_obstacle_distance.distance = distance;
	_front_obstacle_distance.region   = region;
	_front_obstacle_distance.status   = status ;

/////////////////////////////////////////////////////////////////////////////////////////
	distance = MAX_DETECT_RANGE;
	status   = Normal;
	region   = CenterRegion;
	over_detection_cnt = 0;
	LR_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[5].Point.getX(), LEFT_EDGE_TO_CENTER);
	RR_P = Vector2d(_ultrasonic_obstacle_config.UltrasonicLocationArray[5].Point.getX(),-RIGHT_EDGE_TO_CENTER);
	for(i = 4;i < 8;i++)
	{
		if(Normal == u->AbstacleBodyPositionTriangle[i].Status)
		{
			if(u->AbstacleBodyPositionTriangle[i].Position.getY() > 0.93)
			{
				current_distance = (u->AbstacleBodyPositionTriangle[i].Position - LR_P).Length();
				region = current_distance < distance ? LeftRegion : region;
			}
			else if(u->AbstacleBodyPositionTriangle[i].Position.getY() < -0.93)
			{
				current_distance = (u->AbstacleBodyPositionTriangle[i].Position - RR_P).Length();
				region = current_distance < distance ? RightRegion : region;
			}
			else
			{
				current_distance = fabs(u->AbstacleBodyPositionTriangle[i].Position.getX() - _ultrasonic_obstacle_config.UltrasonicLocationArray[5].Point.getX());
				region = current_distance < distance ? CenterRegion : region;
			}
			distance = current_distance < distance ? current_distance : distance;
			status = Normal;
		}
		else if(BlindZone == u->AbstacleBodyPositionTriangle[i].Status)
		{
			distance = MIN_DETECT_RANGE;
			region   = CenterRegion;
			status   = BlindZone;
			break;
		}
		else if(OverDetection == u->AbstacleBodyPositionTriangle[i].Status)
		{
			over_detection_cnt++;
		}
		else if(Noise == u->AbstacleBodyPositionTriangle[i].Status)
		{
			distance = MAX_DETECT_RANGE;
			region   = CenterRegion;
			status   = Noise;
		}
		else if(InvalidPoint == u->AbstacleBodyPositionTriangle[i].Status)
		{
			if(0 == u->UltrasonicPacket[i].status)
			{
				if(0 == u->UltrasonicPacket[i].Distance1)
				{
					distance = MAX_DETECT_RANGE;
					region   = CenterRegion;
					status   = OverDetection;
				}
				else
				{
					current_distance = u->UltrasonicPacket[i].Distance1;
					if(current_distance < distance)
					{
						switch(i)
						{
							case 4:
								region   = LeftRegion;
							break;

							case 5:
							case 6:
								region   = CenterRegion;
							break;

							case 7:
								region   = RightRegion;
							break;

							default:
								break;
						}
					}
					distance = current_distance < distance ? current_distance : distance;
					status = Normal;
				}
			}
			else if(16 == u->UltrasonicPacket[i].status)
			{
				distance = MIN_DETECT_RANGE;
				region   = CenterRegion;
				status   = BlindZone;
				break;
			}
			else if(2 == u->UltrasonicPacket[i].status)
			{
				distance = MAX_DETECT_RANGE;
				region   = CenterRegion;
				status   = Noise;
			}
		}
	}
	if(4 == over_detection_cnt)
	{
		distance = MAX_DETECT_RANGE;
		region   = CenterRegion;
		status   = OverDetection;
		for(i=4;i<8;i++)
		{
			if(0 == u->UltrasonicPacket[i].status)
			{
				if(0 != u->UltrasonicPacket[i].Distance1)
				{
					current_distance = u->UltrasonicPacket[i].Distance1;
					if(current_distance < distance)
					{
						switch(i)
						{
							case 0:
								region   = LeftRegion;
							break;

							case 1:
							case 2:
								region   = CenterRegion;
							break;

							case 3:
								region   = RightRegion;
							break;

							default:
								break;
						}
					}
					distance = current_distance < distance ? current_distance : distance;
					status = Normal;
				}
			}
			else if(16 == u->UltrasonicPacket[i].status)
			{
				distance = MIN_DETECT_RANGE;
				region   = CenterRegion;
				status   = BlindZone;
				break;
			}
			else if(2 == u->UltrasonicPacket[i].status)
			{
				distance = MAX_DETECT_RANGE;
				region   = CenterRegion;
				status   = Noise;
				break;
			}
		}
	}
	_rear_obstacle_distance.distance = distance;
	_rear_obstacle_distance.region   = region;
	_rear_obstacle_distance.status   = status;
}

void UltrasonicObstaclePercption::UltrasonicCollisionDiatanceV1_1(Ultrasonic *u)
{
	CollisionDiatanceCalculate(u,0,4,1,&_front_obstacle_distance);
	CollisionDiatanceCalculate(u,4,8,5,&_rear_obstacle_distance);

}

void UltrasonicObstaclePercption::UltrasonicCollisionDiatanceV1_2(Ultrasonic *u)
{
	CollisionDiatanceCalculate(u,0,4,1,&_front_obstacle_distance_temp);
	CollisionDiatanceCalculate(u,4,8,5,&_rear_obstacle_distance_temp);

	ObstacleDistanceProcess(&_front_obstacle_distance_temp,&_front_obstacle_distance,&_front_obstacle_process_state,&front_obstacle_cnt);
	ObstacleDistanceProcess(&_rear_obstacle_distance_temp,&_rear_obstacle_distance,&_rear_obstacle_process_state,&rear_obstacle_cnt);

}
/***********************************************************************************************/
uint16_t UltrasonicObstaclePercption::getPositionListLength()
{
	return (uint16_t)_ultrasonic_position_list->Length();
}

uint16_t UltrasonicObstaclePercption::getLocationListLength()
{
	return (uint16_t)_ultrasonic_triangle_location_list->Length();
}

uint16_t UltrasonicObstaclePercption::getLeftEdgeListLength()
{
	return (uint16_t)_left_edge_position_list->Length();
}

uint16_t UltrasonicObstaclePercption::getRightEdgeListLength()
{
	return (uint16_t)_right_edge_position_list->Length();
}

////////////////////////////////////////////////////////////////////////
LocationStatus UltrasonicObstaclePercption::getUltrasonicLocationStatus()           { return  _ultrasonic_location_sts;}
void  UltrasonicObstaclePercption::setUltrasonicLocationStatus(LocationStatus value){ _ultrasonic_location_sts = value;}
////////////////////////////////////////////////////////////////////////
