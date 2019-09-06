/*
 * ultrasonic_abstacle_percption.h
 *
 *  Created on: 2019年1月29日
 *      Author: zhuguohua
 */

#ifndef ULTRASONICPERCAPTION_ULTRASONIC_OBSTACLE_PERCPTION_H_
#define ULTRASONICPERCAPTION_ULTRASONIC_OBSTACLE_PERCPTION_H_

#include "./Percaption/Interface/percaption.h"

// 选择运行的平台：嵌入式 -> 0 ; PC -> 1;
#define EMBEDDED_PLATFORM              (0)
#define PC_PLATFORM                    (1)
#define RUNNING_PLATFORM               EMBEDDED_PLATFORM
// 选择车辆入库的方向，可选的是x轴还是y轴
#define X_AXIS_ENTER				   (0)
#define Y_AXIS_ENTER				   (1)
#define ENTER_PARK_DIRECTION           (Y_AXIS_ENTER)

// 入库过程库位调整相关参数
#define DISTANCE_ERR_THRESHOLD        (0.6f)
#define LEVEL_THRESHOLD               (2.2f)
#define WIDTH_THRESHOLD               (30.0f)
#define DISTANCE_THRESHOLD            (0.5f)
#define STEP_DISTANCE                 (0.1f)
#define MIN_LOCATION_NUM              (50)
// 进库后的相关参数调整
#define FIT_LINE_STEP_DISTANCE        (0.05f)
#define FIT_LINE_STEP_LEVEL_THRESHOLD (3.0f)
#define MIN_FIT_NUM                   (20)
#define MIN_FIT_DISTANCE              (1.0)


#define MAX_DETECT_RANGE              (3.5f)
#define MIN_DETECT_RANGE              (0.0f)
// 库位边沿查找算法
typedef enum _EdgeFindingState
{
	ObstacleWaitEdge= 0,
	VehicleEdgeWaitstate,
	WaitEnterDenseArea,
	JudgeParkingValid,
	waitParking
}EdgeFindingState;

typedef enum _EdgeFindingStateV1_0
{
	EdgeJudge= 0,
	EdgeLooking
}EdgeFindingStateV1_0;

typedef enum _LocationStatus
{
	LocationReady= 0,
	LocationDataPush,
	LocationCalculate,
	LocationFinish
}LocationStatus;

typedef enum _UltrasonicLocationPushState
{
	WaitPushStart= 0,
	ParkingEdgeUltrasonicDataPush,
	ParkingCenterUltrasonicDataPush,
}UltrasonicLocationPushState;

typedef enum _UltrasonicLocationCalculateState
{
	WaitCommandForCalculate = 0,
	FrontEdgeCalculate,
	RearEdgeCalculate,
	ParkingCenterCalculate
}UltrasonicLocationCalculateState;

typedef enum _ObstacleDistanceProcessState
{
	WaitObstacleState = 0,
	ObstacleUpdateState,
}ObstacleDistanceProcessState;

class UltrasonicObstaclePercption : public Percaption
{
public:
	UltrasonicObstaclePercption();
	virtual ~UltrasonicObstaclePercption();

	void Init();

	void Push(LinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat);
	/*
	 * 有序数据的推送
	 * */
	void OrderedPush(LinkList *list,ObstacleLocationPacket p_dat);
	/*
	 * 无序数据的推送
	 * */
	void DisorderPush(LinkList *list,ObstacleLocationPacket p_dat);

	void ParkingCenterPush(LinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat);
	/*******************************************************************************************/
	float WeightCalculation(float d);

	void DataCredibilityCalculate(LinkList *list);

	/////////////////////////////////////////////////////////////////////////////////////////////
	void EdgeFinding(LinkList *list);
	void EdgeFinding_V1_0(LinkList *list);
	// 求取最高分布值的最优解
	float HighestDistribution(uint8_t group_number,uint16_t* group_value_array,float min_value);
	// 求取最高分布的索引值
	uint8_t HighestDistributionBase(uint8_t group_number,uint16_t* group_value_array);
	void ValueDistributed(LinkList *valid_list);
	/*
	 * 数值分布求取
	 * valid_list：原始的分布数据集，
	 * fit_list：分布最高的数据集
	 * */
	void ValueDistributedFilter(LinkList *valid_list,LinkList *fit_list);
	// 根据有效数据拟合车库边沿
	void EdgeLineFitParkingCenterCalculate();
	/******************************库位重新定位的状态机***************************************/
	// 库位重新定位
	/*
	 * 数据推送状态机，负责有效超声数据的推送；
	 * PC端需50ms调用一次该函数,嵌入式直接放入5ms定时器即可
	 * */
	void DataPushStateMachine(Ultrasonic* u_dat);
	/*
	 * 库位位置和中心点位置，重新计算的状态机
	 * 使用时无时序要求，放入最低优先级任务中即可，比如While(1)死循环中
	 * 该状态机有返回值
	 * 如果返回值为:-1 -> 计算未完成；0 -> 计算完成
	 * */
	int8_t ParkingCalculateStateMachine(void);

	/*********************************************超声避障相关函数***************************************/
	// 计算避障距离的基本函数
	void CollisionDiatanceCalculate(Ultrasonic *u,uint8_t start,uint8_t end,uint8_t p_id,ObstacleDistancePacket *odp);

	void ObstacleDistanceProcess(ObstacleDistancePacket *before,ObstacleDistancePacket *after,ObstacleDistanceProcessState *state,uint8_t *cnt);

	int8_t UltrasonicCollisionStatus(Ultrasonic *u,MessageManager *msg);

	void UltrasonicCollisionDiatance(Ultrasonic *u,MessageManager *msg);

	void UltrasonicCollisionDiatanceV1_0(Ultrasonic *u,MessageManager *msg);

	void UltrasonicCollisionDiatanceV1_1(Ultrasonic *u);

	void UltrasonicCollisionDiatanceV1_2(Ultrasonic *u);
	/*********************************************基础函数***************************************/
	uint16_t getPositionListLength();
	uint16_t getLocationListLength();
	uint16_t getLeftEdgeListLength();
	uint16_t getRightEdgeListLength();
	/////////////////////////////////////////////////////////////////////////////////////////////////
	LocationStatus getUltrasonicLocationStatus();
	void    setUltrasonicLocationStatus(LocationStatus value);
	Property<UltrasonicObstaclePercption,LocationStatus,READ_WRITE> UltrasonicLocationStatus;
private:
	LocationStatus _ultrasonic_location_sts;
	EdgeFindingState _edge_finding_state;
	EdgeFindingStateV1_0 _edge_finding_state_v1_0;

	UltrasonicLocationPushState      _data_push_state;
	UltrasonicLocationCalculateState _parking_calculate_state;

	LinkList *_ultrasonic_position_list;
	LinkList *_ultrasonic_triangle_location_list;

	LinkList *_left_edge_position_list;
	LinkList *_right_edge_position_list;

	LinkList *_left_fit_edge_list;
	LinkList *_right_fit_edge_list;

	ObstacleInformationPacket _parking_position;
	ObstacleInformationPacket _vehicle_position;

	uint8_t _push_cnt;
	/******************************************/
	/////////////////////////////////////////////
	CurveFitting _line_fit;
	/******************************************/
	/**************超声避障相关参数************/
	/******************************************/
	VehilceConfig _ultrasonic_obstacle_config;

	ObstacleDistanceProcessState _front_obstacle_process_state;
	ObstacleDistanceProcessState _rear_obstacle_process_state;

	uint8_t front_obstacle_cnt;
	uint8_t rear_obstacle_cnt;

	ObstacleDistancePacket _front_obstacle_distance_temp;
	ObstacleDistancePacket _rear_obstacle_distance_temp;
};

#endif /* ULTRASONICPERCAPTION_ULTRASONIC_ABSTACLE_PERCPTION_H_ */
