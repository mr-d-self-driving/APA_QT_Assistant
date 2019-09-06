/*
 * system_config.h
 *
 *  Created on: 2019年3月30日
 *      Author: zhuguohua
 */

#ifndef CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_
#define CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_
/****************Task Shedule Priority Level******************/
#define TIME_5MS_TASK (1)
#define CAN0_TASK     (2)
#define CAN1_TASK     (3)
#define CAN2_TASK     (4)

/****************System Configure******************/
//#define CHANGAN
//#define BORUI
#define DONG_FENG_E70

#define SIMULATION 0
/********************是否使用超声波避障使能按钮***********************/
#define ULTRASONIC_COLLISION_ENABLE  ( 1 ) // 超声避障使能按钮


#endif /* CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_ */
