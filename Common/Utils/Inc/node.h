/*
 * node.h
 *
 *  Created on: 2019年4月25日
 *      Author: zhuguohua
 */

#ifndef UTILS_NODE_H_
#define UTILS_NODE_H_

#include "./Interaction/Ultrasonic/Ultrasonic.h"

class Node {
public:
	Node();
	virtual ~Node();

	ObstacleLocationPacket data;//值域
	Node *next;//指针域，指向下一个节点的指针
};

#endif /* UTILS_NODE_H_ */
