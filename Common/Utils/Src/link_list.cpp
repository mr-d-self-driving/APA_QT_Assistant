/*
 * link_list.cpp
 *
 *  Created on: 2019年4月25日
 *      Author: zhuguohua
 */

#include "./Common/Utils/Inc/link_list.h"


LinkList::LinkList() {
	HeadNode.setContainer(this);
	HeadNode.getter(&LinkList::getHeadNode);

	EndNode.setContainer(this);
	EndNode.getter(&LinkList::getEndNode);
	// TODO Auto-generated constructor stub
	_list_length = 0;

	_head_node = NULL;//头节点
	_end_node  = NULL;//尾节点
	_node      = NULL;//
}

LinkList::~LinkList() {
	// TODO Auto-generated destructor stub
	_list_length = 0;
	delete _head_node;//头节点
	delete _end_node;//尾节点
	delete _node;//

	_head_node = NULL;
	_end_node  = NULL;
	_node      = NULL;
}

void LinkList::Add(ObstacleLocationPacket dat)
{
	_node = new Node;//申请一个新的节点
	if( _node != NULL)
	{
		_node->data = dat;
		if(_end_node == NULL)
		{
			_head_node = _node;//头节点
			_end_node  = _node;//尾节点
		}
		else
		{
			_end_node->next = _node;
			_end_node = _node;
		}
		_list_length++;
	}
}

void LinkList::Delete(void)
{
	Node* _free_node;//
	if(_list_length > 0)
	{
		while(_head_node->next != NULL)
		{
			_free_node = _head_node;
			_head_node = _head_node->next;
			delete _free_node;
			_free_node = NULL;
		}
		_list_length = 0;
		delete _head_node;
		_head_node = NULL;
		_end_node  = NULL;
	}
}

uint32_t LinkList::Length()//返回链表个数
{
	return _list_length;
}

Node* LinkList::getHeadNode() { return  _head_node;}
Node* LinkList::getEndNode()  { return  _end_node; }
