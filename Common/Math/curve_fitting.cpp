/*****************************************************************************/
/* FILE NAME: curve_fitting.cpp                     COPYRIGHT (c) Motovis 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to fit the curve line  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     May 20 2019         Initial Version                  */
/*****************************************************************************/
#include "./Common/Math/curve_fitting.h"

CurveFitting::CurveFitting() {
	// TODO Auto-generated constructor stub

}

CurveFitting::~CurveFitting() {
	// TODO Auto-generated destructor stub
}

void CurveFitting::LineFitting(LinkList *l,float *a,float *b)
{
	float sum_x,sum_y,sum_xy,sum_x2,sum_n;
	float denominator,molecule_a,molecule_b;
	Node *current_node;
	current_node = l->HeadNode;

	sum_x = 0;
	sum_y = 0;
	sum_xy = 0;
	sum_x2 = 0;
	sum_n  = 0;
	while(current_node->next != NULL)
	{
		sum_x += current_node->data.Position.getX();
		sum_y += current_node->data.Position.getY();
		sum_xy += current_node->data.Position.getX()*current_node->data.Position.getY();
		sum_x2 += current_node->data.Position.getX()*current_node->data.Position.getX();
		sum_n++;
		current_node = current_node->next;
	}
	denominator = sum_n * sum_x2 - sum_x * sum_x;
	if(denominator == 0)
	{
		*a = PI_2;
	}
	else
	{
		molecule_a  = sum_n * sum_xy - sum_x * sum_y;
		molecule_b  = sum_x2 * sum_y - sum_x * sum_xy;
		*a = atanf(molecule_a/denominator);
		*b = molecule_b/denominator;
	}
}
