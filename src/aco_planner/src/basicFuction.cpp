#pragma once//防止重定义
#include "basicFunction.h"

vector<Node> connecte(vector<Node> v1, vector<Node> v2)
{
	for(int i=0; i<v2.size(); i++)
	{
		v1.insert(v1.end(),v2.at(i));
	}
	return v1;
}

float angle(Node point1, Node point2)
{
	float dx, dy, angle; 

	dx = point2.x - point1.x;
	dy = point2.y - point1.y;

	if(fabs(dx) <= 0.0001)
	{
		if(dy  > 0)
		{
			angle = pai / 2;
		}
		else
		{
			angle = 0.0 - pai / 2;
		}
	}
	else
	{
		angle = atan2(dy,dx);
		if(angle > 0)
		{
			if(dy < 0)
			{
				angle = angle - pai;
			}

		}
		else if(angle < 0)
		{
			if(dy > 0)
			{
				angle = angle + pai;
			}
		}
		else
		{
			if(dx < 0)
			{
				angle = angle + pai;
			}
		}
	}       
    return angle;
}

float dist(Node point1, Node point2)
{
	float dx, dy, distance; 
	dx = point2.x - point1.x;
	dy = point2.y - point1.y;
	distance = sqrt( dx*dx + dy*dy );
	return distance;
}

double rnd(int low,double uper)	//获得随机数
{
	double p = ( rand() / (double) RAND_MAX ) * ((uper) - (low)) + (low);
	return (p);
};

int rnd(int uper)
{
	return (rand()%uper);
};