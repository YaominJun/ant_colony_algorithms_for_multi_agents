#pragma once//防止重定义

#include <ros/ros.h>
#include <ros/package.h>

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <string>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"




using namespace std;

//Timer
#define RUN 2009


#define Max_robotNuumber 50         //环境中最多可以存在的机器人数量
#define RobotNumber 5  				//机器人数量
#define pai 3.14159
#define realmapHeight 200           //单位为米
#define realmapWidth 200            //单位为米
#define singleGridSize 1            //单位为米
#define vehicleSize 2.0
#define WheelBase 2.5772           //byd前后轴距
#define Track 1.51                 //byd左右轮距
#define DisBetweenWheel 0.331      //p3at左右轮距
#define taskNodeNumber 20        //任务节点数量
//#define Base_R  2                //机器人起始点移动的半径大小
const int mapWidth = realmapWidth / singleGridSize;
const int mapHeight = realmapHeight / singleGridSize;
//#define Robot_R 10.55; //机器人半径，用于显示

typedef struct _Pose
{
    float x;
    float y;
	float theta;
    _Pose(float X, float Y, float T)
    {
        x = X;
        y = Y;
		theta = T;
    }
    _Pose()
	{
		x = 0;
		y = 0;
		theta = 0;
	}
}Pose;

typedef struct _Status
{
	float v;
	float w;
	 _Status(float V, float W)
    {
        v = V;
        w = W;
    }
    _Status()
	{
		v = 0;
		w = 0;
	}
}Status;

typedef struct _Node
{
    float x;
    float y;
	_Node(float X,float Y)
	{
		x = X;
		y = Y;
	}
    _Node()
	{
		x = 0;
		y = 0;
	}
}Node;

struct pos
{
	double x;
	double y;
};

struct pos_int
{
	int x;
	int y;
};

struct state_struct
{
	pos position;
	double s;//弧长
	double theta;//弧度，车头与正东方向夹角，heading=pi/2-theta; heading_g=heading_l+heading_v
	bool forward;//前进或者后退

	//state_struct用于描述原始全局期望路径，以下属性基本没用
	double steering_angle;
};
