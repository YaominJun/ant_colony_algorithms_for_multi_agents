#pragma once
#include "notation.h"
//#include "m_map.h"
#include "astar.h"
#include "aco.h"
//#include "pointcloudplanner3d.h"
#include "basicFunction.h"



class Aco_Robot
{
private:
public:
    vector<int> nodeSequence;
    vector<Node> robotPath;	
};

class aco_init
{
private:
public:
    void init_node();
    void init_plan();
    void aco_plan();
    void show_map();
    void coordPlan2Map(vector<Node>* pathInPlan);
    Node coordPlan2Map(Node nodeInPlan);
    void DrawPathLine(vector<Node> path, CvScalar cvcolor);
    void DrawPathPoint(vector<Node> TotalTaskNode, CvScalar cvcolor);
    void task_nodeCallback(const nav_msgs::Path &task_node_list);
    
    Caco m_aco;
	Castar m_astar;
    vector<Node> TotalTaskNode;
	vector<double> timeless;  //时效性系数
    Aco_Robot robots[Max_robotNuumber];
    vector<Node> pathInMap;
    int m_srcImgWidth;
    int m_srcImgHeight;


    ros::NodeHandle n;
	ros::Publisher pub;
    ros::Subscriber sub;
    void init();

    IplImage* m_srcImg; 
    CvScalar cvRobotColor[Max_robotNuumber];

};

