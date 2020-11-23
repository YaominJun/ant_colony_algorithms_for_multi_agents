#pragma once
#include "notation.h"
//#include "basicFunction.h"
//#include "MultiRobotPlanDlg.h"
//#include "map.h"
//using namespace std;

//values for Astar
const int expandFactor = 2;
const int found = 1, nonexistent = 2, errorputin = 3; // Astar 返回值
const int walkable = 0, unwalkable = 1;// walkability array constants
const int onOpenList = 1, onClosedList = 2;

class Castar
{
private:

	int path[mapWidth*mapHeight+2][2];     //stores the found path for the user
	int walkability[mapHeight][mapWidth];

	bool m_bFind;
	IplImage* m_expandImg;       //用于路径规划
	vector<Node> m_path;        //存放路径搜索结果
	double moveCost;               //路径消耗
	int nodeNumber;             //节点个数
   
//	void GetFinalShortestPath();
    
public:

    Castar();
	~Castar(void);
	void ExpandPlanMap();
	bool IsFindPath();
	void InitWalkabilityMap();
	int FindPath(Node startNode, Node endNode);	
	void ShowMapwithPath();
	vector<Node>* GetPathVector(); 
	double GetmoveCost();
};
