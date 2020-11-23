#pragma once                               //防止重定义

#include "notation.h"
#include "basicFunction.h"

//蚁群算法参数
const int Max_salesmanNumber = 5;              //最多旅行商人数
const int iCityCount = taskNodeNumber;         //城市数量
const int iAntCount = 80;		               //蚂蚁数量
const int iItCount = 20000;	                   //最大迭代次数
const int Threshold = 2000;                    //Threshold次不更新最优解也是会跳出循环的  
const double man_v = 2;                        //旅行商的估计速度

static double disMat[iCityCount][iCityCount];                 //路径长度
static double timeless[iCityCount];                      //存放时效性系数
static double m_dTrial[iCityCount][iCityCount];               //信息素值

const double Q = 100;
const double q = 0.5;  //q和（1 - q）分别为路径消耗和时效性消耗的重要性占比
const double alpha = 1;
const double beta = 1;
const double rou = 0.5; //信息素蒸发（或挥发）系数
const int salesmanNumber = RobotNumber;

//const int salesmanNumber = 3;              //旅行商人数

class ant
{
private:
	int m_iCityCount;
	int ChooseNextCity();			    //选择城市	
	int speChooseNextCity();			//选择城市	
	int AllowedCity[iCityCount];		//没有走过的城市
	double prob[iCityCount];            //选择概率

public:
	int tabu[iCityCount+Max_salesmanNumber];             //存放路径
	double m_dLength;                                    //路径消耗
	double m_tCost;                                      //时效性消耗
	double m_cost;                                       //总消耗
	
	ant();
	~ant(void);
	void setSalesmanNumber();
	void addcity(int cityNumber);	
	void addstartcity(int cityNumber);	
	void Clear();
	void UpdateResult();	
	void Move();
	void speMove();
	void move2last();
};

class Caco
{
private:

public:	
	int besttour[iCityCount+Max_salesmanNumber];	             //最优路径列表
	vector<int> nodeSequence;                                    //存放最优路径序列
	double m_best_route_Cost;                                    //最优总消耗
	double m_best_route_Length;                                  //最优路径消耗
	double m_best_route_Timeless;                                //最优时效性消耗
	double disMatfromOut[iCityCount][iCityCount];                //距离矩阵
	double timelessfromOut[iCityCount];                               //存放时效性系数
	double m_dDeltTrial[iCityCount][iCityCount];                 //信息素变化值
	ant ants[iAntCount];
	double weight;
	
	Caco();
	~Caco(void);	
	void initmap();
	void copydisMat();
	void GetAnt();
	void UpdateTrial();
	void UpdateTrial(int bestOne);
	void StartSearch();	
};

