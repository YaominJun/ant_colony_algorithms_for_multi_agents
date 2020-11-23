#pragma once                               //防止重定义
//#include "StdAfx.h"

#include"aco.h"

//#include "MultiRobotPlan.h"
//#include "MultiRobotPlanDlg.h"

//int Caco::salesmanNumber = RobotNumber;
//int ant::salesmanNumber = RobotNumber;

Caco::Caco()
{
    //initial map.
	initmap();
	weight = q;
	nodeSequence.clear();
	m_best_route_Cost = 10e9;
	m_best_route_Length = 10e9;
	m_best_route_Timeless = 10e9;
}

Caco::~Caco(void)
{
}

void Caco::initmap()
{	
	for(int i=0; i<iCityCount; i++)
	{
		for (int j=0; j<iCityCount; j++)
		{
			m_dTrial[i][j] = 1;
		    m_dDeltTrial[i][j] = 0;
		}
	}
}

void Caco::copydisMat()
{
	for(int i=0; i<iCityCount; i++)
	{
		for (int j=0; j<iCityCount; j++)
		{
			disMat[i][j] = disMatfromOut[i][j];
		}
		timeless[i] = timelessfromOut[i];
	}
}

void Caco::UpdateTrial()  //calculate the changes of trial information
{
	int i, j;

    //计算信息素变化矩阵
	for(i=0; i<iAntCount; i++)
	{
		if( ants[0].m_tCost>0.05) //有时效性
		{
			for (j=0; j<iCityCount+salesmanNumber-1; j++)
			{
				m_dDeltTrial[ants[i].tabu[j]][ants[i].tabu[j+1]] += q * Q / ants[i].m_dLength + (1-q) * Q / ants[i].m_tCost;
				m_dDeltTrial[ants[i].tabu[j+1]][ants[i].tabu[j]] += q * Q / ants[i].m_dLength + (1-q) * Q / ants[i].m_tCost;
			}
		}
		else                     //无时效性
		{
			for (j=0; j<iCityCount+salesmanNumber-1; j++)
			{
				m_dDeltTrial[ants[i].tabu[j]][ants[i].tabu[j+1]] +=  Q / ants[i].m_dLength;
				m_dDeltTrial[ants[i].tabu[j+1]][ants[i].tabu[j]] +=  Q / ants[i].m_dLength;
			}
		}
	}

	//更新信息素矩阵
	for (i=0; i<iCityCount; i++)
	{
		for (j=0; j<iCityCount; j++)
		{
		   m_dTrial[i][j] = (rou * m_dTrial[i][j] + m_dDeltTrial[i][j] ); //边(i,j)的信息素乘以rou衰减因子+信息素增量
		   m_dDeltTrial[i][j] = 0;
		}
	}
}

//MMAS信息素更新方式
void Caco::UpdateTrial(int bestOne)  //calculate the changes of trial information
{
	int j;
    //计算信息素变化矩阵	
	for (j=0; j<iCityCount+salesmanNumber-1; j++)
	{
		m_dDeltTrial[ants[bestOne].tabu[j]][ants[bestOne].tabu[j+1]] += Q / ants[bestOne].m_dLength ;
		m_dDeltTrial[ants[bestOne].tabu[j+1]][ants[bestOne].tabu[j]] += Q / ants[bestOne].m_dLength;
	}
	//m_dDeltTrial[ants[i].tabu[iCityCount-1]][ants[i].tabu[0]] += Q / ants[i].m_dLength;
	//m_dDeltTrial[ants[i].tabu[0]][ants[i].tabu[iCityCount-1]] += Q / ants[i].m_dLength;
	//更新信息素矩阵
	for(int i=0; i<iCityCount; i++)
	{
		for(j=0; j<iCityCount; j++)
		{
		   m_dTrial[i][j] = (rou * m_dTrial[i][j] + m_dDeltTrial[i][j] );
		   m_dDeltTrial[i][j] = 0;
		}
	}
}

void Caco::GetAnt()		//randomly put ant into map
{
	for(int i=0; i<iAntCount; i++)
	{		
		ants[i].Clear();
	}
}

void Caco::StartSearch()	//搜索最佳路径
{
	//CMultiRobotPlanDlg* mainDlg = (CMultiRobotPlanDlg*) AfxGetApp()->m_pMainWnd;    //主窗口句柄
	//Caco::salesmanNumber = RobotNumber;                                    //获取机器人数量	
	
	initmap();
	nodeSequence.clear();
	m_best_route_Cost = 10e9;	 //初始化搜索信息
	m_best_route_Length = 10e9;
	m_best_route_Timeless = 10e9;
	int times=0;                 //用于记录循环次数
	int notChangetimes = 0;      //记录多少次最优解没有更新，确定是否跳出搜索
	int i, j, t;
	int average_CN = 0, rest_CN = 0;
	int bestAnt;
	int nodes4salesman[Max_salesmanNumber];
	double tempMinCost = 10e9;
	double tempMinLength = 10e9;
	double tempMinTimeless = 10e9;
	int temptour[iCityCount+Max_salesmanNumber];
	GetAnt();                   //获取蚂蚁 //randomly put ant into map随机选取一个蚂蚁，则ants的数组中该蚂蚁index变为0
	// ants[0].setSalesmanNumber();

	//平均分配每个旅行商应访问的城市数目
	average_CN = floor( (double)(iCityCount-1) /salesmanNumber );
	rest_CN = (iCityCount-1) % salesmanNumber;
	for(i=0; i<salesmanNumber; i++)
	{
		nodes4salesman[i] = average_CN;
	}
	for(i=0; i<rest_CN; i++)
	{
		nodes4salesman[i] += 1;
	}

    ofstream outfile0("src/aco_planner/Debug/Route_costhistory.txt"); 
	outfile0<<"The shortest toure in one searche is :"<<endl;
	//outfile0<<tempMinCost<<" "<<m_best_route_Length<<endl;

	//每只蚂蚁都会为每个旅行商寻路
	while(times < iItCount)
	{	
		for(j=0; j<iAntCount; j++)                      //每只蚂蚁寻路
		{
			for(t=0; t<salesmanNumber; t++)              //为每个旅行商寻路
			{
				for(i=0; i<nodes4salesman[t]; i++)        //保证每个旅行商需要遍历的城市数      
				{
					if(i == (nodes4salesman[t]-1))                //到该旅行商的最后一个城市，需要回到起点
					{
						ants[j].speMove();
					}
					else
					{
						ants[j].Move();
					}						
				}
			}		
		   //ants[j].addcity(0);                    //回到起点，完成访问任务
		   ants[j].UpdateResult();                 //计算消耗值
		}

	    //取出本次最佳路径
		tempMinCost = ants[0].m_cost;           //将第一只蚂蚁的路径作为初始最佳路径
		for(t=0; t<iCityCount+salesmanNumber; t++)
		{
		   temptour[t] = ants[0].tabu[t]; //第一只蚂蚁走过的路径记录到temptour中
		}
		bestAnt = 0;
		for(j=0; j<iAntCount; j++) //选择总消耗最小的蚂蚁
		{
		   if(ants[j].m_cost < tempMinCost) 
		   {
			   tempMinCost = ants[j].m_cost;	
			   bestAnt = j;		   
		   }
		}		
		tempMinLength = ants[bestAnt].m_dLength;
		tempMinTimeless = ants[bestAnt].m_tCost;
		for(t=0; t<iCityCount+salesmanNumber; t++)
		{
			temptour[t] = ants[bestAnt].tabu[t];
		}

		//更新信息素
		UpdateTrial();                           //普通方法来更新信息素
		//UpdateTrial(bestAnt);                  //采用MMAS方法来更新信息素

		//记录历史最佳路径
		outfile0<<tempMinCost<<"      ";	
		if(tempMinCost < m_best_route_Cost)
		{
		   m_best_route_Cost = tempMinCost; //取得最优路径消耗
		   m_best_route_Length = tempMinLength;
		   m_best_route_Timeless = tempMinTimeless;
		   for(t=0; t<iCityCount+salesmanNumber; t++)
		   {
			   besttour[t] = temptour[t]; //更新最佳路径
		   }
		   notChangetimes = 0; //只要进行了更新，就将notChangetimes变量重置为0
		   outfile0<<m_best_route_Cost<<endl;	
		}
		else
		{
			notChangetimes++; //记录有多少次没有进行更新，相当于在此以后的都是tempMinCost >= m_best_route_Cost的路径。
			outfile0<<endl;	
		}

		//如果超过Threshold次没有更新，就跳出搜索
		if(notChangetimes >= Threshold) 
		{
			break;
		}

		//重置蚂蚁
		for(j=0; j<iAntCount; j++)
		{
		   ants[j].Clear();
		}
		times++;	
	}//while
	outfile0.close();

	ofstream outfile("src/aco_planner/Debug/Route.txt"); 
	outfile<<"Find shortest toure in "<<times++<<" times search."<<endl;
	outfile<<"The best toure cost is "<<m_best_route_Cost<<"."<<endl;
	outfile<<"The best toure Distance_cost is "<<m_best_route_Length<<"."<<endl;
	outfile<<"The best toure Timeless_cost is "<<m_best_route_Timeless<<"."<<endl;
	for(int t=0; t<iCityCount+salesmanNumber; t++)
	{
		outfile<<besttour[t]<<" ";
		nodeSequence.push_back(besttour[t]);
	}
	outfile<<endl;
	for(int t=0; t<iCityCount+salesmanNumber; t++)
	{
		outfile<<timeless[besttour[t]]<<" ";
	}
	outfile<<endl;
	outfile.close();
}


ant::ant()     //构造函数
{
	//salesmanNumber = Caco::salesmanNumber;
	m_dLength =  0;
	m_iCityCount = 0;
	for(int i=0; i<iCityCount; i++)
	{
		AllowedCity[i] = 1;
		prob[i] = 0;
	}
}

ant::~ant(void)
{
}

// void ant::setSalesmanNumber()
// {
// 	//CMultiRobotPlanDlg* mainDlg = (CMultiRobotPlanDlg*) AfxGetApp()->m_pMainWnd;    //主窗口句柄
// 	ant::salesmanNumber = RobotNumber;                                     //获取机器人数量
// }

void ant::addcity(int cityNumber)		//将已选城市加入禁忌表/路径序列中;
{
	tabu[m_iCityCount] = cityNumber;
	m_iCityCount++;
	AllowedCity[cityNumber] = 0; //0代表不能再次进入这个城市节点node
}

void ant::addstartcity(int cityNumber)		//将已选城市加入禁忌表/路径序列中;
{
	tabu[m_iCityCount] = cityNumber;
	m_iCityCount++;
	AllowedCity[cityNumber] = 2;
}

int ant::ChooseNextCity()
{
	//Update the probability of path selection
	//select a path from tabu[m_iCityCount-1] to next
	int i;
	int j = 10000;	
	int curCity = tabu[m_iCityCount-1];     //当前所在城市

	//计算每个供选城市的概率
	double totalVal = 0, sel = 0;
	for(i=0; i<iCityCount; i++)
	{
		if(AllowedCity[i] == 1) //AllowedCity[iCityCount]记录没有走过的城市
		{
			totalVal += pow( (m_dTrial[curCity][i]), alpha ) * pow( (1.0/disMat[curCity][i]), beta ); // 1/dis为启发式值，m_dTrial为信息素值
		}
	}
	for (i=0; i<iCityCount; i++)
	{
		if((AllowedCity[i] == 1))
		{
		   prob[i] = ( pow( (m_dTrial[curCity][i]), alpha) * pow(( 1.0/disMat[curCity][i]), beta ) ) / totalVal; //从curCity到i城市node的转移概率
		   sel += prob[i]; //概率之和
		}
		else
		{
			prob[i] = 0;
		}
	}

	//按概率选择下一城市
	//轮盘赌法
	double mRate = rnd(0,sel); //获取大于0小于总概率之和sel的随机数
	double mSelect = 0;

	for(i=0; i<iCityCount; i++)
	{
		if(AllowedCity[i] == 1) //未曾走过的城市
		{
		   mSelect += prob[i] ; //计算累计概率q[k]
		   if(mSelect >= mRate)  //mSelect大于随机数mRate（0<=mRate<=sel），退出循环
		   {
				j = i; //轮盘赌法：若随机数r<q[1]，则选择个体1，否则，选择个体k，使得：q[k-1]<r≤q[k] 成立。
				break;
		   }
		}
	}

	if(j == 10000) //j为初始值10000，即即便遍历所有未曾走过的城市都没有满足mSelect大于随机数mRate（0<=mRate<=sel）条件，
		//？为什么会有所有转移概率累计起来了还达不到mRate的情况？
		//相当于没有找到下一个城市点node，直接选择第一个城市。
	{
		totalVal = -1;
		for(i=0; i<iCityCount; i++)
		{
			if(AllowedCity[i] == 1) //未曾走过的城市
			{
				if(totalVal < pow( (1.0/disMat[curCity][i]), beta ) * pow( (m_dTrial[curCity][i]), alpha) )    
				{
					 totalVal = pow( (1.0/disMat[curCity][i]), beta ) * pow( (m_dTrial[curCity][i]), alpha );
					 j = i;
				}
			}
		}
	}

	return j;
}

int ant::speChooseNextCity()
{
	//Update the probability of path selection
	//select a path from tabu[m_iCityCount-1] to next
	int i;
	int j = 10000;	
	int curCity = tabu[m_iCityCount-1];     //当前所在城市

	//计算每个供选城市的概率
	double totalVal = 0, sel = 0;
	for(i=0; i<iCityCount; i++)
	{
		if(AllowedCity[i] == 1)
		{
			totalVal += pow( 1.0/(disMat[curCity][i] + disMat[i][0]), beta ) * pow( (m_dTrial[curCity][i] + m_dTrial[i][0]), alpha ); // 1/dis为启发式值，m_dTrial为信息素值
			//该函数相比于ChooseNextCity只是在计算信息素时多加了从下一个城市节点node到起点0的信息素值
		}
	}
	for(i=0; i<iCityCount; i++)
	{
		if((AllowedCity[i] == 1))
		{
		   prob[i] = pow( 1.0/(disMat[curCity][i] + disMat[i][0]), beta ) * pow( (m_dTrial[curCity][i] + m_dTrial[i][0]), alpha ) / totalVal;
		   //该函数相比于ChooseNextCity只是在计算信息素时多加了从下一个城市节点node到起点0的信息素值
		   sel += prob[i];
		}
		else
		{
			prob[i] = 0;
		}
	}

	//按概率选择下一城市
	double mRate = rnd(0,sel);
	//mRate = (rand() % 1001);
	double mSelect = 0;

	for(i=0; i<iCityCount; i++)
	{
		if(AllowedCity[i] == 1)
		{
		   mSelect += prob[i];
		   if(mSelect >= mRate) 
		   {
				j = i;
				break;
		   }
		}	
	}

	if(j == 10000)
	{
		totalVal = -1;
		for(i=0; i<iCityCount; i++)
		{
			if(AllowedCity[i] == 1)
			{
				if(totalVal < pow( (1.0/disMat[curCity][i]), beta ) * pow( (m_dTrial[curCity][i]), alpha) )    
				{
					 totalVal = pow( (1.0/disMat[curCity][i]), beta ) * pow( (m_dTrial[curCity][i]), alpha );
					 j = i;
				}
			}
		}
	}

	return j;
}

void ant::UpdateResult()	// Update the length of tour
{	
	double singleman_dLength = 0;
//	double p = 0.5;
	for(int i=1; i<iCityCount+salesmanNumber; i++)
	{
		m_dLength += disMat[tabu[i-1]][tabu[i]]; //路径消耗
		//tabu禁忌表，存放了已经走过的城市
		if(tabu[i-1] == 0) //为起点时
		{
			singleman_dLength = 0;
		}
		singleman_dLength += disMat[tabu[i-1]][tabu[i]];
		m_tCost = m_tCost + singleman_dLength * timeless[tabu[i]] / man_v; //时效性消耗
	}
	m_cost = q * m_dLength + (1 - q) * m_tCost; //m_cost为总消耗。
	//各路径上信息素：m_tCost为边（i，j）上的信息素量；q和（1-q）分别为路径消耗和时效性消耗的重要性占比
	//m_dLength += disMat[tabu[iCityCount-1]][tabu[0]];
}

void ant::Move()  //the ant move to next town and add town ID to tabu
{
	int j;
	j = ChooseNextCity(); //选择下一个城市node
	addcity(j); //将已选城市加入禁忌表/路径序列中;
}

void ant::speMove()  //the ant move to next town and add town ID to tabu
//tabu, 禁忌表，每只蚂蚁只能走合法路线（经过每个城市1次且仅1次），为此设置禁忌表来控制。
//旅行商只剩下最后两个城市节点需要走时
{
	int j;	
	j = speChooseNextCity();
	addcity(j);
	addcity(0);           //需要将起始点加入到路径中
}

void ant::move2last()
{
	for(int i=0; i<iCityCount; i++)
	{
		if(AllowedCity[i] == 2)
		{
		   addcity(i);
		   break;
		}
	}
}

void ant::Clear()
{	
	int i;
	m_cost = 0;
	m_tCost = 0;
	m_dLength = 0;         //路径长度初始为0
	m_iCityCount = 0;      //已选城市数初始为0
	for(i=0; i<iCityCount; i++)
	{
		prob[i] = 0;         //所有城市概率为0
		AllowedCity[i] = 1;  //所有城市可选
	}
	//初始时将蚂蚁随机放到一个城市上
	int city = 0;	
	//srand( (unsigned)time( NULL ) + rand() );
	//addcity(0);	             //加入起始城市
	//while(city == 0)
	//{
	//	city = rnd(iCityCount);
	//}
	addcity(city);	         //加入第一个城市
}
