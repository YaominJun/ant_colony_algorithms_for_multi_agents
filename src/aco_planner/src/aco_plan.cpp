#include "aco_plan.h"

CvFont font;
void aco_init::init_node() //加载任务点 存放在TotalTaskNode，timeless
{
    std::string full_path = ros::package::getPath("aco_planner") + "/src/Nodes.txt";
    ifstream loadData(full_path);
	double tempX, tempY, tempT;
	int tempF;                   //结束标志
	Node tempNode;
	double factory = realmapHeight/100;
	double factorx = realmapWidth/100;
	while(1)
	{
		loadData>>tempX; //偏离realmap地图中心的x方向距离，单位为200m/100
		loadData>>tempY;
		loadData>>tempT;
		loadData>>tempF;
		tempNode.x = (factorx * tempX + realmapWidth / 2) *  mapWidth/ realmapWidth; //从realmap转化到栅格地图map
		tempNode.y = (factory * (-tempY) + realmapHeight / 2) * mapHeight / realmapHeight;
		TotalTaskNode.push_back(tempNode);

		timeless.push_back(tempT);
		if(tempF == 0)
		{
			loadData.close();
			break;
		}
	}
}

void aco_init::init_plan()  //astar计算距离矩阵 m_aco.disMatfromOut[i][j]
{
    m_astar.InitWalkabilityMap();
	int i=0, j=0;
	ofstream outfile("src/aco_planner/Debug/disMat.txt");
	outfile.setf(ios::showpoint);
	for(i=0; i<TotalTaskNode.size(); i++)
	{
		m_aco.timelessfromOut[i] = timeless.at(i);               //时效性系数赋值，来源于"Nodes.txt"文件中的第三列
		for(j=0; j<TotalTaskNode.size(); j++)
		{
			if(j<i) //保证disMatfromOut矩阵对称性
			{
				m_aco.disMatfromOut[i][j] = m_aco.disMatfromOut[j][i];
				outfile<<setprecision(5)<<m_aco.disMatfromOut[i][j]<<" ";
			}
			else //循环先进入这一步（j=i=0)
			{
				if(m_astar.FindPath(TotalTaskNode.at(i),TotalTaskNode.at(j)) == 1)  //规划路径found = 1, nonexistent = 2, errorputin = 3; // Astar 返回值
				//如果规划出了路径
				{
					m_aco.disMatfromOut[i][j] = m_astar.GetmoveCost();
					outfile<<setprecision(5)<<m_aco.disMatfromOut[i][j]<<" ";
				}
				else
				{
					m_aco.disMatfromOut[i][j] = 10e9;
				}
			}
		}
		outfile<<endl;
	}
	m_aco.copydisMat();
	outfile.close();
}

void aco_init::aco_plan()
{
	//m_astar.InitWalkabilityMap();
	m_aco.StartSearch();                     //关键步骤，用aco算法求解规划问题
	int i = 0, j = 0;
	for(i=0; i<RobotNumber; i++)            //将之前可能在机器人中存在的数据统统清除
	{
		robots[i].nodeSequence.clear();
	}

	for(i=0; i<RobotNumber; )  //将节点序列分给各个机器人 示例节点序列为0 15 6 12 13 7 0 8 9 10 2 1 0 3 11 4 14 5 0 所以这个目的是将0 -> 0 -> 0 -> 0 给三辆车
	{
		for(; j<m_aco.nodeSequence.size(); )
		{
			if(m_aco.nodeSequence.at(j) == 0)
			{
				robots[i].nodeSequence.push_back(m_aco.nodeSequence.at(j));
				j++;
				for(; j<m_aco.nodeSequence.size(); j++)
				{
					robots[i].nodeSequence.push_back(m_aco.nodeSequence.at(j));
					if(m_aco.nodeSequence.at(j) == 0)
					{
						i++;
						break;
					}
				}
			}
			if(i>=RobotNumber)
			{
				break;
			}
		}
	}
}

void aco_init::coordPlan2Map(vector<Node>* pathInPlan)
{
	pathInMap.clear();
	Node temp_node;
	for(int i=pathInPlan->size()-1; i>=0; i--)
	{
		temp_node.x = pathInPlan->at(i).x * m_srcImgWidth / mapWidth;
		temp_node.y = pathInPlan->at(i).y * m_srcImgHeight / mapHeight;
		pathInMap.push_back(temp_node);
	}
}

Node aco_init::coordPlan2Map(Node nodeInPlan)
{
	Node mapNode;
	mapNode.x = nodeInPlan.x * m_srcImgWidth / mapWidth; // mapWidth=200
	mapNode.y = nodeInPlan.y * m_srcImgHeight / mapHeight;
	return mapNode;
}

void aco_init::DrawPathLine(vector<Node> path, CvScalar cvcolor)
{
	int path_thickness = 1;
	CvPoint pt1,pt2;
	if(path.size() < 2)
	{
		return;
	}
	for(int i=0; i<path.size()-1; i++)
	{
		pt1 = cvPoint(path.at(i).x,path.at(i).y);
		pt2 = cvPoint(path.at(i+1).x,path.at(i+1).y);
		cvLine(m_srcImg, pt1, pt2, cvcolor, path_thickness, 4, 0);
	}
 }

 void aco_init::DrawPathPoint(vector<Node> TotalTaskNode, CvScalar cvcolor)
{
	float basePoint_r = 2 * vehicleSize;
	CvPoint pt1;
	Node tempnode;
	for(int i=0; i<TotalTaskNode.size(); i++)
	{
		tempnode = coordPlan2Map(TotalTaskNode.at(i));
		pt1 = cvPoint(tempnode.x, tempnode.y);

		std::string str=std::to_string(i);
		char *p=(char*)str.data();
		cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1.0,1.0,0,1,1);
		cvPutText(m_srcImg,p,pt1,&font,cvScalar(255,0,0,1));

		cvCircle(m_srcImg, pt1, basePoint_r, cvcolor, -1 );
	}
 }

void aco_init::show_map()
{
	cvRobotColor[0] = cvScalar(0,0,255);
	cvRobotColor[1] = cvScalar(0,255,0);
	cvRobotColor[2] = cvScalar(255,0,0);
	cvRobotColor[3] = cvScalar(0,255,255);
	cvRobotColor[4] = cvScalar(255,0,255);

	m_srcImg = cvLoadImage("src/aco_planner/src/m_planImg.jpg",-1);         //把原始图形读进m_srcImg
	if(m_srcImg)
    {
        m_srcImgWidth = m_srcImg->width;
		m_srcImgHeight = m_srcImg->height;
	}
	vector<Node> pathPoint;
	for(int i=0; i<RobotNumber; i++)
	{
		for(int j=0; j<robots[i].nodeSequence.size()-1; j++)
		{
			pathPoint.clear();
			m_astar.FindPath(TotalTaskNode.at(robots[i].nodeSequence.at(j)), TotalTaskNode.at(robots[i].nodeSequence.at(j+1)));
			coordPlan2Map(m_astar.GetPathVector());
			pathPoint = pathInMap;
			robots[i].robotPath = connecte(robots[i].robotPath, pathPoint);
		}
		DrawPathLine(robots[i].robotPath,cvRobotColor[i]);
	}
	DrawPathPoint(TotalTaskNode,cvRobotColor[0]);
	cvSaveImage("src/aco_planner/Debug/result.jpg",m_srcImg);
}

void aco_init::task_nodeCallback(const nav_msgs::Path &task_node_list){
	if(task_node_list.header.frame_id == "task_node"){
		TotalTaskNode.clear();
		Node tempNode;
		for (auto it : task_node_list.poses){
			tempNode.x = it.pose.position.x;
			tempNode.y = it.pose.position.y;
			TotalTaskNode.push_back(tempNode);
			timeless.push_back(0);
			}
		// for(int i=0;i<TotalTaskNode.size();i++){
		// 	std::cout << TotalTaskNode[i].x << " ";
		// }
		// std::cout << std::endl;
		init_plan();
		aco_plan();
		nav_msgs::Path aco_poses;
		aco_poses.poses.clear();
		for(int i=0; i<RobotNumber; i++){
			for(int j=0; j<robots[i].nodeSequence.size(); j++){
				std::cout << robots[i].nodeSequence.at(j) << " ";
				geometry_msgs::PoseStamped tempose;
				string frame = std::to_string(i+1);
				tempose.header.frame_id = frame;
				tempose.pose.position.x = TotalTaskNode[robots[i].nodeSequence.at(j)].x;
				tempose.pose.position.y = TotalTaskNode[robots[i].nodeSequence.at(j)].y;
				aco_poses.poses.push_back(tempose);
				}
				std::cout << std::endl;
	}

	for (int i = 0 ; i < 10 ;  i++){
		pub.publish(aco_poses);
		ros::Duration(0.05).sleep();
	}
	//pub.publish(aco_poses);
	show_map();
	if(aco_poses.poses.size() != 0)
		{
			ros::shutdown();
		}
	}
	// if (nodeSequence.size() != 0){
	// 	break;
	// }
}

void aco_init::init(){
	pub = n.advertise<nav_msgs::Path>("aco_planner_pub",1);
    sub = n.subscribe("task_node_interface",10,&aco_init::task_nodeCallback,this);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"aco_planner");
	aco_init Aco_Init;
	Aco_Init.init();

	ros::Rate loop_rate(1.0);
	while(ros::ok()){

		ros::spinOnce();

	loop_rate.sleep();
	 }
}