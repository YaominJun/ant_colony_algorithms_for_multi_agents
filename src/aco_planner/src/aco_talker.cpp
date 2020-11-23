#include "aco_plan.h"


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




int main(int argc, char **argv)
{
	ros::init(argc,argv,"aco_planner_talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<nav_msgs::Path>("task_node_interface",1);
	
	ros::Rate loop_rate(1.0);

    aco_init Aco_Init;
	Aco_Init.init_node();
	while (ros::ok())
	{
		nav_msgs::Path aco_poses_talker;
		aco_poses_talker.header.frame_id = "task_node";

		for(int i=0; i<Aco_Init.TotalTaskNode.size();i++){
			//std::cout << Aco_Init.TotalTaskNode.at(i).x << " ";
			geometry_msgs::PoseStamped tempose;
			tempose.pose.position.x = Aco_Init.TotalTaskNode.at(i).x ;
			tempose.pose.position.y = Aco_Init.TotalTaskNode.at(i).y ;
			aco_poses_talker.poses.push_back(tempose);
			}
		pub.publish(aco_poses_talker);
		std::cout << aco_poses_talker << std::endl;
		
		loop_rate.sleep();
		ros::spinOnce();
	}
	
}