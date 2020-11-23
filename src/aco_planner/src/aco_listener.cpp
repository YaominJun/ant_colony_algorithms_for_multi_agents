#include "aco_plan.h"

void task_node_plan_Callback(const nav_msgs::Path &task_node_list){
	// if(task_node_list.header.frame_id == "task_node"){
		double tempNode;
		for (auto it : task_node_list.poses){
			tempNode = it.pose.position.x;
			std::cout << tempNode << " ";
			}
			std::cout << std::endl;
	// }
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"aco_planner_listener");	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("aco_planner_pub",10,task_node_plan_Callback);

	ros::Rate loop_rate(1.0);
	while(ros::ok()){

		ros::spinOnce();
    
	loop_rate.sleep();
	 }	
}