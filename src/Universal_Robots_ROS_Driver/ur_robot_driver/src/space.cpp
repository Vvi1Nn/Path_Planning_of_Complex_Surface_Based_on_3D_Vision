// https://blog.csdn.net/weixin_39312052/article/details/88130730
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

#include<moveit_visual_tools/moveit_visual_tools.h>
#include<rviz_visual_tools/rviz_visual_tools.h>

#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>

using namespace std;

int main(int argc, char **argv)
{
//---------------------------初始化----------------------------------------
  ros::init(argc, argv, "movo_moveit");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1);
  spinner.start();

//----------------------------用于visualtools可视化工具---------------------
  namespace rvt=rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

//-----------------------设置MOVEIT的规划组------------------------------------
  moveit::planning_interface::MoveGroupInterface group("manipulator");//ur5对应moveit中选择的规划部分
  
//---------------------移动到初始位置---------------------------------------
  group.setNamedTarget("home");
  group.move();
  sleep(5);
  group.setNamedTarget("start");
  group.move();
  sleep(5);
 
// ---------------------声明发送的数据，对应于moveit中的拖拽----------------
  geometry_msgs::Pose target_pose;

//---------------------声明轨迹数列-----------------------------
  std::vector<geometry_msgs::Pose>waypoints;

//###################################################测量工作范围#########################

  double x_max=0.4;
  double x_min=0.4;
  double y_max=0.1;
  double y_min=0.1;
  double fraction=0.0;
  int maxtries=100;
  int  attempts=0;
  int success=1;
  group.setMaxVelocityScalingFactor(0.02);
  target_pose.orientation.x= 0;
  target_pose.orientation.y = 1;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 0;
  target_pose.position.z = 0.2;
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.1;
//----------------------------声明trajectory对象--------------------
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//---------------------------------------------x_max---------------------------------------

    
    target_pose.position.y = 0.6;
    waypoints.push_back(target_pose);
    target_pose.position.x = 0.6;
    waypoints.push_back(target_pose);
    target_pose.position.y = -0.6;
    waypoints.push_back(target_pose);
    target_pose.position.x = 0.2;
    waypoints.push_back(target_pose);
    target_pose.position.y = 0.1;
    waypoints.push_back(target_pose);
//-----------------------------开始解算轨迹----------------------
    while(fraction<1.0&&attempts<maxtries)
    {
	  fraction=group.computeCartesianPath(waypoints,
					  0.01, //终端步进制
					  0.0,//跳跃阀值
					  trajectory,
					  true,NULL);//避障规划
							
	  attempts+=1;
	  if(attempts %10==0)
		  ROS_INFO("still tring after %d attempts",attempts);
	  }
   
   my_plan.trajectory_=trajectory;
 
   if(fraction==1.0)
    {
	  
          ROS_INFO("path computed succesfully after%d times with x=%f",attempts,target_pose.position.x);
          group.execute(my_plan);
    }
    else {ROS_INFO("path planning filed after %d attempts with x_Max=%f",maxtries,target_pose.position.x);
          success=0;}
    waypoints.clear();
    fraction=0;
    attempts=0;
//###################################################测量工作范围#########################
  group.setNamedTarget("start");
  group.move();
  sleep(5);
  ros::shutdown(); 
  return 0;
//#############################################计算#################################

}



