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
  group.setNamedTarget("start");
  group.move();
  sleep(5);
 
// ---------------------声明发送的数据，对应于moveit中的拖拽----------------
  geometry_msgs::Pose target_pose;

//---------------------声明轨迹数列-----------------------------
  std::vector<geometry_msgs::Pose>waypoints;
/*
//###################################################测量工作范围#########################

  double x=0.4;
  double fraction=0.0;
  int maxtries=100;
  int  attempts=0;
  int success=1;
  group.setMaxVelocityScalingFactor(0.02);
  target_pose.orientation.x= 0;
  target_pose.orientation.y = 1;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 0;
  target_pose.position.z = 0.4;
  target_pose.position.x = x;
  target_pose.position.y = 0.1;
//----------------------------声明trajectory对象--------------------
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  while(success==1)
  {
    x=x+0.02;
    target_pose.position.x = x;
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
  }
//###################################################测量工作范围#########################
*/

//#################################从文件读取数据运动new#####################################
  ifstream f;
  f.open("/mnt/hgfs/ubantushare/data.txt");
  string str;
  vector<vector<int>>pos; 
  int over=0;
  
  while(getline(f,str))
  {
    istringstream input(str);
    vector<int>pos0;
    int a;
    while(input>>a)
        pos0.push_back(a);

    pos.push_back(pos0);
  }
  target_pose.orientation.x= 0
  ;
  target_pose.orientation.y = 1;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 0;
  target_pose.position.z = 0.35;
  target_pose.position.x = 0.4;
  target_pose.position.y = -0.1;
  for(int i=0;i<pos.size();++i)
  { 
  if(pos[i][0]==0&&pos[i][1]==0)
    {
	  over=1;
    }
  else
    {
	  if(over==1)
		{
		  target_pose.position.z = 0.45;
		  waypoints.push_back(target_pose);
		  target_pose.position.x = (pos[i][0])*0.001;
  		  target_pose.position.y = (pos[i][1]-150)*0.001;
		  waypoints.push_back(target_pose);
		  target_pose.position.z = 0.35;
		  waypoints.push_back(target_pose);
                  over=0;
		}
	  else
		{
		  target_pose.position.x = (pos[i][0])*0.001;
  		  target_pose.position.y = (pos[i][1]-150)*0.001;
		  waypoints.push_back(target_pose);
		}
     }
  cout<<target_pose.position.x<<" ";
  cout<<target_pose.position.y<<" ";
  cout<<target_pose.position.z<<" ";
  cout<<endl;
  }

//#################################从文件读取数据运动new########################################

/*
//#################################从文件读取数据运动#####################################
  ifstream f;
  f.open("/mnt/hgfs/paint/data.txt");
  string str;
  vector<vector<int>>pos;
  int over=0;
  
  while(getline(f,str))
  {
    istringstream input(str);
    vector<int>pos0;
    int a;
    while(input>>a)
        pos0.push_back(a);

    pos.push_back(pos0);
  }
  target_pose.orientation.x= 0;
  target_pose.orientation.y = 1;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 0;
  target_pose.position.z = 0.35;
  target_pose.position.x = 0.4;
  target_pose.position.y = -0.1;
  for(int i=0;i<pos.size();++i)
  { 
  if(pos[i][0]==0)
    {
	  over=1;
    }
  else
    {
	  if(over==1)
		{
		  target_pose.position.z = 0.45;
		  waypoints.push_back(target_pose);
		  target_pose.position.x = (pos[i][0])*0.001;
  		  target_pose.position.y = (pos[i][1]-150)*0.001;
		  waypoints.push_back(target_pose);
		  target_pose.position.z = 0.35;
		  waypoints.push_back(target_pose);
                  over=0;
		}
	  else
		{
		  target_pose.position.x = (pos[i][0])*0.001;
  		  target_pose.position.y = (pos[i][2]-150)*0.001;
		  waypoints.push_back(target_pose);
		}
     }
  cout<<target_pose.position.x<<" ";
  cout<<target_pose.position.y<<" ";
  cout<<target_pose.position.z<<" ";
  cout<<pos[i][0]<<" ";
  cout<<pos[i][1]<<" ";
  cout<<pos[i][2]<<" ";
  cout<<pos[i][3]<<" ";
  cout<<endl;
  }
  ofstream file_writer("/mnt/hgfs/paint/data.txt", ios_base::out);
//#################################从文件读取数据运动########################################
*/
/*
//########################################给定点运动########################################
//----------------------------声明trajectory对象--------------------
//---------------------目标坐标1----------------------
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x= 0;
  target_pose1.orientation.y = 1;
  target_pose1.orientation.z = 0;
  target_pose1.orientation.w = 0;
 
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.1;
  target_pose1.position.z = 0.4;
 

  waypoints.push_back(target_pose1);
 group.setMaxVelocityScalingFactor(0.02);
  //group.setPoseTarget(target_pose1);
  // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
  //bool success = group.plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS;;
  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   
  //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
  //if(success)
      //group.execute(my_plan);
 
//---------------------目标坐标2----------------------
geometry_msgs::Pose target_pose2;

  target_pose2.orientation.x= 0;
  target_pose2.orientation.y = 1;
  target_pose2.orientation.z = 0;
  target_pose2.orientation.w = 0;
 
  target_pose2.position.x =0.6;
  target_pose2.position.y =0.1;
  target_pose2.position.z = 0.4;
 
  waypoints.push_back(target_pose2);
  group.setMaxVelocityScalingFactor(0.02);
  //group.setPoseTarget(target_pose2);
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
  //bool success = (ptr_group->plan(my_plan_1) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  //bool success1 = group.plan(my_plan_1)== moveit::planning_interface::MoveItErrorCode::SUCCESS;;
  //ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");   
  //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
  //if(success1)
      //group.execute(my_plan_1);
//---------------------目标坐标3----------------------
geometry_msgs::Pose target_pose3;

   target_pose3.orientation.x= 0;
  target_pose3.orientation.y = 1;
  target_pose3.orientation.z = 0;
  target_pose3.orientation.w = 0;
 
  target_pose3.position.x = 0.8;
  target_pose3.position.y = 0.1;
  target_pose3.position.z = 0.4;
 
  waypoints.push_back(target_pose3);
  group.setMaxVelocityScalingFactor(0.01);
  //group.setPoseTarget(target_pose3);
  // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
  //bool success = (ptr_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  //bool success2 = group.plan(my_plan_2)== moveit::planning_interface::MoveItErrorCode::SUCCESS;;
  //ROS_INFO("Visualizing plan 3 (pose goal) %s",success?"":"FAILED");   
  //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
  //if(success2)
      //group.execute(my_plan_2);

//########################################给定点运动########################################
*/

//#############################################计算#################################
//----------------------初始化计算cartesian轨迹的参数----------------------
  double fraction=0.0;
  int maxtries=100;
  int  attempts=0;
  group.setMaxVelocityScalingFactor(0.02);
//----------------------------声明trajectory对象--------------------
  moveit_msgs::RobotTrajectory trajectory;

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

//-------------------------声明moveit plan对象-----------------------
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_=trajectory;

//------------------------------解算成功执行计划----------------------------
  if(fraction==1.0)
  {
	  ROS_INFO("path computed succesfully after%d times,moving the arm%f",attempts,fraction);
	  group.execute(my_plan);
	  visual_tools.publishPath(waypoints,rvt::LIME_GREEN,rvt::SMALL);
	  visual_tools.trigger();
	  sleep(5);
	  ROS_INFO("path execution complete");
  }
  else ROS_INFO("path planning filed with only %f success after %d attempts",fraction,maxtries);
  group.setNamedTarget("start");
  group.move();
  sleep(5);
  ros::shutdown(); 
  return 0;
//#############################################计算#################################

}



