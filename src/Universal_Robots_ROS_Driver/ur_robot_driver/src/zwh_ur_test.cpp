#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

#include <moveit/trajectory_processing/time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

const double trans = 180 / M_PI;

int main(int argc, char **argv)
{
/*ROS初始化*/
  ros::init(argc, argv, "movo_moveit");//不太清楚为什么是move_moveit
  ros::NodeHandle node_handle; 
  setlocale(LC_ALL, "");//使ROS_INFO可以正常显示中文

/*开启指针：获取机器人状态*/
  ros::AsyncSpinner spinner(1);
  spinner.start();

/*规划组设置*/
  static const std::string PLANNING_GROUP = "manipulator";//规划组设置为manipulator
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

/*可视化设置*/
  namespace rvt=rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

/*参数申明*/
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;//存放规划结果

  bool success = false;//轨迹解算是否成功的标志量

  moveit_msgs::RobotTrajectory trajectory;//存放轨迹

  const double jump_threshold = 0;//跳跃阈值
  const double eef_step = 0.01;//末端执行器步距
  double fraction = 0;//笛卡尔路径计算返回值，范围0～1，如果错误返回-1.0

  int max_calculation = 100;//最大计算次数
  int current_calculation = 0;//当前计算次数

  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "manipulator");//TOTG算法轨迹对象

  cout << "初始化完成" << endl;

/*第一步*/
/*
  visual_tools.prompt("移动至Start姿态");

  move_group_interface.setNamedTarget("start");
  move_group_interface.move();

  visual_tools.prompt("点击next进行第1步");
  //设置目标点姿态
  geometry_msgs::Pose test_pose;
  test_pose.position.x = 0.25;
  test_pose.position.y = 0.25;
  test_pose.position.z = 0.15;
  test_pose.orientation.x = 1;
  test_pose.orientation.y = 0;
  test_pose.orientation.z = 0;
  test_pose.orientation.w = 1;
  move_group_interface.setPoseTarget(test_pose);  

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("zwh", "测试部分的规划结果： %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("zwh", "显示测试部分的轨迹点连线");
  //visual_tools.publishAxisLabeled(test_pose, "pose1");//打印第一个点坐标系的名称“pose1”
  //visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);//打印“Pose Gaol”字样
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  visual_tools.prompt("点击next执行规划");

  move_group_interface.execute(my_plan);

  visual_tools.prompt("测试结束 点击next进行下一步");
*/

/*第二步*/
/*
  visual_tools.prompt("第二步:点击next返回start姿态");

  move_group_interface.setNamedTarget("start");
  move_group_interface.move();

  visual_tools.prompt("点击next导入姿态点waypoint");

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose Start_Pose;
  Start_Pose.orientation.x = 0;
  Start_Pose.orientation.y = 0;
  Start_Pose.orientation.z = 1;
  Start_Pose.orientation.w = -0.2;
  Start_Pose.position.x = 0.3;
  Start_Pose.position.y = 0.3;
  Start_Pose.position.z = 0.2;
  waypoints.push_back(Start_Pose);

  geometry_msgs::Pose Current_Pose;
  Current_Pose = Start_Pose;

  Current_Pose.position.x -= 0.05;
  waypoints.push_back(Current_Pose);//姿态1

  Current_Pose.position.y -= 0.05;
  waypoints.push_back(Current_Pose);//姿态2

  Current_Pose.position.x -= 0.05;
  Current_Pose.position.y -= 0.05;
  Current_Pose.position.z += 0.1;
  waypoints.push_back(Current_Pose);//姿态3

  visual_tools.prompt("点击next进入轨迹解算");

  jump_threshold = 0.1;
  eef_step = 0.03;
  fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("可视化Cartesian path成功进度: %.2f%%", fraction * 100.0);

  //my_plan.trajectory_=trajectory;

  visual_tools.deleteAllMarkers();
  //visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);//显示一系列相连圆柱标记，waypoint是用于圆柱体连接的一系列点
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools.trigger();
  
  visual_tools.prompt("点击next执行轨迹");

  move_group_interface.execute(trajectory);
*/

/*第三步*/
/*
  visual_tools.prompt("第三步:点击next返回start姿态");

  move_group_interface.setNamedTarget("start");
  move_group_interface.move();

  move_group_interface.setMaxVelocityScalingFactor(0.03);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  visual_tools.prompt("点击next导入姿态点waypoint");

  std::vector<geometry_msgs::Pose> waypoints_2nd;
  
  geometry_msgs::Pose Start_Pose_2nd;
  Start_Pose_2nd.position.x = 0.3;
  Start_Pose_2nd.position.y = 0.3;
  Start_Pose_2nd.position.z = 0.2;
  Start_Pose_2nd.orientation.x = 1;
  Start_Pose_2nd.orientation.y = 0;
  Start_Pose_2nd.orientation.z = 0;
  Start_Pose_2nd.orientation.w = 0;
  visual_tools.deleteAllMarkers();
  for(int i=0;i<50;i++)
  {
    waypoints_2nd.push_back(Start_Pose_2nd);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.publishPath(waypoints_2nd, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishAxisLabeled(waypoints_2nd[i], "pt" + std::to_string(i), rvt::SMALL);
    
    Start_Pose_2nd.position.x += 0.01;
    Start_Pose_2nd.orientation.w += 0.03;
    ROS_INFO("current times: %d, w = %lf",i,Start_Pose_2nd.orientation.w);
  }
  move_group_interface.computeCartesianPath(waypoints_2nd, eef_step, jump_threshold, trajectory);
  visual_tools.trigger();
  move_group_interface.execute(trajectory);
*/

/*第四步*//*第四步*//*第四步*/
/*第四步*//*第四步*//*第四步*/
/*第四步*//*第四步*//*第四步*/
  visual_tools.prompt("初始姿态");
  move_group_interface.setNamedTarget("my_pose");
  move_group_interface.move();
  move_group_interface.setMaxVelocityScalingFactor(0.03);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);
  cout << "姿态完成" << endl;

  visual_tools.prompt("计算");
  
  geometry_msgs::Pose Start_Pose_4th;
  Start_Pose_4th.position.x = 0.6;
  Start_Pose_4th.position.y = -0.03;
  Start_Pose_4th.position.z = 0.35;
  Start_Pose_4th.orientation.w = 0;
  Start_Pose_4th.orientation.x = 0;
  Start_Pose_4th.orientation.y = 1;
  Start_Pose_4th.orientation.z = 0;

  std::vector<geometry_msgs::Pose> waypoints_4th;

  double dx = 0.015;
  int step = 10;

  for(int i = 0; i < step; i++)
  {
    //waypoints_4th.push_back(Start_Pose_4th);

    ROS_INFO("第%d组, 位置: (%lf, %lf, %lf), 姿态: (%lf, %lf, %lf, %lf)",
    i,
    Start_Pose_4th.position.x, Start_Pose_4th.position.y, Start_Pose_4th.position.z,
    Start_Pose_4th.orientation.w, Start_Pose_4th.orientation.x, Start_Pose_4th.orientation.y, Start_Pose_4th.orientation.z);

    Start_Pose_4th.position.x += dx;
    //Start_Pose_4th.orientation.w += 0.03;

    waypoints_4th.push_back(Start_Pose_4th);
  }

  fraction = move_group_interface.computeCartesianPath(waypoints_4th, eef_step, jump_threshold, trajectory);

  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization IPTP;// 五次样条曲线插补
  trajectory_processing::IterativeSplineParameterization ISP;// 三次样条曲线
  trajectory_processing::TimeOptimalTrajectoryGeneration TOTG;
  
  success = TOTG.computeTimeStamps(rt, 1.0, 1.0);
  ROS_INFO("计算时间辍： %s",success?"成功":"失败");
  rt.getRobotTrajectoryMsg(trajectory);

  my_plan.trajectory_ = trajectory;

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.publishPath(waypoints_4th, rvt::PINK, rvt::XXXSMALL);
  for (std::size_t j = 0; j < waypoints_4th.size(); ++j)
  {
    visual_tools.publishAxisLabeled(waypoints_4th[j], "Point" + std::to_string(j), rvt::XXXSMALL);
  }
  visual_tools.trigger();

  visual_tools.prompt("执行轨迹");
  visual_tools.prompt("确定执行");

  if(fraction != -1.0)
  {
	  move_group_interface.execute(trajectory);
	  ROS_INFO("路径执行完成");
  }
  else ROS_INFO("路径规划失败");

/*结束*/
  ros::shutdown();
  return 0;
}