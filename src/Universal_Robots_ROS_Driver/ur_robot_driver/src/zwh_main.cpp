/*---------------------------------------- C++ ----------------------------------------*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <math.h>

/*---------------------------------------- 机械臂 ----------------------------------------*/
/*moveit*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 

/*轨迹算法*/
#include <moveit/trajectory_processing/time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

/*可视化*/
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

/*---------------------------------------- PCL ----------------------------------------*/
/*PCL_ROS库*/
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

/*PCL1.12库*/
#include <pcl/point_types.h>//点类型
#include <pcl/io/pcd_io.h>//读写类
#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>//直通滤波
#include <pcl/filters/voxel_grid.h>//体素滤波
#include <pcl/filters/uniform_sampling.h>//均匀采样
#include <pcl/filters/conditional_removal.h>//条件滤波
#include <pcl/filters/radius_outlier_removal.h>//半径滤波

#include <pcl/visualization/pcl_visualizer.h>//可视化
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/kdtree/kdtree_flann.h>//KD树
#include <pcl/features/normal_3d_omp.h>//法向量



/*---------------------------------------- 全局定义(参数+类) ----------------------------------------*/
namespace RVT = rviz_visual_tools;

ros::Publisher PCL_Publisher;

const bool test = true;

/*规划组*/
moveit::planning_interface::MoveGroupInterface::Plan My_Plan;//规划结果

/*姿态和路径点*/
geometry_msgs::Pose Waypose;//姿态
std::vector<geometry_msgs::Pose> Waypoints;//路径点

/*计算笛卡尔路径*/
bool success = false;//轨迹解算成功标志量
moveit_msgs::RobotTrajectory Trajectory;//存放轨迹
const double jump_threshold = 0;//跳跃阈值
const double eef_step = 0.01;//末端执行器步距
double fraction = 0;//笛卡尔路径计算返回值，范围0～1，如果错误，返回-1.0

/*轨迹优化算法*/
trajectory_processing::IterativeParabolicTimeParameterization IPTP;//五次样条Class
trajectory_processing::IterativeSplineParameterization ISP;// 三次样条Class
trajectory_processing::TimeOptimalTrajectoryGeneration TOTG;//时间最优Class

/*点云*/
pcl::PCDReader Reader;//PCD读取Class
pcl::PCDWriter Writer;//PCD写入Class

pcl::PointCloud<pcl::PointXYZ>::Ptr Initial_Cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);//初始点云

pcl::UniformSampling<pcl::PointXYZ> US;//均匀采样Class
pcl::PointCloud<pcl::PointXYZ>::Ptr US_Cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);//均匀采样点云
pcl::PointXYZ US_Cloud_Min;//3D边界最小值
pcl::PointXYZ US_Cloud_Max;//3D边界最大值

pcl::ConditionalRemoval<pcl::PointXYZ> CR;//条件滤波Class
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_0(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_1(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_2(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_3(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_4(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_5(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_6(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_7(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_8(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_9(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_10(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_11(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_12(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::ConditionAnd<pcl::PointXYZ>::Ptr Range_Condition_13(new pcl::ConditionAnd<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_4(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_5(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_6(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_7(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_8(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_9(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_10(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_11(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_12(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud_13(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CR_Cloud(new pcl::PointCloud<pcl::PointXYZ>);

pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> NE;//法线计算Class
pcl::search::KdTree<pcl::PointXYZ>::Ptr Kd_Tree(new pcl::search::KdTree<pcl::PointXYZ>);//法向量计算方法
pcl::PointCloud<pcl::Normal>::Ptr Normals(new pcl::PointCloud<pcl::Normal>);//法线数据(Nx, Ny, Nz, Curvature)

sensor_msgs::PointCloud2 PCL_Output;

static const std::string FRAME = "base";

void OrganizeAndPush_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, unsigned long int i, const bool write);
void RemoveNaN_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const bool write);
bool RemovePoint_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int number, const bool write);
geometry_msgs::Quaternion NormalsToQuaternion_Zwh(pcl::PointCloud<pcl::Normal>::Ptr Normal, int num);


/*---------------------------------------- 主函数 ----------------------------------------*/
int main(int argc, char** argv)
{
    /*-------------------- ROS初始化 --------------------*/
    ros::init(argc, argv, "zwh_main");
    ros::NodeHandle Node_Handle; 

    setlocale(LC_ALL, "");

    /*-------------------- 创建话题 --------------------*/
    PCL_Publisher = Node_Handle.advertise <sensor_msgs::PointCloud2> ("zwh_main_pcl", 1);

    /*-------------------- 开启指针：获取机器人状态 --------------------*/
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*-------------------- 规划组设置+信息显示 --------------------*/
    moveit::planning_interface::MoveGroupInterface Move_Group("manipulator");
    const moveit::core::JointModelGroup* Joint_Model_Group = Move_Group.getCurrentState()->getJointModelGroup("manipulator");

    ROS_INFO("UR5移动组信息如下:");
    ROS_INFO("Planning frame: %s", Move_Group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", Move_Group.getEndEffectorLink().c_str());
    ROS_INFO("Available Planning Groups:");
    std::copy(Move_Group.getJointModelGroupNames().begin(),
              Move_Group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
    cout << " " << endl;

    /*-------------------- 可视化设置 --------------------*/
    moveit_visual_tools::MoveItVisualTools VT(FRAME);
    VT.deleteAllMarkers();
    VT.trigger();
    
    /*-------------------- 读取点云 --------------------*/
    ROS_INFO("读取点云");
    Reader.read("/home/zwh/blade_100w.pcd", *Initial_Cloud_XYZ);

    /*-------------------- 均匀采样 --------------------*/
    ROS_INFO("均匀采样");
    US.setInputCloud(Initial_Cloud_XYZ);
    US.setRadiusSearch(0.01f);
    US.filter(*US_Cloud_XYZ);

    //Writer.write("/home/zwh/zwh_main_data/US_Cloud_XYZ.pcd", *US_Cloud_XYZ);

    pcl::getMinMax3D(*US_Cloud_XYZ, US_Cloud_Min, US_Cloud_Max);

    /*-------------------- 条件滤波+排序 --------------------*/
    ROS_INFO("条件滤波+排序");

    const double step = (US_Cloud_Max.x - US_Cloud_Min.x) / 13;
    const double section = 5.0;
    double CR_min[14], CR_max[14];

    for(int i = 0; i < 14; i++)
    {
        CR_min[i] = US_Cloud_Min.x + step * i - step / section;
        CR_max[i] = US_Cloud_Min.x + step * i + step / section;

        CR.setInputCloud(US_Cloud_XYZ);
        CR.setKeepOrganized(false);

        switch(i)
        {
            case 0:
            {
                Range_Condition_0->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_0->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_0);
                CR.filter(*CR_Cloud_0);
                OrganizeAndPush_Zwh(CR_Cloud_0, i, false);
                if(CR_Cloud_0->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_0.pcd", *CR_Cloud_0);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 1:
            {
                Range_Condition_1->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_1->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_1);
                CR.filter(*CR_Cloud_1);
                OrganizeAndPush_Zwh(CR_Cloud_1, i, false);
                if(CR_Cloud_1->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_1.pcd", *CR_Cloud_1);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 2:
            {
                Range_Condition_2->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_2->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_2);
                CR.filter(*CR_Cloud_2);
                OrganizeAndPush_Zwh(CR_Cloud_2, i, false);
                if(CR_Cloud_2->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_2.pcd", *CR_Cloud_2);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 3:
            {
                Range_Condition_3->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_3->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_3);
                CR.filter(*CR_Cloud_3);
                OrganizeAndPush_Zwh(CR_Cloud_3, i, false);
                if(CR_Cloud_3->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_3.pcd", *CR_Cloud_3);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 4:
            {
                Range_Condition_4->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_4->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_4);
                CR.filter(*CR_Cloud_4);
                OrganizeAndPush_Zwh(CR_Cloud_4, i, false);
                if(CR_Cloud_4->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_4.pcd", *CR_Cloud_4);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 5:
            {
                Range_Condition_5->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_5->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_5);
                CR.filter(*CR_Cloud_5);
                OrganizeAndPush_Zwh(CR_Cloud_5, i, false);
                if(CR_Cloud_5->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_5.pcd", *CR_Cloud_5);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 6:
            {
                Range_Condition_6->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_6->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_6);
                CR.filter(*CR_Cloud_6);
                OrganizeAndPush_Zwh(CR_Cloud_6, i, false);
                if(CR_Cloud_6->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_6.pcd", *CR_Cloud_6);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 7:
            {
                Range_Condition_7->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_7->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_7);
                CR.filter(*CR_Cloud_7);
                OrganizeAndPush_Zwh(CR_Cloud_7, i, false);
                if(CR_Cloud_7->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_7.pcd", *CR_Cloud_7);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 8:
            {
                Range_Condition_8->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_8->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_8);
                CR.filter(*CR_Cloud_8);
                OrganizeAndPush_Zwh(CR_Cloud_8, i, false);
                if(CR_Cloud_8->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_8.pcd", *CR_Cloud_8);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 9:
            {
                Range_Condition_9->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_9->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_9);
                CR.filter(*CR_Cloud_9);
                OrganizeAndPush_Zwh(CR_Cloud_9, i, false);
                if(CR_Cloud_9->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_9.pcd", *CR_Cloud_9);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 10:
            {
                Range_Condition_10->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_10->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_10);
                CR.filter(*CR_Cloud_10);
                OrganizeAndPush_Zwh(CR_Cloud_10, i, false);
                if(CR_Cloud_10->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_10.pcd", *CR_Cloud_10);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 11:
            {
                Range_Condition_11->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_11->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_11);
                CR.filter(*CR_Cloud_11);
                OrganizeAndPush_Zwh(CR_Cloud_11, i, false);
                if(CR_Cloud_11->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_11.pcd", *CR_Cloud_11);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 12:
            {
                Range_Condition_12->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_12->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_12);
                CR.filter(*CR_Cloud_12);
                OrganizeAndPush_Zwh(CR_Cloud_12, i, false);
                if(CR_Cloud_12->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_12.pcd", *CR_Cloud_12);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            case 13:
            {
                Range_Condition_13->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::GT, CR_min[i])));//GT: >=
                Range_Condition_13->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>
                    ("x", pcl::ComparisonOps::LT, CR_max[i])));//LT: <=
                CR.setCondition(Range_Condition_13);
                CR.filter(*CR_Cloud_13);
                OrganizeAndPush_Zwh(CR_Cloud_13, i, true);
                if(CR_Cloud_13->points.size()>0) Writer.write("/home/zwh/zwh_main_data/CR_Cloud_13.pcd", *CR_Cloud_13);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            default: cout << "条件滤波: Case Error" << endl; break;
        }
    }

    /*-------------------- 计算法线 --------------------*/
    ROS_INFO("计算法线");

    NE.setNumberOfThreads(10);//openMP线程数
    NE.setInputCloud(CR_Cloud_0);
    NE.setSearchSurface(Initial_Cloud_XYZ);
    NE.setSearchMethod(Kd_Tree);
    NE.setRadiusSearch(0.005);
    //NE.setKSearch(5);
    NE.compute(*Normals);

    for(unsigned long int num = 0; num < Normals->points.size(); num++)
    {
        if(Normals->points[num].normal_z > 0)
        {
            Normals->points[num].normal_x = -Normals->points[num].normal_x;
            Normals->points[num].normal_y = -Normals->points[num].normal_y;
            Normals->points[num].normal_z = -Normals->points[num].normal_z;
        }
    }

    Writer.write("/home/zwh/zwh_main_data/Normals.pcd", *Normals);

    /*-------------------- PCL可视化 --------------------*/
    const bool PCL_Viewer = false;
    if(PCL_Viewer)
    {
        if(!test) VT.prompt("PCL可视化");
        else ROS_INFO("PCL可视化");

        boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer(new pcl::visualization::PCLVisualizer("Zwh Viewer"));

        Viewer->setWindowName("Zwh-Main-Viewer");//创建视窗对象，定义标题栏名称
        Viewer->setBackgroundColor(0.3, 0.3, 0.3);
        Viewer->addCoordinateSystem(0.5);
        Viewer->addText("Zwh_Main", 50, 50, 0, 1, 0, "v1_text");
        Viewer->addPointCloud<pcl::PointXYZ>(CR_Cloud, "Zwh-XYZNormal");//将点云添加到视窗对象中，并定义一个唯一的ID

        //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(CR_Cloud, "x");
	    //Viewer->addPointCloud(CR_Cloud, rgb, "sample cloud");

        Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "Zwh-XYZNormal");//点云附色，三个字段，每个字段范围0-1
        Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Zwh-XYZNormal");//点云大小6
        Viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(CR_Cloud, Normals, 1, 0.01, "Normals");//每1个点显示一个法线，长度为0.005

        while (!Viewer->wasStopped())
        {
            Viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    /*-------------------- UR初始位置 --------------------*/
    VT.prompt("Start姿态");

    Move_Group.setNamedTarget("my_pose");
    Move_Group.move();
    Move_Group.setMaxVelocityScalingFactor(0.2);
    Move_Group.setMaxAccelerationScalingFactor(0.1);

    /*-------------------- 发布点云 --------------------*/
    VT.prompt("发布点云");

    sensor_msgs::PointCloud2 PCL_Output;
    pcl::toROSMsg(*CR_Cloud_0, PCL_Output);
    PCL_Output.header.frame_id = FRAME;
    ros::Rate loop_rate(1);
    PCL_Publisher.publish(PCL_Output);
    ros::spinOnce();
    loop_rate.sleep();

    /*-------------------- 计算姿态+导入轨迹点 --------------------*/
    VT.prompt("计算姿态+导入轨迹点");

    // Waypose.orientation.w = 0;
    // Waypose.orientation.x = 0;
    // Waypose.orientation.y = 1;
    // Waypose.orientation.z = 0;

    //测试
    bool orientation_test = false;  
    if(orientation_test)
    {
        double ex,ey,ez;
        geometry_msgs::Quaternion q_msg;
        double roll,pitch,yaw;
        Normals->points[0].normal_x = 0;
        Normals->points[0].normal_y = 0;
        Normals->points[0].normal_z = 0;
        char a;
        while(1)
        {
            cin >> Normals->points[0].normal_x >> Normals->points[0].normal_y >> Normals->points[0].normal_z >> a;
            ex = acos(
                sqrt(
                    (Normals->points[0].normal_x * Normals->points[0].normal_x + Normals->points[0].normal_z * Normals->points[0].normal_z) /
                    (Normals->points[0].normal_x * Normals->points[0].normal_x + Normals->points[0].normal_y * Normals->points[0].normal_y + Normals->points[0].normal_z * Normals->points[0].normal_z)));
            if(Normals->points[0].normal_y > 0) ex = 2 * 3.1415926535 - ex;
            ey = atan2(Normals->points[0].normal_x, Normals->points[0].normal_z);
            if(Normals->points[0].normal_y < 0) ey += 3.1415926535;
            if(Normals->points[0].normal_x < 0) ey += 3.1415926535;
            ez = 0;
            q_msg = tf::createQuaternionMsgFromRollPitchYaw(ex, ey, ez);
            cout << "q_msg.w= " << q_msg.w << "  q_msg.x= " << q_msg.x << "  q_msg.y= " << q_msg.x << "  q_msg.z= " << q_msg.z << endl;
            Waypose.orientation.w = q_msg.w;
            Waypose.orientation.x = q_msg.x;
            Waypose.orientation.y = q_msg.y;
            Waypose.orientation.z = q_msg.z;
            tf::Quaternion q;
            tf::quaternionMsgToTF(Waypose.orientation, q);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            cout << "roll= " << roll << "  pitch= " << pitch << "  yaw= " << yaw << endl;
            if(a == 'n') break;
        }
        VT.prompt("测试等待");
    }
    
    //实际
    else
    {
        geometry_msgs::Quaternion Q;
        for(unsigned long int i = 0; i < CR_Cloud_0->points.size(); i++)
        {
            Q = NormalsToQuaternion_Zwh(Normals, i);
            Waypose.orientation.w = Q.w;
            Waypose.orientation.x = Q.x;
            Waypose.orientation.y = Q.y;
            Waypose.orientation.z = Q.z;
            Waypose.position.x = CR_Cloud_0->points[i].x;
            Waypose.position.y = CR_Cloud_0->points[i].y;
            Waypose.position.z = CR_Cloud_0->points[i].z;
            Waypoints.push_back(Waypose);
        }
    }

    /*-------------------- 轨迹算法 --------------------*/
    VT.prompt("轨迹算法");

    fraction = Move_Group.computeCartesianPath(Waypoints, eef_step, jump_threshold, Trajectory);

    robot_trajectory::RobotTrajectory RT(Move_Group.getCurrentState()->getRobotModel(), "manipulator");//轨迹优化算法Class
    RT.setRobotTrajectoryMsg(*Move_Group.getCurrentState(), Trajectory);
    success = TOTG.computeTimeStamps(RT, 1.0, 1.0);
    ROS_INFO("计算时间辍： %s",success?"成功":"失败");
    RT.getRobotTrajectoryMsg(Trajectory);

    My_Plan.trajectory_ = Trajectory;

    /*-------------------- 轨迹可视化 --------------------*/
    VT.prompt("轨迹可视化");

    VT.publishTrajectoryLine(My_Plan.trajectory_, Joint_Model_Group);
    VT.publishPath(Waypoints, RVT::PINK, RVT::XXXSMALL);
    for (std::size_t j = 0; j < Waypoints.size(); ++j)
        VT.publishAxisLabeled(Waypoints[j], "pt" + std::to_string(j), RVT::XXXSMALL);
    VT.trigger();

    /*-------------------- 执行轨迹 --------------------*/
    VT.prompt("执行轨迹");

    if(fraction != -1.0)
    {
        Move_Group.execute(Trajectory);
        ROS_INFO("路径执行完成");
    }
    else ROS_INFO("路径规划失败");

    /*-------------------- 结束 --------------------*/
    ros::shutdown();
    return 0;
}



/*---------------------------------------- 子函数 ----------------------------------------*/
/*-------------------- 删除NaN --------------------*/
void RemoveNaN_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const bool write = false)
{
    cloud_in->is_dense = false;

    pcl::PointCloud<pcl::PointXYZ>::PointType p_nan;
	p_nan.x = std::numeric_limits<float>::quiet_NaN();
	p_nan.y = std::numeric_limits<float>::quiet_NaN();
	p_nan.z = std::numeric_limits<float>::quiet_NaN();
	cloud_in->push_back(p_nan);

    std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, indices);
    
    if(write)
    {
        pcl::PCDWriter writer;
        writer.write("/home/zwh/zwh_main_data/RemoveNaN_Zwh.pcd", *cloud_out);
    }
}

/*-------------------- 删除特定点 --------------------*/
bool RemovePoint_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int number, const bool write = false)
{
    pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();

    int size = cloud->points.size();
    if(number > size)
    return false;
    else
    {
        index = cloud->begin() + number - 1;
        cloud->erase(index);
        if(write)
        {
            pcl::PCDWriter writer;
            writer.write("/home/zwh/zwh_main_data/RemovePoint_Zwh.pcd", *cloud);
        }
    }
    return true;
}

/*-------------------- 排序压栈 --------------------*/
void OrganizeAndPush_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, unsigned long int i, const bool write = false)
{
    double x, y, z;

    for(unsigned long int k = 0; k < cloud->points.size(); k++)
    {
        if(i % 2 == 0)
        {
            for(unsigned long int j = 0; j < (cloud->points.size()-k); j++)
            {
                if(cloud->points[k].y > cloud->points[k+j].y)
                {
                    y = cloud->points[k].y;
                    cloud->points[k].y = cloud->points[k+j].y;
                    cloud->points[k+j].y = y;
                    
                    x = cloud->points[k].x;
                    cloud->points[k].x = cloud->points[k+j].x;
                    cloud->points[k+j].x = x;

                    z = cloud->points[k].z;
                    cloud->points[k].z = cloud->points[k+j].z;
                    cloud->points[k+j].z = z;
                }
            }
        }
        else
        {
            for(unsigned long int j = 0; j < (cloud->points.size()-k); j++)
            {
                if(cloud->points[k].y < cloud->points[k+j].y)
                {
                    y = cloud->points[k].y;
                    cloud->points[k].y = cloud->points[k+j].y;
                    cloud->points[k+j].y = y;
                    
                    x = cloud->points[k].x;
                    cloud->points[k].x = cloud->points[k+j].x;
                    cloud->points[k+j].x = x;

                    z = cloud->points[k].z;
                    cloud->points[k].z = cloud->points[k+j].z;
                    cloud->points[k+j].z = z;
                }
            }
        }
        CR_Cloud->push_back(cloud->points[k]);
    }

    if(write && cloud->points.size()>0)
    Writer.write("/home/zwh/zwh_main_data/CR_" + std::to_string(i) + ".pcd", *CR_Cloud);
}

/*-------------------- 方向向量->四元数 --------------------*/
geometry_msgs::Quaternion NormalsToQuaternion_Zwh(pcl::PointCloud<pcl::Normal>::Ptr Normal, int num)
{
    double Roll, Pitch, Yaw;

    Roll = acos
              (
                 sqrt(
                       (Normal->points[num].normal_x * Normal->points[num].normal_x + Normal->points[num].normal_z * Normal->points[num].normal_z)
                       /
                       (Normal->points[num].normal_x * Normal->points[num].normal_x + Normal->points[num].normal_y * Normal->points[num].normal_y + Normal->points[num].normal_z * Normal->points[num].normal_z)
                     )
              );
    //if(Normal->points[num].normal_y > 0) Roll = 2 * 3.1415926535 - Roll;

    Pitch = atan2(Normal->points[num].normal_x, Normal->points[num].normal_z);
    //if(Normal->points[num].normal_y < 0) Pitch += 3.1415926535;
    //if(Normal->points[num].normal_x < 0) Pitch += 3.1415926535;

    Yaw = 0;

    return tf::createQuaternionMsgFromRollPitchYaw(Roll, Pitch, Yaw);
}