/*---------------------------------------- C++ ----------------------------------------*/
/*---------------------------------------- C++ ----------------------------------------*/
/*---------------------------------------- C++ ----------------------------------------*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <math.h>

/*---------------------------------------- PCL ----------------------------------------*/
/*---------------------------------------- PCL ----------------------------------------*/
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
/*---------------------------------------- 全局定义(参数+类) ----------------------------------------*/
/*---------------------------------------- 全局定义(参数+类) ----------------------------------------*/
ros::Publisher PCL_Publisher;

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

void OrganizeAndPush_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int i, const bool write);


/*---------------------------------------- 主函数 ----------------------------------------*/
/*---------------------------------------- 主函数 ----------------------------------------*/
/*---------------------------------------- 主函数 ----------------------------------------*/
int main(int argc, char** argv)
{
    /*-------------------- ROS初始化 --------------------*/
    /*-------------------- ROS初始化 --------------------*/
    ros::init(argc, argv, "zwh_main");
    ros::NodeHandle Node_Handle; 

    setlocale(LC_ALL, "");

    /*-------------------- 创建话题 --------------------*/
    /*-------------------- 创建话题 --------------------*/
    PCL_Publisher = Node_Handle.advertise <sensor_msgs::PointCloud2> ("zwh_main_pcl", 1);
    
    /*-------------------- 读取点云 --------------------*/
    /*-------------------- 读取点云 --------------------*/
    ROS_INFO("读取点云");
    Reader.read("/home/zwh/blade_100w.pcd", *Initial_Cloud_XYZ);

    /*-------------------- 均匀采样 --------------------*/
    /*-------------------- 均匀采样 --------------------*/
    ROS_INFO("均匀采样");
    US.setInputCloud(Initial_Cloud_XYZ);
    US.setRadiusSearch(0.01f);
    US.filter(*US_Cloud_XYZ);
    pcl::getMinMax3D(*US_Cloud_XYZ, US_Cloud_Min, US_Cloud_Max);

    /*-------------------- 条件滤波+排序 --------------------*/
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
                OrganizeAndPush_Zwh(CR_Cloud_13, i, false);
                cout << "CR_CLoud_" << i << " 排序完成" << endl;
                break;
            }

            default: cout << "条件滤波: Case Error" << endl; break;
        }
    }





    /*-------------------- 测试 --------------------*/
    /*-------------------- 测试 --------------------*/

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(CR_Cloud);
    int count = 0; int all_count = 0;
    double min = 1e-06;
    int K = 2;
    std::vector<int> pointId(K);
    std::vector<float> pointSquareDistance(K);

    for(long unsigned int ID; ID < CR_Cloud->points.size(); ID++)
    {
        kdtree.nearestKSearch(CR_Cloud->points[ID], K, pointId, pointSquareDistance);
        cout << "搜索对象: " << ID << endl;
        for (size_t i = 1; i < pointId.size(); ++i)      
        {
            if(pointSquareDistance[i] < min && pointSquareDistance[i] > 0) count++;
            cout << "[" << pointId[i] << "] " << " 平方距离: " << pointSquareDistance[i] << endl;
        }
        cout << "距离过小数量: " << count << endl;
        cout << endl;
        all_count += count;
        count = 0;
    }
    cout << "总和: " << all_count << endl;
    //return 0;





    /*-------------------- 计算法线 --------------------*/
    /*-------------------- 计算法线 --------------------*/
    ROS_INFO("计算法线");

    NE.setNumberOfThreads(10);//openMP线程数
    NE.setInputCloud(CR_Cloud);
    NE.setSearchSurface(Initial_Cloud_XYZ);
    NE.setSearchMethod(Kd_Tree);
    NE.setRadiusSearch(0.002);
    //NE.setKSearch(5);
    NE.compute(*Normals);

    for(long unsigned int num = 0; num < Normals->points.size(); num++)
    {
        if(Normals->points[num].normal_z > 0)
        {
            Normals->points[num].normal_x = -Normals->points[num].normal_x;
            Normals->points[num].normal_y = -Normals->points[num].normal_y;
            Normals->points[num].normal_z = -Normals->points[num].normal_z;
        }
    }

    /*-------------------- PCL可视化 --------------------*/
    /*-------------------- PCL可视化 --------------------*/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer(new pcl::visualization::PCLVisualizer("Zwh Viewer"));

    Viewer->setWindowName("Zwh-Main-Viewer");//创建视窗对象，定义标题栏名称
    Viewer->setBackgroundColor(0.3, 0.3, 0.3);
    Viewer->addCoordinateSystem(0.5);
    Viewer->addText("Zwh_Main", 50, 50, 0, 1, 0, "v1_text");
    Viewer->addPointCloud<pcl::PointXYZ>(CR_Cloud, "Zwh-XYZNormal");//将点云添加到视窗对象中，并定义一个唯一的ID
    Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "Zwh-XYZNormal");//点云附色，三个字段，每个字段范围0-1
    Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "Zwh-XYZNormal");//点云大小
    Viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(CR_Cloud, Normals, 1, 0.005, "Normals");//每1个点显示一个法线，长度为0.5

    while (!Viewer->wasStopped())
    {
        Viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    /*-------------------- 结束 --------------------*/
    /*-------------------- 结束 --------------------*/
    ros::shutdown();
    return 0;
}



/*---------------------------------------- 子函数 ----------------------------------------*/
/*---------------------------------------- 子函数 ----------------------------------------*/
/*---------------------------------------- 子函数 ----------------------------------------*/
/*-------------------- 排序压栈 --------------------*/
/*-------------------- 排序压栈 --------------------*/
void OrganizeAndPush_Zwh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int i, const bool write = false)
{
    double x, y, z;

    for(long unsigned int k = 0; k < cloud->points.size(); k++)
    {
        if(i % 2 == 0)
        {
            for(long unsigned int j = 0; j < (cloud->points.size()-k); j++)
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
            for(long unsigned int j = 0; j < (cloud->points.size()-k); j++)
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