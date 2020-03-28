#include <ros/ros.h>
#include <iostream> //标准输入输出头文件
#include <pcl/io/pcd_io.h> //io操作头文件
#include <pcl/point_types.h> //点类型定义头文件
#include <pcl/registration/icp.h> //ICP配准类所有相关的头文件

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_icp");
  ros::NodeHandle nh;

  //定义两个点云变量，一个输入，一个输出
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_in->width =6; //设置点云宽度
  cloud_in->height=1; //设置点云为无序点云
  cloud_in->is_dense=false;
  cloud_in->points.resize(cloud_in->width*cloud_in->height);

  //给申明的点云随机赋值
  for(size_t i=0;i<cloud_in->points.size();++i)
  {
    cloud_in->points[i].x=1024*rand()/(RAND_MAX+1.0f);
    cloud_in->points[i].y=1024*rand()/(RAND_MAX+1.0f);
    cloud_in->points[i].z=1024*rand()/(RAND_MAX+1.0f);
  }

  //打印输入点云信息
  std::cout<<"input cloudPoints"<<cloud_in->points.size()<<"data points to input:"<<std::endl;

  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    std::cout << "    " <<cloud_in->points[i].x <<
                 " " << cloud_in->points[i].y <<
                 " " << cloud_in->points[i].z << std::endl;
  }

  //先将输入的点云赋给输出点云
  *cloud_out=*cloud_in;

  std::cout<<"size: "<<cloud_out->points.size()<<std::endl;

  //将所有点沿Z方向平移,等价于将输出点云的x坐标都加上一个数,比如0.6f.
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_out->points[i].z = cloud_in->points[i].z + 0.6f;
  }

  //打印平移后的点
  std::cout<<"Transformed "<<cloud_in->points.size()<<"data points:"<<std::endl;

  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
                 cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

  //以上,实现了一个简单的点云刚体变换,以构造目标点云,并再次打印处数据集.

  pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp; //创建一个IterativeClosestPoint的对象
  icp.setInputCloud(cloud_in); //设置源点云
  icp.setInputTarget(cloud_out); //设置目标点云

  pcl::PointCloud<pcl::PointXYZ> Final; //存储经过配准变换源点云后的点云
  icp.align(Final);  //执行配准存储变换后的源点云到Final

  //打印配准相关输入信息
  std::cout<<"has converged: "<<icp.hasConverged()<<"  "<<"score: "<<icp.getFitnessScore()<<std::endl;

  //打印输出最终估计的变换矩阵.
  std::cout<<icp.getFinalTransformation()<<std::endl;

  
  ROS_INFO("Hello world!");

  return 0;
}
