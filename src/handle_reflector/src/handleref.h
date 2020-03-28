#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h> //io操作头文件
#include <pcl/point_types.h> //点类型定义头文件
#include <pcl/registration/icp.h> //ICP配准类所有相关的头文件


#include <cmath>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

class HandleRef
{
public:
  ros::NodeHandle n;
  HandleRef();
  virtual ~HandleRef(){}

  float getDis(float x1, float y1, float x2, float y2);
  bool findcircles(std::vector<float> num, int count, std::vector<Eigen::Matrix2d>& info);

  //TF Scalar Listener
  tf::TransformListener transform_listener;
  tf::StampedTransform transform;

  /*Callback Functions*/
  void scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg);

  ros::Subscriber scan_sub;


  /*Public Functions*/
  ros::Publisher robot_pub;

  //测试用pub
  ros::Publisher test_point_pub;
  ros::Publisher test_point_pub2;

};
