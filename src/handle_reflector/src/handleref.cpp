#include "handleref.h"
#include <cstdio>

using namespace std;

//这里是所有的常数定义
float RADIUS = 0.5;

float ANGLE_MIN;
float ANGLE_INCREMENT;


//这里是所有的全局变量
//1. 储存了全局地图的相关信息
int reflactor_number = 5;

float x_1 = -2; float y_1 = -2.2;
float x_2 = 2.7; float y_2 = -7.5;
float x_3 = 3.3; float y_3 = 2.4;
float x_4 = -1.9; float y_4 = 8.4;
float x_5 = -7.2; float y_5 = 3.8;

//用来储存测得圆心
vector<Eigen::Vector2d> Points;

//2. 用来储存发布信息
geometry_msgs::PointStamped robot_location;
geometry_msgs::PointStamped test_point;

//3. 用来申明将会在各个函数中交换信息的变量

//可见反光柱的相关位置与距离信息
Eigen::Matrix<float, 3, 4> M_ref_sensor;
Eigen::Matrix<float, 3, 4> M_ref_real;
//Eigen::Matrix<float, 1, 4> M_dis_sensor;
vector<float> M_dis_sensor;
Eigen::Matrix<float, 1, 4> M_dis_real;

float d1, d2, d3, d4;

//什么是有效距离？
std::vector<float> valid_dis;


HandleRef::HandleRef()
{
	scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_filtered",1,&HandleRef::scan_Callback,this);
	robot_pub = n.advertise<geometry_msgs::PointStamped>("robot_location", 1);
	test_point_pub = n.advertise<geometry_msgs::PointStamped>("test_point", 1);
	test_point_pub2 = n.advertise<geometry_msgs::PointStamped>("test_point2", 1);
}

struct point
{
	point() {}
	std::vector<float> x;
	std::vector<float> y;
	std::vector<int> ID;
};
point reflectors;

// 获得两点间的直线距离
float HandleRef::getDis(float x1, float y1, float x2, float y2)
{
	float Dis;
	Dis = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

	return Dis;

}

// 从获取的激光数据中，搜索出圆弧片段的起点与终点
bool HandleRef::findcircles(std::vector<float> msg, int count, vector<Eigen::Matrix2d>& count_start_end)
{
	Eigen::Matrix2d start_end;

	bool sign = 0;

	cout<<"找到了以下的圆弧："<<endl;

	for(int i=1;i<count;i++){
		if(!isnan(msg.at(i)) && sign == 0){
			start_end(0,0) = i;
			cout<<"start="<<i<<endl;
			sign = 1;
		}

		if(isnan(msg.at(i)) && sign == 1){
			start_end(1,0) = i;
			count_start_end.push_back(start_end);
			cout<<"end="<<i<<endl;
			sign = 0;
		}
	}

	cout<<"======"<<endl;
	return 0;
}


void HandleRef::scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_WARN_STREAM("CALL BACK");

	Points.clear();
	
	M_dis_real << 0,0,0,0;
	cout<<"初始化M_dis_real:"<<M_dis_real<<endl;

	ANGLE_MIN = msg->angle_min;
	ANGLE_INCREMENT = msg->angle_increment;
	int ranges = msg->ranges.size();
	std::vector<float> ranges_vec = msg->ranges;
	vector<Eigen::Matrix2d> msg_start_end;
	HandleRef::findcircles(ranges_vec, ranges, msg_start_end);


	for(auto it = msg_start_end.begin(); it != msg_start_end.end(); it++) {
		// count就直接等于圆弧中间的那个位置
		int count = ((*it)(0,0)+(*it)(1,0))/2;
		float angle = count * ANGLE_INCREMENT;
		float point_x = -ranges_vec.at(count) * cos(angle);
		float point_y = -ranges_vec.at(count) * sin(angle);

		Eigen::Vector2d point;
		point << point_x, point_y;

		ROS_WARN_STREAM("mid point: "<<point_x<<" "<<point_y);

		/*
		// 装入mdis，及传感器扫描到的距离。
		M_dis_sensor.push_back(sqrt(point_x*point_x+point_y*point_y));

		if(it == msg_start_end.begin())
		{
			test_point.point.x = point_x;
			test_point.point.y = point_y;
			test_point.point.z = 0;
			test_point.header.frame_id = "/scan";
			test_point_pub.publish(test_point);
		}

		if(it == msg_start_end.begin()+2)
		{
			test_point.point.x = point_x;
			test_point.point.y = point_y;
			test_point.point.z = 0;
			test_point.header.frame_id = "/scan";
			test_point_pub2.publish(test_point);
		}
		*/
		Points.push_back(point);

	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_in->width =8; //设置点云宽度
	cloud_in->height=1; //设置点云为无序点云
	cloud_in->is_dense=false;
	cloud_in->points.resize(cloud_in->width*cloud_in->height);


	cloud_out->width =8; //设置点云宽度
	cloud_out->height=1; //设置点云为无序点云
	cloud_out->is_dense=false;
	cloud_out->points.resize(cloud_out->width*cloud_out->height);

	//给申明的点云随机赋值
	for(size_t i=0;i<Points.size();++i)
	{
		cloud_in->points[i].x=Points.at(i)(0);
		cloud_in->points[i].y=Points.at(i)(1);
		cloud_in->points[i].z=0;
	}

	//打印输入点云信息
	std::cout<<"input cloudPoints"<<cloud_in->points.size()<<"data points to input:"<<std::endl;

	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		std::cout << "    " <<cloud_in->points[i].x <<
			" " << cloud_in->points[i].y <<
			" " << cloud_in->points[i].z << std::endl;
	}


	//设置全局点云
	cloud_out->points[0].x = x_1;
	cloud_out->points[0].y = y_1;

	cloud_out->points[1].x = x_2;
	cloud_out->points[1].y = y_2;

	cloud_out->points[2].x = x_3;
	cloud_out->points[2].y = y_3;

	cloud_out->points[3].x = x_4;
	cloud_out->points[3].y = y_4;

	cloud_out->points[4].x = x_5;
	cloud_out->points[4].y = y_5;

	//打印平移后的点
	std::cout<<"Transformed "<<cloud_in->points.size()<<"data points:"<<std::endl;

	for (size_t i = 0; i < cloud_out->points.size (); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
			cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	//以上,实现了一个简单的点云刚体变换,以构造目标点云,并再次打印处数据集.

	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp; //创建一个IterativeClosestPoint的对象
	icp.setInputSource(cloud_out); //设置源点云
	icp.setInputTarget(cloud_in); //设置目标点云

	pcl::PointCloud<pcl::PointXYZ> Final; //存储经过配准变换源点云后的点云
	icp.align(Final);  //执行配准存储变换后的源点云到Final

	//打印配准相关输入信息
	std::cout<<"has converged: "<<icp.hasConverged()<<"  "<<"score: "<<icp.getFitnessScore()<<std::endl;

	//打印输出最终估计的变换矩阵.
	std::cout<<icp.getFinalTransformation()<<std::endl;




	// 这里输出位置，需要改为输出odom类的信息用于karman_filter
	robot_location.point.x = icp.getFinalTransformation()(0,3);
	robot_location.point.y = icp.getFinalTransformation()(1,3);
	robot_location.point.z = 0;
	robot_location.header.frame_id = "/scan";
	robot_pub.publish(robot_location);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "handleref");

	HandleRef handle_laser; //实体化一个在头文件中的class，在class里面就包含了回调函数的声明

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
