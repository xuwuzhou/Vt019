#include<cmath>
#include<stdio.h>
#include<stdlib.h>
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<fstream>
#include<eigen3/Eigen/Dense>
#include"pointDefinition.h"

const double PI =3.1415926;

const int keepVoDataNum = 30;
double voDataTime[keepVoDataNum] = {0};
double voRx[keepVoDataNum] = {0};
double voRy[keepVoDataNum] = {0};
double voRz[keepVoDataNum] = {0};
double voTx[keepVoDataNum] = {0};
double voTy[keepVoDataNum] = {0};
double voTz[keepVoDataNum] = {0};
int voDataInd = -1;
int voRegInd = 0;
int systemInitCount = 0;
int systemDelay = 11;

//测试
int z_num_sum = 0;

	
pcl::PointCloud<pcl::PointXYZI>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr syncCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointXYZI>());

double timeRec = 0;
double rxRec = 0,ryRec = 0,rzRec = 0;
double txRec = 0,tyRec = 0,tzRec = 0;



Eigen::Matrix4f transform_matrix ;//存储的世界坐标系变换到当前坐标系的变换矩阵
Eigen::Matrix4f lidar_camera_matrix ;
Eigen::Matrix4f Last_transform_matrix = Eigen::Matrix4f::Identity(4,4) ; //存储的世界坐标系变换到上一帧坐标系的矩阵
Eigen::Matrix4f Now_Last_transform_matrix ;//存储的当前坐标系相对于上一帧坐标系的变换矩阵 

bool systemInited = false;
double initTime;
 
int startCount = -1;
const int startSkipNum = 5;

ros::Publisher *depthCloudPubPointer = NULL;

void voDataHandler(const nav_msgs::Odometry::ConstPtr& voData)
{
	double time = voData->header.stamp.toSec();
	
	double roll ,pitch ,yaw;
	geometry_msgs::Quaternion geoQuat = voData->pose.pose.orientation;
	//tf::Matrix3x3(tf::Quaternion(geoQuat.z,-geoQuat.x,-geoQuat.y,geoQuat.w)).getRPY(roll,pitch,yaw);
	//求出本帧和上一帧每一项的差值//源代码中使用的是欧拉角直接相加减，但是应该不对，因为欧拉角直接相加减的前提是小角度假设，这个不能恒成立（https://zhuanlan.zhihu.com/p/49426776）
	Eigen::Quaternionf q(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
//	Eigen::Quaternionf qu = q.normalized();
	Eigen::Matrix3f rm = q.matrix();
//	double rx = roll-rxRec;
//	double ry = pitch-ryRec;
//	double rz = yaw-rzRec;

//	if(ry< -PI){
//	ry =ry+2*PI;	
//	}else if(ry>PI){
//	ry= ry-2*PI;	
//	}

	Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	
	//生成变换矩阵(r,t,0,1)(这个变换矩阵是把世界坐标系变换成当前坐标系)
	transform_matrix(0,0)=rm(0,0);transform_matrix(0,1)=rm(0,1);transform_matrix(0,2)=rm(0,2);transform_matrix(0,3)=voData->pose.pose.position.x;
	transform_matrix(1,0)=rm(1,0);transform_matrix(1,1)=rm(1,1);transform_matrix(1,2)=rm(1,2);transform_matrix(1,3)=voData->pose.pose.position.y;
	transform_matrix(2,0)=rm(2,0);transform_matrix(2,1)=rm(2,1);transform_matrix(2,2)=rm(2,2);transform_matrix(2,3)=voData->pose.pose.position.z;
	transform_matrix(3,0)=0      ;transform_matrix(3,1)=0      ;transform_matrix(3,2)=0      ;transform_matrix(3,3)=1                           ;
//	transform_matrix <<     rm(0,0),rm(0,1),rm(0,2),  voData->pose.pose.position.x,
//				rm(1,0),rm(1,1),rm(1,2),  voData->pose.pose.position.y,
//				rm(2,0),rm(2,1),rm(2,2),  voData->pose.pose.position.z, 
//				0      ,      0,      0,  1,
	
	Now_Last_transform_matrix = transform_matrix * Last_transform_matrix.inverse() ;//这个变换矩阵实现了将当前点的在世界坐标系的坐标变换到上一帧坐标系
	
	Last_transform_matrix = transform_matrix ;

	
//	double tx = voData->pose.pose.position.x-txRec;
//	double ty = voData->pose.pose.position.y-tyRec;
//	double tz = voData->pose.pose.position.z-tzRec;

//	rxRec = roll;
//	ryRec = pitch;
//	rzRec = yaw;

//	txRec = voData->pose.pose.position.x;
//	tyRec = voData->pose.pose.position.y;
//	tzRec = voData->pose.pose.position.z;

	//下面的操作是把平移从世界坐标系转换到上一帧坐标系
//	double x1 = cos(yaw)*tx+sin(yaw)*tz;
//	double y1 = ty;
//	double z1 = -sin(yaw) * tx +cos(yaw) *tz;

//	double x2 = x1;
//	double y2 = cos(pitch)*y1-sin(pitch)*z1;
//	double z2 = sin(pitch)*y1+cos(pitch)*z1;

//	tx = cos(roll) *x2 +sin(roll) * y2;
//	ty = -sin(roll) *x2 +cos(roll) * y2;
//	tz = z2;
	//voDataInd取值为0-29
	voDataInd = (voDataInd+ 1)%keepVoDataNum;
	voDataTime[voDataInd] = time;

	//rx-ry中存的是R_lc的旋转量，旋转方向是z->x->y,参考坐标系是上一帧,所以也就是说上一帧按照R_lc=ry*rx*rz(旋转方向自右向左)的顺序旋转可以得到当前帧的坐标，tx~tz存的就是T_lc的位移量,当前坐标系相对于上一帧坐标系,在当前坐标系下表示的位移增量。R_lc和R_cl的区别就是:R_lc=ry*rx*rz，R_cl=-rz*-rx*-ry(旋转顺序从右往左看)
//	voRx[voDataInd] = rx;
//	voRy[voDataInd] = ry;
//	voRz[voDataInd] = rz;
//	voTx[voDataInd] = tx;
//	voTy[voDataInd] = ty;
//	voTz[voDataInd] = tz;

//	double cosrx =cos(rx);
//	double sinrx =sin(rx);
//	double cosry =cos(ry);
//	double sinry =sin(ry);
//	double cosrz =cos(rz);
//	double sinrz =sin(rz);

//	if(time - timeRec <0.5){
	pcl::PointXYZI point;
	tempCloud->clear();
	double x1,y1,z1,x2,y2,z2;
	int depthCloudNum = depthCloud->points.size();
	int numpoints = 0;//记录最终加入深度图点的个数
	printf("即将参与运算的激光点云点的数量为%d\n",depthCloudNum);
	for(int i = 0;i<depthCloudNum;i++){
	point = depthCloud->points[i];
	
//	x1 = cosry* point.x - sinry*point.z;
//	y1 = point.y;
//	z1 = sinry*point.x + cosry *point.z;

//	x2 = x1;
//	y2 = cosrx *y1 +sinrx *z1;
//	z2 = -sinrx *y1 + cosrx*z1;
	//tx-tz存的是当前坐标系相对于上一帧坐标系，在当前坐标系下表示的位移增量，因为rx = voData->twist.twist.angular.x - rxRec;所以基准坐标系是上一帧，即rxRec
	Eigen::Vector4f pointpose;
	pointpose[0] = point.x;
	pointpose[1] = point.y;
	pointpose[2] = point.z;
	pointpose[3] = 1;
	
//	point.x = cosrz * x2 + sinrz *y2-tx;
//	point.y = -sinrz * x2 + cosrz * y2 -ty;
//	point.z = z2- tz;
	Eigen::Vector4f newpointpose;
	newpointpose = Now_Last_transform_matrix * pointpose;
	point.x = newpointpose[0];
	point.y = newpointpose[1];
	point.z = newpointpose[2];

	double pointDis = sqrt(point.x *point.x +point.y*point.y+point.z*point.z);
	double timeDis = time -initTime- point.intensity;
	if(pointDis<15){
	tempCloud->push_back(point);	
	}
	}	
	printf("这一帧深度图点云中加入了%d个点\n",numpoints);
	depthCloud->clear();
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
	downSizeFilter.setInputCloud(tempCloud);
	downSizeFilter.setLeafSize(0.7,0.7,0.7);
	downSizeFilter.filter(*depthCloud);
	depthCloudNum = depthCloud->points.size();
	printf("深度图经过滤波后，点还剩%d个\n",depthCloud->points.size());
	tempCloud->clear();
	for(int i=0;i<depthCloudNum;i++){
	point = depthCloud->points[i];

	//if(fabs(point.x/point.z)<1&&fabs(point.y/point.z)<0.6){
	point.intensity = depthCloud->points[i].z;
	point.x *=10/ depthCloud->points[i].z;	
	point.y *=10/ depthCloud->points[i].z;
	point.z =10;	

	tempCloud->push_back(point);		
	//}	
	}

	tempCloud2->clear();
	downSizeFilter.setInputCloud(tempCloud);
	downSizeFilter.setLeafSize(0.7,0.7,0.7);
	downSizeFilter.filter(*tempCloud2);
	int tempCloud2Num =tempCloud2->points.size();

	for(int i=0;i<tempCloud2Num;i++){
	tempCloud2->points[i].z = tempCloud2->points[i].intensity;
	tempCloud2->points[i].x *= tempCloud2->points[i].z/10 ;
	tempCloud2->points[i].y *= tempCloud2->points[i].z/10 ;
	tempCloud2->points[i].intensity = 10;

 	}
	printf("最后输出的深度图的点个数%d\n",tempCloud2->size());
	sensor_msgs::PointCloud2 depthCloud2;
	pcl::toROSMsg(*tempCloud2,depthCloud2);
	depthCloud2.header.frame_id = "camera2";
	depthCloud2.header.stamp = voData->header.stamp;
	depthCloudPubPointer->publish(depthCloud2);
//	}
	timeRec = time;
}


void syncCloudHandler(const sensor_msgs::PointCloud2ConstPtr& syncCloud2)
{

	if (!systemInited)
    	{ 
	
        systemInitCount++;
	printf("第%d帧点云数据,抛弃\n",systemInitCount);
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
	    printf("前%d帧点云数据因为没有对应的里程计数据，均已抛弃，下面可以进行深度图创建\n",systemInitCount);
        }
        else
            return;
    	}	

	

	initTime = syncCloud2->header.stamp.toSec();

	double time = syncCloud2->header.stamp.toSec();
	double timeLasted =time -initTime;

	syncCloud->clear();
	pcl::fromROSMsg(*syncCloud2,*syncCloud);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*syncCloud, *syncCloud, indices);//移除空点，经验证，这一条基本没用，移除前后基本没变化

//从逻辑上来讲，测试kitti，用不到下面这段，因为kitti的雷达点云经过矫正，并且时间序列和图像集是一模一样，但是实机测试是要用到的
//	double scale = 0;
//	int voPreInd = keepVoDataNum -1;
//	if(voDataInd >= 0){
//	while(voDataTime[voRegInd]<=time&&voRegInd!=voDataInd){
//	voRegInd = (voRegInd+1)%keepVoDataNum;		
//	}//轮转查询与当前点云时间戳最相近的里程计时间戳

//	voPreInd = (voRegInd+keepVoDataNum-1)%keepVoDataNum;	
//	double voTimePre = voDataTime[voPreInd];
//	double voTimeReg = voDataTime[voRegInd];

//	if(voTimeReg - voTimePre<0.5){
//	double scale = (voTimeReg-time) / (voTimeReg-voTimePre);
//	if(scale>1){//与哪个时间戳最近，就把哪个里程计信息作为变换的依据
//	scale = 1;
//	}else if(scale<0){
//	scale = 0;	
//	}
//	}
//	}
	
	//这里的代码问题，因为是测试kitti数据集，所以并不需要通过插值得到点云所对应的坐标系，并且因为每一帧点云坐标系都与每一帧的里程计坐标系的之间的关系是确定的，所以，下面的总共三步：1、通过插值得到点云对应的坐标系，2、把插值得到的坐标系下的点转换到voRegInd指向的那一帧，3、将点云中的点一直变换到当前相机坐标系。可以汇总为一步，将当前点云中的点变换到相机坐标系，因为相机坐标系和点云坐标系是标定好的。
	//汇总步骤：将当前点云中的点变换到相机坐标系：
//	int z_num = 0;
//	for(int i=0;i<syncCloud->points.size();i++){
//	if(syncCloud->points[i].z!=0){z_num++;z_num_sum++;}
//	}
//	printf("当前点云帧的z值非0点个数为%d,总共接收到的点的个数为%d\n",z_num,syncCloud->points.size());
//	printf("累计已经接收到的点的非0点个数为%d\n",z_num_sum);
	
	//将点云从点云坐标系变换到左相机坐标系

//	//通过插值得到与点云对应的坐标系，rx2-rz2,tx2-tz2指的是点云对应的帧与voRegInd指向的帧的旋转变换关系
//	double rx2 = voRx[voRegInd]*1;//scale暂定为1
//	double ry2 = voRy[voRegInd]*1;
//	double rz2 = voRz[voRegInd]*1;

//	double tx2 = voTx[voRegInd]*1;
//	double ty2 = voTy[voRegInd]*1;
//	double tz2 = voTz[voRegInd]*1;

//	double cosrx2 = cos(rx2);
//	double sinrx2 = sin(rx2);
//	double cosry2 = cos(ry2);
//	double sinry2 = sin(ry2);
//	double cosrz2 = cos(rz2);
//	double sinrz2 = cos(rz2);

	pcl::PointXYZI point;
//	double x1,y1,z1,x2,y2,z2;
	int syncCloudNum = syncCloud->points.size();
	//printf("接收到的初始点云点数量为%d\n",syncCloud->points.size());
	//std::ofstream foutC("/home/xuwuzhou/catkin_vins/depthmappoints.csv",std::ios::app);
	//foutC.setf(std::ios::fixed,std::ios::floatfield);
	
	for(int i=0;i<syncCloudNum;i++){
		
	point.x = syncCloud->points[i].x;
	point.y = syncCloud->points[i].y;
	point.z = syncCloud->points[i].z;
	point.intensity = timeLasted;
	
	//foutC.precision(5);
	//foutC   << point.x <<","
	//	<< point.y <<","
	//	<< point.z <<","<<std::endl;

//	//把插值得到的坐标系下的点转换到voRegInd指向的那一帧
//	x1 = cosry2*point.x - sinry2*point.z;
//	y1 = point.y;
//	z1 = sinry2*point.x + cosry2*point.z;

//	x2 = x1;
//	y2 = cosrx2*y1 + sinrx2*z1;
//	z2 =-sinrx2*y1 + cosrx2*z1;
	
	Eigen::Vector4f tmppoint;
	tmppoint[0] = point.x;
	tmppoint[1] = point.y;
	tmppoint[2] = point.z;
	tmppoint[3] = 1;
	//tmppoint << point.x << point.y << point.z << 1;//接收到的雷达点
	Eigen::Vector4f camerapoint;//将雷达点变换到相机坐标系下
	
	
	camerapoint = lidar_camera_matrix * tmppoint;
	point.x = camerapoint[0];
	point.y = camerapoint[1];
	point.z = camerapoint[2];

//	//将点云的点的坐标一直变换到最新的帧的坐标系下
//	if(voDataInd >= 0){
//	int voAftInd = (voRegInd+1) % keepVoDataNum;
//	while(voAftInd != (voDataInd+1)%keepVoDataNum){
//	double rx = voRx[voAftInd];
//	double ry = voRy[voAftInd];
//	double rz = voRz[voAftInd];

//	double tx = voTx[voAftInd];
//	double ty = voTy[voAftInd];
//	double tz = voTz[voAftInd];

//	double cosrx = cos(rx);
//	double sinrx = sin(rx);
//	double cosry = cos(ry);
//	double sinry = sin(ry);
//	double cosrz = cos(rz);
//	double sinrz = sin(rz);

//	x1 = cosry*point.x - sinry*point.z;
//	y1 = point.y;
//	z1 = sinry*point.x + cosry*point.z;

//	x2 = x1;
//	y2 = cosrx*y1 + sinrx*z1;
//	z2 = -sinrx *y1 + cosrx * z1;

//	point.x = cosrz*x2+sinrz*y2-tx;
//	point.y = -sinrz *x2 + cosrz *y2 -ty;
//	point.z = z2-tz;

//	voAftInd = (voAftInd+1) %keepVoDataNum; 	
//	}	
//	}
	
	double pointDis = sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
	//printf("输出第一次筛选的条件值,point.x/point.z=%d,point.y/point.z=%d,point.z=%d,pointDis=%d\n",point.x/point.z,point.y/point.z,point.z,pointDis);
	//fabs(point.x/point.z)<2 fabs(point.y/point.z)<1.5&&point.z>0.5&&
	if(pointDis<15){
	depthCloud->push_back(point);	
	}
		
	}
	//foutC.close();
	//printf("第一次处理后点云点的数量为%d,从下面开始接收里程计信息进行转换\n",depthCloud->points.size());
}


int main(int argc, char **argv)
{ 
	ros::init(argc,argv,"processDepthmap");
	ros::NodeHandle nh;
	transform_matrix <<1,0,0,0,
		   0,1,0,0,
		   0,0,1,0,
		   0,0,0,1;
	lidar_camera_matrix <<    -1.857739385241e-03,-9.999659513510e-01,-8.039975204516e-03,-4.784029760483e-03,
		-6.481465826011e-03,8.051860151134e-03 ,-9.999466081774e-01,-7.337429464231e-02,
		9.999773098287e-01 ,-1.805528627661e-03,-6.496203536139e-03,-3.339968064433e-01,
		0		   ,0  		,0		,1			;//雷达坐标系和相机坐标系的外参矫正矩阵04序列
//	lidar_camera_matrix << 4.276802385584e-04,-9.999672484946e-01,-8.084491683471e-03,-1.198459927713e-02,
//				-7.210626507497e-03,8.081198471645e-03,-9.999413164504e-01,-5.403984729748e-02,
//				9.999738645903e-01,4.859485810390e-04,-7.206933692422e-03,-2.921968648686e-01,
//				0,0,0,1;//雷达坐标系和相机坐标系的外参矫正矩阵01序列
//	lidar_camera_matrix << -1.857739385241e-03,-9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
//				 -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
//				 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
//				0,0,0,1;//10序列的外参矫正矩阵
	ros::Subscriber voDataSub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry",100,voDataHandler);//接收不到前11帧的里程计数据
	ros::Subscriber syncCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",100,syncCloudHandler);//所以点云数据的前11帧也要相对应的不进行接收
	ros::Publisher depthCloudPub = nh.advertise<sensor_msgs::PointCloud2>("depth_cloud",100);
	depthCloudPubPointer = &depthCloudPub;
	ros::spin();
	return 0;
}

