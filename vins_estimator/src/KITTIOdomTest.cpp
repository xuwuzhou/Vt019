/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "estimator/parameters.h"
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<sensor_msgs::PointCloud2Ptr> depth_cloudBuf;
std::mutex m_buf;
int num,num0,num1; 

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

void depthCloud_callback(const sensor_msgs::PointCloud2Ptr &depth_cloud)
{
	m_buf.lock();
	depth_cloudBuf.push(depth_cloud);
	m_buf.lock();
}

void callback(const sensor_msgs::ImageConstPtr &img_msg0,const sensor_msgs::ImageConstPtr &img_msg1)
{ 
    estimator.inputImage(img_msg0->header.stamp.toSec(),getImageFromMsg(img_msg0),getImageFromMsg(img_msg1));
    num++;
    printf("处理第%d帧",num);

}


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
    num0++;
    printf("缓存队列加入了第%d左帧\n",num0);
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
    num1++;
    printf("缓存队列加入了第%d右帧\n",num1);
}


// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
               
                
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                
            }
            m_buf.unlock();
            if(!image0.empty()){
                estimator.inputImage(time, image0, image1);
                num++;
                printf("将第%d左右帧送入里程计处理\n",num);}
        }
        else
        {
           
        }

        
    }
}


int main(int argc, char** argv)
{
        num=0;
        num0=0;
        num1=0;
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	


	if(argc != 2)
	{
		printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_odom_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
			   "/media/tony-ws1/disk_D/kitti/odometry/sequences/00/ \n");
		return 1;
	}

	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
        
        //将参数从config中读取设置，主要包括是否是双目（是），是否使用IMU（否），和相机外参矫正参数，并且是单线程
	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);
       ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 1100, img0_callback);
       ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 1100, img1_callback);
//        message_filters::Subscriber<sensor_msgs::Image> image_LEFT(n, IMAGE0_TOPIC,2000);
//        message_filters::Subscriber<sensor_msgs::Image> image_RIGHT(n, IMAGE1_TOPIC, 2000);
//        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_LEFT, image_RIGHT, 2000);
 //       sync.registerCallback(boost::bind(&callback, _1, _2));
        //depthCloud是将点云信息的坐标系转换到当前图像帧平面坐标系，但是还没有进行投影
        ros::Subscriber subdepthMap = n.subscribe("/depth_cloud",1100,depthCloud_callback);
        std::thread sync_thread{sync_process};

        ros::spin();
	return 0;
}
