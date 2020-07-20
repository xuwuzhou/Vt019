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
std::mutex m_buf;
vector<cv::Mat> img0_NOBAG;
vector<cv::Mat> img1_NOBAG;
vector<cv::Mat> img0_BAG;
vector<cv::Mat> img1_BAG;
vector<double> imageTimeList_BAG;
vector<double> imageTimeList_NOBAG;

bool equalMAT(cv::Mat mat1,cv::Mat mat2)
{
    int Rows = mat1.rows;
    int Cols = mat1.cols * mat1.channels();
    int IsStopOutLoop = 0;
    bool bRet = true;

    
        for (int i = 0; i < Rows; i++)
        {
            uchar *data1 = mat1.ptr<uchar>(i);
            uchar *data2 = mat2.ptr<uchar>(i);
            for (int j = 0; j < Cols; j++)
            {
                if (data1[j] != data2[j])
                {
                    IsStopOutLoop++;
                    bRet = false;
                    break;
                }
            }
            if (IsStopOutLoop != 0)
                break;
        }
        //bRet = true;
    
    return bRet;
}

void duibi()
{ 
    cv::Mat buff1,buff2;
    int numNOBAG_img,numBAG_img,numNOBAG_time,numBAG_time;
    numNOBAG_img=img0_NOBAG.size();
    numNOBAG_time=imageTimeList_NOBAG.size();
    numBAG_img=img0_BAG.size();
    numBAG_time=imageTimeList_BAG.size();
    printf("NOBAGIMG%f  NOBAGTIME%f BAGIMG%f BAGTIME%f\n",numNOBAG_img,numNOBAG_time,numBAG_img,numBAG_time);
    
    for(int i=0;i<numBAG_img;i++){
       buff1=img0_NOBAG[i].clone();
       buff2=img1_NOBAG[i].clone();
       if(!equalMAT(buff1,buff2)){printf("第 %d 帧图像bag和nobag不对应", i);break;}
    }
    printf("检测完成，图像均对应");
    
}

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

void callback(const sensor_msgs::ImageConstPtr &img_msg0,const sensor_msgs::ImageConstPtr &img_msg1)
{ 
    printf("bag开始处理");
    cv::Mat image0_BAG, image1_BAG;
    m_buf.lock();
    image0_BAG=getImageFromMsg(img_msg0);
    image1_BAG=getImageFromMsg(img_msg1);
    img0_BAG.push_back(image0_BAG.clone());
    img1_BAG.push_back(image1_BAG.clone());
    img0_buf.push(img_msg0);
    img1_buf.push(img_msg1); 
    imageTimeList_BAG.push_back(img_msg0->header.stamp.toSec());
    m_buf.unlock();
}

/*
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}
*/

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
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
           
        }

        
    }
}

int main(int argc, char** argv)
{

        ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage",1000);
	ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage",1000);

	if(argc != 3)
	{
		printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_odom_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
			   "/media/tony-ws1/disk_D/kitti/odometry/sequences/00/ \n");
		return 1;
	}

	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
	string sequence = argv[2];
	printf("read sequence: %s\n", argv[2]);
	string dataPath = sequence + "/";

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);

         message_filters::Subscriber<sensor_msgs::Image> image_LEFT(n, IMAGE0_TOPIC,500);
        message_filters::Subscriber<sensor_msgs::Image> image_RIGHT(n, IMAGE1_TOPIC, 500);
        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_LEFT, image_RIGHT, 50);
        sync.registerCallback(boost::bind(&callback, _1, _2));
        
        printf("当前bag处理%d帧,当前nobag处理%d帧",imageTimeList_BAG,imageTimeList_NOBAG);


	// load image list
	FILE* file;
	file = std::fopen((dataPath + "times.txt").c_str() , "r");
	if(file == NULL){
	    printf("cannot find file: %stimes.txt\n", dataPath.c_str());
	    ROS_BREAK();
	    return 0;          
	}
	double imageTime_NOBAG;
	
	while ( fscanf(file, "%lf", &imageTime_NOBAG) != EOF)
	{
	    imageTimeList_NOBAG.push_back(imageTime_NOBAG);
	}
	std::fclose(file);

	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
//	FILE* outFile;
//	outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
//	if(outFile == NULL)
//		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
        while(img0_NOBAG.size()<1100){
	for (size_t i = 0; i < imageTimeList_NOBAG.size(); i++)
	{	
		
			printf("\nprocess image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(6) << i;
			leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
			rightImagePath = dataPath + "image_1/" + ss.str() + ".png";
			//printf("%lu  %f \n", i, imageTimeList[i]);
			//printf("%s\n", leftImagePath.c_str() );
			//printf("%s\n", rightImagePath.c_str() );

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
			imLeftMsg->header.stamp = ros::Time(imageTimeList_NOBAG[i]);
			pubLeftImage.publish(imLeftMsg);
                        img0_NOBAG.push_back(imLeft);

			imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
			imRightMsg->header.stamp = ros::Time(imageTimeList_NOBAG[i]);
			pubRightImage.publish(imRightMsg);
                        img1_NOBAG.push_back(imRight);
           printf("当前bag处理%d帧,当前nobag处理%d帧",imageTimeList_BAG,imageTimeList_NOBAG);
//			estimator.inputImage(imageTimeList[i], imLeft, imRight);
			
		//	Eigen::Matrix<double, 4, 4> pose;
//			estimator.getPoseInWorldFrame(pose);
//			if(outFile != NULL)
//				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
//																	       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
//																	       pose(2,0), pose(2,1), pose(2,2),pose(2,3));
			
			//cv::imshow("leftImage", imLeft);
			//cv::imshow("rightImage", imRight);
			//cv::waitKey(2);
		
//		else
	//		break;
	}}
//	if(outFile != NULL)
//		fclose (outFile);
//	return 0;        


//	ros::init(argc, argv, "vins_estimator");
//	ros::NodeHandle n("~");
	//ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	


/*	if(argc != 2)
	{
		printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_odom_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
			   "/media/tony-ws1/disk_D/kitti/odometry/sequences/00/ \n");
		return 1;
	}*/

//	string config_file = argv[1];
//	printf("config_file: %s\n", argv[1]);

//	readParameters(config_file);
//	estimator.setParameter();
//	registerPub(n);
        
       
//        std::thread sync_thread{sync_process};
        if(imageTimeList_BAG.size()==1100){duibi();}
        ros::spin();
	return 0;
}
