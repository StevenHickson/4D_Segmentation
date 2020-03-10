/*
Copyright (C) 2014 Steven Hickson

MIT License
*/

#ifndef FLOW_H
#define FLOW_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"

//DEFINES constants for kinect cameras
#define KINECT_CX_C 3.5094272028759258e+02
#define KINECT_CY_C 2.4251931828128443e+02
#define KINECT_FX_C 5.2921508098293293e+02
#define KINECT_FY_C 5.2556393630057437e+02


//I preinvert all the depth focal points so they can be multiplied instead of divided. This is because I'm smart
#define KINECT_CX_D 3.2330780975300314e+02
#define KINECT_CY_D 2.4273913761751615e+02
#define KINECT_FX_D 1.6828944189289601e-03
#define KINECT_FY_D 1.6919313269589566e-03

template <typename T> inline T Clamp(T a, T minn, T maxx)
{ return (a < minn) ? minn : ( (a > maxx) ? maxx : a ); }

inline int Round (float a)  
{
	//assert( !_isnan( a ) );
	//Recently started happening. This has to be a pointer error somewhere that I need to figure out
	if(std::isnan( a )) {
		//printf("This should never happen!!");
		return 0;
	}
	return static_cast<int>(a>=0 ? a+0.5f : a-0.5f); 
} 

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudBgr;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudInt;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;

void ComputeOpticalFlow(const cv::Mat &past, const cv::Mat &current, const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &pastCloud, const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &currCloud, pcl::PointCloud<pcl::Normal>::Ptr &flow);
void ComputeOpticalFlow(const pcl::PointCloud<pcl::PointXYZRGBA> &pastCloud, const pcl::PointCloud<pcl::PointXYZRGBA> &currCloud, pcl::PointCloud<pcl::Normal> *flow);
//void ComputeOpticalFlowGPU(const cv::Mat &past, const cv::Mat &current, const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &pastCloud, const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &currCloud, pcl::PointCloud<pcl::Normal>::Ptr &flow);
void Downsample2x2(const cv::Mat &in, cv::Mat &out);

#endif //FLOW_H
