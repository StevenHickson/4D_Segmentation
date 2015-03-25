#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Edges.h"
#include "OpticalFlow.h"
#include "GraphSegmentation.h"

#include <string.h>

/****************
Normals are currently only supported with use_normals=true and use_time=false.
number_of_frames currently does nothing right now. It is set as 8.
use_fast_method=true runs a quicker method that only matches 1 frame at a time instead of 8.
For use_time=true, the first 7 frames will return null or empty point clouds.
*****************/
class SegmentationOptions {
public:
	float sigma_depth;
	float sigma_color;
	float sigma_normals;
	float c_depth;
	float c_color;
	float c_normals;
	int depth_min_size;
	int color_min_size;
	int normals_min_size;
	float max_depth;
	bool use_normals;
	bool use_time;
	bool use_fast_method;
	int number_of_frames;

	SegmentationOptions() : sigma_depth(2.5f), 
		sigma_color(0.8f), 
		sigma_normals(0.8f), 
		c_depth(800), 
		c_color(800), 
		c_normals(25), 
		depth_min_size(1000), 
		color_min_size(1000), 
		normals_min_size(50), 
		max_depth(6.0f), 
		use_normals(false), 
		use_time(true), 
		use_fast_method(false), 
		number_of_frames(8) { }
};

class RGBDTSegmentation {
private:
	SegmentationOptions options;
	Segment4DBig seg4d;
	Segment3D seg3d;

public:
	RGBDTSegmentation() { }
	RGBDTSegmentation(SegmentationOptions &in) {
		options = in;
	}

	void AddSlice(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &in, 
		pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_color);
};