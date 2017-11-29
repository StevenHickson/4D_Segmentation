/*
Copyright (C) 2014 Steven Hickson

MIT License
*/
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "4D_Segmentation.h"

using namespace std;
using namespace pcl;
using namespace cv;

inline void MakeCloudDense(PointCloud<PointXYZRGBA> &cloud) {
	PointCloud<PointXYZRGBA>::iterator p = cloud.begin();
	cloud.is_dense = true;
	for(int j = 0; j < cloud.height; j++) {
		for(int i = 0; i < cloud.width; i++) {
			if(isnan(p->z)) {
				p->x = float(((float)i - KINECT_CX_D) * KINECT_FX_D);
				p->y = float(((float)j - KINECT_CY_D) * KINECT_FY_D);
				p->z = 0;
			}
			//p->a = 255;
			++p;
		}
	}
}

inline void MakeCloudDense(PointCloud<PointNormal>::Ptr &cloud) {
	PointCloud<PointNormal>::iterator p = cloud->begin();
	cloud->is_dense = true;
	for(int j = 0; j < cloud->height; j++) {
		for(int i = 0; i < cloud->width; i++) {
			if(isnan(p->z)) {
				p->x = float(((float)i - KINECT_CX_D) * KINECT_FX_D);
				p->y = float(((float)j - KINECT_CY_D) * KINECT_FY_D);
				p->z = 0;
				p->normal_x = p->normal_y = p->normal_z = 0;

			}
			//p->a = 255;
			++p;
		}
	}
}

void RGBDTSegmentation::AddSlice(const PointCloud<PointXYZRGBA>::ConstPtr &in, 
								 pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
								 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_color) {
									 boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
									 copyPointCloud(*in,*cloud);
									 MakeCloudDense(*cloud);
									 if(options.use_time) {
										 if(options.use_fast_method || options.number_of_frames == 1) {
											 seg3d.AddSlice(*cloud, options.sigma_depth, options.c_depth, options.depth_min_size, options.sigma_color, options.c_color, options.color_min_size, out, out_color);
										 } else {
											 seg4d.AddSlice(*cloud, options.sigma_depth, options.c_depth, options.depth_min_size, options.max_depth, options.sigma_color, options.c_color, options.color_min_size);
											 *out = seg4d.labels[curr];
											 *out_color = seg4d.labels_colored[curr];
											 ++curr;
											 if(curr == 8)
												 curr = 4;
										 }
									 } else if(options.use_normals) {
										 boost::shared_ptr<pcl::PointCloud<pcl::PointNormal> > normals;
										 EstimateNormals(cloud, normals, false);
										 MakeCloudDense(normals);
										 SegmentColorAndNormals(*cloud, normals, options.sigma_depth, options.sigma_color, options.c_normals, options.normals_min_size, out, out_color);
									 } else {
										 SHGraphSegment(*cloud, options.sigma_depth, options.c_depth, options.depth_min_size, options.sigma_color, options.c_color, options.color_min_size, out, out_color);
									 }
}
