/*
Copyright (C) 2014 Steven Hickson

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

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
			if(_isnan(p->z)) {
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
			if(_isnan(p->z)) {
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

inline void EstimateNormals(const PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointCloud<PointNormal>::Ptr &normals, bool fill) {
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
	if(fill) {
		PointCloudNormal::iterator p = normals->begin();
		while(p != normals->end()) {
			if(_isnan(p->normal_x))
				p->normal_x = 0;
			if(_isnan(p->normal_y))
				p->normal_y = 0;
			if(_isnan(p->normal_z))
				p->normal_z = 0;
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
											 *out = seg4d.labels[7];
											 *out_color = seg4d.labels_colored[7];
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
