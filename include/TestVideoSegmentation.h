#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Microsoft_grabber.h"
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Edges.h"
#include "OpticalFlow.h"
#include "GraphSegmentation.h"

#include <Shlwapi.h>
#include <string.h>

inline void EstimateNormals(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &normals, bool fill = false);
void BuildNYUDataset(std::string direc);

#define NUM_LABELS 894