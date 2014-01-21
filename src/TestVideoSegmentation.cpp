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
/*#include <FaceTrackLib.h>
#include <KinectInteraction.h>
#include <NuiKinectFusionApi.h>
#include <NuiKinectFusionDepthProcessor.h>
#include <NuiKinectFusionVolume.h>*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Edges.h"
#include "OpticalFlow.h"
#include "GraphSegmentation.h"

using namespace std;
using namespace pcl;
using namespace cv;

void Display2dOF(Mat flow) {
	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

	//translate magnitude to range [0;1]
	double mag_min, mag_max;
	cv::minMaxLoc(magnitude, &mag_min, &mag_max);
	magnitude.convertTo(magnitude, -1, 1.0/mag_max);

	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = angle;
	_hsv[1] = Mat::ones(angle.size(), CV_32F);
	_hsv[2] = magnitude;
	cv::merge(_hsv, 3, hsv);

	//convert to BGR and show
	Mat bgr;//CV_32FC3 matrix
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	imshow("Optical flow",bgr);
}

class SimpleOFViewer
{
public:
	SimpleOFViewer () : viewer(new pcl::visualization::PCLVisualizer ("PCL Microsoft Viewer")), normals(new pcl::PointCloud<pcl::Normal>), 
		sharedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), first(true), update(false) {}

	void cloud_cb_ (const boost::shared_ptr<const KinectData> &data)
	{
		if(!data->cloud.empty()) {
			normalMutex.lock();
			if(!first) {
				//double begin = pcl::getTime();
				ComputeOpticalFlowGPU(past,data->image,sharedCloud,data->cloud.makeShared(),normals);
				//double end = pcl::getTime();
				//cout << "Time: " << (end - begin) << endl;
				update = true;
			} else
				first = false;
			past = data->image;
			copyPointCloud(data->cloud,*sharedCloud);
			normalMutex.unlock();
		}
	}

	void run ()
	{
		// create a new grabber for OpenNI devices
		pcl::Grabber* my_interface = new pcl::MicrosoftGrabber();

		// make callback function from member function
		boost::function<void (const boost::shared_ptr<const KinectData>&)> f =
			boost::bind (&SimpleOFViewer::cloud_cb_, this, _1);

		my_interface->registerCallback (f);

		//viewer.setBackgroundColor(0.0, 0.0, 0.5);
		my_interface->start ();
		boost::this_thread::sleep (boost::posix_time::seconds (1));
		while (!viewer->wasStopped())
		{
			normalMutex.lock();
			if(update) {
				viewer->removePointCloud("cloud");
				viewer->removePointCloud("original");
				viewer->addPointCloud(sharedCloud,"original");
				viewer->addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(sharedCloud, normals);
				update = false;
			}
			viewer->spinOnce();
			normalMutex.unlock();
		}

		my_interface->stop ();
	}

	boost::shared_ptr<pcl::PointCloud<pcl::Normal> > normals;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > sharedCloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	Mat past;
	volatile bool first, update;
	boost::mutex normalMutex;
};

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

inline void minMax(const PointCloud<PointNormal>::ConstPtr &cloud, PointNormal *min, PointNormal *max) {
	PointCloud<PointNormal>::const_iterator p = cloud->begin();
	min->normal_x = max->normal_x = p->normal_x;
	min->normal_y = max->normal_y = p->normal_y;
	min->normal_z = max->normal_z = p->normal_z;
	for(int j = 0; j < cloud->height; j++) {
		for(int i = 0; i < cloud->width; i++) {
			if(!_isnan(p->normal_x)) {
				if(_isnan(min->normal_x))
					min->normal_x = p->normal_x;
				if(_isnan(max->normal_x))
					max->normal_x = p->normal_x;
				if(p->normal_x < min->normal_x)
					min->normal_x = p->normal_x;
				if(p->normal_x > max->normal_x)
					max->normal_x = p->normal_x;
			}
			if(!_isnan(p->normal_y)) {
				if(_isnan(min->normal_y))
					min->normal_y = p->normal_y;
				if(_isnan(max->normal_y))
					max->normal_y = p->normal_y;
				if(p->normal_y < min->normal_y)
					min->normal_y = p->normal_y;
				if(p->normal_y > max->normal_y)
					max->normal_y = p->normal_y;
			}
			if(!_isnan(p->normal_z)) {
				if(_isnan(min->normal_z))
					min->normal_z = p->normal_z;
				if(_isnan(max->normal_z))
					max->normal_z = p->normal_z;
				if(p->normal_z < min->normal_z)
					min->normal_z = p->normal_z;
				if(p->normal_z > max->normal_z)
					max->normal_z = p->normal_z;
			}
			++p;
		}
	}
}

inline void EstimateNormals(const PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointCloud<PointNormal>::Ptr &normals) {
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
}

class SimpleSegmentViewer
{
public:
	SimpleSegmentViewer () : viewer("Original Viewer"), 
		label(new pcl::PointCloud<pcl::PointXYZI>), segment(new pcl::PointCloud<pcl::PointXYZRGBA>), sharedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), normals (new pcl::PointCloud<pcl::PointNormal>), update(false) {}

	void cloud_cb_ (const boost::shared_ptr<const KinectData> &data)
	{
		if(!data->cloud.empty()) {
			normalMutex.lock();
			double begin = pcl::getTime();
			PointCloud<PointXYZRGBA> cloud = data->cloud;
			//const KinectData* convert = data.get();
			sharedCloud = (data->cloud.makeShared());
			EstimateNormals(sharedCloud,normals);
			/*PointNormal min, max;
			minMax(normals,&min,&max);*/
			MakeCloudDense(cloud);
			//MakeCloudDense(normals);
			//io::savePLYFileASCII<PointXYZRGBA>("test2.ply",cloud);
			stseg.AddSlice(cloud,2.5f,700,800,0.5f,10,200,label,segment);
			//SegmentNormals(cloud,normals,0.5f,200,200,label,segment);
			double end = pcl::getTime();
			cout << "Time: " << (end - begin) << endl;
			//io::savePLYFileASCII<PointXYZRGBA>("test3.ply",*segment);
			//pcl::io::savePCDFile("output.pcd",*segment);
			//viewer1->showCloud(data->cloud.makeShared());
			//viewer.showCloud(segment);
			//copyPointCloud(data->cloud,*sharedCloud);
			update = true;
			normalMutex.unlock();
		}
	}

	void run ()
	{
		// create a new grabber for OpenNI devices
		pcl::Grabber* my_interface = new pcl::MicrosoftGrabber();

		// make callback function from member function
		boost::function<void (const boost::shared_ptr<const KinectData>&)> f =
			boost::bind (&SimpleSegmentViewer::cloud_cb_, this, _1);

		my_interface->registerCallback (f);

		//viewer.setBackgroundColor(0.0, 0.0, 0.5);
		my_interface->start ();

		bool finished = false;
		while (!viewer.wasStopped())
		{
			normalMutex.lock();
			if(update) {
				viewer.removePointCloud("cloud");
				viewer.removePointCloud("original");
				viewer.addPointCloud(segment,"original");
				viewer.addPointCloudNormals<pcl::PointXYZRGBA,pcl::PointNormal>(sharedCloud, normals);
				update = false;
			}
			viewer.spinOnce();
			normalMutex.unlock();
		}

		my_interface->stop ();
	}

	boost::shared_ptr<PointCloud<PointXYZI> > label;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > segment, sharedCloud;
	boost::shared_ptr<pcl::PointCloud<pcl::PointNormal> > normals;
	pcl::visualization::PCLVisualizer viewer;
	bool update;
	boost::mutex normalMutex;
	Segment3D stseg;
};

int main (int argc, char** argv) {
	try {
		SimpleSegmentViewer v;
		v.run();
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
	return 0;
}