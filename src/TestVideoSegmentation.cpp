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

#include "TestVideoSegmentation.h"

using namespace std;
using namespace pcl;
using namespace cv;

void Seg4DExample(string data_folder) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("PCL Segmentation Viewer"));
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr label (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr label_color (new pcl::PointCloud<pcl::PointXYZRGBA>);

	SegmentationOptions options;
	//Set up options here

	RGBDTSegmentation segments(options);

	PCDReader reader;
	int i = 6;
	while(!viewer->wasStopped() && i < 107) {
		//read the pointcloud file
		stringstream fileName;
		fileName << data_folder;
		fileName << i;
		fileName << ".pcd";
		reader.read<pcl::PointXYZRGBA> (fileName.str(), *cloud);
		cloud->width = 640;
		cloud->height = 480;

		//run the segmentation
		segments.AddSlice(cloud, label, label_color);

		//display the result
		viewer->removePointCloud("cloud");
		viewer->addPointCloud(label_color);
		viewer->spinOnce();
		++i;
	}
}

int main (int argc, char** argv) {
	try {
		Seg4DExample(argv[1]);
		cout << "Done" << endl;
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
	return 0;
}