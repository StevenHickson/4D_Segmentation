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

class Seg4DViewer
{
public:

	Seg4DViewer() :  viewer(new pcl::visualization::PCLVisualizer ("PCL Segmentation Viewer")), cloud((new pcl::PointCloud<pcl::PointXYZRGBA>)), label(new pcl::PointCloud<pcl::PointXYZI>), label_color(new pcl::PointCloud<pcl::PointXYZRGBA>) { }
	
	void Seg4DExample() {
		SegmentationOptions options;
		//Set up options here

		RGBDTSegmentation segments(options);

		PCDReader reader;
		int i = 6;
		while(!stopped && i < 107) {
			//read the pointcloud file
			stringstream fileName;
			fileName << data_folder;
			fileName << i;
			fileName << ".pcd";
			reader.read<pcl::PointXYZRGBA> (fileName.str(), *cloud);
			cloud->width = 640;
			cloud->height = 480;

			//run the segmentation
			//cloudMutex.lock();
			segments.AddSlice(cloud, label, label_color);
			update = true;
			//cloudMutex.unlock();

			++i;
		}
	}

	void run(string input) {
		data_folder = input;
		stopped = false;
		boost::thread* readerThread = new boost::thread(boost::bind(&Seg4DViewer::Seg4DExample, this));
		while(!viewer->wasStopped()) {
			//cloudMutex.lock();
			if(update) {
				//display the result
				viewer->removePointCloud("cloud");
				viewer->addPointCloud(label_color);
				update = false;
			}
			viewer->spinOnce();
			//cloudMutex.unlock();
		}
		stopped = true;
	}

	string data_folder;
	volatile bool stopped, update;
	boost::mutex cloudMutex;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr label;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr label_color;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

int main (int argc, char** argv) {
	try {
		Seg4DViewer example;
		example.run(argv[1]);
		cout << "Done" << endl;
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
	return 0;
}