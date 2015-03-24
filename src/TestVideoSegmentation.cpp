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
			PointNormal min, max;
			minMax(normals,&min,&max);
			MakeCloudDense(cloud);
			//MakeCloudDense(normals);
			stseg.AddSlice(cloud,2.5f,700,800,0.5f,10,200,label,segment);
			//SegmentNormals(cloud,normals,0.5f,300,300,label,segment);
			SegmentColorAndNormals(cloud,normals,0.8f,0.5f,25,50,label,segment);
			double end = pcl::getTime();
			cout << "Time: " << (end - begin) << endl;
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
		//SimpleSegmentViewer v;
		//v.run();
		BuildNYUDataset(string(argv[1]));
		cout << "Done" << endl;
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
	return 0;
}