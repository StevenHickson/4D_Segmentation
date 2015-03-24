#include "OpticalFlow.h"

using namespace cv;
using namespace cv::gpu;
using namespace pcl;
using namespace std;

void ComputeOpticalFlow(const Mat &past, const Mat &current, const PointCloud<PointXYZRGBA>::ConstPtr &pastCloud, const PointCloud<PointXYZRGBA>::ConstPtr &currCloud, PointCloud<Normal>::Ptr &flow) {
	Mat in1, in2, flow2d;
	cvtColor(past,in1,CV_BGR2GRAY);
	cvtColor(current,in2,CV_BGR2GRAY);
	calcOpticalFlowFarneback(in1,in2,flow2d,0.5f,2,5,2,7,1.5,0);
	flow->height = flow2d.rows;
	flow->width = flow2d.cols;
	flow->is_dense = false;
	flow->resize(flow->height * flow->width);
	flow->sensor_origin_.setZero();
	PointCloud<Normal>::iterator pOut = flow->begin();
	PointCloud<PointXYZRGBA>::const_iterator pCloud = currCloud->begin();
	Mat_<Vec2f>::iterator pIn = flow2d.begin<Vec2f>();
	int safeWidth = pastCloud->width - 1, safeHeight = pastCloud->height - 1;
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(int j = 0; j < pastCloud->height; j++) {
		for(int i = 0; i < pastCloud->width ; i++) {
			if(pCloud->z == 0 || pCloud->z == bad_point) {
				pOut->normal_x = pOut->normal_y = pOut->normal_z = bad_point;
			} else {
				/*pOut->x = pCloud->x;
				pOut->y = pCloud->y;
				pOut->z = pCloud->z;*/
				pOut->normal_x = (*pIn)[0] * pCloud->z * KINECT_FX_D;
				pOut->normal_y = (*pIn)[1] * pCloud->z * KINECT_FY_D;
				pOut->normal_z = (pCloud->z - (*pastCloud)(Clamp(int(i - (*pIn)[0]),0,safeWidth), Clamp(int(j - (*pIn)[1]),0,safeHeight)).z);
			}
			++pIn; ++pOut; ++pCloud;
		}
	}
}

void ComputeOpticalFlow(const PointCloud<PointXYZRGBA> &pastCloud, const PointCloud<PointXYZRGBA> &currCloud, PointCloud<Normal> *flow) {
	Mat flow2d;
	Mat in1 = Mat(pastCloud.height,pastCloud.width,CV_8UC1);
	Mat in2 = Mat(pastCloud.height,pastCloud.width,CV_8UC1);
	PointCloud<PointXYZRGBA>::const_iterator pPast = pastCloud.begin(), pCloud = currCloud.begin();
	Mat_<char>::iterator pi1 = in1.begin<char>(), pi2 = in2.begin<char>();
	int tmp;
	while(pi1 != in1.end<char>()) {
		tmp = (pPast->b + 6*pPast->g + 3*pPast->r) / 10;
		*pi1 = min(max(tmp, 0), 255);
		tmp = (pCloud->b + 6*pCloud->g + 3*pCloud->r) / 10;
		*pi2 = min(max(tmp, 0), 255);
		++pi1; ++pi2; ++pPast; ++pCloud;
	}
	calcOpticalFlowFarneback(in1,in2,flow2d,0.5f,2,5,2,7,1.5,0);
	flow->height = flow2d.rows;
	flow->width = flow2d.cols;
	flow->is_dense = false;
	flow->resize(flow->height * flow->width);
	flow->sensor_origin_.setZero();
	PointCloud<Normal>::iterator pOut = flow->begin();
	pCloud = currCloud.begin();
	Mat_<Vec2f>::iterator pIn = flow2d.begin<Vec2f>();
	int safeWidth = pastCloud.width - 1, safeHeight = pastCloud.height - 1;
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(int j = 0; j < pastCloud.height; j++) {
		for(int i = 0; i < pastCloud.width ; i++) {
			if(pCloud->z == 0 || pCloud->z == bad_point) {
				pOut->normal_x = pOut->normal_y = pOut->normal_z = bad_point;
			} else {
				/*pOut->x = pCloud->x;
				pOut->y = pCloud->y;
				pOut->z = pCloud->z;*/
				pOut->normal_x = (*pIn)[0] * pCloud->z * KINECT_FX_D;
				pOut->normal_y = (*pIn)[1] * pCloud->z * KINECT_FY_D;
				pOut->normal_z = (pCloud->z - pastCloud(Clamp(int(i - (*pIn)[0]),0,safeWidth), Clamp(int(j - (*pIn)[1]),0,safeHeight)).z);
			}
			++pIn; ++pOut; ++pCloud;
		}
	}
}

//void ComputeOpticalFlowGPU(const Mat &past, const Mat &current, const PointCloud<PointXYZRGBA>::ConstPtr &pastCloud, const PointCloud<PointXYZRGBA>::ConstPtr &currCloud, PointCloud<Normal>::Ptr &flow) {
//	try {
//	Mat in1, in2;
//	cvtColor(past,in1,CV_BGR2GRAY);
//	cvtColor(current,in2,CV_BGR2GRAY);
//	GpuMat d_frameL(in1), d_frameR(in2);
//	GpuMat d_flowx, d_flowy;
//	cv::gpu::FarnebackOpticalFlow calc_flow;
//	Mat flowx, flowy;
//	calc_flow(d_frameL, d_frameR, d_flowx, d_flowy);
//	d_flowx.download(flowx);
//	d_flowy.download(flowy);
//	//calcOpticalFlowFarneback(in1,in2,flow2d,0.5f,2,5,2,7,1.5,0);
//	flow->height = flowx.rows;
//	flow->width = flowx.cols;
//	flow->is_dense = false;
//	flow->resize(flow->height * flow->width);
//	flow->sensor_origin_.setZero();
//	PointCloud<Normal>::iterator pOut = flow->begin();
//	PointCloud<PointXYZRGBA>::const_iterator pCloud = currCloud->begin();
//	Mat_<float>::iterator pInX = flowx.begin<float>();
//	Mat_<float>::iterator pInY = flowy.begin<float>();
//	int safeWidth = pastCloud->width - 1, safeHeight = pastCloud->height - 1;
//	float bad_point = std::numeric_limits<float>::quiet_NaN ();
//	for(int j = 0; j < pastCloud->height; j++) {
//		for(int i = 0; i < pastCloud->width ; i++) {
//			if(pCloud->z == 0 || pCloud->z == bad_point) {
//				pOut->normal_x = pOut->normal_y = pOut->normal_z = bad_point;
//			} else {
//				/*pOut->x = pCloud->x;
//				pOut->y = pCloud->y;
//				pOut->z = pCloud->z;*/
//				pOut->normal_x = *pInX * pCloud->z * KINECT_FX_D;
//				pOut->normal_y = *pInY * pCloud->z * KINECT_FY_D;
//				pOut->normal_z = (pCloud->z - (*pastCloud)(Clamp(int(i - *pInX),0,safeWidth), Clamp(int(j - *pInY),0,safeHeight)).z);
//			}
//			++pInX; ++pInY; ++pOut; ++pCloud;
//		}
//	}
//	} catch(cv::Exception &e) {
//		cout << e.what() << endl;
//	} catch(std::exception &e) {
//		cout << e.what() << endl;
//	} catch(...) {
//
//	}
//}
//

void Downsample2x2(const Mat &in, Mat &out) { resize(in,out,Size(),0.5f,0.5f); }