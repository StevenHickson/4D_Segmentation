/*
Copyright (C) 2014 Steven Hickson

MIT License
*/
#ifndef SEGMENTS_H
#define SEGMENTS_H

#include "OpticalFlow.h"
#include "RegionTree.h"
#include <deque>

class Segment3D {
private:
	RegionTree3D prevTree, currTree;
	//std::vector<Region3D*> prevList, currList;
	cv::Vec3b *m_colors;
	bool m_init;
	int m_maxLabel, m_count;

	int Initialize(const pcl::PointCloud<pcl::PointXYZRGBA> &in, 
		float sigma_depth, 
		float c_depth, 
		int depth_min_size,
		float color_depth,
		float c_color, 
		int color_min_size,
		pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_color);

	void Merge();
public:
	Segment3D() : m_init(false), m_maxLabel(0) { };
	~Segment3D() { };

	int AddSlice(const pcl::PointCloud<pcl::PointXYZRGBA> &in, 
		float sigma_depth, 
		float c_depth, 
		int depth_min_size,
		float sigma_color,
		float c_color, 
		int color_min_size,
		pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_color);
};

class Segment4DBig {
private:
	//Universe prevU_C, currU_C, prevU_D, currU_D;
	//map<int, map<int, bool>> prevC_Map, prevD_Map;
	//Edge3D *prevE, *currE;
	std::deque< pcl::PointCloud<pcl::PointXYZRGBA> > clouds;
	RegionTree4DBig prevTree, currTree;
	std::vector<Region4DBig*> prevList, currList;
	std::deque< pcl::PointCloud<pcl::Normal> > flows;
	cv::Vec3b *m_colors;
	int m_init;
	int m_maxLabel;
	int m_merge_iter;

	void Merge();
	void Merge2();
public:
	pcl::PointCloud<pcl::PointXYZRGBA> labels_colored[NUM_FRAMES];
	pcl::PointCloud<pcl::PointXYZI> labels[NUM_FRAMES], labels_old[NUM_FRAMES];

	Segment4DBig() : m_init(0), m_maxLabel(0) { 
		//clouds.reserve(NUM_FRAMES);
		//flows.reserve(NUM_FRAMES);
		m_merge_iter = NUM_FRAMES / 2;
	};
	~Segment4DBig() { };


	//uses linear combination
	int AddSlice(const pcl::PointCloud<pcl::PointXYZRGBA> &in, 
		float alpha,
		float sigma_depth, 
		float sigma_color,
		float c, 
		int min_size);

	//uses two-step
	int AddSlice(const pcl::PointCloud<pcl::PointXYZRGBA> &in, 
		float sigma_depth, 
		float c_depth, 
		int depth_min_size,
		float max_depth,
		float sigma_color,
		float c_color, 
		int color_min_size);
};


#endif //SEGMENTS_H
