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
#ifndef REGION_TREE_H
#define REGION_TREE_H

#include "OpticalFlow.h"
#include "Edges.h"
#include <vector>
#include <amp.h>
#include <amp_math.h>

#define NUM_BINS 20
#define NUM_BINS_XYZ 30
//HIST_MUL = NUM BINS / (RANGE)

//RANGE = 100
#define HIST_MUL_L 0.2f
//RANGE = 257
#define HIST_MUL_A 0.0778210116731517509727626459144f
#define HIST_MUL_B 0.0778210116731517509727626459144f
//RANGE = -3 - 3
#define HIST_MUL_X 5.0f
//RANGE = -2 - 2
#define HIST_MUL_Y 7.5f
//RANGE = 0 - 6
#define HIST_MUL_Z 5.0f
//RANGE = -1 - 1
#define HIST_MUL_N 10.0f //might need one of these just for w.

//Modifier for Depth in Histogram Difference
#define HIST_DEPTH_MOD 1.0f
#define HIST_NORMAL_MOD 0.75f
#define HIST_COLOR_MOD 1.25f

#define SIZE_MUL 0.5f
#define CENTROID_MUL 50

//minimum SAD distance a region can move in t
#define MIN_REGION_DIST 0.003f
//minimum SAD Hist Distance a region can change in t (3.4 for noncombined method)
#define MIN_REGION_HIST 3.4f
//minimum size difference a region can change (in percent) in t
#define MIN_REGION_SIZE 5.0f
//max converging size to allow small segments to merge
#define MAX_CONVERGING_SIZE 5000

#define PRIOR_MUL 0.5f

#define NUM_FRAMES 8

#define TREE_LEVEL 0.5f

class LAB {
public:
	int l, a, b;

	LAB() : l(0), a(0), b(0) { }
	LAB(int r_val, int g_val, int b_val);
	inline void RGB2XYZ(int r, int g, int b, float *x, float *y, float *z);
	inline void XYZ2LAB(float x, float y, float z, int *l, int *a, int *b);
};

class LABXYZUVW {
public:
	int l, a, b;
	float x, y, z, u, v, w;

	LABXYZUVW() : l(0), a(0), b(0), x(0.0f), y(0.0f), z(0.0f), u(0.0f), v(0.0f), w(0.0f) { }
	//LABD(int l_val, int a_val, int b_val, float d_val) : l(l_val), a(a_val), b(b_val), d(d_val) { }
	LABXYZUVW(int r_val, int g_val, int b_val, float x_val, float y_val, float z_val, float u_val, float v_val, float w_val);
};

class LABXYZ {
public:
	int l, a, b;
	float x, y, z;

	LABXYZ() : l(0), a(0), b(0), x(0.0f), y(0.0f), z(0.0f) { }
	//LABD(int l_val, int a_val, int b_val, float d_val) : l(l_val), a(a_val), b(b_val), d(d_val) { }
	LABXYZ(int r_val, int g_val, int b_val, float x_val, float y_val, float z_val);
};

class Region3D {
public:
	Region3D() : m_numRegions(0), m_level(0), m_size(0), m_hist(NULL) { m_regions[0] = NULL; m_regions[1] = NULL; };
	Region3D* m_regions[2]; //regions beneath this one
	std::vector<pcl::PointXYZI*> m_nodes;
	int m_numRegions; //number of regions
	int m_level; //tree level (1 is the lowest that is a region)
	unsigned int m_size; //Size of my region
	//int m_label; //region label
	cv::Point2f m_centroid; //if m_leaf 2D location, else centroid
	pcl::PointXYZI m_centroid3D, m_min3D, m_max3D; //if a leaf (level 1), 3D location and value, else centroid and label
	LABXYZUVW *m_hist;
	std::vector<int> m_neighbors; //neighboring regions

	inline void InitializeRegion(pcl::PointXYZI *in, cv::Vec3b &color, const pcl::PointNormal &normal, int label, const int i, const int j, const int level);
	inline void InitializeRegion(Region3D *node1, Region3D *node2, int level, int min_size);
	inline void AddNode(pcl::PointXYZI *in, cv::Vec3b &color, const pcl::PointNormal &normal, const int i, const int j);

	bool operator<(const Region3D &other) const {
		return m_centroid3D.intensity < other.m_centroid3D.intensity;
	}

	void Release() {
		if(m_numRegions > 0 && m_regions != NULL) {
			for(int i = 0; i < m_numRegions; i++) {
				if(m_regions[i] != NULL) {
					m_regions[i]->Release();
					delete m_regions[i];
				}
			}
			if(m_hist != NULL) {
				delete[] m_hist;
				m_hist = NULL;
			}
			m_regions[0] = m_regions[1] = NULL;
		}
		m_numRegions = m_level = -1;
		m_size = 0;
		m_nodes.clear();
		m_neighbors.clear();
	}
};

template<class T, class HistContainer, class ColorContainer, class LabelContainer>
class RegionTreeType {
public:
	T** m_nodes;
	//T *region_list;
	std::vector<T*> region_list;
	std::vector<T*> top_regions;
	int m_size,m_width,m_height;
	bool m_propagated;
	int numRegions, totRegions; //temporary, lets add every time we new

	HistContainer *m_hist_array; //NUM_BINS * size;
	unsigned int m_hist_num;
	//PointCloudBgr m_cloud;

	RegionTreeType() : m_size(0), m_width(0), m_height(0), m_nodes(NULL), m_propagated(false) { }
	RegionTreeType(int regions, int width, int height) : m_size(regions), m_width(width), m_height(height), m_propagated(false) {
		//need to figure out why this is
		int tmp = (regions + 2) << 1 + 1;
		m_nodes = new T*[tmp]();
		for(int i = 0; i < tmp; i++)
			m_nodes[i] = NULL;
	}

	void Release() {
		//Need to call region release for each region
		/*for(int i = 0; i < region_list.size(); i++)
		region_list[i].Release();*/
		if(m_nodes != NULL) {
			(*m_nodes)->Release();
			delete[] m_nodes;
			m_nodes = NULL;
		}
		region_list.clear();
		m_size = m_width = m_height = 0;
		m_propagated = false;
	}

	void Create(const ColorContainer &in, LabelContainer &labels, const pcl::PointCloud<pcl::PointNormal> &normals, int num_segments, int start_label);
	void TemporalCorrection(RegionTreeType<T,HistContainer,ColorContainer,LabelContainer> &past, int level);
	void PropagateRegionHierarchy(int min_size = 0);
	void UpdateCloud(int level);
	/*
	void GetRegionList(float level, vector<T*> *list);
	void UpdateRegionList(vector<T*> &list);
	void TemporalCorrection(vector<T*> &past_list, int level);
	void TemporalCorrection(vector<T*> &past_list, vector<T*> &curr_list, int level);
	*/

	void ImplementSegmentation(float level) {
		if(!m_propagated)
			PropagateRegionHierarchy();
		//now that the tree is fully built, when can select the level using the percentage the user selected
		int loc = level * (*m_nodes)->m_level;
		UpdateCloud(loc);
	}
};

typedef RegionTreeType<Region3D,LABXYZUVW,pcl::PointCloud<pcl::PointXYZRGBA>,pcl::PointCloud<pcl::PointXYZI> > RegionTree3D;

#endif //REGION_TREE_H