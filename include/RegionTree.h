/*
Copyright (C) 2012 Steven Hickson

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
#include "blepo.h"
#include "OpticalFlow.h"
#include "Edges.h"

using namespace blepo;

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
//RANGE = -0.2 - 0.2
#define HIST_MUL_OF 50.0f //might need one of these just for w.

//Modifier for Depth in Histogram Difference
#define HIST_DEPTH_MOD 1.5f

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

typedef struct
{
	int rank;
	int p;
	int size;
} uni_elt;

class Universe
{
public:
	Universe() : num(0) { }
	Universe(int elements)
	{
		num = elements;
		elts.resize(num);
		std::vector<uni_elt>::iterator p = elts.begin();
		int i = 0;
		while(p != elts.end()) {
			p->rank = 0;
			p->size = 1;
			p->p = i;
			p++;
			i++;
		}
	}
	~Universe(){};
	int find(int x)
	{
		int y = x;
		while (y != elts[y].p)
			y = elts[y].p;
		elts[x].p = y;
		return y;
	};  
	void join(int x, int y)
	{
		if (elts[x].rank > elts[y].rank)
		{
			elts[y].p = x;
			elts[x].size += elts[y].size;
		} 
		else
		{
			elts[x].p = y;
			elts[y].size += elts[x].size;
			if (elts[x].rank == elts[y].rank)
				elts[y].rank++;
		}
		num--;
	}
	void release() {
		elts.clear();
	}
	int size(int x) const { return elts[x].size; }
	int num_sets() const { return num; }
	//should be private but I need to access some things
	std::vector<uni_elt>elts;
	int num;
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

class Region {
public:
	Region** m_regions; //regions beneath this one
	vector<int*> m_nodes;
	int m_numRegions; //number of regions
	int m_level; //tree level (1 is the lowest that is a region)
	unsigned int m_size; //Size of my region
	int m_label; //region label
	Point m_centroid; //if m_leaf 2D location, else centroid
	LAB *m_hist;
	vector<int> m_neighbors; //neighboring regions

	inline void InitializeRegion(int *in, Bgr &color, const int label, const int i, const int j, const int level);
	inline void InitializeRegion(Region *node1, Region *node2, int level, int min_size);
	inline void AddNode(int *in, Bgr &color, const int i, const int j);

	void Release() {
		if(m_numRegions > 0 && m_regions != NULL) {
			for(int i = 0; i < m_numRegions;i++) {
				m_regions[i]->Release();
				delete m_regions[i];
			}
		}
		delete m_regions;
		delete m_hist;
		m_nodes.clear();
		m_neighbors.clear();
	}
};

class Region3D {
public:
	Region3D* m_regions[2]; //regions beneath this one
	vector<Point3D<int>*> m_nodes;
	int m_numRegions; //number of regions
	int m_level; //tree level (1 is the lowest that is a region)
	unsigned int m_size; //Size of my region
	//int m_label; //region label
	Point m_centroid; //if m_leaf 2D location, else centroid
	Point3D<int> m_centroid3D; //if a leaf (level 1), 3D location and value, else centroid and label
	LABXYZ *m_hist;
	LABXYZ *m_hist_array;
	unsigned int m_hist_num;
	vector<int> m_neighbors; //neighboring regions

	inline void InitializeRegion(Point3D<int> *in, Bgr &color, const int label, const int i, const int j, const int level);
	inline void InitializeRegion(Region3D *node1, Region3D *node2, int level, int min_size);
	inline void AddNode(Point3D<int> *in, Bgr &color, const int i, const int j);

	bool operator<(const Region3D &other) const {
		return m_centroid3D.value < other.m_centroid3D.value;
	}

	void Release() {
		m_numRegions = m_level = -1;
		m_size = 0;
		//printf("Deleting hist\n");
		if(m_hist != NULL) {
			delete m_hist;
			m_hist = NULL;
		}
		m_regions[0] = m_regions[1] = NULL;
		m_nodes.clear();
		m_neighbors.clear();
	}
};

class Region4D {
public:
	Region4D* m_regions[2]; //regions beneath this one
	vector<Point3D<int>*> m_nodes;
	int m_numRegions; //number of regions
	int m_level; //tree level (1 is the lowest that is a region)
	unsigned int m_size; //Size of my region
	//int m_label; //region label
	Point m_centroid; //if m_leaf 2D location, else centroid
	Point3D<int> m_centroid3D; //if a leaf (level 1), 3D location and value, else centroid and label
	LABXYZUVW *m_hist;
	vector<int> m_neighbors; //neighboring regions

	//Need to change to all LAB and need to use a lot less new() calls (make the histogram one big array with NUM_BINS * 2 * n + 1 being the size
	inline void InitializeRegion(Point3D<int> *in, Bgr &color, Flow &flow, const int label, const int i, const int j, const int level);
	inline void InitializeRegion(Region4D *node1, Region4D *node2, int level, int min_size);
	inline void AddNode(Point3D<int> *in, Bgr &color, Flow &flow, const int i, const int j);

	void Release() {
		if(m_numRegions > 0 && m_regions != NULL) {
			for(int i = 0; i < m_numRegions;i++) {
				m_regions[i]->Release();
				delete m_regions[i];
			}
		}
		//delete m_regions;
		if(m_hist != NULL)
			delete m_hist;
		m_nodes.clear();
		m_neighbors.clear();
	}
};

class Region4DBig {
public:
	Region4DBig* m_regions[2]; //regions beneath this one
	vector<Point3D<int>*> m_nodes;
	int m_numRegions; //number of regions
	int m_level; //tree level (1 is the lowest that is a region)
	unsigned int m_size; //Size of my region
	//int m_label; //region label
	Point m_centroid; //if m_leaf 2D location, else centroid
	Point3D<int> m_centroid3D; //if a leaf (level 1), 3D location and value, else centroid and label
	LABXYZUVW *m_hist;
	vector<int> m_neighbors; //neighboring regions
	map<int, Region4DBig*> m_neighbor_map;

	//Need to change to all LAB and need to use a lot less new() calls (make the histogram one big array with NUM_BINS * 2 * n + 1 being the size
	inline void InitializeRegion(Point3D<int> *in, Bgr &color, Flow &flow, const int label, const int i, const int j, const int level);
	inline void InitializeRegion(Region4DBig *node1, Region4DBig *node2, int level, int min_size);
	inline void AddNode(Point3D<int> *in, Bgr &color, Flow &flow, const int i, const int j);

	bool operator<(const Region4DBig &other) const {
		return m_centroid3D.value < other.m_centroid3D.value;
	}

	void Release() {
		if(m_numRegions > 0 && m_regions != NULL) {
			for(int i = 0; i < m_numRegions;i++) {
				m_regions[i]->Release();
				delete m_regions[i];
			}
		}
		//delete m_regions;
		if(m_hist != NULL)
			delete m_hist;
		m_nodes.clear();
		m_neighbors.clear();
	}
};

template<class T, class HistContainer, class ColorContainer, class LabelContainer>
class RegionTreeType {
public:
	T** m_nodes;
	//T *region_list;
	vector<T> region_list;
	int m_size,m_width,m_height;
	bool m_propagated;
	int numRegions, totRegions; //temporary, lets add every time we new

	HistContainer *m_hist_array; //NUM_BINS * size;
	unsigned int m_hist_num;
	//PointCloudBgr m_cloud;

	RegionTreeType() : m_size(0), m_width(0), m_height(0), m_nodes(NULL), m_propagated(false) { }
	RegionTreeType(int regions, int width, int height) : m_size(regions), m_width(width), m_height(height), m_propagated(false) {
		//need to figure out why this is
		m_nodes = new T*[(regions + 2) << 1 + 1]();
	}

	void Release() {
		/*printf("region_list 5: %p\n",region_list);
		if(region_list != NULL) {
		//Need to call region release for each region
		for(int i = 0; i < numRegions; i++)
		region_list[i].Release();
		printf("region_list 6: %p\n",region_list);
		//free(region_list);
		//delete region_list;
		region_list = NULL;
		}*/
		/*for(int i = 0; i < region_list.size(); i++)
		region_list[i].Release();*/
		/*region_list.clear();
		if(m_nodes != NULL) {
		delete[] m_nodes;
		m_nodes = NULL;
		}
		m_size = m_width = m_height = 0;
		m_propagated = false;*/
	}

	void Create(ColorContainer &in, LabelContainer &labels, int num_segments, int start_label);
	void Create(PointCloudBgr &in1, PointCloudInt &labels1, PointCloudBgr &in2, PointCloudInt &labels2, FlowInfo &flow, int num_segments, int start_label);
	void Create(deque<PointCloudBgr> &in, deque<FlowInfo> &flow, PointCloudInt *labels, int num_segments, int start_label);
	void UpdateCloud(int level);
	void GetRegionList(float level, vector<T*> *list);
	void UpdateRegionList(vector<T*> &list);
	void TemporalCorrection(RegionTreeType<T,HistContainer,ColorContainer,LabelContainer> &past, int level);
	void TemporalCorrection(vector<T*> &past_list, int level);
	void TemporalCorrection(vector<T*> &past_list, vector<T*> &curr_list, int level);
	void PropagateRegionHierarchy(int min_size = 0);

	void ImplementSegmentation(float level) {
		if(!m_propagated)
			PropagateRegionHierarchy();
		//now that the tree is fully built, when can select the level using the percentage the user selected
		int loc = level * (*m_nodes)->m_level;
		UpdateCloud(loc);
	}
};

typedef RegionTreeType<Region,LAB,ImgBgr,ImgInt> RegionTree;
typedef RegionTreeType<Region3D,LABXYZ,PointCloudBgr,PointCloudInt> RegionTree3D;
typedef RegionTreeType<Region4D,LABXYZUVW,PointCloudBgr,PointCloudInt> RegionTree4D;
typedef RegionTreeType<Region4DBig,LABXYZUVW,PointCloudBgr,PointCloudInt> RegionTree4DBig;


#endif