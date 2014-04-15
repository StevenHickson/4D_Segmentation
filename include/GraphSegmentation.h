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

#ifndef GRAPH_SEGMENTATION_H
#define GRAPH_SEGMENTATION_H

#include <vector>
#include "OpticalFlow.h"
#include "Edges.h"
#include "Segments.h"
#include "BuildGraph.h"
#include "RegionTree.h"
#include <amp.h>
#include <amp_math.h>

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

int SegmentNormals(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, 
	const pcl::PointCloud<pcl::PointNormal>::ConstPtr &in, 
	float sigma, 
	float c, 
	int min_size,
	pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_color);

int SegmentColorAndNormals(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, 
	const pcl::PointCloud<pcl::PointNormal>::ConstPtr &in, 
	float sigma_normal, 
	float sigma_color,
	float c, 
	int min_size,
	pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_color);

int SHGraphSegment(
	pcl::PointCloud<pcl::PointXYZRGBA> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	pcl::PointCloud<pcl::PointXYZI> *out,
	pcl::PointCloud<pcl::PointXYZRGBA> *out_color);

#endif //GRAPH_SEGMENTATION_H