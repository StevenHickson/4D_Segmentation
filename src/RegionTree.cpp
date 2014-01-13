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
#include "stdafx.h"
#include "RegionTree.h"
//#include "GraphSegmentation.h"

LABXYZUVW::LABXYZUVW(int r_val, int g_val, int b_val, float x_val, float y_val, float z_val, float u_val, float v_val, float w_val) {
	LAB lab = LAB(r_val, g_val, b_val);
	l = lab.l;
	a = lab.a;
	b = lab.b;
	u = u_val + 0.2f;  //Need to shift these as well
	v = v_val + 0.2f;
	w = w_val + 6.2f;  //not sure about these numbers
	x = x_val + 3.0f; //to shift to all positive values
	y = y_val + 2.0f; //for the histogram
	z = z_val; //should minimize this one later to specify values that are within a small time range
}

LABXYZ::LABXYZ(int r_val, int g_val, int b_val, float x_val, float y_val, float z_val) {
	LAB lab = LAB(r_val, g_val, b_val);
	l = lab.l;
	a = lab.a;
	b = lab.b;
	x = x_val + 3.0f; //to shift to all positive values
	y = y_val + 2.0f; //for the histogram
	z = z_val;
}

inline void Region4D::InitializeRegion(Point3D<int> *in, Bgr &color, Flow &flow, const int label, const int i, const int j, const int level) {
	m_level  = level;
	m_size = 1;
	m_hist = new LABXYZUVW[NUM_BINS_XYZ]();
	LABXYZUVW labxyzuvw = LABXYZUVW(color.r,color.g,color.b,in->x, in->y, in->z, flow.u, flow.v, flow.w);
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.l * HIST_MUL_L),0,NUM_BINS)].l++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.a * HIST_MUL_A),0,NUM_BINS)].a++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.b * HIST_MUL_B),0,NUM_BINS)].b++;
	//Need to shift all these to make them positive
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.x * HIST_MUL_X),0,NUM_BINS_XYZ)].x++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.y * HIST_MUL_Y),0,NUM_BINS_XYZ)].y++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.z * HIST_MUL_Z),0,NUM_BINS_XYZ)].z++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.u * HIST_MUL_OF),0,NUM_BINS)].u++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.v * HIST_MUL_OF),0,NUM_BINS)].v++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.w * HIST_MUL_OF),0,NUM_BINS)].w++;
	m_centroid = Point(i,j);
	m_centroid3D.x = in->x;
	m_centroid3D.y = in->y;
	m_centroid3D.z = in->z;
	m_centroid3D.value = label;
	m_nodes.reserve(76800);
	m_neighbors.reserve(8);
	m_nodes.push_back(in);
	m_regions[0] = m_regions[1] = NULL;
	m_numRegions = 0;
}

inline void Region4D::InitializeRegion(Region4D *node1, Region4D *node2, int level, int min_size) {
	if(node1->m_size <= 0 || node2->m_size <= 0) {
		printf("Error, this should never happen\n");
	}
	/*if(node1->m_level == node2->m_level) {
	//m_level = node1->m_level + 1;
	if(node1->m_size > node2->m_size)
	m_centroid3D.value = node1->m_centroid3D.value;
	else
	m_centroid3D.value = node2->m_centroid3D.value;
	} else*/ if(node1->m_level > node2->m_level) {
		//m_level = node1->m_level + 1;
		m_centroid3D.value = node1->m_centroid3D.value;
	} else {
		//m_level = node2->m_level + 1;
		m_centroid3D.value = node2->m_centroid3D.value;
	}
	m_size = node1->m_size + node2->m_size;
	if(m_size == 0 || m_size > min_size)
		m_level = level;
	else
		m_level = 1;
	m_centroid.x = (node1->m_centroid.x * node1->m_size + node2->m_centroid.x * node2->m_size) / m_size;
	m_centroid.y = (node1->m_centroid.y * node1->m_size + node2->m_centroid.y * node2->m_size) / m_size;
	m_centroid3D.x = (node1->m_centroid3D.x * node1->m_size + node2->m_centroid3D.x * node2->m_size) / m_size;
	m_centroid3D.y = (node1->m_centroid3D.y * node1->m_size + node2->m_centroid3D.y * node2->m_size) / m_size;
	m_centroid3D.z = (node1->m_centroid3D.z * node1->m_size + node2->m_centroid3D.z * node2->m_size) / m_size;
	m_hist = new LABXYZUVW[NUM_BINS_XYZ]();
	LABXYZUVW *pHist = m_hist, *pHistFirstEnd = m_hist + NUM_BINS, *pHistEnd = m_hist + NUM_BINS_XYZ, *pHist1 = node1->m_hist, *pHist2 = node2->m_hist;
	while(pHist != pHistFirstEnd) {
		pHist->l = pHist1->l + pHist2->l;
		pHist->a = pHist1->a + pHist2->a;
		pHist->b = pHist1->b + pHist2->b;
		pHist->x = pHist1->x + pHist2->x;
		pHist->y = pHist1->y + pHist2->y;
		pHist->z = pHist1->z + pHist2->z;
		pHist->u = pHist1->u + pHist2->u;
		pHist->v = pHist1->v + pHist2->v;
		pHist->w = pHist1->w + pHist2->w;
		pHist++; pHist1++; pHist2++;
	}
	while(pHist != pHistEnd) {
		pHist->x = pHist1->x + pHist2->x;
		pHist->y = pHist1->y + pHist2->y;
		pHist->z = pHist1->z + pHist2->z;
		pHist++; pHist1++; pHist2++;
	}
	//I might want to add the neighbors later but for now I don't think it matters
	m_numRegions = 2;
	m_regions[0] = node1;
	m_regions[1] = node2;
}

inline void Region4D::AddNode(Point3D<int> *in, Bgr &color, Flow &flow, const int i, const int j) {
	LABXYZUVW labxyzuvw = LABXYZUVW(color.r,color.g,color.b,in->x, in->y, in->z, flow.u, flow.v, flow.w);
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.l * HIST_MUL_L),0,NUM_BINS)].l++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.a * HIST_MUL_A),0,NUM_BINS)].a++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.b * HIST_MUL_B),0,NUM_BINS)].b++;
	//Need to shift all these to make them positive
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.x * HIST_MUL_X),0,NUM_BINS_XYZ)].x++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.y * HIST_MUL_Y),0,NUM_BINS_XYZ)].y++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.z * HIST_MUL_Z),0,NUM_BINS_XYZ)].z++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.u * HIST_MUL_OF),0,NUM_BINS)].u++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.v * HIST_MUL_OF),0,NUM_BINS)].v++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.w * HIST_MUL_OF),0,NUM_BINS)].w++;
	/*int new_size = m_size + 1;
	m_centroid.x = (m_centroid.x * m_size + i) / new_size;
	m_centroid.y = (m_centroid.y * m_size + j) / new_size;
	m_centroid3D.x = (m_centroid3D.x * m_size + in->x) / new_size;
	m_centroid3D.y = (m_centroid3D.y * m_size + in->y) / new_size;
	m_centroid3D.z = (m_centroid3D.z * m_size + in->z) / new_size;
	m_size = new_size;*/
	m_centroid.x += i;
	m_centroid.y += j;
	m_centroid3D.x += in->x;
	m_centroid3D.y += in->y;
	m_centroid3D.z += in->z;
	m_size++;
	m_nodes.push_back(in);
}

inline void Region4DBig::InitializeRegion(Point3D<int> *in, Bgr &color, Flow &flow, const int label, const int i, const int j, const int level) {
	m_level  = level;
	m_size = 1;
	m_hist = new LABXYZUVW[NUM_BINS_XYZ]();
	LABXYZUVW labxyzuvw = LABXYZUVW(color.r,color.g,color.b,in->x, in->y, in->z, flow.u, flow.v, flow.w);
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.l * HIST_MUL_L),0,NUM_BINS)].l++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.a * HIST_MUL_A),0,NUM_BINS)].a++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.b * HIST_MUL_B),0,NUM_BINS)].b++;
	//Need to shift all these to make them positive
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.x * HIST_MUL_X),0,NUM_BINS_XYZ)].x++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.y * HIST_MUL_Y),0,NUM_BINS_XYZ)].y++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.z * HIST_MUL_Z),0,NUM_BINS_XYZ)].z++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.u * HIST_MUL_OF),0,NUM_BINS)].u++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.v * HIST_MUL_OF),0,NUM_BINS)].v++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.w * HIST_MUL_OF),0,NUM_BINS)].w++;
	m_centroid = Point(i,j);
	m_centroid3D.x = in->x;
	m_centroid3D.y = in->y;
	m_centroid3D.z = in->z;
	m_centroid3D.value = label;
	m_nodes.reserve(76800);
	m_neighbors.reserve(8);
	m_nodes.push_back(in);
	m_regions[0] = m_regions[1] = NULL;
	m_numRegions = 0;
}

inline void Region4DBig::InitializeRegion(Region4DBig *node1, Region4DBig *node2, int level, int min_size) {
	if(node1->m_size <= 0 || node2->m_size <= 0) {
		printf("Error, this should never happen\n");
	}
	/*if(node1->m_level == node2->m_level) {
	//m_level = node1->m_level + 1;
	if(node1->m_size > node2->m_size)
	m_centroid3D.value = node1->m_centroid3D.value;
	else
	m_centroid3D.value = node2->m_centroid3D.value;
	} else*/ if(node1->m_level > node2->m_level) {
		//m_level = node1->m_level + 1;
		m_centroid3D.value = node1->m_centroid3D.value;
	} else {
		//m_level = node2->m_level + 1;
		m_centroid3D.value = node2->m_centroid3D.value;
	}
	m_size = node1->m_size + node2->m_size;
	if(m_size == 0 || m_size > min_size)
		m_level = level;
	else
		m_level = 1;
	m_centroid.x = (node1->m_centroid.x * node1->m_size + node2->m_centroid.x * node2->m_size) / m_size;
	m_centroid.y = (node1->m_centroid.y * node1->m_size + node2->m_centroid.y * node2->m_size) / m_size;
	m_centroid3D.x = (node1->m_centroid3D.x * node1->m_size + node2->m_centroid3D.x * node2->m_size) / m_size;
	m_centroid3D.y = (node1->m_centroid3D.y * node1->m_size + node2->m_centroid3D.y * node2->m_size) / m_size;
	m_centroid3D.z = (node1->m_centroid3D.z * node1->m_size + node2->m_centroid3D.z * node2->m_size) / m_size;
	m_hist = new LABXYZUVW[NUM_BINS_XYZ]();
	LABXYZUVW *pHist = m_hist, *pHistFirstEnd = m_hist + NUM_BINS, *pHistEnd = m_hist + NUM_BINS_XYZ, *pHist1 = node1->m_hist, *pHist2 = node2->m_hist;
	while(pHist != pHistFirstEnd) {
		pHist->l = pHist1->l + pHist2->l;
		pHist->a = pHist1->a + pHist2->a;
		pHist->b = pHist1->b + pHist2->b;
		pHist->x = pHist1->x + pHist2->x;
		pHist->y = pHist1->y + pHist2->y;
		pHist->z = pHist1->z + pHist2->z;
		pHist->u = pHist1->u + pHist2->u;
		pHist->v = pHist1->v + pHist2->v;
		pHist->w = pHist1->w + pHist2->w;
		pHist++; pHist1++; pHist2++;
	}
	while(pHist != pHistEnd) {
		pHist->x = pHist1->x + pHist2->x;
		pHist->y = pHist1->y + pHist2->y;
		pHist->z = pHist1->z + pHist2->z;
		pHist++; pHist1++; pHist2++;
	}
	//I might want to add the neighbors later but for now I don't think it matters
	m_numRegions = 2;
	m_regions[0] = node1;
	m_regions[1] = node2;
}

inline void Region4DBig::AddNode(Point3D<int> *in, Bgr &color, Flow &flow, const int i, const int j) {
	LABXYZUVW labxyzuvw = LABXYZUVW(color.r,color.g,color.b,in->x, in->y, in->z, flow.u, flow.v, flow.w);
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.l * HIST_MUL_L),0,NUM_BINS)].l++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.a * HIST_MUL_A),0,NUM_BINS)].a++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.b * HIST_MUL_B),0,NUM_BINS)].b++;
	//Need to shift all these to make them positive
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.x * HIST_MUL_X),0,NUM_BINS_XYZ)].x++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.y * HIST_MUL_Y),0,NUM_BINS_XYZ)].y++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.z * HIST_MUL_Z),0,NUM_BINS_XYZ)].z++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.u * HIST_MUL_OF),0,NUM_BINS)].u++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.v * HIST_MUL_OF),0,NUM_BINS)].v++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyzuvw.w * HIST_MUL_OF),0,NUM_BINS)].w++;
	/*int new_size = m_size + 1;
	m_centroid.x = (m_centroid.x * m_size + i) / new_size;
	m_centroid.y = (m_centroid.y * m_size + j) / new_size;
	m_centroid3D.x = (m_centroid3D.x * m_size + in->x) / new_size;
	m_centroid3D.y = (m_centroid3D.y * m_size + in->y) / new_size;
	m_centroid3D.z = (m_centroid3D.z * m_size + in->z) / new_size;
	m_size = new_size;*/
	m_centroid.x += i;
	m_centroid.y += j;
	m_centroid3D.x += in->x;
	m_centroid3D.y += in->y;
	m_centroid3D.z += in->z;
	m_size++;
	m_nodes.push_back(in);
}

inline void Region3D::InitializeRegion(Point3D<int> *in, Bgr &color, const int label, const int i, const int j, const int level) {;
m_level  = level;
m_size = 1;
m_hist = new LABXYZ[NUM_BINS_XYZ]();
LABXYZ labxyz = LABXYZ(color.r,color.g,color.b,in->x, in->y, in->z);
m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.l * HIST_MUL_L),0,NUM_BINS)].l++;
m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.a * HIST_MUL_A),0,NUM_BINS)].a++;
m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.b * HIST_MUL_B),0,NUM_BINS)].b++;
m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.x * HIST_MUL_X),0,NUM_BINS_XYZ)].x++;
m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.y * HIST_MUL_Y),0,NUM_BINS_XYZ)].y++;
m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.z * HIST_MUL_Z),0,NUM_BINS_XYZ)].z++;
m_centroid = Point(i,j);
m_centroid3D.x = in->x;
m_centroid3D.y = in->y;
m_centroid3D.z = in->z;
m_centroid3D.value = label;
m_nodes.reserve(76800);
m_neighbors.reserve(8);
m_nodes.push_back(in);
m_regions[0] = m_regions[1] = NULL;
m_numRegions = 0;
}

inline void Region3D::InitializeRegion(Region3D *node1, Region3D *node2, int level, int min_size) {
	if(node1->m_size <= 0 || node2->m_size <= 0) {
		printf("Error, this should never happen\n");
	}
	/*if(node1->m_level == node2->m_level) {
	//m_level = node1->m_level + 1;
	if(node1->m_size > node2->m_size)
	m_centroid3D.value = node1->m_centroid3D.value;
	else
	m_centroid3D.value = node2->m_centroid3D.value;
	} else*/ if(node1->m_level > node2->m_level) {
		//m_level = node1->m_level + 1;
		m_centroid3D.value = node1->m_centroid3D.value;
	} else {
		//m_level = node2->m_level + 1;
		m_centroid3D.value = node2->m_centroid3D.value;
	}
	m_size = node1->m_size + node2->m_size;
	if(m_size == 0 || m_size > min_size)
		m_level = level;
	else
		m_level = 1;
	m_centroid.x = (node1->m_centroid.x * node1->m_size + node2->m_centroid.x * node2->m_size) / m_size;
	m_centroid.y = (node1->m_centroid.y * node1->m_size + node2->m_centroid.y * node2->m_size) / m_size;
	m_centroid3D.x = (node1->m_centroid3D.x * node1->m_size + node2->m_centroid3D.x * node2->m_size) / m_size;
	m_centroid3D.y = (node1->m_centroid3D.y * node1->m_size + node2->m_centroid3D.y * node2->m_size) / m_size;
	m_centroid3D.z = (node1->m_centroid3D.z * node1->m_size + node2->m_centroid3D.z * node2->m_size) / m_size;
	m_hist = new LABXYZ[NUM_BINS_XYZ]();
	LABXYZ *pHist = m_hist, *pHistFirstEnd = m_hist + NUM_BINS, *pHistEnd = m_hist + NUM_BINS_XYZ, *pHist1 = node1->m_hist, *pHist2 = node2->m_hist;
	while(pHist != pHistFirstEnd) {
		pHist->l = pHist1->l + pHist2->l;
		pHist->a = pHist1->a + pHist2->a;
		pHist->b = pHist1->b + pHist2->b;
		pHist->x = pHist1->x + pHist2->x;
		pHist->y = pHist1->y + pHist2->y;
		pHist->z = pHist1->z + pHist2->z;
		pHist++; pHist1++; pHist2++;
	}
	while(pHist != pHistEnd) {
		pHist->x = pHist1->x + pHist2->x;
		pHist->y = pHist1->y + pHist2->y;
		pHist->z = pHist1->z + pHist2->z;
		pHist++; pHist1++; pHist2++;
	}
	//I might want to add the neighbors later but for now I don't think it matters
	m_numRegions = 2;
	m_regions[0] = node1;
	m_regions[1] = node2;
}

inline void Region3D::AddNode(Point3D<int> *in, Bgr &color, const int i, const int j) {
	LABXYZ labxyz = LABXYZ(color.r,color.g,color.b,in->x,in->y,in->z);
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.l * HIST_MUL_L), 0, NUM_BINS)].l++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.a * HIST_MUL_A), 0, NUM_BINS)].a++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.b * HIST_MUL_B), 0, NUM_BINS)].b++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.x * HIST_MUL_X), 0, NUM_BINS_XYZ)].x++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.y * HIST_MUL_Y), 0, NUM_BINS_XYZ)].y++;
	m_hist[blepo_ex::Clamp(blepo_ex::Round(labxyz.z * HIST_MUL_Z), 0, NUM_BINS_XYZ)].z++;
	m_size++;
	m_centroid.x += i;
	m_centroid.y += j;
	m_centroid3D.x += in->x;
	m_centroid3D.y += in->y;
	m_centroid3D.z += in->z;
	m_nodes.push_back(in);
}

inline void Region::InitializeRegion(int *in, Bgr &color, const int label, const int i, const int j, const int level) {;
m_level  = level;
m_size = 1;
m_hist = new LAB[NUM_BINS]();
LAB lab = LAB(color.r,color.g,color.b);
m_hist[blepo_ex::Round(lab.l * HIST_MUL_L)].l++;
m_hist[blepo_ex::Round(lab.a * HIST_MUL_A)].a++;
m_hist[blepo_ex::Round(lab.b * HIST_MUL_B)].b++;
m_centroid = Point(i,j);
m_label = label;
m_nodes.reserve(76800);
m_neighbors.reserve(8);
m_nodes.push_back(in);
m_regions = NULL;
m_numRegions = 0;
}

inline void Region::InitializeRegion(Region *node1, Region *node2, int level, int min_size) {
	if(node1->m_size <= 0 || node2->m_size <= 0) {
		printf("Error, this should never happen\n");
	}
	/*if(node1->m_level == node2->m_level) {
	//m_level = node1->m_level + 1;
	if(node1->m_size > node2->m_size)
	m_centroid3D.value = node1->m_centroid3D.value;
	else
	m_centroid3D.value = node2->m_centroid3D.value;
	} else*/ if(node1->m_level > node2->m_level) {
		//m_level = node1->m_level + 1;
		m_label = node1->m_label;
	} else {
		//m_level = node2->m_level + 1;
		m_label = node2->m_label;
	}
	m_size = node1->m_size + node2->m_size;
	if(m_size == 0 || m_size > min_size)
		m_level = level;
	else
		m_level = 1;
	m_centroid.x = (node1->m_centroid.x * node1->m_size + node2->m_centroid.x * node2->m_size) / m_size;
	m_centroid.y = (node1->m_centroid.y * node1->m_size + node2->m_centroid.y * node2->m_size) / m_size;
	m_hist = new LAB[NUM_BINS]();
	LAB *pHist = m_hist, *pHistEnd = m_hist + NUM_BINS, *pHist1 = node1->m_hist, *pHist2 = node2->m_hist;
	while(pHist != pHistEnd) {
		pHist->l = pHist1->l + pHist2->l;
		pHist->a = pHist1->a + pHist2->a;
		pHist->b = pHist1->b + pHist2->b;
		pHist++; pHist1++; pHist2++;
	}
	//I might want to add the neighbors later but for now I don't think it matters
	m_numRegions = 2;
	m_regions = new Region*[2];
	m_regions[0] = node1;
	m_regions[1] = node2;
}

inline void Region::AddNode(int *in, Bgr &color, const int i, const int j) {
	LAB lab = LAB(color.r,color.g,color.b);
	m_hist[blepo_ex::Round(lab.l * HIST_MUL_L)].l++;
	m_hist[blepo_ex::Round(lab.a * HIST_MUL_A)].a++;
	m_hist[blepo_ex::Round(lab.b * HIST_MUL_B)].b++;
	m_size++;
	m_centroid.x += i;
	m_centroid.y += j;
	m_nodes.push_back(in);
}

void RegionTree4D::Create(PointCloudBgr &in, PointCloudInt &labels, int num_segments, int start_label) {
	BLEPO_ERROR("Wrong Create Function");
}

void RegionTree4D::Create(PointCloudBgr &in1, PointCloudInt &labels1, PointCloudBgr &in2, PointCloudInt &labels2, FlowInfo &flow, int num_segments, int start_label) {
	*this = RegionTree4D(num_segments,in1.Width(),in1.Height());
	//the original region tree is only two levels, the original segments and then the voxels(leafs)

	//create a lookup table to map the labels (valid lookups will have to start at 0)
	int min, max1, max2, max, current = 0;
	MinMax(labels1,&min,&max1);
	MinMax(labels2,&min,&max2);
	if(max1 > max2)
		max = max1;
	else
		max = max2;
	max++;
	int *lookup = new int[max]();
	int *pLook = lookup, *pLookEnd = lookup + max;
	while(pLook != pLookEnd)
		*pLook++ = -1;
	PointCloudBgr::Iterator pIn = in1.Begin();
	PointCloudInt::Iterator pLabel = labels1.Begin();
	Image<Flow>::Iterator pFlow = flow.Begin();
	int loc, label, i, j, k;
	const int safeWidth = in1.Width() - 1, safeHeight = in1.Height() - 1;
	Region4D* pRegion;
	numRegions = 0;
	//not sure about this
	totRegions = (num_segments + 2) << 1 + 1;
	region_list.resize(totRegions);
	for(k = 0; k < 2; k++) {
		for(j = 0; j < in1.Height(); j++) {
			for(i = 0; i < in1.Width(); i++) {
				//for each voxel, add to the appropriate region and compute important values
				loc = lookup[pLabel->value];
				//Am I a new region?
				if(loc == -1) {				
					//I'm a new region, initialize
					if(pLabel->value >= max) {
						printf("Vector out of range\n");
					}
					lookup[pLabel->value] = current;
					if(numRegions >= totRegions) {
						printf("Vector out of range\n");
					}
					pRegion = &(region_list[numRegions]);
					//pRegion->InitializeRegion(pLabel, pIn->value, pLabel->value, i, j, 1);
					region_list[numRegions].InitializeRegion(pLabel, pIn->value, *pFlow, current + start_label, i, j, 1);
					if(current >= totRegions) {
						printf("Vector out of range\n");
					}
					m_nodes[current] = pRegion;
					numRegions++;
					current++;
				}
				pIn++; pLabel++; pFlow++;
			}
		}
		pIn = in2.Begin();
		pLabel = labels2.Begin();
		//I messed up, need to figure out what to do about flow here for the second image.
		pFlow = flow.Begin();
	}
	pIn = in1.Begin();
	pLabel = labels1.Begin();
	pFlow = flow.Begin();
	Region4D *tmp;
	for(k = 0; k < 2; k++) {
		for(j = 0; j < in1.Height(); j++) {
			for(i = 0; i < in1.Width(); i++) {
				//I'm not new, add me appropriately
				//Add node to appropriately region list
				loc = lookup[pLabel->value];
				pRegion = &(region_list[loc]);
				region_list[loc].AddNode(pLabel,pIn->value,*pFlow,i,j);
				//Check for neighbors
				if(i < safeWidth) {
					label = lookup[(pLabel + 1)->value];
					tmp = &(region_list[label]);
					if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
					if(j < safeHeight) {
						label = lookup[(pLabel + in1.Width() + 1)->value];
						tmp = &(region_list[label]);
						if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
							pRegion->m_neighbors.push_back(label);
					}
					if(j > 0) {
						label = lookup[(pLabel - in1.Width() + 1)->value];
						tmp = &(region_list[label]);
						if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
							pRegion->m_neighbors.push_back(label);
					}
				}
				if(j < safeHeight) {
					label = lookup[(pLabel + in1.Width())->value];
					tmp = &(region_list[label]);
					if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
				}
				pIn++; pLabel++;
			}
		}
		pIn = in2.Begin();
		pLabel = labels2.Begin();
		//again not sure about this
		pFlow = flow.Begin();
	}
	//adjust means and centroids and such
	//pRegion = out->m_nodes[0];
	for(i = 0; i < num_segments; i++) {
		if(m_nodes[i] != NULL) {
			m_nodes[i]->m_centroid.x /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid.y /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid3D.x /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid3D.y /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid3D.z /= m_nodes[i]->m_size;
		}
	}
	delete[] lookup;
}

void RegionTree4DBig::Create(PointCloudBgr &in, PointCloudInt &labels, int num_segments, int start_label) {
	BLEPO_ERROR("Wrong Create Function");
}

void RegionTree4DBig::Create(deque<PointCloudBgr> &in, deque<FlowInfo> &flow, PointCloudInt *labels, int num_segments, int start_label) {
	*this = RegionTree4DBig(num_segments,in[0].Width(),in[0].Height());
	//the original region tree is only two levels, the original segments and then the voxels(leafs)

	//create a lookup table to map the labels (valid lookups will have to start at 0)
	int min, max = 0, current;
	for(int k = 0; k < NUM_FRAMES; k++) {
		MinMax(labels[k],&min,&current);
		if(current > max)
			max = current;
	}
	max++;
	current = 0;
	int *lookup = new int[max]();
	int *pLook = lookup, *pLookEnd = lookup + max;
	while(pLook != pLookEnd)
		*pLook++ = -1;
	int loc, label, i, j, k;
	const int safeWidth = in[0].Width() - 1, safeHeight = in[0].Height() - 1;
	Region4DBig* pRegion;
	numRegions = 0;
	//not sure about this
	totRegions = (num_segments + 2) << 1 + 1;
	region_list.resize(totRegions);
	for(k = 0; k < NUM_FRAMES; k++) {
		//need to do something about the optical flow of the first image
		PointCloudBgr::Iterator pIn = in[k].Begin();
		PointCloudInt::Iterator pLabel = labels[k].Begin();
		//ComputeOpticalFlow(in[k-1],in[k],&flow);
		Image<Flow>::Iterator pFlow = flow[k].Begin();
		for(j = 0; j < in[0].Height(); j++) {
			for(i = 0; i < in[0].Width(); i++) {
				//for each voxel, add to the appropriate region and compute important values
				loc = lookup[pLabel->value];
				//Am I a new region?
				if(loc == -1) {				
					//I'm a new region, initialize
					if(pLabel->value >= max) {
						printf("Vector out of range\n");
					}
					lookup[pLabel->value] = current;
					if(numRegions >= totRegions) {
						printf("Vector out of range\n");
					}
					pRegion = &(region_list[numRegions]);
					//pRegion->InitializeRegion(pLabel, pIn->value, pLabel->value, i, j, 1);
					region_list[numRegions].InitializeRegion(pLabel, pIn->value, *pFlow, current + start_label, i, j, 1);
					if(current >= totRegions) {
						printf("Vector out of range\n");
					}
					m_nodes[current] = pRegion;
					numRegions++;
					current++;
				}
				pIn++; pLabel++; pFlow++;
			}
		}
	}
	Region4DBig *tmp;
	for(k = 0; k < NUM_FRAMES; k++) {
		PointCloudBgr::Iterator pIn = in[k].Begin();
		PointCloudInt::Iterator pLabel = labels[k].Begin();
		//ComputeOpticalFlow(in[k-1],in[k],&flow);
		Image<Flow>::Iterator pFlow = flow[k].Begin();
		for(j = 0; j < in[k].Height(); j++) {
			for(i = 0; i < in[k].Width(); i++) {
				//I'm not new, add me appropriately
				//Add node to appropriately region list
				loc = lookup[pLabel->value];
				pRegion = &(region_list[loc]);
				region_list[loc].AddNode(pLabel,pIn->value,*pFlow,i,j);
				//Check for neighbors
				if(i < safeWidth) {
					label = lookup[(pLabel + 1)->value];
					tmp = &(region_list[label]);
					if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end()) {
						pRegion->m_neighbors.push_back(label);
						pRegion->m_neighbor_map[label] = tmp;
					}
					if(j < safeHeight) {
						label = lookup[(pLabel + in[k].Width() + 1)->value];
						tmp = &(region_list[label]);
						if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end()) {
							pRegion->m_neighbors.push_back(label);
							pRegion->m_neighbor_map[label] = tmp;
						}
					}
					if(j > 0) {
						label = lookup[(pLabel - in[k].Width() + 1)->value];
						tmp = &(region_list[label]);
						if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end()) {
							pRegion->m_neighbors.push_back(label);
							pRegion->m_neighbor_map[label] = tmp;
						}
					}
				}
				if(j < safeHeight) {
					label = lookup[(pLabel + in[k].Width())->value];
					tmp = &(region_list[label]);
					if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end()) {
						pRegion->m_neighbors.push_back(label);
						pRegion->m_neighbor_map[label] = tmp;
					}
				}
				pIn++; pLabel++;
			}
		}
	}
	//adjust means and centroids and such
	//pRegion = out->m_nodes[0];
	for(i = 0; i < num_segments; i++) {
		if(m_nodes[i] != NULL) {
			m_nodes[i]->m_centroid.x /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid.y /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid3D.x /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid3D.y /= m_nodes[i]->m_size;
			m_nodes[i]->m_centroid3D.z /= m_nodes[i]->m_size;
		}
	}
	delete[] lookup;
}

void RegionTree3D::Create(PointCloudBgr &in, PointCloudInt &labels, int num_segments, int start_label) {
	this->Release();
	*this = RegionTree3D(num_segments,in.Width(),in.Height());
	//the original region tree is only two levels, the original segments and then the voxels(leafs)

	//create a lookup table to map the labels (valid lookups will have to start at 0)
	int min, max, current = 0;
	MinMax(labels,&min,&max);
	max++;
	int *lookup = new int[max]();
	int *pLook = lookup, *pLookEnd = lookup + max;
	while(pLook != pLookEnd)
		*pLook++ = -1;
	PointCloudBgr::Iterator pIn = in.Begin();
	PointCloudInt::Iterator pLabel = labels.Begin();
	int loc, label, i, j;
	const int safeWidth = in.Width() - 1, safeHeight = in.Height() - 1;
	Region3D* pRegion;
	//printf("Original num of Segments: %d\n",num_segments);
	numRegions = 0;
	totRegions = (num_segments + 2) << 1;
	//region_list = new Region3D[totRegions]();
	//delete region_list;
	region_list.resize(totRegions);
	//region_list = (Region3D*) malloc(totRegions * sizeof(Region3D));
	//printf("region_list 1: %p\n",region_list);
	for(j = 0; j < in.Height(); j++) {
		for(i = 0; i < in.Width(); i++) {
			//for each voxel, add to the appropriate region and compute important values
			loc = lookup[pLabel->value];
			//Am I a new region?
			if(loc == -1) {
				//I'm a new region, initialize
				lookup[pLabel->value] = current;
				pRegion = &(region_list[numRegions]);
				//pRegion->InitializeRegion(pLabel, pIn->value, pLabel->value, i, j, 1);
				region_list[numRegions].InitializeRegion(pLabel, pIn->value, current + start_label, i, j, 1);
				m_nodes[current] = pRegion;
				numRegions++;
				current++;
			}
			pIn++; pLabel++;
		}
	}
	pIn = in.Begin();
	pLabel = labels.Begin();
	Region3D *tmp;
	for(j = 0; j < in.Height(); j++) {
		for(i = 0; i < in.Width(); i++) {
			//I'm not new, add me appropriately
			//Add node to appropriately region list
			loc = lookup[pLabel->value];
			pRegion = &(region_list[loc]);
			region_list[loc].AddNode(pLabel,pIn->value,i,j);
			//Check for neighbors
			if(i < safeWidth) {
				label = lookup[(pLabel + 1)->value];
				tmp = &(region_list[label]);
				if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
					pRegion->m_neighbors.push_back(label);
				if(j < safeHeight) {
					label = lookup[(pLabel + in.Width() + 1)->value];
					tmp = &(region_list[label]);
					if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
				}
				if(j > 0) {
					label = lookup[(pLabel - in.Width() + 1)->value];
					tmp = &(region_list[label]);
					if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
				}
			}
			if(j < safeHeight) {
				label = lookup[(pLabel + in.Width())->value];
				tmp = &(region_list[label]);
				if(pLabel->value != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->value) == tmp->m_neighbors.end())
					pRegion->m_neighbors.push_back(label);
			}
			pIn++; pLabel++;
		}
	}
	//adjust means and centroids and such
	//pRegion = out->m_nodes[0];
	/*for(i = 0; i < num_segments; i++) {
	if(region_list[i].m_hist != NULL) {
	region_list[i].m_centroid.x /= pRegion->m_size;
	region_list[i].m_centroid.y /= pRegion->m_size;
	region_list[i].m_centroid3D.x /= pRegion->m_size;
	region_list[i].m_centroid3D.y /= pRegion->m_size;
	region_list[i].m_centroid3D.z /= pRegion->m_size;
	}
	}*/
	delete[] lookup;
}

void SetBranch(Region3D* region, int level, int label) {
	assert(region != NULL);
	if(label == -1 && region->m_level <= level) {
		//I should set the label
		label = region->m_centroid3D.value;
	}
	if(region->m_numRegions != 0 && region->m_regions != NULL) {
		//I am not at the leaf level, tell my children to do proper
		Region3D **branch = region->m_regions;
		for(int i = 0; i < region->m_numRegions; i++) {
			SetBranch(*branch,level, label);
			branch++;
		}
	} else {
		//I contain the leaves, Set each one.
		assert(label != -1);
		//set my label
		region->m_centroid3D.value = label;
		vector<Point3D<int>*>::iterator pNode = region->m_nodes.begin();
		while(pNode != region->m_nodes.end()) {
			(*pNode)->value = label;
			pNode++;
		}
	}
}

void SetBranch(Region4D* region, int level, int label) {
	assert(region != NULL);
	if(label == -1 && region->m_level <= level) {
		//I should set the label
		label = region->m_centroid3D.value;
	}
	if(region->m_numRegions != 0 && region->m_regions != NULL) {
		//I am not at the leaf level, tell my children to do proper
		Region4D **branch = region->m_regions;
		for(int i = 0; i < region->m_numRegions; i++) {
			SetBranch(*branch,level, label);
			branch++;
		}
	} else {
		//I contain the leaves, Set each one.
		assert(label != -1);
		//set my label
		region->m_centroid3D.value = label;
		vector<Point3D<int>*>::iterator pNode = region->m_nodes.begin();
		while(pNode != region->m_nodes.end()) {
			(*pNode)->value = label;
			pNode++;
		}
	}
}

void SetBranch(Region4DBig* region, int level, int label) {
	assert(region != NULL);
	if(label == -1 && region->m_level <= level) {
		//I should set the label
		label = region->m_centroid3D.value;
	}
	if(region->m_numRegions != 0 && region->m_regions != NULL) {
		//I am not at the leaf level, tell my children to do proper
		Region4DBig **branch = region->m_regions;
		for(int i = 0; i < region->m_numRegions; i++) {
			SetBranch(*branch,level, label);
			branch++;
		}
	} else {
		//I contain the leaves, Set each one.
		assert(label != -1);
		//set my label
		region->m_centroid3D.value = label;
		vector<Point3D<int>*>::iterator pNode = region->m_nodes.begin();
		while(pNode != region->m_nodes.end()) {
			(*pNode)->value = label;
			pNode++;
		}
	}
}

void RegionTree3D::UpdateCloud(int level) {
	//start by finding the right level for each branch
	int i;
	Region3D** branch = m_nodes;
	for(i = 0; i < m_size; i++) {
		SetBranch(*branch, level, -1);
		branch++;
	}
}

void RegionTree4D::UpdateCloud(int level) {
	//start by finding the right level for each branch
	int i;
	Region4D** branch = m_nodes;
	for(i = 0; i < m_size; i++) {
		SetBranch(*branch, level, -1);
		branch++;
	}
}

void RegionTree4DBig::UpdateCloud(int level) {
	//start by finding the right level for each branch
	int i;
	Region4DBig** branch = m_nodes;
	for(i = 0; i < m_size; i++) {
		SetBranch(*branch, level, -1);
		branch++;
	}
}

template<class T, class HistContainer, class ColorContainer, class LabelContainer>
void RegionTreeType<T,HistContainer,ColorContainer,LabelContainer>::UpdateRegionList(vector<T*> &list) {
	vector<T*>::iterator p = list.begin();
	while(p != list.end()) {
		SetBranch(*p,(*p)->m_level,(*p)->m_centroid3D.value);
		p++;
	}
}

//perhaps this is wrong, perhaps I should normalize somehow first, maybe by the size in order to get percentages?
inline float HistDifference(Region3D &reg1, Region3D &reg2) {
	if(reg1.m_hist != NULL && reg2.m_hist != NULL) {
		float sad = 0.0f;
		LABXYZ *p1 = reg1.m_hist, *p2 = reg2.m_hist;
		int i;
		for(i = 0; i < NUM_BINS; i++, p1++, p2++) {
			sad += blepo_ex::Abs(float(p1->a) / reg1.m_size - float(p2->a) / reg2.m_size) + blepo_ex::Abs(float(p1->b) / reg1.m_size - float(p2->b) / reg2.m_size) + blepo_ex::Abs(float(p1->l) / reg1.m_size - float(p2->l) / reg2.m_size) + HIST_DEPTH_MOD * (blepo_ex::Abs(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + blepo_ex::Abs(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + blepo_ex::Abs(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size)); 
		}
		while(i < NUM_BINS_XYZ) {
			sad += HIST_DEPTH_MOD * (blepo_ex::Abs(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + blepo_ex::Abs(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + blepo_ex::Abs(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size));
			++i; ++p1; ++p2;
		}
		float min = (blepo_ex::Abs(reg1.m_centroid3D.x - reg2.m_centroid3D.x) + blepo_ex::Abs(reg1.m_centroid3D.y - reg2.m_centroid3D.y) + blepo_ex::Abs(reg1.m_centroid3D.z - reg2.m_centroid3D.z)) / (reg1.m_size + reg2.m_size / 2);
		float size_diff = float(blepo_ex::Abs(int(reg1.m_size) - int(reg2.m_size))) / (float(reg1.m_size + reg2.m_size) / 2);
		return sad + CENTROID_MUL * min + size_diff * SIZE_MUL;
	}
	return 1000000;
}

inline float HistDifference(Region4D &reg1, Region4D &reg2) {
	float sad = 0.0f;
	LABXYZUVW *p1 = reg1.m_hist, *p2 = reg2.m_hist;
	int i;
	for(i = 0; i < NUM_BINS; i++, p1++, p2++) {
		sad += blepo_ex::Abs(float(p1->a) / reg1.m_size - float(p2->a) / reg2.m_size) + blepo_ex::Abs(float(p1->b) / reg1.m_size - float(p2->b) / reg2.m_size) + blepo_ex::Abs(float(p1->l) / reg1.m_size - float(p2->l) / reg2.m_size) + HIST_DEPTH_MOD * (blepo_ex::Abs(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + blepo_ex::Abs(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + blepo_ex::Abs(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size)) + blepo_ex::Abs(float(p1->u) / reg1.m_size - float(p2->u) / reg2.m_size) + blepo_ex::Abs(float(p1->v) / reg1.m_size - float(p2->v) / reg2.m_size) + blepo_ex::Abs(float(p1->w) / reg1.m_size - float(p2->w) / reg2.m_size); 
	}
	while(i < NUM_BINS_XYZ) {
		sad += HIST_DEPTH_MOD * (blepo_ex::Abs(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + blepo_ex::Abs(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + blepo_ex::Abs(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size));
		++i; ++p1; ++p2;
	}
	return sad;
}

//lets include size and centroid difference in this as well.
inline float HistDifference(Region4DBig &reg1, Region4DBig &reg2) {
	if(reg1.m_hist != NULL && reg2.m_hist != NULL) {
		float sad = 0.0f;
		LABXYZUVW *p1 = reg1.m_hist, *p2 = reg2.m_hist;
		int i;
		for(i = 0; i < NUM_BINS; i++, p1++, p2++) {
			sad += blepo_ex::Abs(float(p1->a) / reg1.m_size - float(p2->a) / reg2.m_size) + blepo_ex::Abs(float(p1->b) / reg1.m_size - float(p2->b) / reg2.m_size) + blepo_ex::Abs(float(p1->l) / reg1.m_size - float(p2->l) / reg2.m_size) + blepo_ex::Abs(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + blepo_ex::Abs(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + blepo_ex::Abs(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size) + blepo_ex::Abs(float(p1->u) / reg1.m_size - float(p2->u) / reg2.m_size) + blepo_ex::Abs(float(p1->v) / reg1.m_size - float(p2->v) / reg2.m_size) + blepo_ex::Abs(float(p1->w) / reg1.m_size - float(p2->w) / reg2.m_size); 
		}
		while(i < NUM_BINS_XYZ) {
			sad += blepo_ex::Abs(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + blepo_ex::Abs(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + blepo_ex::Abs(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size);
			++i; ++p1; ++p2;
		}
		float min = (blepo_ex::Abs(reg1.m_centroid3D.x - reg2.m_centroid3D.x) + blepo_ex::Abs(reg1.m_centroid3D.y - reg2.m_centroid3D.y) + blepo_ex::Abs(reg1.m_centroid3D.z - reg2.m_centroid3D.z)) / (reg1.m_size + reg2.m_size / 2);
		float size_diff = float(blepo_ex::Abs(int(reg1.m_size) - int(reg2.m_size))) / (float(reg1.m_size + reg2.m_size) / 2);
		return sad + CENTROID_MUL * min + size_diff * SIZE_MUL;
	}
	return 1000000;
}


template<class T>
void GetLevel(T *root, int level, vector<T*> *list) {
	if(root->m_level <= level || root->m_numRegions == 0)
		list->push_back(root);
	else {
		GetLevel(root->m_regions[0],level,list);
		GetLevel(root->m_regions[1],level,list);
	}
}

//Lets construct a list of the Region Branches that satisfy a certain level (used for temporal correction)
void RegionTree3D::GetRegionList(float level, vector<Region3D*> *list) {
	list->reserve(76800); //this is my average guess
	GetLevel(*m_nodes,int(level * (float)m_nodes[0]->m_level),list);
}

void RegionTree4D::GetRegionList(float level, vector<Region4D*> *list) {
	list->reserve(76800); //this is my average guess
	GetLevel(*m_nodes,int(level * (float)m_nodes[0]->m_level),list);
}

void RegionTree4DBig::GetRegionList(float level, vector<Region4DBig*> *list) {
	list->reserve(76800); //this is my average guess
	GetLevel(*m_nodes,int(level * (float)m_nodes[0]->m_level),list);
}


//assumes tree is not propagated
//void RegionTree3D::TemporalCorrection(RegionTree3D &past, int level) {
//	assert(!m_propagated && !past.m_propagated);
//	//Match based on the centroid of each region
//	//Region3D **pCurr = m_nodes, **pPast;
//	vector<Region3D>::iterator pCurr = region_list.begin(), pPast;
//	for(int i = 0; i < m_size; i++) {
//		//find the best match
//		pPast = past.region_list.begin();
//		float min[3];
//		float hist_diff[3];
//		int size_diff[3];
//		min[0] = min[1] = min[2] = blepo_ex::Abs((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
//		hist_diff[0] = hist_diff[1] = hist_diff[2] = HistDifference(*pCurr,*pPast);
//		Region3D minLoc[3];
//		minLoc[0] = minLoc[1] = minLoc[2] = *pPast;
//		pPast++;
//		//find the 3 with the closest centroids
//		for(int j = 0; j < past.m_size - 1; j++) {
//			//float val = blepo_ex::Abs((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
//			float val = HistDifference(*pCurr,*pPast);
//			if(val < hist_diff[0]) {
//				hist_diff[2] = hist_diff[1];
//				minLoc[2] = minLoc[1];
//				hist_diff[1] = hist_diff[0];
//				minLoc[1] = minLoc[0];
//				hist_diff[0] = val;
//				minLoc[0] = *pPast;
//			} else if(val < hist_diff[1]) {
//				hist_diff[2] = hist_diff[1];
//				minLoc[2] = minLoc[1];
//				hist_diff[1] = val;
//				minLoc[1] = *pPast;
//			} else if(val < hist_diff[2]) {
//				hist_diff[2] = val;
//				minLoc[2] = *pPast;
//			}
//			pPast++;
//		}
//		//of those 3, lets pick the one with the closest histogram and size. If none fit, give it a new segment
//		/*hist_diff[0] = HistDifference(*pCurr,minLoc[0]);
//		hist_diff[1] = HistDifference(*pCurr,minLoc[1]);
//		hist_diff[2] = HistDifference(*pCurr,minLoc[2]);
//		//sort by minimum hist_diff
//		if(hist_diff[1] < hist_diff[0]) {
//		int tmp = hist_diff[0];
//		Region3D tmp2 = minLoc[0];
//		float tmp3 = min[0];
//		hist_diff[0] = hist_diff[1];
//		minLoc[0] = minLoc[1];
//		min[0] = min[1];
//		hist_diff[1] = tmp;
//		minLoc[1] = tmp2;
//		min[1] = tmp3;
//		}
//		if(hist_diff[2] < hist_diff[0]) {
//		int tmp = hist_diff[0];
//		Region3D tmp2 = minLoc[0];
//		float tmp3 = min[0];
//		size_diff[0] = size_diff[2];
//		minLoc[0] = minLoc[2];
//		min[0] = min[2];
//		hist_diff[2] = tmp;
//		minLoc[2] = tmp2;
//		min[2] = tmp3;
//		}
//		if(hist_diff[2] < hist_diff[1]) {
//		int tmp = hist_diff[1];
//		Region3D tmp2 = minLoc[1];
//		float tmp3 = min[1];
//		hist_diff[1] = hist_diff[2];
//		minLoc[1] = minLoc[2];
//		min[1] = min[2];
//		hist_diff[2] = tmp;
//		minLoc[2] = tmp2;
//		min[2] = tmp3;
//		}*/
//		min[0] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[0].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[0].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[0].m_centroid3D.z)) / pCurr->m_size;
//		min[1] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[1].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[1].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[1].m_centroid3D.z)) / pCurr->m_size;
//		min[2] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[2].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[2].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[2].m_centroid3D.z)) / pCurr->m_size;
//		size_diff[0] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[0].m_size));
//		size_diff[1] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[1].m_size));
//		size_diff[2] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[2].m_size));
//		//printf("2 - Hist: %f, Size: %d, Dist: %f\n",hist_diff[1],size_diff[1],min[1]);
//		//printf("3 - Hist: %f, Size: %d, Dist: %f\n",hist_diff[2],size_diff[2],min[2]);
//		//Need a way to prevent big segments from overmerging but allow small segments to join despite the MIN_REGION_SIZE constraint
//		//if(hist_diff[0] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[0] <= (pCurr)->m_size * MIN_REGION_SIZE && min[0] < MIN_REGION_DIST)))
//		SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[0].m_centroid3D.value);
//		/*else if(hist_diff[1] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[1] <= (pCurr)->m_size * MIN_REGION_SIZE && min[1] < MIN_REGION_DIST)))
//		SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[1].m_centroid3D.value);
//		else if(hist_diff[2] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[2] <= (pCurr)->m_size * MIN_REGION_SIZE && min[2] < MIN_REGION_DIST)))
//		SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[2].m_centroid3D.value);*/
//		/*else
//		printf("NEW SEGMENT!! 1 - Hist: %f, Size: %d, MaxSize: %f, Dist: %f\n",hist_diff[0],size_diff[0],float((pCurr)->m_size) * MIN_REGION_SIZE,min[0]); */
//		pCurr++;
//	}
//}

void RegionTree3D::TemporalCorrection(RegionTree3D &past, int level) {
	if(!m_propagated && !past.m_propagated) {
		map<Region3D,Region3D> currSeg, pastSeg;
		//Match based on the centroid of each region
		//Region3D **pCurr = m_nodes, **pPast;
		vector<Region3D>::iterator pCurr = region_list.begin(), pPast;
		//FILE *fp;
		//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\stats.txt","a");
		for(int i = 0; i < m_size; i++) {
			//find the best match
			pPast = past.region_list.begin();
			float min;
			float hist_diff;
			int size_diff;
			min = blepo_ex::Abs((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
			hist_diff = HistDifference(*pCurr,*pPast);
			Region3D minLoc;
			minLoc = *pPast;
			pPast++;
			for(int j = 0; j < past.m_size - 1; j++) {
				float val = HistDifference(*pCurr,*pPast);
				if(val < hist_diff) {
					hist_diff = val;
					minLoc = *pPast;
				}
				pPast++;
			}
			currSeg[*pCurr] = minLoc;
			pCurr++;
		}
		pPast = past.region_list.begin();
		for(int i = 0; i < past.m_size; i++) {
			//find the best match
			pCurr = region_list.begin();
			float hist_diff = HistDifference(*pCurr,*pPast);
			Region3D minLoc;
			minLoc = *pCurr;
			pCurr++;
			//find the 3 with the closest centroids
			for(int j = 0; j < m_size - 1; j++) {
				float val = HistDifference(*pCurr,*pPast);
				if(val < hist_diff) {
					hist_diff = val;
					minLoc = *pCurr;
				}
				pCurr++;
			}
			//fprintf(fp,"%f, %d, %d, %f, %d, %d, %f, %f, %f, %d, %d, %f, %f, %f\n",hist_diff[0], pCurr->m_size, size_diff[0], min[0], int(pCurr->m_centroid.x), int(pCurr->m_centroid.y), pCurr->m_centroid3D.x, pCurr->m_centroid3D.y, pCurr->m_centroid3D.z, int(minLoc[0].m_centroid.x), int(minLoc[0].m_centroid.y), minLoc[0].m_centroid3D.x, minLoc[0].m_centroid3D.y, minLoc[0].m_centroid3D.z);
			pastSeg[*pPast] = minLoc;
			pPast++;
		}

		pCurr = region_list.begin();
		for(int i = 0; i < m_size; i++) {
			//bipartite matching with red-black trees
			if(pCurr->m_centroid3D.value == pastSeg[currSeg[*pCurr]].m_centroid3D.value) {
				//the min should be normalized by the size somehow
				//float min = (blepo_ex::Abs((pCurr)->m_centroid3D.x - currSeg[*pCurr].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y -currSeg[*pCurr].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - currSeg[*pCurr].m_centroid3D.z)) / pCurr->m_size;
				//int size_diff = blepo_ex::Abs(int((pCurr)->m_size) - int(currSeg[*pCurr].m_size));
				//float hist_diff = HistDifference(*pCurr,currSeg[*pCurr]);
				//if(hist_diff <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff <= (pCurr)->m_size * MIN_REGION_SIZE && min < MIN_REGION_DIST)))
				SetBranch(&(*pCurr),(pCurr)->m_level,currSeg[*pCurr].m_centroid3D.value);
			}
			++pCurr;
		}
		//fclose(fp);
	} 
}

void RegionTree4D::TemporalCorrection(RegionTree4D &past, int level) {
	assert(!m_propagated && !past.m_propagated);
	//Match based on the centroid of each region
	//Region3D **pCurr = m_nodes, **pPast;
	vector<Region4D>::iterator pCurr = region_list.begin(), pPast;
	//FILE *fp;
	//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\rgbd_dataset_freiburg2_coke\\stats.txt","w");
	for(int i = 0; i < m_size; i++) {
		//find the best match
		pPast = past.region_list.begin();
		float min[3];
		float hist_diff[3];
		int size_diff[3];
		min[0] = min[1] = min[2] = blepo_ex::Abs((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
		hist_diff[0] = hist_diff[1] = hist_diff[2] = HistDifference(*pCurr,*pPast);
		Region4D minLoc[3];
		minLoc[0] = minLoc[1] = minLoc[2] = *pPast;
		pPast++;
		//find the 3 with the closest centroids
		for(int j = 0; j < past.m_size - 1; j++) {
			float val = HistDifference(*pCurr,*pPast);
			if(val < hist_diff[0]) {
				hist_diff[2] = hist_diff[1];
				minLoc[2] = minLoc[1];
				hist_diff[1] = hist_diff[0];
				minLoc[1] = minLoc[0];
				hist_diff[0] = val;
				minLoc[0] = *pPast;
			} else if(val < hist_diff[1]) {
				hist_diff[2] = hist_diff[1];
				minLoc[2] = minLoc[1];
				hist_diff[1] = val;
				minLoc[1] = *pPast;
			} else if(val < hist_diff[2]) {
				hist_diff[2] = val;
				minLoc[2] = *pPast;
			}
			pPast++;
		}
		//of those 3, lets pick the one with the closest histogram and size. If none fit, give it a new segment
		//the min should be normalized by the size somehow
		min[0] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[0].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[0].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[0].m_centroid3D.z)) / pCurr->m_size;
		min[1] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[1].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[1].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[1].m_centroid3D.z)) / pCurr->m_size;
		min[2] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[2].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[2].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[2].m_centroid3D.z)) / pCurr->m_size;
		size_diff[0] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[0].m_size));
		size_diff[1] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[1].m_size));
		size_diff[2] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[2].m_size));
		//fprintf(fp,"%f, %d, %d, %f, %d, %d, %f, %f, %f, %d, %d, %f, %f, %f\n",hist_diff[0], pCurr->m_size, size_diff[0], min[0], int(pCurr->m_centroid.x), int(pCurr->m_centroid.y), pCurr->m_centroid3D.x, pCurr->m_centroid3D.y, pCurr->m_centroid3D.z, int(minLoc[0].m_centroid.x), int(minLoc[0].m_centroid.y), minLoc[0].m_centroid3D.x, minLoc[0].m_centroid3D.y, minLoc[0].m_centroid3D.z);
		//Need a way to prevent big segments from overmerging but allow small segments to join despite the MIN_REGION_SIZE constraint
		if(hist_diff[0] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[0] <= (pCurr)->m_size * MIN_REGION_SIZE && min[0] < MIN_REGION_DIST)))
			SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[0].m_centroid3D.value);
		else if(hist_diff[1] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[1] <= (pCurr)->m_size * MIN_REGION_SIZE && min[1] < MIN_REGION_DIST)))
			SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[1].m_centroid3D.value);
		else if(hist_diff[2] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[2] <= (pCurr)->m_size * MIN_REGION_SIZE && min[2] < MIN_REGION_DIST)))
			SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[2].m_centroid3D.value);
		pCurr++;
	}
	//fclose(fp);
}

/*void RegionTree4DBig::TemporalCorrection(RegionTree4DBig &past, int level) {
assert(!m_propagated && !past.m_propagated);
//Match based on the centroid of each region
//Region3D **pCurr = m_nodes, **pPast;
vector<Region4DBig>::iterator pCurr = region_list.begin(), pPast;
//FILE *fp;
//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\stats.txt","a");
for(int i = 0; i < m_size; i++) {
//find the best match
pPast = past.region_list.begin();
float min[3];
float hist_diff[3];
int size_diff[3];
min[0] = min[1] = min[2] = blepo_ex::Abs((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
hist_diff[0] = hist_diff[1] = hist_diff[2] = HistDifference(*pCurr,*pPast);
Region4DBig minLoc[3];
minLoc[0] = minLoc[1] = minLoc[2] = *pPast;
pPast++;
//find the 3 with the closest centroids
for(int j = 0; j < past.m_size - 1; j++) {
float val = HistDifference(*pCurr,*pPast);
if(val < hist_diff[0]) {
hist_diff[2] = hist_diff[1];
minLoc[2] = minLoc[1];
hist_diff[1] = hist_diff[0];
minLoc[1] = minLoc[0];
hist_diff[0] = val;
minLoc[0] = *pPast;
} else if(val < hist_diff[1]) {
hist_diff[2] = hist_diff[1];
minLoc[2] = minLoc[1];
hist_diff[1] = val;
minLoc[1] = *pPast;
} else if(val < hist_diff[2]) {
hist_diff[2] = val;
minLoc[2] = *pPast;
}
pPast++;
}
//of those 3, lets pick the one with the closest histogram and size. If none fit, give it a new segment
//the min should be normalized by the size somehow
min[0] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[0].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[0].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[0].m_centroid3D.z)) / pCurr->m_size;
min[1] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[1].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[1].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[1].m_centroid3D.z)) / pCurr->m_size;
min[2] = (blepo_ex::Abs((pCurr)->m_centroid3D.x - minLoc[2].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - minLoc[2].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - minLoc[2].m_centroid3D.z)) / pCurr->m_size;
size_diff[0] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[0].m_size));
size_diff[1] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[1].m_size));
size_diff[2] = blepo_ex::Abs(int((pCurr)->m_size) - int(minLoc[2].m_size));
//fprintf(fp,"%f, %d, %d, %f, %d, %d, %f, %f, %f, %d, %d, %f, %f, %f\n",hist_diff[0], pCurr->m_size, size_diff[0], min[0], int(pCurr->m_centroid.x), int(pCurr->m_centroid.y), pCurr->m_centroid3D.x, pCurr->m_centroid3D.y, pCurr->m_centroid3D.z, int(minLoc[0].m_centroid.x), int(minLoc[0].m_centroid.y), minLoc[0].m_centroid3D.x, minLoc[0].m_centroid3D.y, minLoc[0].m_centroid3D.z);
//Need a way to prevent big segments from overmerging but allow small segments to join despite the MIN_REGION_SIZE constraint
if(hist_diff[0] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[0] <= (pCurr)->m_size * MIN_REGION_SIZE && min[0] < MIN_REGION_DIST)))
SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[0].m_centroid3D.value);
else if(hist_diff[1] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[1] <= (pCurr)->m_size * MIN_REGION_SIZE && min[1] < MIN_REGION_DIST)))
SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[1].m_centroid3D.value);
else if(hist_diff[2] <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff[2] <= (pCurr)->m_size * MIN_REGION_SIZE && min[2] < MIN_REGION_DIST)))
SetBranch(&(*pCurr),(pCurr)->m_level,minLoc[2].m_centroid3D.value);
pCurr++;
}
//fclose(fp);
}*/

/* RegionTree4DBig::TemporalCorrection(RegionTree4DBig &past, int level) {
assert(!m_propagated && !past.m_propagated);
map<Region4DBig,Region4DBig> currSeg, pastSeg;
//Match based on the centroid of each region
//Region3D **pCurr = m_nodes, **pPast;
vector<Region4DBig>::iterator pCurr = region_list.begin(), pPast;
//FILE *fp;
//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\stats.txt","a");
for(int i = 0; i < m_size; i++) {
//find the best match
pPast = past.region_list.begin();
float min;
float hist_diff;
int size_diff;
min = blepo_ex::Abs((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
hist_diff = HistDifference(*pCurr,*pPast);
Region4DBig minLoc;
minLoc = *pPast;
pPast++;
for(int j = 0; j < past.m_size - 1; j++) {
float val = HistDifference(*pCurr,*pPast);
if(val < hist_diff) {
hist_diff = val;
minLoc = *pPast;
}
pPast++;
}
currSeg[*pCurr] = minLoc;
pCurr++;
}
pPast = past.region_list.begin();
for(int i = 0; i < past.m_size; i++) {
//find the best match
pCurr = region_list.begin();
float hist_diff = HistDifference(*pCurr,*pPast);
Region4DBig minLoc;
minLoc = *pCurr;
pCurr++;
//find the 3 with the closest centroids
for(int j = 0; j < m_size - 1; j++) {
float val = HistDifference(*pCurr,*pPast);
if(val < hist_diff) {
hist_diff = val;
minLoc = *pCurr;
}
pCurr++;
}
//fprintf(fp,"%f, %d, %d, %f, %d, %d, %f, %f, %f, %d, %d, %f, %f, %f\n",hist_diff[0], pCurr->m_size, size_diff[0], min[0], int(pCurr->m_centroid.x), int(pCurr->m_centroid.y), pCurr->m_centroid3D.x, pCurr->m_centroid3D.y, pCurr->m_centroid3D.z, int(minLoc[0].m_centroid.x), int(minLoc[0].m_centroid.y), minLoc[0].m_centroid3D.x, minLoc[0].m_centroid3D.y, minLoc[0].m_centroid3D.z);
pastSeg[*pPast] = minLoc;
pPast++;
}

pCurr = region_list.begin();
for(int i = 0; i < m_size; i++) {
//bipartite matching with red-black trees
if(pCurr->m_centroid3D.value == pastSeg[currSeg[*pCurr]].m_centroid3D.value) {
//the min should be normalized by the size somehow
float min = (blepo_ex::Abs((pCurr)->m_centroid3D.x - currSeg[*pCurr].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y -currSeg[*pCurr].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - currSeg[*pCurr].m_centroid3D.z)) / pCurr->m_size;
int size_diff = blepo_ex::Abs(int((pCurr)->m_size) - int(currSeg[*pCurr].m_size));
float hist_diff = HistDifference(*pCurr,currSeg[*pCurr]);
//if(hist_diff <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff <= (pCurr)->m_size * MIN_REGION_SIZE && min < MIN_REGION_DIST)))
SetBranch(&(*pCurr),(pCurr)->m_level,currSeg[*pCurr].m_centroid3D.value);
}
++pCurr;
}
//fclose(fp);
}*/

void iMergeRegions(Region4DBig &past, Region4DBig *curr) {
	if(curr != NULL && curr->m_size == 1 && past.m_size == 1) {
		//SetBranch(&(*pCurr),(pCurr)->m_level,currSeg[*pCurr].m_centroid3D.value);
		SetBranch(curr,curr->m_level,past.m_centroid3D.value);
	}
	if(curr!= NULL && curr->m_level != 1 && past.m_level != 1) {
		map<Region4DBig,Region4DBig> currSeg, pastSeg;
		Region4DBig **pCurr = curr->m_regions, **pPast;
		//FILE *fp;
		//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\stats.txt","a");
		for(int i = 0; i < curr->m_numRegions; i++) {
			//find the best match
			pPast = past.m_regions;
			float min;
			float hist_diff;
			int size_diff;
			min = blepo_ex::Abs((*pCurr)->m_centroid3D.x - (*pPast)->m_centroid3D.x) + blepo_ex::Abs((*pCurr)->m_centroid3D.y - (*pPast)->m_centroid3D.y) + blepo_ex::Abs((*pCurr)->m_centroid3D.z - (*pPast)->m_centroid3D.z);
			hist_diff = HistDifference(**pCurr,**pPast);
			Region4DBig minLoc;
			minLoc = **pPast;
			pPast++;
			for(int j = 0; j < past.m_numRegions - 1; j++) {
				float val = HistDifference(**pCurr,**pPast);
				if(val < hist_diff) {
					hist_diff = val;
					minLoc = **pPast;
				}
				pPast++;
			}
			currSeg[**pCurr] = minLoc;
			pCurr++;
		}
		pPast = past.m_regions;
		for(int i = 0; i < past.m_numRegions; i++) {
			//find the best match
			pCurr = curr->m_regions;
			float hist_diff = HistDifference(**pCurr,**pPast);
			Region4DBig minLoc;
			minLoc = **pCurr;
			pCurr++;
			//find the 3 with the closest centroids
			for(int j = 0; j < curr->m_numRegions - 1; j++) {
				float val = HistDifference(**pCurr,**pPast);
				if(val < hist_diff) {
					hist_diff = val;
					minLoc = **pCurr;
				}
				pCurr++;
			}
			//fprintf(fp,"%f, %d, %d, %f, %d, %d, %f, %f, %f, %d, %d, %f, %f, %f\n",hist_diff[0], pCurr->m_size, size_diff[0], min[0], int(pCurr->m_centroid.x), int(pCurr->m_centroid.y), pCurr->m_centroid3D.x, pCurr->m_centroid3D.y, pCurr->m_centroid3D.z, int(minLoc[0].m_centroid.x), int(minLoc[0].m_centroid.y), minLoc[0].m_centroid3D.x, minLoc[0].m_centroid3D.y, minLoc[0].m_centroid3D.z);
			pastSeg[**pPast] = minLoc;
			pPast++;
		}

		pCurr = curr->m_regions;
		for(int i = 0; i < curr->m_numRegions; i++) {
			//bipartite matching with red-black trees
			if((*pCurr)->m_centroid3D.value == pastSeg[currSeg[**pCurr]].m_centroid3D.value) {
				//the min should be normalized by the size somehow
				//float min = (blepo_ex::Abs((*pCurr)->m_centroid3D.x - currSeg[**pCurr].m_centroid3D.x) + blepo_ex::Abs((*pCurr)->m_centroid3D.y -currSeg[**pCurr].m_centroid3D.y) + blepo_ex::Abs((*pCurr)->m_centroid3D.z - currSeg[**pCurr].m_centroid3D.z)) / (*pCurr)->m_size;
				//int size_diff = blepo_ex::Abs(int((pCurr)->m_size) - int(currSeg[*pCurr].m_size));
				//float hist_diff = HistDifference(*pCurr,currSeg[*pCurr]);
				//if(hist_diff <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff <= (pCurr)->m_size * MIN_REGION_SIZE && min < MIN_REGION_DIST)))
				SetBranch(*pCurr,(*pCurr)->m_level,currSeg[**pCurr].m_centroid3D.value);
				if(currSeg[**pCurr].m_regions != NULL && (*pCurr)->m_regions != NULL)
					iMergeRegions(currSeg[**pCurr], *pCurr);
			}
			++pCurr;
		}
	}
}

void RegionTree4DBig::TemporalCorrection(RegionTree4DBig &past, int level) {
	if(!m_propagated && !past.m_propagated) {
		map<Region4DBig,Region4DBig> currSeg, pastSeg;
		//Match based on the centroid of each region
		//Region3D **pCurr = m_nodes, **pPast;
		vector<Region4DBig>::iterator pCurr = region_list.begin(), pPast;
		//FILE *fp;
		//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\stats.txt","a");
		for(int i = 0; i < m_size; i++) {
			//find the best match
			pPast = past.region_list.begin();
			float min;
			float hist_diff;
			int size_diff;
			min = blepo_ex::Abs((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
			hist_diff = HistDifference(*pCurr,*pPast);
			Region4DBig minLoc;
			minLoc = *pPast;
			pPast++;
			for(int j = 0; j < past.m_size - 1; j++) {
				float val = HistDifference(*pCurr,*pPast);
				if(val < hist_diff) {
					hist_diff = val;
					minLoc = *pPast;
				}
				pPast++;
			}
			currSeg[*pCurr] = minLoc;
			pCurr++;
		}
		pPast = past.region_list.begin();
		for(int i = 0; i < past.m_size; i++) {
			//find the best match
			pCurr = region_list.begin();
			float hist_diff = HistDifference(*pCurr,*pPast);
			Region4DBig minLoc;
			minLoc = *pCurr;
			pCurr++;
			//find the 3 with the closest centroids
			for(int j = 0; j < m_size - 1; j++) {
				float val = HistDifference(*pCurr,*pPast);
				if(val < hist_diff) {
					hist_diff = val;
					minLoc = *pCurr;
				}
				pCurr++;
			}
			//fprintf(fp,"%f, %d, %d, %f, %d, %d, %f, %f, %f, %d, %d, %f, %f, %f\n",hist_diff[0], pCurr->m_size, size_diff[0], min[0], int(pCurr->m_centroid.x), int(pCurr->m_centroid.y), pCurr->m_centroid3D.x, pCurr->m_centroid3D.y, pCurr->m_centroid3D.z, int(minLoc[0].m_centroid.x), int(minLoc[0].m_centroid.y), minLoc[0].m_centroid3D.x, minLoc[0].m_centroid3D.y, minLoc[0].m_centroid3D.z);
			pastSeg[*pPast] = minLoc;
			pPast++;
		}

		pCurr = region_list.begin();
		for(int i = 0; i < m_size; i++) {
			//bipartite matching with red-black trees
			if(pCurr->m_centroid3D.value == pastSeg[currSeg[*pCurr]].m_centroid3D.value) {
				//the min should be normalized by the size somehow
				//float min = (blepo_ex::Abs((pCurr)->m_centroid3D.x - currSeg[*pCurr].m_centroid3D.x) + blepo_ex::Abs((pCurr)->m_centroid3D.y -currSeg[*pCurr].m_centroid3D.y) + blepo_ex::Abs((pCurr)->m_centroid3D.z - currSeg[*pCurr].m_centroid3D.z)) / pCurr->m_size;
				//int size_diff = blepo_ex::Abs(int((pCurr)->m_size) - int(currSeg[*pCurr].m_size));
				//float hist_diff = HistDifference(*pCurr,currSeg[*pCurr]);
				//if(hist_diff <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff <= (pCurr)->m_size * MIN_REGION_SIZE && min < MIN_REGION_DIST)))
				SetBranch(&(*pCurr),(pCurr)->m_level,currSeg[*pCurr].m_centroid3D.value);
			}
			++pCurr;
		}
		//fclose(fp);
	} else {
		iMergeRegions(*past.m_nodes[0],m_nodes[0]);
	}
}

//currently very crude
void RegionTree3D::TemporalCorrection(vector<Region3D*> &past_list, int level) {
	vector<Region3D*> curr_list;
	GetRegionList(level,&curr_list);
	TemporalCorrection(past_list,curr_list,level);
}

template<>
void RegionTree3D::TemporalCorrection(vector<Region3D*> &past_list, vector<Region3D*> &curr_list, int level) {
	//Match based on the centroid of each region
	//Region3D **pCurr = m_nodes, **pPast;
	vector<Region3D*>::iterator pCurr = curr_list.begin(), pPast;
	//int size = (curr_list.size() < past_list.size() ? curr_list.size() : past_list.size());
	//for(int i = 0; i < size; i++) {
	while(pCurr != curr_list.end()) {
		//find the best match
		pPast = past_list.begin();
		float min[3];
		min[0] = min[1] = min[2] = blepo_ex::Abs((*pCurr)->m_centroid3D.x - (*pPast)->m_centroid3D.x) + blepo_ex::Abs((*pCurr)->m_centroid3D.y - (*pPast)->m_centroid3D.y) + blepo_ex::Abs((*pCurr)->m_centroid3D.z - (*pPast)->m_centroid3D.z);
		Region3D *minLoc[3];
		minLoc[0] = minLoc[1] = minLoc[2] = *pPast;
		pPast++;
		//find the 3 with the closest centroids
		while(pPast != past_list.end()) {
			float val = blepo_ex::Abs((*pCurr)->m_centroid3D.x - (*pPast)->m_centroid3D.x) + blepo_ex::Abs((*pCurr)->m_centroid3D.y - (*pPast)->m_centroid3D.y) + blepo_ex::Abs((*pCurr)->m_centroid3D.z - (*pPast)->m_centroid3D.z);
			if(val < min[0]) {
				min[2] = min[1];
				minLoc[2] = minLoc[1];
				min[1] = min[0];
				minLoc[1] = minLoc[0];
				min[0] = val;
				minLoc[0] = *pPast;
			} else if(val < min[1]) {
				min[2] = min[1];
				minLoc[2] = minLoc[1];
				min[1] = val;
				minLoc[1] = *pPast;
			} else if(val < min[2]) {
				min[2] = val;
				minLoc[2] = *pPast;
			}
			pPast++;
		}
		//of those 3, lets pick the one with the closest histogram and size. If none fit, give it a new segment
		float hist_diff[3];
		int size_diff[3];
		hist_diff[0] = (float)HistDifference(**pCurr,*(minLoc[0]));
		hist_diff[1] = (float)HistDifference(**pCurr,*(minLoc[1]));
		hist_diff[2] = (float)HistDifference(**pCurr,*(minLoc[2]));
		//sort by minimum hist_diff
		if(hist_diff[1] < hist_diff[0]) {
			int tmp = hist_diff[0];
			Region3D *tmp2 = minLoc[0];
			float tmp3 = min[0];
			hist_diff[0] = hist_diff[1];
			minLoc[0] = minLoc[1];
			min[0] = min[1];
			hist_diff[1] = tmp;
			minLoc[1] = tmp2;
			min[1] = tmp3;
		}
		if(hist_diff[2] < hist_diff[0]) {
			int tmp = hist_diff[0];
			Region3D *tmp2 = minLoc[0];
			float tmp3 = min[0];
			size_diff[0] = size_diff[2];
			minLoc[0] = minLoc[2];
			min[0] = min[2];
			hist_diff[2] = tmp;
			minLoc[2] = tmp2;
			min[2] = tmp3;
		}
		if(hist_diff[2] < hist_diff[1]) {
			int tmp = hist_diff[1];
			Region3D *tmp2 = minLoc[1];
			float tmp3 = min[1];
			hist_diff[1] = hist_diff[2];
			minLoc[1] = minLoc[2];
			min[1] = min[2];
			hist_diff[2] = tmp;
			minLoc[2] = tmp2;
			min[2] = tmp3;
		}
		size_diff[0] = blepo_ex::Abs((*pCurr)->m_size - minLoc[0]->m_size);
		size_diff[1] = blepo_ex::Abs((*pCurr)->m_size - minLoc[1]->m_size);
		size_diff[2] = blepo_ex::Abs((*pCurr)->m_size - minLoc[2]->m_size);
		if(hist_diff[0] <= MIN_REGION_HIST && size_diff[0] <= (*pCurr)->m_size * MIN_REGION_SIZE && min[0] < MIN_REGION_DIST)
			SetBranch(*pCurr,(*pCurr)->m_level,minLoc[0]->m_centroid3D.value);
		else if(hist_diff[1] <= MIN_REGION_HIST && size_diff[1] <= (*pCurr)->m_size * MIN_REGION_SIZE && min[1] < MIN_REGION_DIST)
			SetBranch(*pCurr,(*pCurr)->m_level,minLoc[1]->m_centroid3D.value);
		else if(hist_diff[2] <= MIN_REGION_HIST && size_diff[2] <= (*pCurr)->m_size * MIN_REGION_SIZE && min[2] < MIN_REGION_DIST)
			SetBranch(*pCurr,(*pCurr)->m_level,minLoc[2]->m_centroid3D.value);
		pCurr++;
	}
}

void UpdateTable(Region3D* child1, Region3D* child2, Region3D* father, Region3D**  lookup, int size) {
	//for every value in the table, if it is = to child1 or = to child2, set it = to father
	Region3D **p = lookup, **pEnd = lookup + size;
	while(p != pEnd) {
		if(*p == child1 || *p == child2)
			*p = father;
		p++;
	}
}

void UpdateTable(Region4D* child1, Region4D* child2, Region4D* father, Region4D**  lookup, int size) {
	//for every value in the table, if it is = to child1 or = to child2, set it = to father
	Region4D **p = lookup, **pEnd = lookup + size;
	while(p != pEnd) {
		if(*p == child1 || *p == child2)
			*p = father;
		p++;
	}
}

void UpdateTable(Region4DBig* child1, Region4DBig* child2, Region4DBig* father, Region4DBig**  lookup, int size) {
	//for every value in the table, if it is = to child1 or = to child2, set it = to father
	Region4DBig **p = lookup, **pEnd = lookup + size;
	while(p != pEnd) {
		if(*p == child1 || *p == child2)
			*p = father;
		p++;
	}
}

void UpdateTable(Region4DBig* child1, Region4DBig* child2, Region4DBig* father, vector<Region4DBig> &lookup) {
	//for every value in the table, if it is = to child1 or = to child2, set it = to father
	vector<Region4DBig>::iterator p = lookup.begin();
	while(p != lookup.end()) {
		if(&(*p) == child1 || &(*p) == child2)
			*p = *father;
		p++;
	}
}



void RegionTree3D::PropagateRegionHierarchy(int min_size) {
	//lets start by making a handy label lookup table
	Region3D** lookup;
	//find max label
	Region3D **p = m_nodes, **pEnd = m_nodes + m_size;
	int num_edges = 0;
	int max = (*p)->m_centroid3D.value;
	while(p != pEnd) {
		int label = (*p)->m_centroid3D.value;
		if(label > max)
			max = label;
		num_edges += (*p)->m_neighbors.size();
		p++;
	}
	lookup = new Region3D*[max]();
	int i = 0;
	p = m_nodes;
	while(p != pEnd) {
		lookup[(*p)->m_centroid3D.value] = *p;
		p++;
	}
	//printf("Max found as %d, size: %d, num_edges: %d\n",max,m_size,num_edges);
	//lets build the a new universe with edges between the regions and their neighbors with the index being the segment label
	Edge* edges = new Edge[num_edges]();
	p = m_nodes;
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(p != pEnd) {
		vector<int>::const_iterator pNeighbors = (*p)->m_neighbors.begin(), pNeighborEnd = (*p)->m_neighbors.end();
		while(pNeighbors != pNeighborEnd) {
			pEdge->a = (*p)->m_centroid3D.value;
			pEdge->b = *pNeighbors;
			//I'm not sure which of these is better, basing it off the size or the Histogram difference. Should be tested empirically.
			pEdge->w = (float)HistDifference(**p,*(lookup[*pNeighbors]));// (float)(*p)->m_size;
			//pEdge->w = (float)(*p)->m_size;
			pNeighbors++;
			pEdge++;
			i++;
		}
		p++;
	}

	//printf("Graph built, went through %d Edges\n",i);
	//now that we are done building the graph, join it upwards recursively after sorting it
	blepo_ex::Sort(edges, edgesEnd);

	//now we should combine neighbors into higher level regions, this is tricky
	//for each edge, create a new region that points to the regions the edge connects, then recompute the new region statistics and point the lookup table to this new region
	pEdge = edges;
	i=0;
	int current = 0;
	//printf("Original: %d, ",numRegions);
	Region3D *father;
	while(pEdge != edgesEnd) {			
		Region3D *reg1 = lookup[pEdge->a], *reg2 = lookup[pEdge->b];
		if(reg1 != reg2 && reg1->m_centroid3D.value != reg2->m_centroid3D.value) {
			father = &(region_list[numRegions]);
			numRegions++;
			assert(numRegions < totRegions);
			father->InitializeRegion(reg1,reg2,current,min_size);
			//Instead of this, I should go through the whole table looking up the old pointer values and replace them with the new pointer values
			UpdateTable(reg1,reg2,father,lookup,max);
			//this is just in case, I probably don't need to do this
			lookup[pEdge->a] = father;
			lookup[pEdge->b] = father;
		}
		pEdge++;
		i++;
		current++;
		//printf("Edge no: %d\n",i);
	}
	//printf("Graph joined, went through %d Edges\n",i);
	//If everything goes well, there should be only one region left.
	delete[] m_nodes;
	m_size = 1;
	m_nodes = new Region3D*[1]();
	//printf("Final: %d\n",numRegions);
	pEdge--;
	m_nodes[0] = lookup[pEdge->a];
	//m_nodes[0] = new Region3D();
	//*(m_nodes[0] ) = *(lookup[pEdge->a]);
	m_propagated = true;
	//delete lookup; //somehow I must have already deleted this?
	delete[] edges;
}

void RegionTree4D::PropagateRegionHierarchy(int min_size) {
	//lets start by making a handy label lookup table
	Region4D** lookup;
	//find max label
	Region4D **p = m_nodes, **pEnd = m_nodes + m_size;
	int num_edges = 0;
	int max = (*p)->m_centroid3D.value;
	while(p != pEnd) {
		int label = (*p)->m_centroid3D.value;
		if(label > max)
			max = label;
		num_edges += (*p)->m_neighbors.size();
		p++;
	}
	lookup = new Region4D*[max]();
	int i = 0;
	p = m_nodes;
	while(p != pEnd) {
		lookup[(*p)->m_centroid3D.value] = *p;
		p++;
	}

	//printf("Max found as %d, size: %d, num_edges: %d\n",max,m_size,num_edges);
	//lets build the a new universe with edges between the regions and their neighbors with the index being the segment label
	Edge* edges = new Edge[num_edges]();
	p = m_nodes;
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(p != pEnd) {
		vector<int>::const_iterator pNeighbors = (*p)->m_neighbors.begin(), pNeighborEnd = (*p)->m_neighbors.end();
		while(pNeighbors != pNeighborEnd) {
			pEdge->a = (*p)->m_centroid3D.value;
			pEdge->b = *pNeighbors;
			pEdge->w = (float)HistDifference(**p,*(lookup[*pNeighbors]));// (float)(*p)->m_size;
			pNeighbors++;
			pEdge++;
			i++;
		}
		p++;
	}

	//printf("Graph built, went through %d Edges\n",i);
	//now that we are done building the graph, join it upwards recursively after sorting it
	blepo_ex::Sort(edges, edgesEnd);

	//now we should combine neighbors into higher level regions, this is tricky
	//for each edge, create a new region that points to the regions the edge connects, then recompute the new region statistics and point the lookup table to this new region
	pEdge = edges;
	i=0;
	int current = 0;
	while(pEdge != edgesEnd) {			
		Region4D *reg1 = lookup[pEdge->a], *reg2 = lookup[pEdge->b];
		if(reg1 != reg2 && reg1->m_centroid3D.value != reg2->m_centroid3D.value) {
			Region4D *father = new Region4D();
			father->InitializeRegion(reg1,reg2,current,min_size);
			//Instead of this, I should go through the whole table looking up the old pointer values and replace them with the new pointer values
			UpdateTable(reg1,reg2,father,lookup,max);
			//this is just in case, I probably don't need to do this
			lookup[pEdge->a] = father;
			lookup[pEdge->b] = father;
		}
		pEdge++;
		i++;
		current++;
		//printf("Edge no: %d\n",i);
	}
	//printf("Graph joined, went through %d Edges\n",i);
	//If everything goes well, there should be only one region left.
	delete m_nodes;
	m_size = 1;
	m_nodes = new Region4D*[1]();
	pEdge--;
	m_nodes[0] = lookup[pEdge->a];
	m_propagated = true;
	//delete lookup; //somehow I must have already deleted this?
	delete edges;
}

void RegionTree4DBig::PropagateRegionHierarchy(int min_size) {
	//lets start by making a handy label lookup table
	//find max label

	vector<Region4DBig>::iterator p = region_list.begin();
	int num_edges = 0;
	while(p != region_list.end()) {
		num_edges += p->m_neighbors.size();
		p++;
	}
	int i = 0;
	//printf("Max found as %d, size: %d, num_edges: %d\n",max,m_size,num_edges);
	//lets build the a new universe with edges between the regions and their neighbors with the index being the segment label
	Edge* edges = new Edge[num_edges]();
	p =region_list.begin();
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(p != region_list.end()) {
		vector<int>::const_iterator pNeighbors = p->m_neighbors.begin(), pNeighborEnd = p->m_neighbors.end();
		while(pNeighbors != pNeighborEnd) {
			//if(lookup[*pNeighbors] != NULL) {
			pEdge->a = i;
			pEdge->b = *pNeighbors;
			//pEdge->w = (float)HistDifference(**p,*(lookup[*pNeighbors]));// (float)(*p)->m_size;
			//pEdge->w = (float)HistDifference(**p,*(*p)->m_neighbor_map[*pNeighbors]);// (float)(*p)->m_size;
			pEdge->w = (float)HistDifference(*p,region_list[*pNeighbors]);
			pEdge++;
			//}
			pNeighbors++;
		}
		i++;
		p++;
	}

	//printf("Graph built, went through %d Edges\n",i);
	//now that we are done building the graph, join it upwards recursively after sorting it
	blepo_ex::Sort(edges, edgesEnd);

	//now we should combine neighbors into higher level regions, this is tricky
	//for each edge, create a new region that points to the regions the edge connects, then recompute the new region statistics and point the lookup table to this new region
	pEdge = edges;
	i=0;
	int current = 0;
	while(pEdge != edgesEnd) {			
		Region4DBig reg1 = region_list[pEdge->a], reg2 = region_list[pEdge->b];
		if(reg1.m_centroid3D.value != reg2.m_centroid3D.value) {
			Region4DBig *father = new Region4DBig();
			father->InitializeRegion(&reg1,&reg2,current,min_size);
			//Instead of this, I should go through the whole table looking up the old pointer values and replace them with the new pointer values
			UpdateTable(&reg1,&reg2,father,region_list);
			//this is just in case, I probably don't need to do this
			region_list[pEdge->a] = *father;
			region_list[pEdge->b] = *father;
		}
		pEdge++;
		i++;
		current++;
		//printf("Edge no: %d\n",i);
	}
	//printf("Graph joined, went through %d Edges\n",i);
	//If everything goes well, there should be only one region left.
	delete m_nodes;
	m_size = 1;
	m_nodes = new Region4DBig*[1]();
	pEdge--;
	m_nodes[0] = &region_list[pEdge->a];
	m_propagated = true;
	//delete lookup; //somehow I must have already deleted this?
	delete edges;
}
