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
#include "RegionTree.h"
//#include "GraphSegmentation.h"

using namespace std;
using namespace pcl;
using namespace cv;

inline void LAB::RGB2XYZ(int r, int g, int b, float *x, float *y, float *z) {
	//convert RGB to xyz
	float rr = r / 255.0f;
	float gg = g / 255.0f;
	float bb = b / 255.0f;

	if (rr > 0.04045f) { rr = powf((rr + 0.055f) / 1.055f, 2.4f); }
	else { rr = rr / 12.92f; }
	if ( gg > 0.04045f){ gg = powf((gg + 0.055f) / 1.055f, 2.4f); }
	else { gg = gg / 12.92f; }
	if (bb > 0.04045f){ bb = powf((bb + 0.055f) / 1.055f, 2.4f); }
	else {	bb = bb / 12.92f; }
	rr *= 100.0f;
	gg *= 100.0f;
	bb *= 100.0f;

	*x = rr * 0.4124f + gg * 0.3576f + bb * 0.1805f;
	*y = rr * 0.2126f + gg * 0.7152f + bb * 0.0722f;
	*z = rr * 0.0193f + gg * 0.1192f + bb * 0.9505f;
}

inline void LAB::XYZ2LAB(float x, float y, float z, int *l, int *a, int *b) {
	//convert xyz to cielab
	/*x /=  95.047f;
	y /=  100.0f;
	z /=  108.883f;

	const float expo = 1.0f/3.0f, expo2 = 16.0f / 116.0f;
	if ( x > 0.008856f ) { x = Pow( x , expo ); }
	else { x = ( 7.787f * x ) + ( expo2 ); }
	if ( y > 0.008856f ) { y = Pow( y , expo ); }
	else { y = ( 7.787f * y ) + ( expo2 ); }
	if ( z > 0.008856f ) { z = Pow( z , expo ); }
	else { z = ( 7.787f * z ) + ( expo2 ); }

	l = int(( 116.0f * y ) - 16.0f);
	a = int(500.0f * ( x - y ) + 128.0f); //to make between 0 and 256
	b = int(200.0f * ( y - z ) + 128.0f); //to make between 0 and 256
	*/

	//convert xyz to Hunter lab
	const float sqrt_y = sqrtf(y);
	if(sqrt_y == 0.0f) {
		*l = 0;
		*a = *b = 128;
	} else {
		*l = int(10.0f * sqrt_y);
		*a = int(17.5f * (1.02f * x - y) / sqrt_y + 128.0f);
		*b = int( 7.0f * (y - 0.847f * z ) / sqrt_y + 128.0f);
	}
}

LAB::LAB(int r_val, int g_val, int b_val) {
	float xx,yy,zz;
	RGB2XYZ(r_val,g_val,b_val,&xx,&yy,&zz);
	XYZ2LAB(xx,yy,zz,&l,&a,&b);
}


LABXYZUVW::LABXYZUVW(int r_val, int g_val, int b_val, float x_val, float y_val, float z_val, float u_val, float v_val, float w_val) {
	LAB lab = LAB(r_val, g_val, b_val);
	l = lab.l;
	a = lab.a;
	b = lab.b;
	u = u_val + 1.0f;  //Need to shift these as well
	v = v_val + 1.0f;
	w = w_val + 1.0f;  //not sure about these numbers
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


inline void Region3D::InitializeRegion(PointXYZI *in, Vec3b color, const pcl::PointNormal &normal, const int label, const int i, const int j, const int level) {
	//if(!isnan(in->z)) {
	m_level  = level;
	m_size = 1;
	m_hist = new LABXYZUVW[NUM_BINS_XYZ]();
	LABXYZUVW labxyz = LABXYZUVW(color[2],color[1],color[0],in->x, in->y, in->z,normal.normal_x,normal.normal_y,normal.normal_z);
	m_hist[Clamp(Round(labxyz.l * HIST_MUL_L),0,NUM_BINS)].l++;
	m_hist[Clamp(Round(labxyz.a * HIST_MUL_A),0,NUM_BINS)].a++;
	m_hist[Clamp(Round(labxyz.b * HIST_MUL_B),0,NUM_BINS)].b++;
	m_hist[Clamp(Round(labxyz.x * HIST_MUL_X),0,NUM_BINS_XYZ-1)].x++;
	m_hist[Clamp(Round(labxyz.y * HIST_MUL_Y),0,NUM_BINS_XYZ-1)].y++;
	m_hist[Clamp(Round(labxyz.z * HIST_MUL_Z),0,NUM_BINS_XYZ-1)].z++;
	if(!isnan(labxyz.u))
		m_hist[Clamp(Round(labxyz.u * HIST_MUL_N),0,NUM_BINS)].u++;
	if(!isnan(labxyz.v))
		m_hist[Clamp(Round(labxyz.v * HIST_MUL_N),0,NUM_BINS)].v++;
	if(!isnan(labxyz.w))
		m_hist[Clamp(Round(labxyz.w * HIST_MUL_N),0,NUM_BINS)].w++;
	m_centroid = Point(i,j);
	m_centroid3D.x = m_min3D.x = m_max3D.x = in->x;
	m_centroid3D.y = m_min3D.y = m_max3D.y = in->y;
	m_centroid3D.z = m_min3D.z = m_max3D.z = in->z;
	m_centroid3D.intensity = label;
	m_nodes.reserve(76800);
	m_neighbors.reserve(8);
	m_nodes.push_back(in);
	m_regions[0] = m_regions[1] = NULL;
	m_numRegions = 0;
	//}
}

inline void Region3D::InitializeRegion(Region3D *node1, Region3D *node2, int level, int min_size) {
	assert(node1 != NULL && node2 != NULL);
	if(node1 == NULL || node2 == NULL || node1->m_size <= 0 || node2->m_size <= 0) {
		printf("Error, this should never happen\n");
	}
	/*if(node1->m_level == node2->m_level) {
	//m_level = node1->m_level + 1;
	if(node1->m_size > node2->m_size)
	m_centroid3D.intensity = node1->m_centroid3D.intensity;
	else
	m_centroid3D.intensity = node2->m_centroid3D.intensity;
	} else*/ if(node1->m_level > node2->m_level) {
		//m_level = node1->m_level + 1;
		m_centroid3D.intensity = node1->m_centroid3D.intensity;
	} else {
		//m_level = node2->m_level + 1;
		m_centroid3D.intensity = node2->m_centroid3D.intensity;
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
	m_min3D = node1->m_min3D;
	m_max3D = node1->m_max3D;
	if(m_min3D.x > node2->m_min3D.x)
		m_min3D.x = node2->m_min3D.x;
	if(m_min3D.y > node2->m_min3D.y)
		m_min3D.y = node2->m_min3D.y;
	if(m_min3D.z > node2->m_min3D.z)
		m_min3D.z = node2->m_min3D.z;
	if(m_max3D.x < node2->m_max3D.x)
		m_max3D.x = node2->m_max3D.x;
	if(m_max3D.y < node2->m_max3D.y)
		m_max3D.y = node2->m_max3D.y;
	if(m_max3D.z < node2->m_max3D.z)
		m_max3D.z = node2->m_max3D.z;
	//I might want to add the neighbors later but for now I don't think it matters
	m_numRegions = 2;
	m_regions[0] = node1;
	m_regions[1] = node2;
}

inline void Region3D::AddNode(PointXYZI *in, Vec3b color, const pcl::PointNormal &normal, const int i, const int j) {
	//if(!isnan(in->z)) {
	LABXYZUVW labxyz = LABXYZUVW(color[2],color[1],color[0],in->x,in->y,in->z,normal.normal_x,normal.normal_y,normal.normal_z);
	m_hist[Clamp(Round(labxyz.l * HIST_MUL_L), 0, NUM_BINS)].l++;
	m_hist[Clamp(Round(labxyz.a * HIST_MUL_A), 0, NUM_BINS)].a++;
	m_hist[Clamp(Round(labxyz.b * HIST_MUL_B), 0, NUM_BINS)].b++;
	m_hist[Clamp(Round(labxyz.x * HIST_MUL_X), 0, NUM_BINS_XYZ-1)].x++;
	m_hist[Clamp(Round(labxyz.y * HIST_MUL_Y), 0, NUM_BINS_XYZ-1)].y++;
	m_hist[Clamp(Round(labxyz.z * HIST_MUL_Z), 0, NUM_BINS_XYZ-1)].z++;
	if(!isnan(labxyz.u))
		m_hist[Clamp(Round(labxyz.u * HIST_MUL_N),0,NUM_BINS)].u++;
	if(!isnan(labxyz.v))
		m_hist[Clamp(Round(labxyz.v * HIST_MUL_N),0,NUM_BINS)].v++;
	if(!isnan(labxyz.w))
		m_hist[Clamp(Round(labxyz.w * HIST_MUL_N),0,NUM_BINS)].w++;
	m_size++;
	m_centroid.x += i;
	m_centroid.y += j;
	m_centroid3D.x += in->x;
	m_centroid3D.y += in->y;
	m_centroid3D.z += in->z;
	m_nodes.push_back(in);
	if(m_min3D.x > in->x)
		m_min3D.x = in->x;
	if(m_min3D.y > in->y)
		m_min3D.y = in->y;
	if(m_min3D.z > in->z)
		m_min3D.z = in->z;
	if(m_max3D.x < in->x)
		m_max3D.x = in->x;
	if(m_max3D.y < in->y)
		m_max3D.y = in->y;
	if(m_max3D.z < in->z)
		m_max3D.z = in->z;
	//}
}

inline void Region4DBig::InitializeRegion(PointXYZI *in, Vec3b color, const Normal &flow, const int label, const int i, const int j, const int level) {
	m_level  = level;
	m_size = 1;
	m_hist = new LABXYZUVW[NUM_BINS_XYZ]();
	LABXYZUVW labxyzuvw = LABXYZUVW(color[2],color[1],color[0],in->x, in->y, in->z, flow.normal_x, flow.normal_y, flow.normal_z);
	m_hist[Clamp(Round(labxyzuvw.l * HIST_MUL_L),0,NUM_BINS)].l++;
	m_hist[Clamp(Round(labxyzuvw.a * HIST_MUL_A),0,NUM_BINS)].a++;
	m_hist[Clamp(Round(labxyzuvw.b * HIST_MUL_B),0,NUM_BINS)].b++;
	//Need to shift all these to make them positive
	m_hist[Clamp(Round(labxyzuvw.x * HIST_MUL_X),0,NUM_BINS_XYZ-1)].x++;
	m_hist[Clamp(Round(labxyzuvw.y * HIST_MUL_Y),0,NUM_BINS_XYZ-1)].y++;
	m_hist[Clamp(Round(labxyzuvw.z * HIST_MUL_Z),0,NUM_BINS_XYZ-1)].z++;
	m_hist[Clamp(Round(labxyzuvw.u * HIST_MUL_OF),0,NUM_BINS)].u++;
	m_hist[Clamp(Round(labxyzuvw.v * HIST_MUL_OF),0,NUM_BINS)].v++;
	m_hist[Clamp(Round(labxyzuvw.w * HIST_MUL_OF),0,NUM_BINS)].w++;
	m_centroid = Point(i,j);
	m_centroid3D.x = in->x;
	m_centroid3D.y = in->y;
	m_centroid3D.z = in->z;
	m_centroid3D.intensity = label;
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
	m_centroid3D.intensity = node1->m_centroid3D.intensity;
	else
	m_centroid3D.intensity = node2->m_centroid3D.intensity;
	} else*/ if(node1->m_level > node2->m_level) {
		//m_level = node1->m_level + 1;
		m_centroid3D.intensity = node1->m_centroid3D.intensity;
	} else {
		//m_level = node2->m_level + 1;
		m_centroid3D.intensity = node2->m_centroid3D.intensity;
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

inline void Region4DBig::AddNode(PointXYZI *in, Vec3b color, const Normal &flow, const int i, const int j) {
	LABXYZUVW labxyzuvw = LABXYZUVW(color[2],color[1],color[0],in->x, in->y, in->z, flow.normal_x, flow.normal_y, flow.normal_z);
	m_hist[Clamp(Round(labxyzuvw.l * HIST_MUL_L),0,NUM_BINS)].l++;
	m_hist[Clamp(Round(labxyzuvw.a * HIST_MUL_A),0,NUM_BINS)].a++;
	m_hist[Clamp(Round(labxyzuvw.b * HIST_MUL_B),0,NUM_BINS)].b++;
	//Need to shift all these to make them positive
	m_hist[Clamp(Round(labxyzuvw.x * HIST_MUL_X),0,NUM_BINS_XYZ-1)].x++;
	m_hist[Clamp(Round(labxyzuvw.y * HIST_MUL_Y),0,NUM_BINS_XYZ-1)].y++;
	m_hist[Clamp(Round(labxyzuvw.z * HIST_MUL_Z),0,NUM_BINS_XYZ-1)].z++;
	m_hist[Clamp(Round(labxyzuvw.u * HIST_MUL_OF),0,NUM_BINS)].u++;
	m_hist[Clamp(Round(labxyzuvw.v * HIST_MUL_OF),0,NUM_BINS)].v++;
	m_hist[Clamp(Round(labxyzuvw.w * HIST_MUL_OF),0,NUM_BINS)].w++;
	/*int new_size = m_size + 1;
	m_centroid.x = (m_centroid.x * m_size + i) / new_size;
	m_centroid.y = (m_centroid.y * m_size + j) / new_size;
	m_centroid3D.x = (m_centroid3D.x * m_size + in->x) / new_size;
	m_centroid3D.y = (m_centroid3D.y * m_size + in->y) / new_size;
	m_centroid3D.z = (m_centroid3D.z * m_size + in->z) / new_size;
	m_size = new_size;*/
	m_size++;
	m_centroid.x += i;
	m_centroid.y += j;
	m_centroid3D.x += in->x;
	m_centroid3D.y += in->y;
	m_centroid3D.z += in->z;
	m_nodes.push_back(in);
	if(m_min3D.x > in->x)
		m_min3D.x = in->x;
	if(m_min3D.y > in->y)
		m_min3D.y = in->y;
	if(m_min3D.z > in->z)
		m_min3D.z = in->z;
	if(m_max3D.x < in->x)
		m_max3D.x = in->x;
	if(m_max3D.y < in->y)
		m_max3D.y = in->y;
	if(m_max3D.z < in->z)
		m_max3D.z = in->z;
}


inline void MinMax(const PointCloudInt &cloud, int *min, int *max) {
	PointCloudInt::const_iterator p = cloud.begin();
	*min = *max = int(p->intensity);
	while(p != cloud.end()) {
		if(p->intensity < *min)
			*min = p->intensity;
		else if(p->intensity> *max)
			*max = p->intensity;
		++p;
	}
}

template<>
void RegionTree3D::Create(const PointCloudBgr &in, PointCloudInt &labels, const pcl::PointCloud<pcl::PointNormal> &normals, int num_segments, int start_label) {
	this->Release();
	*this = RegionTree3D(num_segments,in.width,in.height);
	//the original region tree is only two levels, the original segments and then the voxels(leafs)

	//create a lookup table to map the labels (valid lookups will have to start at 0)
	int min, max, current = 0;
	MinMax(labels,&min,&max);
	max++;
	int *lookup = new int[max]();
	int *pLook = lookup, *pLookEnd = lookup + max;
	while(pLook != pLookEnd)
		*pLook++ = -1;
	PointCloudBgr::const_iterator pIn = in.begin();
    PointCloudInt::iterator pLabel = labels.begin();
	PointCloudNormal::const_iterator pNormal = normals.begin();
	int loc, label, i, j;
	const int safeWidth = in.width - 1, safeHeight = in.height - 1;
	Region3D* pRegion = NULL;
	//printf("Original num of Segments: %d\n",num_segments);
	numRegions = 0;
	totRegions = ((num_segments + 2) << 1) + 1;
	region_list.resize(totRegions);
	//float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(j = 0; j < in.height; j++) {
		for(i = 0; i < in.width; i++) {
			//if(!isnan(pIn->z)) {
			//for each voxel, add to the appropriate region and compute important values
			loc = lookup[int(pLabel->intensity)];
			//Am I a new region?
			if(loc == -1) {
				//I'm a new region, initialize
				if(pLabel->intensity>= max) {
					printf("Vector out of range\n");
				}
				lookup[int(pLabel->intensity)] = current;
				if(numRegions >= totRegions) {
					printf("Vector out of range\n");
				}
				region_list[numRegions] = new Region3D();
				region_list[numRegions]->InitializeRegion(&*pLabel, Vec3b(pIn->b,pIn->g,pIn->r), *pNormal, current + start_label, i, j, 1);
				pRegion = region_list[numRegions];
				if(current >= totRegions) {
					printf("Vector out of range\n");
				}
				m_nodes[current] = pRegion;
				numRegions++;
				current++;
			}
			//}
			pIn++; pLabel++; pNormal++;
		}
	}
	//region_list.shrink_to_fit();
	region_list.resize(numRegions);
	pIn = in.begin();
	pLabel = labels.begin();
	pNormal = normals.begin();
	Region3D *tmp = NULL;
	for(j = 0; j < in.height; j++) {
		for(i = 0; i < in.width; i++) {
			//if(!isnan(pIn->z)) {
			//I'm not new, add me appropriately
			//Add node to appropriately region list
			loc = lookup[int(pLabel->intensity)];
			pRegion = region_list[loc];
			assert(pRegion != NULL);
			region_list[loc]->AddNode(&*pLabel, Vec3b(pIn->b,pIn->g,pIn->r),*pNormal,i,j);
			//Check for neighbors
			if(i < safeWidth) {
				label = lookup[int((pLabel + 1)->intensity)];
				tmp = region_list[label];
				assert(tmp != NULL);
				if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
					pRegion->m_neighbors.push_back(label);
				if(j < safeHeight) {
					label = lookup[int((pLabel + in.width + 1)->intensity)];
					tmp = region_list[label];
					assert(tmp != NULL);
					if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
				}
				if(j > 0) {
					label = lookup[int((pLabel - in.width + 1)->intensity)];
					tmp = region_list[label];
					assert(tmp != NULL);
					if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
				}
			}
			if(j < safeHeight) {
				label = lookup[int((pLabel + in.width)->intensity)];
				tmp = region_list[label];
				assert(tmp != NULL);
				if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
					pRegion->m_neighbors.push_back(label);
			}
			//}
			pIn++; pLabel++; pNormal++;
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

template<>
void RegionTree4DBig::Create(deque<PointCloudBgr> &in, deque< pcl::PointCloud<pcl::Normal> > &flow, PointCloudInt *labels, int num_segments, int start_label) {
	//this->Release();
	*this = RegionTree4DBig(num_segments,in[0].width,in[0].height);
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
	const int safeWidth = in[0].width - 1, safeHeight = in[0].height - 1;
	Region4DBig* pRegion;
	numRegions = 0;
	//not sure about this
	totRegions = ((num_segments + 2) << 1) + 1;
	region_list.resize(totRegions);
	for(k = 0; k < NUM_FRAMES; k++) {
		//need to do something about the optical flow of the first image
		PointCloudBgr::const_iterator pIn = in[k].begin();
		PointCloudInt::iterator pLabel = labels[k].begin();
		//ComputeOpticalFlow(in[k-1],in[k],&flow);
		pcl::PointCloud<pcl::Normal>::const_iterator pFlow = flow[k].begin();
		for(j = 0; j < in[0].height; j++) {
			for(i = 0; i < in[0].width; i++) {
				//for each voxel, add to the appropriate region and compute important values
				loc = lookup[int(pLabel->intensity)];
				//Am I a new region?
				if(loc == -1) {				
					//I'm a new region, initialize
					if(int(pLabel->intensity) >= max) {
						printf("Vector out of range\n");
					}
					lookup[int(pLabel->intensity)] = current;
					if(numRegions >= totRegions) {
						printf("Vector out of range\n");
					}
					region_list[numRegions] = new Region4DBig();
					pRegion = (region_list[numRegions]);
					//pRegion->InitializeRegion(pLabel, pIn->intensity, pLabel->intensity, i, j, 1);
					region_list[numRegions]->InitializeRegion(&*pLabel, Vec3b(pIn->b,pIn->g,pIn->r), *pFlow, current + start_label, i, j, 1);
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
		PointCloudBgr::const_iterator pIn = in[k].begin();
		PointCloudInt::iterator pLabel = labels[k].begin();
		//ComputeOpticalFlow(in[k-1],in[k],&flow);
		pcl::PointCloud<pcl::Normal>::const_iterator pFlow = flow[k].begin();
		for(j = 0; j < in[k].height; j++) {
			for(i = 0; i < in[k].width; i++) {
				//I'm not new, add me appropriately
				//Add node to appropriately region list
				loc = lookup[int(pLabel->intensity)];
				pRegion = (region_list[loc]);
				assert(pRegion != NULL);
				region_list[loc]->AddNode(&*pLabel,Vec3b(pIn->b,pIn->g,pIn->r),*pFlow,i,j);
				//Check for neighbors
				if(i < safeWidth) {
					label = lookup[int((pLabel + 1)->intensity)];
					tmp = (region_list[label]);
					if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end()) {
						pRegion->m_neighbors.push_back(label);
						pRegion->m_neighbor_map[label] = tmp;
					}
					if(j < safeHeight) {
						label = lookup[int((pLabel + in[k].width + 1)->intensity)];
						tmp = (region_list[label]);
						if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end()) {
							pRegion->m_neighbors.push_back(label);
							pRegion->m_neighbor_map[label] = tmp;
						}
					}
					if(j > 0) {
						label = lookup[int((pLabel - in[k].width + 1)->intensity)];
						tmp = (region_list[label]);
						if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end()) {
							pRegion->m_neighbors.push_back(label);
							pRegion->m_neighbor_map[label] = tmp;
						}
					}
				}
				if(j < safeHeight) {
					label = lookup[int((pLabel + in[k].width)->intensity)];
					tmp = (region_list[label]);
					if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end()) {
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

void SetBranch(RegionTree3D *tree, Region3D* region, int level, int label) {
	assert(region != NULL);
	if(label == -1 && region->m_level <= level) {
		//I should set the label
		label = region->m_centroid3D.intensity;
		tree->top_regions.push_back(region);
	}
	if(region->m_numRegions != 0 && region->m_regions[0] != NULL && region->m_regions[1] != NULL) {
		//I am not at the leaf level, tell my children to do proper
		Region3D **branch = region->m_regions;
		for(int i = 0; i < region->m_numRegions; i++) {
			SetBranch(tree, *branch,level, label);
			branch++;
		}
	} else {
		//I contain the leaves, Set each one.
		assert(label != -1);
		//set my label
		region->m_centroid3D.intensity = label;
		vector<PointXYZI*>::iterator pNode = region->m_nodes.begin();
		while(pNode != region->m_nodes.end()) {
			(*pNode)->intensity = label;
			pNode++;
		}
	}
}

void SetBranch(RegionTree4DBig *tree, Region4DBig* region, int level, int label) {
	assert(region != NULL);
	if(label == -1 && region->m_level <= level) {
		//I should set the label
		label = region->m_centroid3D.intensity;
		tree->top_regions.push_back(region);
	}
	if(region->m_numRegions != 0) {
		//I am not at the leaf level, tell my children to do proper
		Region4DBig **branch = region->m_regions;
		for(int i = 0; i < region->m_numRegions; i++) {
			SetBranch(tree, *branch,level, label);
			branch++;
		}
	} else {
		//I contain the leaves, Set each one.
		assert(label != -1);
		//set my label
		region->m_centroid3D.intensity = label;
		vector<PointXYZI*>::iterator pNode = region->m_nodes.begin();
		while(pNode != region->m_nodes.end()) {
			(*pNode)->intensity = label;
			pNode++;
		}
	}
}

template<>
void RegionTree3D::UpdateCloud(int level) {
	//start by finding the right level for each branch
	int i;
	Region3D** branch = m_nodes;
	for(i = 0; i < m_size; i++) {
		SetBranch(this, *branch, level, -1);
		branch++;
	}
}

template<>
void RegionTree4DBig::UpdateCloud(int level) {
	//start by finding the right level for each branch
	int i;
	Region4DBig** branch = m_nodes;
	for(i = 0; i < m_size; i++) {
		SetBranch(this, *branch, level, -1);
		branch++;
	}
}

//perhaps this is wrong, perhaps I should normalize somehow first, maybe by the size in order to get percentages?
inline float HistDifference(Region3D &reg1, Region3D &reg2) {
	if(reg1.m_hist != NULL && reg2.m_hist != NULL) {
		float sad = 0.0f;
		LABXYZUVW *p1 = reg1.m_hist, *p2 = reg2.m_hist;
		int i;
		for(i = 0; i < NUM_BINS; i++, p1++, p2++) {
			sad += HIST_COLOR_MOD * (fabsf(float(p1->a) / reg1.m_size - float(p2->a) / reg2.m_size) + fabsf(float(p1->b) / reg1.m_size - float(p2->b) / reg2.m_size) + fabsf(float(p1->l) / reg1.m_size - float(p2->l) / reg2.m_size)) + HIST_DEPTH_MOD * (fabsf(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + fabsf(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + fabsf(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size)) + HIST_NORMAL_MOD * (fabsf(float(p1->u) / reg1.m_size - float(p2->u) / reg2.m_size) + fabsf(float(p1->v) / reg1.m_size - float(p2->v) / reg2.m_size) + fabsf(float(p1->w) / reg1.m_size - float(p2->w) / reg2.m_size)); 
		}
		while(i < NUM_BINS_XYZ) {
			sad += HIST_DEPTH_MOD * (fabsf(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + fabsf(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + fabsf(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size));
			++i; ++p1; ++p2;
		}
		float min = (fabsf(reg1.m_centroid3D.x - reg2.m_centroid3D.x) + fabsf(reg1.m_centroid3D.y - reg2.m_centroid3D.y) + fabsf(reg1.m_centroid3D.z - reg2.m_centroid3D.z)) / (reg1.m_size + reg2.m_size / 2);
		float size_diff = float(abs(int(reg1.m_size) - int(reg2.m_size))) / (float(reg1.m_size + reg2.m_size) / 2);
		return sad + CENTROID_MUL * min + size_diff * SIZE_MUL;
	}
	return 1000000;
}

//lets include size and centroid difference in this as well.
inline float HistDifference(Region4DBig &reg1, Region4DBig &reg2) {
	if(reg1.m_hist != NULL && reg2.m_hist != NULL) {
		float sad = 0.0f;
		LABXYZUVW *p1 = reg1.m_hist, *p2 = reg2.m_hist;
		int i;
		for(i = 0; i < NUM_BINS; i++, p1++, p2++) {
			sad += HIST_COLOR_MOD * (fabsf(float(p1->a) / reg1.m_size - float(p2->a) / reg2.m_size) + fabsf(float(p1->b) / reg1.m_size - float(p2->b) / reg2.m_size) +fabsf(float(p1->l) / reg1.m_size - float(p2->l) / reg2.m_size) +fabsf(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) +fabsf(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) +fabsf(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size) +fabsf(float(p1->u) / reg1.m_size - float(p2->u) / reg2.m_size) +fabsf(float(p1->v) / reg1.m_size - float(p2->v) / reg2.m_size) +fabsf(float(p1->w) / reg1.m_size - float(p2->w) / reg2.m_size)); 
		}
		while(i < NUM_BINS_XYZ) {
			sad += HIST_DEPTH_MOD * (fabsf(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + fabsf(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) +fabsf(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size));
			++i; ++p1; ++p2;
		}
		float min = (fabsf(reg1.m_centroid3D.x - reg2.m_centroid3D.x) +fabsf(reg1.m_centroid3D.y - reg2.m_centroid3D.y) +fabsf(reg1.m_centroid3D.z - reg2.m_centroid3D.z)) / (reg1.m_size + reg2.m_size / 2);
		float size_diff = float(abs(int(reg1.m_size) - int(reg2.m_size))) / (float(reg1.m_size + reg2.m_size) / 2);
		return sad + CENTROID_MUL * min + size_diff * SIZE_MUL;
	}
	return 1000000;
}


template<>
void RegionTree3D::TemporalCorrection(RegionTree3D &past, int level) {
	if(!m_propagated && !past.m_propagated) {
		map<Region3D,Region3D> currSeg, pastSeg;
		//Match based on the centroid of each region
		//Region3D **pCurr = m_nodes, **pPast;
		vector<Region3D*>::iterator pCurr = region_list.begin(), pPast;
		//FILE *fp;
		//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\stats.txt","a");
		for(int i = 0; i < m_size; i++) {
			//find the best match
			pPast = past.region_list.begin();
			float min;
			float hist_diff;
			min = fabsf((*pCurr)->m_centroid3D.x - (*pPast)->m_centroid3D.x) + fabsf((*pCurr)->m_centroid3D.y - (*pPast)->m_centroid3D.y) + fabsf((*pCurr)->m_centroid3D.z - (*pPast)->m_centroid3D.z);
			hist_diff = HistDifference(**pCurr,**pPast);
			Region3D minLoc;
			minLoc = **pPast;
			pPast++;
			for(int j = 0; j < past.m_size - 1; j++) {
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
		pPast = past.region_list.begin();
		for(int i = 0; i < past.m_size; i++) {
			//find the best match
			pCurr = region_list.begin();
			float hist_diff = HistDifference(**pCurr,**pPast);
			Region3D minLoc;
			minLoc = **pCurr;
			pCurr++;
			//find the 3 with the closest centroids
			for(int j = 0; j < m_size - 1; j++) {
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

		pCurr = region_list.begin();
		for(int i = 0; i < m_size; i++) {
			//bipartite matching with red-black trees
			if((*pCurr)->m_centroid3D.intensity == pastSeg[currSeg[**pCurr]].m_centroid3D.intensity) {
				//the min should be normalized by the size somehow
				SetBranch(this,*pCurr,(*pCurr)->m_level,currSeg[**pCurr].m_centroid3D.intensity);
			}
			++pCurr;
		}
		//fclose(fp);
	} 
}

void iMergeRegions(RegionTree4DBig *tree, Region4DBig &past, Region4DBig *curr) {
	if(curr != NULL && curr->m_size == 1 && past.m_size == 1) {
		//SetBranch(&(*pCurr),(pCurr)->m_level,currSeg[*pCurr].m_centroid3D.intensity);
		SetBranch(tree,curr,curr->m_level,past.m_centroid3D.intensity);
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
			min = fabsf((*pCurr)->m_centroid3D.x - (*pPast)->m_centroid3D.x) + fabsf((*pCurr)->m_centroid3D.y - (*pPast)->m_centroid3D.y) + fabsf((*pCurr)->m_centroid3D.z - (*pPast)->m_centroid3D.z);
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
			if((*pCurr)->m_centroid3D.intensity == pastSeg[currSeg[**pCurr]].m_centroid3D.intensity) {
				SetBranch(tree,*pCurr,(*pCurr)->m_level,currSeg[**pCurr].m_centroid3D.intensity);
				iMergeRegions(tree,currSeg[**pCurr], *pCurr);
			}
			++pCurr;
		}
	}
}

template<>
void RegionTree4DBig::TemporalCorrection(RegionTree4DBig &past, int level) {
	if(!m_propagated && !past.m_propagated) {
		map<Region4DBig,Region4DBig> currSeg, pastSeg;
		//Match based on the centroid of each region
		//Region3D **pCurr = m_nodes, **pPast;
		vector<Region4DBig*>::iterator pCurr = region_list.begin(), pPast;
		//FILE *fp;
		//fopen_s(&fp,"C:\\Users\\Steve\\Documents\\Data\\stats.txt","a");
		for(int i = 0; i < m_size; i++) {
			//find the best match
			pPast = past.region_list.begin();
			float min;
			float hist_diff;
			min = fabsf((*pCurr)->m_centroid3D.x - (*pPast)->m_centroid3D.x) + fabsf((*pCurr)->m_centroid3D.y - (*pPast)->m_centroid3D.y) + fabsf((*pCurr)->m_centroid3D.z - (*pPast)->m_centroid3D.z);
			hist_diff = HistDifference(**pCurr,**pPast);
			Region4DBig minLoc;
			minLoc = **pPast;
			pPast++;
			for(int j = 0; j < past.m_size - 1; j++) {
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
		pPast = past.region_list.begin();
		for(int i = 0; i < past.m_size; i++) {
			//find the best match
			pCurr = region_list.begin();
			float hist_diff = HistDifference(**pCurr,**pPast);
			Region4DBig minLoc;
			minLoc = **pCurr;
			pCurr++;
			//find the 3 with the closest centroids
			for(int j = 0; j < m_size - 1; j++) {
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

		pCurr = region_list.begin();
		for(int i = 0; i < m_size; i++) {
			//bipartite matching with red-black trees
			if((*pCurr)->m_centroid3D.intensity == pastSeg[currSeg[**pCurr]].m_centroid3D.intensity) {
				SetBranch(this,*pCurr,(*pCurr)->m_level,currSeg[**pCurr].m_centroid3D.intensity);
			}
			++pCurr;
		}
		//fclose(fp);
	} else {
		iMergeRegions(this,*past.m_nodes[0],m_nodes[0]);
	}
}

void UpdateTable(Region3D* child1, Region3D* child2, Region3D* father, vector<Region3D*> &lookup) {
	//for every value in the table, if it is = to child1 or = to child2, set it = to father
	vector<Region3D*>::iterator p = lookup.begin();
	while(p != lookup.end()) {
		if(*p == child1 || *p == child2)
			*p = father;
		p++;
	}
}

void UpdateTable(Region4DBig* child1, Region4DBig* child2, Region4DBig* father, vector<Region4DBig*> &lookup) {
	//for every value in the table, if it is = to child1 or = to child2, set it = to father
	vector<Region4DBig*>::iterator p = lookup.begin();
	while(p != lookup.end()) {
		if(*p == child1 || *p == child2)
			*p = father;
		p++;
	}
}


template<>
void RegionTree3D::PropagateRegionHierarchy(int min_size) {
	//lets start by making a handy label lookup table
	//find max label

	vector<Region3D*>::iterator p = region_list.begin();
	int num_edges = 0;
	while(p != region_list.end()) {
		num_edges += (*p)->m_neighbors.size();
		p++;
	}
	int i = 0;
	//printf("Max found as %d, size: %d, num_edges: %d\n",max,m_size,num_edges);
	//lets build the a new universe with edges between the regions and their neighbors with the index being the segment label
	Edge* edges = new Edge[num_edges]();
	p = region_list.begin();
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(p != region_list.end()) {
		vector<int>::const_iterator pNeighbors = (*p)->m_neighbors.begin(), pNeighborEnd = (*p)->m_neighbors.end();
		while(pNeighbors != pNeighborEnd) {
			//if(lookup[*pNeighbors] != NULL) {
			pEdge->a = i;
			pEdge->b = *pNeighbors;
			//pEdge->w = (float)HistDifference(**p,*(lookup[*pNeighbors]));// (float)(*p)->m_size;
			//pEdge->w = (float)HistDifference(**p,*(*p)->m_neighbor_map[*pNeighbors]);// (float)(*p)->m_size;
			//assert(region_list[*pNeighbors] != NULL);
			pEdge->w = (float)HistDifference(**p,*(region_list[*pNeighbors]));
			pEdge++;
			//}
			pNeighbors++;
		}
		i++;
		p++;
	}

	//printf("Graph built, went through %d Edges\n",i);
	//now that we are done building the graph, join it upwards recursively after sorting it
	sort(edges, edgesEnd);

	//now we should combine neighbors into higher level regions, this is tricky
	//for each edge, create a new region that points to the regions the edge connects, then recompute the new region statistics and point the lookup table to this new region
	pEdge = edges;
	i=0;
	int current = 0;
	while(pEdge != edgesEnd) {			
		Region3D *reg1 = region_list[pEdge->a], *reg2 = region_list[pEdge->b];
		if(reg1->m_centroid3D.intensity != reg2->m_centroid3D.intensity) {
			Region3D *father = new Region3D();
			father->InitializeRegion(reg1,reg2,current,min_size);
			//Instead of this, I should go through the whole table looking up the old pointer values and replace them with the new pointer values
			UpdateTable(reg1,reg2,father,region_list);
			//this is just in case, I probably don't need to do this
			region_list[pEdge->a] = father;
			region_list[pEdge->b] = father;
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
	pEdge--;
	m_nodes[0] = region_list[pEdge->a];
	m_propagated = true;
	//delete lookup; //somehow I must have already deleted this?
	delete[] edges;
}

template<>
void RegionTree4DBig::PropagateRegionHierarchy(int min_size) {
	//lets start by making a handy label lookup table
	//find max label

	vector<Region4DBig*>::iterator p = region_list.begin();
	int num_edges = 0;
	while(p != region_list.end()) {
		num_edges += (*p)->m_neighbors.size();
		p++;
	}
	int i = 0;
	//printf("Max found as %d, size: %d, num_edges: %d\n",max,m_size,num_edges);
	//lets build the a new universe with edges between the regions and their neighbors with the index being the segment label
	Edge* edges = new Edge[num_edges]();
	p = region_list.begin();
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(p != region_list.end()) {
		vector<int>::const_iterator pNeighbors = (*p)->m_neighbors.begin(), pNeighborEnd = (*p)->m_neighbors.end();
		while(pNeighbors != pNeighborEnd) {
			//if(lookup[*pNeighbors] != NULL) {
			pEdge->a = i;
			pEdge->b = *pNeighbors;
			//pEdge->w = (float)HistDifference(**p,*(lookup[*pNeighbors]));// (float)(*p)->m_size;
			//pEdge->w = (float)HistDifference(**p,*(*p)->m_neighbor_map[*pNeighbors]);// (float)(*p)->m_size;
			pEdge->w = (float)HistDifference(**p,*(region_list[*pNeighbors]));
			pEdge++;
			//}
			pNeighbors++;
		}
		i++;
		p++;
	}

	//printf("Graph built, went through %d Edges\n",i);
	//now that we are done building the graph, join it upwards recursively after sorting it
	sort(edges, edgesEnd);

	//now we should combine neighbors into higher level regions, this is tricky
	//for each edge, create a new region that points to the regions the edge connects, then recompute the new region statistics and point the lookup table to this new region
	pEdge = edges;
	i=0;
	int current = 0;
	while(pEdge != edgesEnd) {			
		Region4DBig *reg1 = region_list[pEdge->a], *reg2 = region_list[pEdge->b];
		if(reg1->m_centroid3D.intensity != reg2->m_centroid3D.intensity) {
			Region4DBig *father = new Region4DBig();
			father->InitializeRegion(reg1,reg2,current,min_size);
			//Instead of this, I should go through the whole table looking up the old pointer values and replace them with the new pointer values
			UpdateTable(reg1,reg2,father,region_list);
			//this is just in case, I probably don't need to do this
			region_list[pEdge->a] = father;
			region_list[pEdge->b] = father;
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
	m_nodes[0] = region_list[pEdge->a];
	m_propagated = true;
	//delete lookup; //somehow I must have already deleted this?
	delete[] edges;
}
