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
using namespace concurrency;

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

	//Observer. = 2°, Illuminant = D65
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
	if ( x > 0.008856f ) { x = blepo_ex::Pow( x , expo ); }
	else { x = ( 7.787f * x ) + ( expo2 ); }
	if ( y > 0.008856f ) { y = blepo_ex::Pow( y , expo ); }
	else { y = ( 7.787f * y ) + ( expo2 ); }
	if ( z > 0.008856f ) { z = blepo_ex::Pow( z , expo ); }
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

inline void Region3D::InitializeRegion(PointXYZI *in, Vec3b &color, const int label, const int i, const int j, const int level) {
	if(!_isnan(in->z)) {
		m_level  = level;
		m_size = 1;
		m_hist = new LABXYZ[NUM_BINS_XYZ]();
		LABXYZ labxyz = LABXYZ(color[2],color[1],color[0],in->x, in->y, in->z);
		m_hist[Clamp(Round(labxyz.l * HIST_MUL_L),0,NUM_BINS)].l++;
		m_hist[Clamp(Round(labxyz.a * HIST_MUL_A),0,NUM_BINS)].a++;
		m_hist[Clamp(Round(labxyz.b * HIST_MUL_B),0,NUM_BINS)].b++;
		m_hist[Clamp(Round(labxyz.x * HIST_MUL_X),0,NUM_BINS_XYZ)].x++;
		m_hist[Clamp(Round(labxyz.y * HIST_MUL_Y),0,NUM_BINS_XYZ)].y++;
		m_hist[Clamp(Round(labxyz.z * HIST_MUL_Z),0,NUM_BINS_XYZ)].z++;
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

inline void Region3D::AddNode(PointXYZI *in, Vec3b &color, const int i, const int j) {
	if(!_isnan(in->z)) {
		LABXYZ labxyz = LABXYZ(color[2],color[1],color[0],in->x,in->y,in->z);
		m_hist[Clamp(Round(labxyz.l * HIST_MUL_L), 0, NUM_BINS)].l++;
		m_hist[Clamp(Round(labxyz.a * HIST_MUL_A), 0, NUM_BINS)].a++;
		m_hist[Clamp(Round(labxyz.b * HIST_MUL_B), 0, NUM_BINS)].b++;
		m_hist[Clamp(Round(labxyz.x * HIST_MUL_X), 0, NUM_BINS_XYZ)].x++;
		m_hist[Clamp(Round(labxyz.y * HIST_MUL_Y), 0, NUM_BINS_XYZ)].y++;
		m_hist[Clamp(Round(labxyz.z * HIST_MUL_Z), 0, NUM_BINS_XYZ)].z++;
		m_size++;
		m_centroid.x += i;
		m_centroid.y += j;
		m_centroid3D.x += in->x;
		m_centroid3D.y += in->y;
		m_centroid3D.z += in->z;
		m_nodes.push_back(in);
	}
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

void RegionTree3D::Create(const PointCloudBgr &in, PointCloudInt &labels, int num_segments, int start_label) {
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
	int loc, label, i, j;
	const int safeWidth = in.width - 1, safeHeight = in.height - 1;
	Region3D* pRegion;
	//printf("Original num of Segments: %d\n",num_segments);
	numRegions = 0;
	totRegions = (num_segments + 2) << 1;
	//region_list = new Region3D[totRegions]();
	//delete region_list;
	region_list.resize(totRegions);
	//region_list = (Region3D*) malloc(totRegions * sizeof(Region3D));
	//printf("region_list 1: %p\n",region_list);
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(j = 0; j < in.height; j++) {
		for(i = 0; i < in.width; i++) {
			if(!_isnan(pIn->z)) {
				//for each voxel, add to the appropriate region and compute important values
				loc = lookup[int(pLabel->intensity)];
				//Am I a new region?
				if(loc == -1) {
					//I'm a new region, initialize
					lookup[int(pLabel->intensity)] = current;
					pRegion = &(region_list[numRegions]);
					//pRegion->InitializeRegion(pLabel, pIn->value, pLabel->value, i, j, 1);
					region_list[numRegions].InitializeRegion(pLabel._Ptr, Vec3b(pIn->b,pIn->g,pIn->r), current + start_label, i, j, 1);
					m_nodes[current] = pRegion;
					numRegions++;
					current++;
				}
			}
			pIn++; pLabel++;
		}
	}
	pIn = in.begin();
	pLabel = labels.begin();
	Region3D *tmp;
	for(j = 0; j < in.height; j++) {
		for(i = 0; i < in.width; i++) {
			if(!_isnan(pIn->z)) {
				//I'm not new, add me appropriately
				//Add node to appropriately region list
				loc = lookup[int(pLabel->intensity)];
				pRegion = &(region_list[loc]);
				region_list[loc].AddNode(pLabel._Ptr, Vec3b(pIn->b,pIn->g,pIn->r),i,j);
				//Check for neighbors
				if(i < safeWidth) {
					label = lookup[int((pLabel + 1)->intensity)];
					tmp = &(region_list[label]);
					if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
					if(j < safeHeight) {
						label = lookup[int((pLabel + in.width + 1)->intensity)];
						tmp = &(region_list[label]);
						if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
							pRegion->m_neighbors.push_back(label);
					}
					if(j > 0) {
						label = lookup[int((pLabel - in.width + 1)->intensity)];
						tmp = &(region_list[label]);
						if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
							pRegion->m_neighbors.push_back(label);
					}
				}
				if(j < safeHeight) {
					label = lookup[int((pLabel + in.width)->intensity)];
					tmp = &(region_list[label]);
					if(pLabel->intensity != label && find(pRegion->m_neighbors.begin(),pRegion->m_neighbors.end(),label) == pRegion->m_neighbors.end() && find(tmp->m_neighbors.begin(),tmp->m_neighbors.end(),pLabel->intensity) == tmp->m_neighbors.end())
						pRegion->m_neighbors.push_back(label);
				}
			}
			pIn++; pLabel++;
		}
	}
	//adjust means and centroids and such
	//pRegion = out->m_nodes[0];
	for(i = 0; i < num_segments; i++) {
		if(region_list[i].m_hist != NULL) {
			region_list[i].m_centroid.x /= pRegion->m_size;
			region_list[i].m_centroid.y /= pRegion->m_size;
			region_list[i].m_centroid3D.x /= pRegion->m_size;
			region_list[i].m_centroid3D.y /= pRegion->m_size;
			region_list[i].m_centroid3D.z /= pRegion->m_size;
		}
	}
	delete[] lookup;
}

void SetBranch(Region3D* region, int level, int label) {
	assert(region != NULL);
	if(label == -1 && region->m_level <= level) {
		//I should set the label
		label = region->m_centroid3D.intensity;
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
		region->m_centroid3D.intensity = label;
		vector<PointXYZI*>::iterator pNode = region->m_nodes.begin();
		while(pNode != region->m_nodes.end()) {
			(*pNode)->intensity = label;
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

//perhaps this is wrong, perhaps I should normalize somehow first, maybe by the size in order to get percentages?
inline float HistDifference(Region3D &reg1, Region3D &reg2) {
	if(reg1.m_hist != NULL && reg2.m_hist != NULL) {
		float sad = 0.0f;
		LABXYZ *p1 = reg1.m_hist, *p2 = reg2.m_hist;
		int i;
		for(i = 0; i < NUM_BINS; i++, p1++, p2++) {
			sad += fabsf(float(p1->a) / reg1.m_size - float(p2->a) / reg2.m_size) + fabsf(float(p1->b) / reg1.m_size - float(p2->b) / reg2.m_size) + fabsf(float(p1->l) / reg1.m_size - float(p2->l) / reg2.m_size) + HIST_DEPTH_MOD * (fabsf(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + fabsf(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + fabsf(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size)); 
		}
		while(i < NUM_BINS_XYZ) {
			sad += HIST_DEPTH_MOD * (fabsf(float(p1->x) / reg1.m_size - float(p2->x) / reg2.m_size) + fabsf(float(p1->y) / reg1.m_size - float(p2->y) / reg2.m_size) + fabsf(float(p1->z) / reg1.m_size - float(p2->z) / reg2.m_size));
			++i; ++p1; ++p2;
		}
		float min = (fabsf(reg1.m_centroid3D.x - reg2.m_centroid3D.x) + fabsf(reg1.m_centroid3D.y - reg2.m_centroid3D.y) + fabsf(reg1.m_centroid3D.z - reg2.m_centroid3D.z)) / (reg1.m_size + reg2.m_size / 2);
		float size_diff = float(fabsf(int(reg1.m_size) - int(reg2.m_size))) / (float(reg1.m_size + reg2.m_size) / 2);
		return sad + CENTROID_MUL * min + size_diff * SIZE_MUL;
	}
	return 1000000;
}

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
			min = fabsf((pCurr)->m_centroid3D.x - (pPast)->m_centroid3D.x) + fabsf((pCurr)->m_centroid3D.y - (pPast)->m_centroid3D.y) + fabsf((pCurr)->m_centroid3D.z - (pPast)->m_centroid3D.z);
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
			if(pCurr->m_centroid3D.intensity == pastSeg[currSeg[*pCurr]].m_centroid3D.intensity) {
				//the min should be normalized by the size somehow
				//float min = (absf((pCurr)->m_centroid3D.x - currSeg[*pCurr].m_centroid3D.x) + absf((pCurr)->m_centroid3D.y -currSeg[*pCurr].m_centroid3D.y) + absf((pCurr)->m_centroid3D.z - currSeg[*pCurr].m_centroid3D.z)) / pCurr->m_size;
				//int size_diff = absf(int((pCurr)->m_size) - int(currSeg[*pCurr].m_size));
				//float hist_diff = HistDifference(*pCurr,currSeg[*pCurr]);
				//if(hist_diff <= MIN_REGION_HIST && ((pCurr)->m_size < MAX_CONVERGING_SIZE || (size_diff <= (pCurr)->m_size * MIN_REGION_SIZE && min < MIN_REGION_DIST)))
				SetBranch(&(*pCurr),(pCurr)->m_level,currSeg[*pCurr].m_centroid3D.intensity);
			}
			++pCurr;
		}
		//fclose(fp);
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

void RegionTree3D::PropagateRegionHierarchy(int min_size) {
	//lets start by making a handy label lookup table
	Region3D** lookup;
	//find max label
	Region3D **p = m_nodes, **pEnd = m_nodes + m_size;
	int num_edges = 0;
	int max = (*p)->m_centroid3D.intensity;
	while(p != pEnd) {
		int label = (*p)->m_centroid3D.intensity;
		if(label > max)
			max = label;
		num_edges += (*p)->m_neighbors.size();
		p++;
	}
	lookup = new Region3D*[max]();
	int i = 0;
	p = m_nodes;
	while(p != pEnd) {
		lookup[int((*p)->m_centroid3D.intensity)] = *p;
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
			pEdge->a = (*p)->m_centroid3D.intensity;
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
	concurrency::parallel_sort(edges, edgesEnd);

	//now we should combine neighbors into higher level regions, this is tricky
	//for each edge, create a new region that points to the regions the edge connects, then recompute the new region statistics and point the lookup table to this new region
	pEdge = edges;
	i=0;
	int current = 0;
	//printf("Original: %d, ",numRegions);
	Region3D *father;
	while(pEdge != edgesEnd) {			
		Region3D *reg1 = lookup[pEdge->a], *reg2 = lookup[pEdge->b];
		if(reg1 != reg2 && reg1->m_centroid3D.intensity != reg2->m_centroid3D.intensity) {
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
