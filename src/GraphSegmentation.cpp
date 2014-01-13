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
//#include "TestVideoSegmentation.h"
#include <amp.h>
#include <amp_math.h>
#include "GraphSegmentation.h"
//#include "RegionTree.h"

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

// ================> begin local functions (available only to this translation unit)
using namespace blepo;
using namespace concurrency;

class Segments {
public:
	int m_loc, m_size;

	Segments() {
		m_loc = m_size = 0;
	}
	Segments(int loc, int size) {
		m_loc = loc;
		m_size = size;
	}
	bool operator==(int value) {
		return (value == m_loc);
	}
	bool operator!=(int value) {
		return (value != m_loc);
	}
};

#define WIDTH 4.0
#define THRESHOLD(size, c) (c/size)

template <class T>
inline T square(const T &x) { return x*x; }; 

/* make filters */
#define MAKE_FILTER(name, fun)                                \
	std::vector<float> make_ ## name (float sigma)       \
{                                                           \
	sigma = max(sigma, 0.01F);			                            \
	int len = (int)fast_math::ceil(sigma * WIDTH) + 1;                     \
	std::vector<float> mask(len);                               \
	for (int i = 0; i < len; i++)                               \
{                                                           \
	mask[i] = fun;                                              \
}                                                           \
	return mask;                                                \
}

MAKE_FILTER(fgauss, (float) fast_math::expf(-0.5*square(i/sigma)));

void normalize(std::vector<float> &mask)
{
	int len = mask.size();
	float sum = 0;
	int i;
	for (i = 1; i < len; i++) 
	{
		sum += fast_math::fabsf(mask[i]);
	}
	sum = 2*sum + fast_math::fabsf(mask[0]);
	for (i = 0; i < len; i++)
	{
		mask[i] /= sum;
	}
}

/* convolve src with mask.  dst is flipped! */
void convolve_even(ImgFloat& src, ImgFloat *dst, std::vector<float> &mask)
{
	int width = src.Width();
	int height = src.Height();
	int len = mask.size();

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			float sum = mask[0] * src(x, y);
			for (int i = 1; i < len; i++) {
				sum += mask[i] * (src(max(x-i,0), y) + src(min(x+i, width-1), y));
			}
			(*dst)(y, x) = sum;
		}
	}
}

inline float iDiff(const ImgFloat& r, const ImgFloat& g, const ImgFloat& b,
				   int x1, int y1, int x2, int y2) 
{
	return fast_math::sqrtf(square( r(x1,y1) - r(x2,y2) ) + 
		square( g(x1,y1) - g(x2,y2) ) + 
		square( b(x1,y1) - b(x2,y2) ) );
};

inline float iDiff(const Volume<float>& r, const Volume<float>& g, const Volume<float>& b,
				   int x1, int y1, int z1, int x2, int y2, int z2) 
{
	return fast_math::sqrtf(square( r(x1,y1,z1) - r(x2,y2,z2) ) + 
		square( g(x1,y1,z1) - g(x2,y2,z2) ) + 
		square( b(x1,y1,z1) - b(x2,y2,z2) ) );
};

inline float iDiffDepth(const ImgFloat& r, const ImgFloat& g, const ImgFloat& b, const ImgFloat& d,
						int x1, int y1, int x2, int y2) 
{
	return fast_math::sqrtf(square( r(x1,y1) - r(x2,y2) ) + 
		square( g(x1,y1) - g(x2,y2) ) + 
		square( b(x1,y1) - b(x2,y2) ) + 
		square( d(x1,y1) - d(x2,y2) ) );
};

inline float iDiffTime(const ImgFloat& r1, const ImgFloat& g1, const ImgFloat& b1,
					   int x1, int y1, const ImgFloat& r2, const ImgFloat& g2, const ImgFloat& b2, int x2, int y2) 
{

	return fast_math::sqrtf(square( r1(x1,y1) - r2(x2,y2) ) + 
		square( g1(x1,y1) - g2(x2,y2) ) + 
		square( b1(x1,y1) - b2(x2,y2) ) );

};

void iExtractRGBColorSpace(const ImgBgr& img, 
						   ImgFloat* B, 
						   ImgFloat* G,
						   ImgFloat* R)
{
	const int ISIZEX = img.Width(), ISIZEY = img.Height();
	const unsigned char *ptri = img.BytePtr();
	float *ptr1 = B->Begin();
	float *ptr2 = G->Begin();
	float *ptr3 = R->Begin();
	int b, g ,r;
	int i;

	for (i=0 ; i<ISIZEX*ISIZEY ; i++)
	{
		b = *ptri++;
		g = *ptri++;
		r = *ptri++;

		*ptr1++ = (float)b;
		*ptr2++ = (float)g;
		*ptr3++ = (float)r;
	}
}

void iExtractRGBColorSpace(const Volume<Bgr>& in, 
						   Volume<float>* B,
						   Volume<float>* G,
						   Volume<float>* R)
{
	B->Reset(in.Width(),in.Height(),in.Depth());
	G->Reset(in.Width(),in.Height(),in.Depth());
	R->Reset(in.Width(),in.Height(),in.Depth());
	Volume<Bgr>::ConstIterator pI = in.Begin();
	Volume<float>::Iterator pB = B->Begin(),pG = G->Begin(),pR = R->Begin();
	while(pI != in.End()) {
		*pB = (float)pI->b;
		*pG = (float)pI->g;
		*pR = (float)pI->r;
		pI++; pB++; pG++; pR++;
	}
}

void iExtractRGBDColorSpace(const Volume<ColoredPoint>& in, 
							Volume<float>* B,
							Volume<float>* G,
							Volume<float>* R,
							Volume<float>* D)
{
	B->Reset(in.Width(),in.Height(),in.Depth());
	G->Reset(in.Width(),in.Height(),in.Depth());
	R->Reset(in.Width(),in.Height(),in.Depth());
	D->Reset(in.Width(),in.Height(),in.Depth());
	Volume<ColoredPoint>::ConstIterator pI = in.Begin();
	Volume<float>::Iterator pB = B->Begin(),pG = G->Begin(),pR = R->Begin(),pD = D->Begin();
	while(pI != in.End()) {
		*pB = (float)pI->value.b;
		*pG = (float)pI->value.g;
		*pR = (float)pI->value.r;
		*pD = 1000.0f*pI->z;
		pI++; pB++; pG++; pR++, pD++;
	}
}

void iExtractRGBDColorSpace(const PointCloud<Bgr>& in, 
							ImgFloat* B,
							ImgFloat* G,
							ImgFloat* R,
							ImgFloat* D)
{
	B->Reset(in.Width(),in.Height());
	G->Reset(in.Width(),in.Height());
	R->Reset(in.Width(),in.Height());
	D->Reset(in.Width(),in.Height());
	PointCloud<Bgr>::ConstIterator pI = in.Begin();
	ImgFloat::Iterator pB = B->Begin(),pG = G->Begin(),pR = R->Begin(),pD = D->Begin();
	while(pI != in.End()) {
		*pB = (float)pI->value.b;
		*pG = (float)pI->value.g;
		*pR = (float)pI->value.r;
		*pD = 1000.0f*pI->z;
		pI++; pB++; pG++; pR++; pD++;
	}
}

void iExtractRGBDColorSpace(const PointCloud<Bgr>& in, 
							ImgFloat* B,
							ImgFloat* G,
							ImgFloat* R,
							ImgFloat* D,
							ImgBgr* img)
{
	B->Reset(in.Width(),in.Height());
	G->Reset(in.Width(),in.Height());
	R->Reset(in.Width(),in.Height());
	D->Reset(in.Width(),in.Height());
	img->Reset(in.Width(),in.Height());
	PointCloud<Bgr>::ConstIterator pI = in.Begin();
	ImgBgr::Iterator pC = img->Begin();
	ImgFloat::Iterator pB = B->Begin(),pG = G->Begin(),pR = R->Begin(),pD = D->Begin();
	while(pI != in.End()) {
		*pB = (float)pI->value.b;
		*pG = (float)pI->value.g;
		*pR = (float)pI->value.r;
		*pD = 1000.0f * pI->z;
		*pC = pI->value;
		pI++; pB++; pG++; pR++; pD++; pC++;
	}
}


void iSmooth(ImgFloat &src, float sigma, ImgFloat *out)
{
	std::vector<float> mask = make_fgauss(sigma);
	normalize(mask);
	ImgFloat tmp(src.Height(),src.Width());
	convolve_even(src, &tmp, mask);
	convolve_even(tmp, out, mask);
}

/*void ComputeOpticalFlow(PointCloud<Bgr> &past, PointCloud<Bgr> &current, FlowInfo *flow) {
ImgGray in1 = ImgGray(past.Width(),past.Height()),in2 = ImgGray(past.Width(),past.Height());
ImgFloat u,v,w;
PointCloud<Bgr>::ConstIterator pPast = past.Begin(), pCurr = current.Begin();
ImgGray::Iterator pi1 = in1.Begin(), pi2 = in2.Begin();
int tmp;
while(pi1 != in1.End()) {
tmp = (pPast->value.b + 6*pPast->value.g + 3*pPast->value.r) / 10;
*pi1 = blepo_ex::Min(blepo_ex::Max(tmp, static_cast<int>(ImgGray::MIN_VAL)), static_cast<int>(ImgGray::MAX_VAL));
tmp = (pCurr->value.b + 6*pCurr->value.g + 3*pCurr->value.r) / 10;
*pi2 = blepo_ex::Min(blepo_ex::Max(tmp, static_cast<int>(ImgGray::MIN_VAL)), static_cast<int>(ImgGray::MAX_VAL));
pi1++; pi2++; pPast++; pCurr++;
}
OpticalFlowFarneback(in2,in1,0.5f,2,5,2,7,1.5,&u,&v);
flow->Reset(past.Width(),past.Height());
Image<Flow>::Iterator pO = flow->Begin();
ImgFloat::ConstIterator pU = u.Begin(), pV = v.Begin();
//pPast = past.Begin();
pCurr = current.Begin();
int safeWidth = past.Width() - 1, safeHeight = past.Height() - 1;
for(int j = 0; j < past.Height(); j++) {
for(int i = 0; i < past.Width(); i++) {
pO->u = *pU; //* pCurr->z * KINECT_FX_D;
pO->v = *pV; //* pCurr->z * KINECT_FX_D;
pO->w = (pCurr->z - past(blepo_ex::Clamp(int(i - pO->u),0,safeWidth), blepo_ex::Clamp(int(j - pO->v),0,safeHeight)).z);
pO++; pCurr++; pU++; pV++;
}
}
}*/

void iBuildGraph(const ImgFloat& smooth_r, 
				 const ImgFloat& smooth_g,
				 const ImgFloat& smooth_b,
				 std::vector<Edge> *edges,
				 int *num_edges)
{
	int width = smooth_r.Width();
	int height = smooth_r.Height();
	int num = 0;
	int x, y;
	edges->clear();
	edges->reserve(width*height*8);
	for ( y = 0; y < height; y++)
	{
		for ( x = 0; x < width; x++)
		{
			if (x < width-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = y * width + (x+1);
				edge.w = iDiff(smooth_r, smooth_g, smooth_b, x, y, x+1, y);
				edges->push_back(edge);
				num++;
			}

			if (y < height-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + x;
				edge.w = iDiff(smooth_r, smooth_g, smooth_b, x, y, x, y+1);
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y < height-1)) 
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + (x+1);
				edge.w = iDiff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y > 0))
			{
				Edge edge;
				edge.a  = y * width + x;
				edge.b  = (y-1) * width + (x+1);
				edge.w  = iDiff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
				edges->push_back(edge);
				num++;
			}
		}
	}
	*num_edges = num;
}

void iBuildGraph(const ImgFloat& smooth_r, 
				 const ImgFloat& smooth_g,
				 const ImgFloat& smooth_b,
				 const ImgFloat& smooth_d,
				 float alpha,
				 std::vector<Edge> *edges,
				 int *num_edges)
{
	int width = smooth_r.Width();
	int height = smooth_r.Height();
	int num = 0;
	int x, y;
	edges->clear();
	edges->reserve(width*height*8);
	ImgFloat::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
	for ( y = 0; y < height; y++)
	{
		for ( x = 0; x < width; x++)
		{
			if (x < width-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = y * width + (x+1);
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*0.5f*square(*pD - smooth_d(edge.b)));
				edges->push_back(edge);
				num++;
			}

			if (y < height-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + x;
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*0.5f*square(*pD - smooth_d(edge.b)));
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y < height-1)) 
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + (x+1);
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*0.5f*square(*pD - smooth_d(edge.b)));
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y > 0))
			{
				Edge edge;
				edge.a  = y * width + x;
				edge.b  = (y-1) * width + (x+1);
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*0.5f*square(*pD - smooth_d(edge.b)));
				edges->push_back(edge);
				num++;
			}
			pR++; pG++; pB++; pD++;
		}
	}
	*num_edges = num;
}

void iBuildGraph(PointCloud<Bgr> &in,
				 float sigma_depth,
				 float sigma_color,
				 Edge3D *&edges,
				 int *num_edges)
{
	int width = in.Width();
	int height = in.Height();
	int num = 0;
	int x, y, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = in.size()*8;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	ImgFloat R,G,B,D, smooth_r(width, height), smooth_g(width, height), smooth_b(width, height), smooth_d(width, height);
	iExtractRGBDColorSpace(in, &B, &G, &R, &D);
	iSmooth(B, sigma_color, &smooth_b);
	iSmooth(G, sigma_color, &smooth_g);
	iSmooth(R, sigma_color, &smooth_r);
	iSmooth(D, sigma_depth, &smooth_d);

	//Normalize
	//PointCloud<Bgr> norm;
	//Normalize(in,&norm);

	Edge3D *p = edges;
	ImgFloat::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
	//PointCloud<Bgr>::ConstIterator pC = in.Begin();
	for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
	{
		for ( x = 0, xp = 1; x < width; x++, xp++)
		{
			if (x < safeWidth)
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = y * width + xp;
				edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
				edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
				*p++ = edge;
				num++;
			}
			if (y < safeHeight)
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = yp * width + x;
				edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
				edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y < safeHeight)) 
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = yp * width + xp;
				edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
				edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y > 0))
			{
				Edge3D edge;
				edge.a  = y * width + x;
				edge.b  = ym * width + xp;
				edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
				edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
				*p++ = edge;
				num++;
			}
			pR++; pG++; pB++; pD++;
		}
	}
	R.Reset();
	G.Reset();
	B.Reset();
	D.Reset();
	smooth_b.Reset();
	smooth_g.Reset();
	smooth_r.Reset();
	smooth_d.Reset();
	*num_edges = num;
}

void gpuiBuildGraph(PointCloud<Bgr> &in,
					float sigma_depth,
					float sigma_color,
					Edge3D *&edges,
					int *num_edges)
{
	int width = in.Width();
	int height = in.Height();
	int widthMinus = width - 1, heightMinus = height - 1, wh = width*height;
	int num = 0;
	int reserve_size = in.size()*8;
	printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	array_view<Edge3D, 1> gpu_Edges(reserve_size, edges);
	array_view<int, 1> gpu_num(1,&num);
	//printf("Building graph, t = %d\n",t);
	ImgFloat R,G,B,D, smooth_r(width, height), smooth_g(width, height), smooth_b(width, height), smooth_d(width, height);
	iExtractRGBDColorSpace(in, &B, &G, &R, &D);
	iSmooth(B, sigma_color, &smooth_b);
	iSmooth(G, sigma_color, &smooth_g);
	iSmooth(R, sigma_color, &smooth_r);
	iSmooth(D, sigma_depth, &smooth_d);
	array_view<float, 2> gpu_R(height, width, smooth_r);
	array_view<float, 2> gpu_G(height, width, smooth_g);
	array_view<float, 2> gpu_B(height, width, smooth_b);
	array_view<float, 2> gpu_D(height, width, smooth_d);

	parallel_for_each( 
		// Define the compute domain, which is the set of threads that are created.
		gpu_R.extent,
		// Define the code to run on each thread on the accelerator.
		[=](index<2> idx) restrict(amp)
	{
		int x = idx[1], y = idx[0];
		if (x < widthMinus)
		{
			Edge3D edge;
			edge.a = y * width + x;
			edge.b = y * width + (x+1);
			float r = gpu_R(x,y) - gpu_R(x+1,y), g = gpu_G(x,y) - gpu_G(x+1,y), b = gpu_B(x,y) - gpu_B(x+1,y);
			edge.w = fast_math::fabs(gpu_D(x,y) - gpu_D(x+1,y));
			edge.w2 = fast_math::sqrtf(r*r + g*g + b*b);
			gpu_Edges[gpu_num[0]] = edge;
			gpu_num[0]++;
		}

		if (y < heightMinus)
		{
			Edge3D edge;
			edge.a = y * width + x;
			edge.b = (y+1) * width + x;
			//edge.w = iDiff(smooth_R, smooth_G, smooth_B, x, y, x, y+1);
			float r = gpu_R(x,y) - gpu_R(x,y+1), g = gpu_G(x,y) - gpu_G(x,y+1), b = gpu_B(x,y) - gpu_B(x,y+1);
			edge.w = fast_math::fabs(gpu_D(x,y) - gpu_D(x,y+1));
			edge.w2 = fast_math::sqrtf(r*r + g*g + b*b);
			gpu_Edges[gpu_num[0]] = edge;
			gpu_num[0]++;
		}

		if ((x < widthMinus) && (y < heightMinus)) 
		{
			Edge3D edge;
			edge.a = y * width + x;
			edge.b = (y+1) * width + (x+1);
			float r = gpu_R(x,y) - gpu_R(x+1,y+1), g = gpu_G(x,y) - gpu_G(x+1,y+1), b = gpu_B(x,y) - gpu_B(x+1,y+1);
			edge.w = fast_math::fabs(gpu_D(x,y) - gpu_D(x+1,y+1));
			edge.w2 = fast_math::sqrtf(r*r + g*g + b*b);
			gpu_Edges[gpu_num[0]] = edge;
			gpu_num[0]++;
		}

		if ((x < widthMinus) && (y > 0))
		{
			Edge3D edge;
			edge.a  = y * width + x;
			edge.b  =(y-1) * width + (x+1);
			float r = gpu_R(x,y) - gpu_R(x+1,y-1), g = gpu_G(x,y) - gpu_G(x+1,y-1), b = gpu_B(x,y) - gpu_B(x+1,y-1);
			edge.w = fast_math::fabs(gpu_D(x,y) - gpu_D(x+1,y-1));
			edge.w2 = fast_math::sqrtf(r*r + g*g + b*b);
			gpu_Edges[gpu_num[0]] = edge;
			gpu_num[0]++;
		}
	}
	);

	gpu_Edges.synchronize();
	gpu_num.synchronize();
	R.Reset();
	G.Reset();
	B.Reset();
	D.Reset();
	smooth_b.Reset();
	smooth_g.Reset();
	smooth_r.Reset();
	smooth_d.Reset();
	*num_edges = num;
}

void iBuildGraphBad(PointCloud<Bgr> &in,
					float sigma_color,
					float sigma_depth,
					float alpha,
					Edge *&edges,
					int *num_edges)
{
	int width = in.Width();
	int height = in.Height();
	int num = 0;
	int x, y, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = in.size()*8;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge*) malloc(reserve_size*sizeof(Edge));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	ImgFloat R,G,B,D, smooth_r(width, height), smooth_g(width, height), smooth_b(width, height), smooth_d(width, height);
	iExtractRGBDColorSpace(in, &B, &G, &R, &D);
	iSmooth(B, sigma_color, &smooth_b);
	iSmooth(G, sigma_color, &smooth_g);
	iSmooth(R, sigma_color, &smooth_r);
	iSmooth(D, sigma_depth, &smooth_d);

	//Normalize
	PointCloud<Bgr> norm;
	Normalize(in,&norm);

	Edge *p = edges;
	ImgFloat::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
	//PointCloud<Bgr>::ConstIterator pC = in.Begin();
	for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
	{
		for ( x = 0, xp = 1; x < width; x++, xp++)
		{
			if (x < safeWidth)
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = y * width + xp;
				//edge.w = blepo_ex::Max(blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))),alpha* ( *pD - smooth_d(edge.b)));
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*100.0f*square(*pD - smooth_d(edge.b)));
				//edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+square(*pD - smooth_d(edge.b)));
				*p++ = edge;
				num++;
			}
			if (y < safeHeight)
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = yp * width + x;
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*100.0f*square(*pD - smooth_d(edge.b)));
				//edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y < safeHeight)) 
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = yp * width + xp;
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*100.0f*square(*pD - smooth_d(edge.b)));
				//edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y > 0))
			{
				Edge3D edge;
				edge.a  = y * width + x;
				edge.b  = ym * width + xp;
				edge.w = (1.0f-alpha)*blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))+alpha*100.0f*square(*pD - smooth_d(edge.b)));
				//edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
				*p++ = edge;
				num++;
			}
			pR++; pG++; pB++; pD++;
		}
	}
	*num_edges = num;
}

inline void AddPriorKnowledge(Universe &past_D, Universe &past_C, int iter, Edge3D *edge) {
	if(past_D.num_sets() != 0 && past_D.find(edge->a + iter) == past_D.find(edge->b + iter))
		edge->w *= PRIOR_MUL;
	if(past_C.num_sets() != 0 && past_C.find(edge->a + iter) == past_C.find(edge->b + iter))
		edge->w2 *= PRIOR_MUL;
}

inline void AddPriorKnowledge(map<int, map<int, bool>> &past_D, map<int, map<int, bool>> &past_C, int iter, Edge3D *edge) {
	try{
		if(past_D.size() != 0 && past_D.at(edge->a + iter).at(edge->b + iter))
			edge->w *= PRIOR_MUL;
		if(past_C.size() != 0 && past_C.at(edge->a + iter).at(edge->b + iter))
			edge->w2 *= PRIOR_MUL;
	} catch (const std::out_of_range &e) {
		return;
	}
}

void iBuildGraph(const deque<PointCloud<Bgr>> &clouds,
	map<int, map<int, bool>> &past_D,
	map<int, map<int, bool>> &past_C,
	float sigma_depth,
	float sigma_color,
	Edge3D *&edges,
	int *num_edges)
{
	int width = clouds[0].Width();
	int height = clouds[0].Height();
	int num = 0;
	int x, y, z, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = NUM_FRAMES*clouds[0].size()*9;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}

	//Normalize
	//PointCloud<Bgr> norm;
	//Normalize(in,&norm);

	Edge3D *p = edges;
	//PointCloud<Bgr>::ConstIterator pC = in.Begin();
	ImgFloat past_r,past_g,past_b,past_d;
	for( z = 0; z < NUM_FRAMES; z++) {
		ImgFloat R,G,B,D, smooth_r(width, height), smooth_g(width, height), smooth_b(width, height), smooth_d(width, height);
		ImgBgr currImg;
		iExtractRGBDColorSpace(clouds[z], &B, &G, &R, &D, &currImg);
		iSmooth(B, sigma_color, &smooth_b);
		iSmooth(G, sigma_color, &smooth_g);
		iSmooth(R, sigma_color, &smooth_r);
		iSmooth(D, sigma_depth, &smooth_d);
		ImgFloat::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
		int currDepthIter = z * height * width, pastDepthIter = (z - 1) * height * width, priorDepthIter = NUM_FRAMES / 2 * height * width;
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				if (x < safeWidth)
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = y * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					if(z < NUM_FRAMES / 2)
						AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
					*p++ = edge;
					num++;
				}
				if (y < safeHeight)
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = yp * width + x;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					if(z < NUM_FRAMES / 2)
						AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y < safeHeight)) 
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = yp * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					if(z < NUM_FRAMES / 2)
						AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y > 0))
				{
					Edge3D edge;
					edge.a  = y * width + x;
					edge.b  = ym * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					if(z < NUM_FRAMES / 2)
						AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
					*p++ = edge;
					num++;
				}
				if(z > 0) {
					//do the same for the points in the past
					//I need to change it so that these use optical flow
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = y * width + x;
					edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += pastDepthIter;
					if(z < NUM_FRAMES / 2)
						AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = y * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						if(z < NUM_FRAMES / 2)
							AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
						*p++ = edge;
						num++;
					}
					if (y < safeHeight)
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = yp * width + x;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						if(z < NUM_FRAMES / 2)
							AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y < safeHeight)) 
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = yp * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						if(z < NUM_FRAMES / 2)
							AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y > 0))
					{
						Edge3D edge;
						edge.a  = y * width + x;
						edge.b  = ym * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						if(z < NUM_FRAMES / 2)
							AddPriorKnowledge(past_D,past_C,pastDepthIter,&edge);
						*p++ = edge;
						num++;
					}
				}
				pR++; pG++; pB++; pD++;
			}
		}
		past_r = smooth_r;
		past_g = smooth_g;
		past_b = smooth_b;
		past_d = smooth_d;
	}
	*num_edges = num;
}

void iNormalize(ImgFloat &depth) {
	ImgFloat::Iterator p = depth.Begin();
	while(p != depth.End()) {
		*p = *p * 255 / 6;
		++p;
	}
}

void iBuildGraph(const deque<PointCloud<Bgr>> &clouds,
	float sigma_depth,
	float sigma_color,
	Edge3D *&edges,
	int *num_edges)
{
	int width = clouds[0].Width();
	int height = clouds[0].Height();
	int num = 0;
	int x, y, z, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = NUM_FRAMES*clouds[0].size()*9;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}

	//Normalize
	//PointCloud<Bgr> norm;
	//Normalize(in,&norm);

	Edge3D *p = edges;
	//PointCloud<Bgr>::ConstIterator pC = in.Begin();
	ImgFloat past_r,past_g,past_b,past_d;
	for( z = 0; z < NUM_FRAMES; z++) {
		ImgFloat R,G,B,D, smooth_r(width, height), smooth_g(width, height), smooth_b(width, height), smooth_d(width, height);
		ImgBgr currImg;
		iExtractRGBDColorSpace(clouds[z], &B, &G, &R, &D, &currImg);
		//iNormalize(D);
		iSmooth(B, sigma_color, &smooth_b);
		iSmooth(G, sigma_color, &smooth_g);
		iSmooth(R, sigma_color, &smooth_r);
		if(sigma_depth == 0)
			smooth_d = D;
		else
			iSmooth(D, sigma_depth, &smooth_d);
		ImgFloat::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
		int currDepthIter = z * height * width, pastDepthIter = (z - 1) * height * width, priorDepthIter = NUM_FRAMES / 2 * height * width;
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				if (x < safeWidth)
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = y * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if (y < safeHeight)
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = yp * width + x;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y < safeHeight)) 
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = yp * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y > 0))
				{
					Edge3D edge;
					edge.a  = y * width + x;
					edge.b  = ym * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if(z > 0) {
					//do the same for the points in the past
					//I need to change it so that these use optical flow
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = y * width + x;
					edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += pastDepthIter;
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = y * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
					if (y < safeHeight)
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = yp * width + x;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y < safeHeight)) 
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = yp * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y > 0))
					{
						Edge3D edge;
						edge.a  = y * width + x;
						edge.b  = ym * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
				}
				pR++; pG++; pB++; pD++;
			}
		}
		past_r = smooth_r;
		past_g = smooth_g;
		past_b = smooth_b;
		past_d = smooth_d;
	}
	*num_edges = num;
}


void iBuildGraph(const deque<PointCloud<Bgr>> &clouds,
	float sigma_depth,
	float sigma_color,
	float alpha,
	Edge *&edges,
	int *num_edges)
{
	int width = clouds[0].Width();
	int height = clouds[0].Height();
	int num = 0;
	int x, y, z, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = NUM_FRAMES*clouds[0].size()*9;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}

	//Normalize
	//PointCloud<Bgr> norm;
	//Normalize(in,&norm);

	Edge *p = edges;
	//PointCloud<Bgr>::ConstIterator pC = in.Begin();
	ImgFloat past_r,past_g,past_b,past_d;
	for( z = 0; z < NUM_FRAMES; z++) {
		ImgFloat R,G,B,D, smooth_r(width, height), smooth_g(width, height), smooth_b(width, height), smooth_d(width, height);
		ImgBgr currImg;
		iExtractRGBDColorSpace(clouds[z], &B, &G, &R, &D, &currImg);
		iSmooth(B, sigma_color, &smooth_b);
		iSmooth(G, sigma_color, &smooth_g);
		iSmooth(R, sigma_color, &smooth_r);
		iSmooth(D, sigma_depth, &smooth_d);
		ImgFloat::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
		int currDepthIter = z * height * width, pastDepthIter = (z - 1) * height * width;
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				if (x < safeWidth)
				{
					Edge edge;
					edge.a = y * width + x;
					edge.b = y * width + xp;
					edge.w =  alpha * blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if (y < safeHeight)
				{
					Edge edge;
					edge.a = y * width + x;
					edge.b = yp * width + x;
					edge.w =  alpha * blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y < safeHeight)) 
				{
					Edge edge;
					edge.a = y * width + x;
					edge.b = yp * width + xp;
					edge.w =  alpha * blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y > 0))
				{
					Edge edge;
					edge.a  = y * width + x;
					edge.b  = ym * width + xp;
					edge.w =  alpha * blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.a += currDepthIter;
					edge.b += currDepthIter;
					*p++ = edge;
					num++;
				}
				if(z > 0) {
					//do the same for the points in the past
					//I need to change it so that these use optical flow
					Edge edge;
					edge.a = y * width + x;
					edge.b = y * width + x;
					edge.w =  alpha * blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - past_d(edge.b));
					edge.a += currDepthIter;
					edge.b += pastDepthIter;
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge edge;
						edge.a = y * width + x;
						edge.b = y * width + xp;
						edge.w =  alpha * blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - past_d(edge.b));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
					if (y < safeHeight)
					{
						Edge edge;
						edge.a = y * width + x;
						edge.b = yp * width + x;
						edge.w =  alpha * blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - past_d(edge.b));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y < safeHeight)) 
					{
						Edge edge;
						edge.a = y * width + x;
						edge.b = yp * width + xp;
						edge.w =  alpha * blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - past_d(edge.b));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y > 0))
					{
						Edge edge;
						edge.a  = y * width + x;
						edge.b  = ym * width + xp;
						edge.w =  alpha * blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b))) + (1 - alpha) * blepo_ex::Abs(*pD - past_d(edge.b));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
				}
				pR++; pG++; pB++; pD++;
			}
		}
		past_r = smooth_r;
		past_g = smooth_g;
		past_b = smooth_b;
		past_d = smooth_d;
	}
	*num_edges = num;
}

void iBuildGraph(PointCloud<Bgr> &in,
				 PointCloud<Bgr> &past,
				 float sigma_depth,
				 float sigma_color,
				 FlowInfo *flow,
				 Edge3D *&edges,
				 int *num_edges)
{
	int width = in.Width();
	int height = in.Height();
	int num = 0;
	int x, y, z, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = 2*in.size()*9;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}

	//Normalize
	//PointCloud<Bgr> norm;
	//Normalize(in,&norm);

	Edge3D *p = edges;
	//PointCloud<Bgr>::ConstIterator pC = in.Begin();
	ComputeOpticalFlow(past,in,flow);
	ImgFloat past_r,past_g,past_b,past_d;
	for( z = 0; z < 2; z++) {
		ImgFloat R,G,B,D, smooth_r(width, height), smooth_g(width, height), smooth_b(width, height), smooth_d(width, height);
		ImgBgr currImg;
		if(z == 0)
			iExtractRGBDColorSpace(past, &B, &G, &R, &D, &currImg);
		else
			iExtractRGBDColorSpace(in, &B, &G, &R, &D, &currImg);
		iSmooth(B, sigma_color, &smooth_b);
		iSmooth(G, sigma_color, &smooth_g);
		iSmooth(R, sigma_color, &smooth_r);
		iSmooth(D, sigma_depth, &smooth_d);
		ImgFloat::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				if (x < safeWidth)
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = y * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += z * height * width;
					edge.b += z * height * width;
					*p++ = edge;
					num++;
				}
				if (y < safeHeight)
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = yp * width + x;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += z * height * width;
					edge.b += z * height * width;
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y < safeHeight)) 
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = yp * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += z * height * width;
					edge.b += z * height * width;
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y > 0))
				{
					Edge3D edge;
					edge.a  = y * width + x;
					edge.b  = ym * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					edge.a += z * height * width;
					edge.b += z * height * width;
					*p++ = edge;
					num++;
				}
				if(z == 1) {
					//do the same for the points in the past
					//I need to change it so that these use optical flow
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = y * width + x;
					edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
					edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
					edge.a += z * height * width;
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = y * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += z * height * width;
						*p++ = edge;
						num++;
					}
					if (y < safeHeight)
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = yp * width + x;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += z * height * width;
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y < safeHeight)) 
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = yp * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += z * height * width;
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y > 0))
					{
						Edge3D edge;
						edge.a  = y * width + x;
						edge.b  = ym * width + xp;
						edge.w = blepo_ex::Abs(*pD - past_d(edge.b));
						edge.w2 =  blepo_ex::Sqrt(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += z * height * width;
						*p++ = edge;
						num++;
					}
				}
				pR++; pG++; pB++; pD++;
			}
		}
		past_r = smooth_r;
		past_g = smooth_g;
		past_b = smooth_b;
		past_d = smooth_d;
	}
	*num_edges = num;
}

void iBuildDepthGraph(const ImgFloat& smooth_r, 
					  const ImgFloat& smooth_g,
					  const ImgFloat& smooth_b,
					  const ImgFloat& smooth_d, 
					  std::vector<Edge> *edges,
					  int *num_edges)
{
	int width = smooth_r.Width();
	int height = smooth_r.Height();
	int num = 0;
	int x, y;
	edges->clear();
	edges->reserve(width*height*8);
	for ( y = 0; y < height; y++)
	{
		for ( x = 0; x < width; x++)
		{
			if (x < width-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = y * width + (x+1);
				edge.w = iDiffDepth(smooth_r, smooth_g, smooth_b, smooth_d, x, y, x+1, y);
				edges->push_back(edge);
				num++;
			}

			if (y < height-1)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + x;
				edge.w = iDiffDepth(smooth_r, smooth_g, smooth_b, smooth_d, x, y, x, y+1);
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y < height-1)) 
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = (y+1) * width + (x+1);
				edge.w = iDiffDepth(smooth_r, smooth_g, smooth_b, smooth_d, x, y, x+1, y+1);
				edges->push_back(edge);
				num++;
			}

			if ((x < width-1) && (y > 0))
			{
				Edge edge;
				edge.a  = y * width + x;
				edge.b  = (y-1) * width + (x+1);
				edge.w  = iDiffDepth(smooth_r, smooth_g, smooth_b, smooth_d, x, y, x+1, y-1);
				edges->push_back(edge);
				num++;
			}
		}
	}
	*num_edges = num;
}

void gpuiBuildGraphVideo(vector<ImgBgr> &imgs,
						 float sigma,
						 Edge *&edges,
						 int *num_edges)
{
	int width = imgs[0].Width();
	int height = imgs[0].Height();
	int widthMinus = width - 1, heightMinus = height - 1, timeMinus = imgs.size() - 1, wh = width*height;
	int num = 0;
	int t;
	ImgFloat smooth_R(width, height),smooth_G(width, height),smooth_B(width, height),last_R(width, height),last_G(width, height),last_B(width, height);
	int reserve_size = wh*imgs.size()*9;
	printf("Reserve size = %d\n",reserve_size);
	edges = (Edge*) malloc(reserve_size*sizeof(Edge));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	array_view<Edge, 1> gpu_Edges(reserve_size, edges);
	array_view<int, 1> gpu_num(1,&num);
	for( t = 0; t < imgs.size(); t++) {
		//printf("Building graph, t = %d\n",t);
		ImgFloat R(width, height),G(width, height),B(width, height);
		iExtractRGBColorSpace(imgs[t], &B, &G, &R);
		iSmooth(B, sigma, &smooth_B);
		iSmooth(G, sigma, &smooth_G);
		iSmooth(R, sigma, &smooth_R);
		imgs[t].Reset();
		array_view<float, 2> gpu_R(height, width, smooth_R);
		array_view<float, 2> gpu_G(height, width, smooth_G);
		array_view<float, 2> gpu_B(height, width, smooth_B);
		array_view<float, 2> gpu_lastR(height, width, last_R);
		array_view<float, 2> gpu_lastG(height, width, last_G);
		array_view<float, 2> gpu_lastB(height, width, last_B);

		parallel_for_each( 
			// Define the compute domain, which is the set of threads that are created.
			gpu_R.extent,
			// Define the code to run on each thread on the accelerator.
			[=](index<2> idx) restrict(amp)
		{
			int x = idx[1], y = idx[0];
			if(t > 0) {
				Edge edge;
				edge.a = wh*t + y * width + x;
				edge.b = wh*(t-1) + y * width + x;
				float r = gpu_R(x,y) - gpu_lastR(x,y), g = gpu_G(x,y) - gpu_lastG(x,y), b = gpu_B(x,y) - gpu_lastB(x,y);
				edge.w = fast_math::sqrtf(r*r + g*g + b*b);
				//edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x, y);
				gpu_Edges[gpu_num[0]] = edge;
				gpu_num[0]++;
			}
			if (x < widthMinus)
			{
				Edge edge;
				edge.a = wh*t + y * width + x;
				edge.b = wh*t + y * width + (x+1);
				//edge.w = iDiff(smooth_R, smooth_G, smooth_B, x, y, x+1, y);
				float r = gpu_R(x,y) - gpu_R(x+1,y), g = gpu_G(x,y) - gpu_G(x+1,y), b = gpu_B(x,y) - gpu_B(x+1,y);
				edge.w = fast_math::sqrtf(r*r + g*g + b*b);
				gpu_Edges[gpu_num[0]] = edge;
				gpu_num[0]++;
			}

			if (y < heightMinus)
			{
				Edge edge;
				edge.a = wh*t +  y * width + x;
				edge.b = wh*t + (y+1) * width + x;
				//edge.w = iDiff(smooth_R, smooth_G, smooth_B, x, y, x, y+1);
				float r = gpu_R(x,y) - gpu_R(x,y+1), g = gpu_G(x,y) - gpu_G(x,y+1), b = gpu_B(x,y) - gpu_B(x,y+1);
				edge.w = fast_math::sqrtf(r*r + g*g + b*b);
				gpu_Edges[gpu_num[0]] = edge;
				gpu_num[0]++;
			}

			if ((x < widthMinus) && (y < heightMinus)) 
			{
				Edge edge;
				edge.a = wh*t + y * width + x;
				edge.b = wh*t + (y+1) * width + (x+1);
				//edge.w = iDiff(smooth_R, smooth_G, smooth_B, x, y, x+1, y+1);
				float r = gpu_R(x,y) - gpu_R(x+1,y+1), g = gpu_G(x,y) - gpu_G(x+1,y+1), b = gpu_B(x,y) - gpu_B(x+1,y+1);
				edge.w = fast_math::sqrtf(r*r + g*g + b*b);
				gpu_Edges[gpu_num[0]] = edge;
				gpu_num[0]++;
			}

			if ((x < widthMinus) && (y > 0))
			{
				Edge edge;
				edge.a  = wh*t + y * width + x;
				edge.b  = wh*t + (y-1) * width + (x+1);
				//edge.w  = iDiff(smooth_R, smooth_G, smooth_B, x, y, x+1, y-1);
				float r = gpu_R(x,y) - gpu_R(x+1,y-1), g = gpu_G(x,y) - gpu_G(x+1,y-1), b = gpu_B(x,y) - gpu_B(x+1,y-1);
				edge.w = fast_math::sqrtf(r*r + g*g + b*b);
				gpu_Edges[gpu_num[0]] = edge;
				gpu_num[0]++;
			}
			int d = t - 1;
			if(d >= 0) {
				if (x < widthMinus)
				{
					Edge edge;
					edge.a = wh*t + y * width + x;
					edge.b = wh*d + y * width + (x+1);
					//edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x+1, y);
					float r = gpu_R(x,y) - gpu_lastR(x+1,y), g = gpu_G(x,y) - gpu_lastG(x+1,y), b = gpu_B(x,y) - gpu_lastB(x+1,y);
					edge.w = fast_math::sqrtf(r*r + g*g + b*b);
					gpu_Edges[gpu_num[0]] = edge;
					gpu_num[0]++;
				}

				if (y < heightMinus)
				{
					Edge edge;
					edge.a = wh*t +  y * width + x;
					edge.b = wh*d + (y+1) * width + x;
					//edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x, y+1);
					float r = gpu_R(x,y) - gpu_lastR(x,y+1), g = gpu_G(x,y) - gpu_lastG(x,y+1), b = gpu_B(x,y) - gpu_lastB(x,y+1);
					edge.w = fast_math::sqrtf(r*r + g*g + b*b);
					gpu_Edges[gpu_num[0]] = edge;
					gpu_num[0]++;
				}

				if ((x < widthMinus) && (y < heightMinus)) 
				{
					Edge edge;
					edge.a = wh*t + y * width + x;
					edge.b = wh*d + (y+1) * width + (x+1);
					//edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x+1, y+1);
					float r = gpu_R(x,y) - gpu_lastR(x+1,y+1), g = gpu_G(x,y) - gpu_lastG(x+1,y+1), b = gpu_B(x,y) - gpu_lastB(x+1,y+1);
					edge.w = fast_math::sqrtf(r*r + g*g + b*b);
					gpu_Edges[gpu_num[0]] = edge;
					gpu_num[0]++;
				}

				if ((x < widthMinus) && (y > 0))
				{
					Edge edge;
					edge.a  = wh*t + y * width + x;
					edge.b  = wh*d + (y-1) * width + (x+1);
					//edge.w  = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x+1, y-1);
					float r = gpu_R(x,y) - gpu_lastR(x+1,y-1), g = gpu_G(x,y) - gpu_lastG(x+1,y-1), b = gpu_B(x,y) - gpu_lastB(x+1,y-1);
					edge.w = fast_math::sqrtf(r*r + g*g + b*b);
					gpu_Edges[gpu_num[0]] = edge;
					gpu_num[0]++;
				}
			}
		}
		);
		last_R = smooth_R;
		last_G = smooth_G;
		last_B = smooth_B;
		smooth_R.Reset(width,height);
		smooth_G.Reset(width,height);
		smooth_B.Reset(width,height);
	}
	gpu_Edges.synchronize();
	gpu_num.synchronize();
	*num_edges = num;
	printf("num: %d\n",*num_edges);
	imgs.clear();
}


void iBuildGraphVideo(vector<ImgBgr> &imgs,
					  float sigma,
					  Edge *&edges,
					  int *num_edges)
{
	int width = imgs[0].Width();
	int height = imgs[0].Height();
	int widthMinus = width - 1, heightMinus = height - 1, timeMinus = imgs.size() - 1, wh = width*height;
	int num = 0;
	int x, y, t;
	ImgFloat smooth_R(width, height),smooth_G(width, height),smooth_B(width, height),last_R(width, height),last_G(width, height),last_B(width, height);
	int reserve_size = wh*imgs.size()*9;
	printf("Reserve size = %d\n",reserve_size);
	edges = (Edge*) malloc(reserve_size*sizeof(Edge));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}

	for( t = 0; t < imgs.size(); t++) {
		//printf("Building graph, t = %d\n",t);
		ImgFloat R(width, height),G(width, height),B(width, height);
		iExtractRGBColorSpace(imgs[t], &B, &G, &R);
		iSmooth(B, sigma, &smooth_B);
		iSmooth(G, sigma, &smooth_G);
		iSmooth(R, sigma, &smooth_R);
		imgs[t].Reset();
		for ( y = 0; y < height; y++)
		{
			for ( x = 0; x < width; x++)
			{
				if(t > 0) {
					Edge edge;
					edge.a = wh*t + y * width + x;
					edge.b = wh*(t-1) + y * width + x;
					edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x, y);
					edges[num] = edge;
					num++;
				}
				if (x < widthMinus)
				{
					Edge edge;
					edge.a = wh*t + y * width + x;
					edge.b = wh*t + y * width + (x+1);
					edge.w = iDiff(smooth_R, smooth_G, smooth_B, x, y, x+1, y);
					edges[num] = edge;
					num++;
				}

				if (y < heightMinus)
				{
					Edge edge;
					edge.a = wh*t +  y * width + x;
					edge.b = wh*t + (y+1) * width + x;
					edge.w = iDiff(smooth_R, smooth_G, smooth_B, x, y, x, y+1);
					edges[num] = edge;
					num++;
				}

				if ((x < widthMinus) && (y < heightMinus)) 
				{
					Edge edge;
					edge.a = wh*t + y * width + x;
					edge.b = wh*t + (y+1) * width + (x+1);
					edge.w = iDiff(smooth_R, smooth_G, smooth_B, x, y, x+1, y+1);
					edges[num] = edge;
					num++;
				}

				if ((x < widthMinus) && (y > 0))
				{
					Edge edge;
					edge.a  = wh*t + y * width + x;
					edge.b  = wh*t + (y-1) * width + (x+1);
					edge.w  = iDiff(smooth_R, smooth_G, smooth_B, x, y, x+1, y-1);
					edges[num] = edge;
					num++;
				}
				int d = t - 1;
				if(d >= 0) {
					if (x < widthMinus)
					{
						Edge edge;
						edge.a = wh*t + y * width + x;
						edge.b = wh*d + y * width + (x+1);
						edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x+1, y);
						edges[num] = edge;
						num++;
					}

					if (y < heightMinus)
					{
						Edge edge;
						edge.a = wh*t +  y * width + x;
						edge.b = wh*d + (y+1) * width + x;
						edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x, y+1);
						edges[num] = edge;
						num++;
					}

					if ((x < widthMinus) && (y < heightMinus)) 
					{
						Edge edge;
						edge.a = wh*t + y * width + x;
						edge.b = wh*d + (y+1) * width + (x+1);
						edge.w = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x+1, y+1);
						edges[num] = edge;
						num++;
					}

					if ((x < widthMinus) && (y > 0))
					{
						Edge edge;
						edge.a  = wh*t + y * width + x;
						edge.b  = wh*d + (y-1) * width + (x+1);
						edge.w  = iDiffTime(smooth_R, smooth_G, smooth_B, x, y, last_R, last_G, last_B, x+1, y-1);
						edges[num] = edge;
						num++;
					}
				}
			}
		}
		last_R = smooth_R;
		last_G = smooth_G;
		last_B = smooth_B;
		smooth_R.Reset(width,height);
		smooth_G.Reset(width,height);
		smooth_B.Reset(width,height);
	}
	*num_edges = num;
	imgs.clear();
}

void iBuildGraph(const Volume<Bgr> &in,
				 float sigma,
				 Edge *&edges,
				 int *num_edges)
{
	int num = 0;
	int x,y,z, xp, yp, ym, zm;
	int width = in.Width(), height = in.Height(), depth = in.Depth();
	int reserve_size = in.size()*9;
	printf("Reserve size = %d\n",reserve_size);
	edges = (Edge*) malloc(reserve_size*sizeof(Edge));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	Volume<float> tmp_r, tmp_g, tmp_b, smooth_r,smooth_g,smooth_b;
	iExtractRGBColorSpace(in,&tmp_b,&tmp_g,&tmp_r);
	Smooth(tmp_b,sigma,&smooth_b);
	tmp_b.Reset();
	Smooth(tmp_g,sigma,&smooth_g);
	tmp_g.Reset();
	Smooth(tmp_r,sigma,&smooth_r);
	tmp_r.Reset();
	int safeWidth = width - 1, safeHeight = height - 1, safeDepth = depth - 1, wh = width*height;
	Edge *p = edges;
	Volume<float>::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin();
	for(z = 0, zm = -1; z < depth; z++, zm++) 
	{
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				if (x < safeWidth)
				{
					Edge edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*z + y * width + xp;
					edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if (y < safeHeight)
				{
					Edge edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*z + yp * width + x;
					edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y < safeHeight)) 
				{
					Edge edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*z + yp * width + xp;
					edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y > 0))
				{
					Edge edge;
					edge.a  = wh*z + y * width + x;
					edge.b  = wh*z + ym * width + xp;
					edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if(z > 0) {
					Edge edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*zm + y * width + x;
					edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge edge;
						edge.a = wh*z + y * width + x;
						edge.b = wh*zm + y * width + xp;
						edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
					if (y < safeHeight)
					{
						Edge edge;
						edge.a = wh*z + y * width + x;
						edge.b = wh*zm + yp * width + x;
						edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y < safeHeight)) 
					{
						Edge edge;
						edge.a = wh*z + y * width + x;
						edge.b = wh*zm + yp * width + xp;
						edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y > 0))
					{
						Edge edge;
						edge.a  = wh*z + y * width + x;
						edge.b  = wh*zm + ym * width + xp;
						edge.w = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
				}
				pR++;
				pG++;
				pB++;
			}
		}
	}
	smooth_r.Reset();
	smooth_g.Reset();
	smooth_b.Reset();
	*num_edges = num;
}

void iBuildGraph(const Volume<ColoredPoint> &in,
				 float sigma_depth,
				 float sigma_color,
				 Edge3D *&edges,
				 int *num_edges)
{
	int num = 0;
	int x,y,z, xp, yp, ym, zm;
	int width = in.Width(), height = in.Height(), depth = in.Depth();
	int reserve_size = in.size()*9;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	Volume<float> tmp_r, tmp_g, tmp_b, tmp_d, smooth_r,smooth_g,smooth_b, smooth_d;
	iExtractRGBDColorSpace(in,&tmp_b,&tmp_g,&tmp_r,&tmp_d);
	Smooth(tmp_b,sigma_color,&smooth_b);
	tmp_b.Reset();
	Smooth(tmp_g,sigma_color,&smooth_g);
	tmp_g.Reset();
	Smooth(tmp_r,sigma_color,&smooth_r);
	tmp_r.Reset();
	Smooth(tmp_d,sigma_depth,&smooth_d);
	tmp_d.Reset();
	int safeWidth = width - 1, safeHeight = height - 1, safeDepth = depth - 1, wh = width*height;
	Edge3D *p = edges;
	Volume<float>::ConstIterator pR = smooth_r.Begin(), pG = smooth_g.Begin(), pB = smooth_b.Begin(), pD = smooth_d.Begin();
	for(z = 0, zm = -1; z < depth; z++, zm++) 
	{
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				if (x < safeWidth)
				{
					Edge3D edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*z + y * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if (y < safeHeight)
				{
					Edge3D edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*z + yp * width + x;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y < safeHeight)) 
				{
					Edge3D edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*z + yp * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if ((x < safeWidth) && (y > 0))
				{
					Edge3D edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*z + ym * width + xp;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
				}
				if(z > 0) {
					//last = current - depth;
					Edge3D edge;
					edge.a = wh*z + y * width + x;
					edge.b = wh*zm + y * width + x;
					edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
					edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge3D edge;
						edge.a = wh*z + y * width + x;
						edge.b = wh*zm + y * width + xp;
						edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
						edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
					if (y < safeHeight)
					{
						Edge3D edge;
						edge.a = wh*z + y * width + x;
						edge.b = wh*zm + yp * width + x;
						edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
						edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y < safeHeight)) 
					{
						Edge3D edge;
						edge.a = wh*z + y * width + x;
						edge.b = wh*zm + yp * width + xp;
						edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
						edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
					if ((x < safeWidth) && (y > 0))
					{
						Edge3D edge;
						edge.a = wh*z + y * width + x;
						edge.b = wh*zm + ym * width + xp;
						edge.w = blepo_ex::Abs(*pD - smooth_d(edge.b));
						edge.w2 = blepo_ex::Sqrt(square(*pR - smooth_r(edge.b))+square(*pG - smooth_g(edge.b))+square(*pB - smooth_b(edge.b)));
						*p++ = edge;
						num++;
					}
				}
				pR++; pG++; pB++; pD++;
			}
		}
	}
	smooth_r.Reset();
	smooth_g.Reset();
	smooth_b.Reset();
	smooth_d.Reset();
	*num_edges = num;
}


bool lessThan (const Edge& a, const Edge& b) {
	return a.w < b.w;
}

bool lessThan3D (const Edge3D& a, const Edge3D& b) {
	return a.w2 < b.w2;
}

void iSegment_graph(int num_vertices, int num_edges, std::vector<Edge>& edges, float c, Universe *u)
{ 
	// sort edges by weight
	blepo_ex::Sort(edges.begin(), edges.end());

	// make a disjoint-set forest

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	vector<Edge>::iterator pEdge = edges.begin();
	while(pEdge != edges.end())
	{
		// components conected by this edge
		int a = u->find(pEdge->a);
		int b = u->find(pEdge->b);
		if (a != b /*&& a >= 0 && b>= 0 && a < num_vertices && b < num_vertices*/) {
			if ((pEdge->w <= threshold[a]) &&
				(pEdge->w <= threshold[b])) {
					u->join(a, b);
					a = u->find(a);
					if(a < num_vertices && a >= 0)
						threshold[a] = pEdge->w + THRESHOLD(u->size(a), c);
					else
						printf("a is %d, which is out of bounds\n",a);
			}
		}
		pEdge++;
	}

	// free up
	delete threshold;
}

void iSegment_graph(int num_vertices, int num_edges, Edge*& edges, float c, Universe *u)
{ 
	Edge* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	blepo_ex::Sort(pEdge, edgesEnd);

	// make a disjoint-set forest

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		// components conected by this edge
		int a = u->find(pEdge->a);
		int b = u->find(pEdge->b);
		if (a != b /*&& a >= 0 && b>= 0 && a < num_vertices && b < num_vertices*/) {
			if ((pEdge->w <= threshold[a]) &&
				(pEdge->w <= threshold[b])) {
					u->join(a, b);
					a = u->find(a);
					if(a < num_vertices && a >= 0)
						threshold[a] = pEdge->w + THRESHOLD(u->size(a), c);
					else
						printf("a is %d, which is out of bounds\n",a);
			}
		}
		pEdge++;
	}

	// free up
	delete threshold;
}

void iSegment_graph(int num_vertices, int num_edges, Edge3D*& edges, float c, Universe *u)
{ 
	Edge3D* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	blepo_ex::Sort(pEdge, edgesEnd);
	//sort(pEdge,edgesEnd,lessThanDepth);
	//thrust::device_vector<Edge3D> d_vec;
	//thrust::copy(pEdge,edgesEnd,d_vec.begin());
	//thrust::sort(d_vec.begin(),d_vec.end(), thrust::minus<Edge3D>());
	//thrust::copy(d_vec.begin(),d_vec.end(),pEdge);
	// make a disjoint-set forest

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		// components conected by this edge
		int a = u->find(pEdge->a);
		int b = u->find(pEdge->b);
		if (a != b /*&& a >= 0 && b>= 0 && a < num_vertices && b < num_vertices*/) {
			if ((pEdge->w <= threshold[a]) &&
				(pEdge->w <= threshold[b])) {
					u->join(a, b);
					a = u->find(a);
					if(a < num_vertices && a >= 0)
						threshold[a] = pEdge->w + THRESHOLD(u->size(a), c);
					else
						printf("a is %d, which is out of bounds\n",a);
			}
		}
		pEdge++;
	}

	// free up
	delete threshold;
}

void iSegment_graph(int num_vertices, int num_edges, map<int, map<int, bool>> &edgeMap, Edge3D*& edges, float c, Universe *u)
{ 
	Edge3D* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	blepo_ex::Sort(pEdge, edgesEnd);
	//sort(pEdge,edgesEnd,lessThanDepth);
	//thrust::device_vector<Edge3D> d_vec;
	//thrust::copy(pEdge,edgesEnd,d_vec.begin());
	//thrust::sort(d_vec.begin(),d_vec.end(), thrust::minus<Edge3D>());
	//thrust::copy(d_vec.begin(),d_vec.end(),pEdge);
	// make a disjoint-set forest

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		// components conected by this edge
		int a = u->find(pEdge->a);
		int b = u->find(pEdge->b);
		if (a != b /*&& a >= 0 && b>= 0 && a < num_vertices && b < num_vertices*/) {
			if ((pEdge->w <= threshold[a]) &&
				(pEdge->w <= threshold[b])) {
					u->join(a, b);
					edgeMap[a][b] = true;
					a = u->find(a);
					if(a < num_vertices && a >= 0)
						threshold[a] = pEdge->w + THRESHOLD(u->size(a), c);
					else
						printf("a is %d, which is out of bounds\n",a);
			}
		}
		pEdge++;
	}

	// free up
	delete threshold;
}

void iSegmentStep2_graph(int num_vertices, int num_edges, Edge3D*& edges, float c, Universe &u1, Universe *u2)
{ 
	Edge3D* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	blepo_ex::Sort(pEdge, edgesEnd,lessThan3D);
	// make a disjoint-set forest

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		// components conected by this edge
		int a = u1.find(pEdge->a);
		int b = u1.find(pEdge->b);
		if (a == b) {
			a = u2->find(pEdge->a);
			b = u2->find(pEdge->b);
			if(a != b) {
				if ((pEdge->w2 <= threshold[a]) &&
					(pEdge->w2 <= threshold[b])) {
						u2->join(a, b);
						a = u2->find(a);
						if(a < num_vertices && a >= 0)
							threshold[a] = pEdge->w2 + THRESHOLD(u2->size(a), c);
						else
							printf("a is %d, which is out of bounds\n",a);
				}
			}
		}
		pEdge++;
	}

	// free up
	delete threshold;
}

void iSegmentStep2_graph(int num_vertices, int num_edges, map<int, map<int, bool>> &edgeMap, Edge3D*& edges, float c, Universe &u1, Universe *u2)
{ 
	Edge3D* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	blepo_ex::Sort(pEdge, edgesEnd,lessThan3D);
	// make a disjoint-set forest

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		// components conected by this edge
		int a = u1.find(pEdge->a);
		int b = u1.find(pEdge->b);
		if (a == b) {
			a = u2->find(pEdge->a);
			b = u2->find(pEdge->b);
			if(a != b) {
				if ((pEdge->w2 <= threshold[a]) &&
					(pEdge->w2 <= threshold[b])) {
						u2->join(a, b);
						edgeMap[a][b] = true;
						a = u2->find(a);
						if(a < num_vertices && a >= 0)
							threshold[a] = pEdge->w2 + THRESHOLD(u2->size(a), c);
						else
							printf("a is %d, which is out of bounds\n",a);
				}
			}
		}
		pEdge++;
	}

	// free up
	delete threshold;
}


inline void iJoin_graph(Edge *&edges, int num_edges, int min_size, Universe *u) {
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(pEdge != edgesEnd)
	{
		int a = u->find(pEdge->a); 
		int b = u->find(pEdge->b);
		if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
		{
			u->join(a, b);
		}
		pEdge++;
	}
}

inline void iJoin_graph(Edge3D *&edges, int num_edges, int min_size, Universe *u) {
	Edge3D *pEdge = edges, *edgesEnd = edges + num_edges;
	while(pEdge != edgesEnd)
	{
		int a = u->find(pEdge->a); 
		int b = u->find(pEdge->b);
		if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
		{
			u->join(a, b);
		}
		pEdge++;
	}
}

inline void iJoin_graph(map<int, map<int, bool>> &edgeMap, Edge3D *&edges, int num_edges, int min_size, Universe *u) {
	Edge3D *pEdge = edges, *edgesEnd = edges + num_edges;
	while(pEdge != edgesEnd)
	{
		int a = u->find(pEdge->a); 
		int b = u->find(pEdge->b);
		if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
		{
			u->join(a, b);
			edgeMap[a][b] = true;
		}
		pEdge++;
	}
}


void random_rgb(Bgr *c)
{ 
	c->r = rand() % 255 + 1;
	c->g = rand() % 255 + 1;
	c->b = rand() % 255 + 1;
}
// ================< end local functions

int GraphSegmentation(
	const ImgBgr& img, 
	float sigma, 
	float c, 
	int min_size,
	ImgInt *out_labels, 
	ImgBgr *out_pseudocolors) 
{
	int width = img.Width();
	int height = img.Height();
	int x, y;

	ImgFloat R(width, height),G(width, height),B(width, height);
	iExtractRGBColorSpace(img, &B, &G, &R);
	ImgFloat smooth_R(width, height), smooth_G(width, height), smooth_B(width, height);
	out_labels->Reset(width, height);
	out_pseudocolors->Reset(width, height);
	iSmooth(B, sigma, &smooth_B);
	iSmooth(G, sigma, &smooth_G);
	iSmooth(R, sigma, &smooth_R);

	vector<Edge> edges;
	int num_edges;
	iBuildGraph(smooth_R, smooth_G, smooth_B, &edges, &num_edges);
	Universe u(width*height);
	iSegment_graph(width*height, num_edges, edges, c, &u);

	int i;
	for (i = 0; i < num_edges; i++)
	{
		int a = u.find(edges[i].a); 
		int b = u.find(edges[i].b);
		if ((a != b) && ((u.size(a) < min_size) || (u.size(b) < min_size)))
		{
			u.join(a, b);
		}
	}

	int num_ccs = u.num_sets();

	std::vector<Bgr> colors;
	for (i = 0; i < width*height; i++)
	{
		Bgr color;
		random_rgb(&color);
		colors.push_back(color);
	}

	for (y = 0; y < height; y++)
	{
		for ( x = 0; x < width; x++)
		{
			int comp = u.find(y * width + x);
			(*out_labels)(x, y) = comp;
			(*out_pseudocolors)(x, y) = colors[comp];
		}
	}  

	return num_ccs;
}

int GraphSegmentDepth(
	const ImgBgr& img, 
	const ImgGray& depth,
	float sigma, 
	float c, 
	int min_size,
	ImgInt *out_labels, 
	ImgBgr *out_pseudocolors) 
{
	int width = img.Width();
	int height = img.Height();
	int x, y;

	ImgFloat R(width, height),G(width, height),B(width, height);
	iExtractRGBColorSpace(img, &B, &G, &R);
	ImgFloat smooth_R(width, height), smooth_G(width, height), smooth_B(width, height);
	ImgFloat D(width, height), smooth_D(width, height);
	out_labels->Reset(width, height);
	out_pseudocolors->Reset(width, height);
	Convert(depth,&D);
	iSmooth(B, sigma, &smooth_B);
	iSmooth(G, sigma, &smooth_G);
	iSmooth(R, sigma, &smooth_R);
	iSmooth(D, sigma, &smooth_D);

	std::vector<Edge> edges;
	int num_edges;
	iBuildDepthGraph(smooth_R, smooth_G, smooth_B, smooth_D, &edges, &num_edges);
	Universe u(width*height);
	iSegment_graph(width*height, num_edges, edges, c, &u);

	int i;
	for (i = 0; i < num_edges; i++)
	{
		int a = u.find(edges[i].a); 
		int b = u.find(edges[i].b);
		if ((a != b) && ((u.size(a) < min_size) || (u.size(b) < min_size)))
		{
			u.join(a, b);
		}
	}

	int num_ccs = u.num_sets();

	std::vector<Bgr> colors;
	for (i = 0; i < width*height; i++)
	{
		Bgr color;
		random_rgb(&color);
		colors.push_back(color);
	}

	for (y = 0; y < height; y++)
	{
		for ( x = 0; x < width; x++)
		{
			int comp = u.find(y * width + x);
			(*out_labels)(x, y) = comp;
			(*out_pseudocolors)(x, y) = colors[comp];
		}
	}  

	return num_ccs;
}

void GraphSegment(
	Volume<Bgr> &in, 
	float sigma, 
	float c, 
	int min_size,
	Volume<Bgr> *out_labels) 
{

	int i, size = in.size();
	Universe u(size);
	Edge* edges = NULL;
	int num_edges;

	iBuildGraph(in, sigma, edges, &num_edges);
	if(edges == NULL || num_edges == 0) {
		printf("Error, graph has no edges\n");
		return;
	}
	iSegment_graph(size, num_edges, edges, c, &u);
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(pEdge != edgesEnd)
	{
		int a = u.find(pEdge->a); 
		int b = u.find(pEdge->b);
		if ((a != b) && ((u.size(a) < min_size) || (u.size(b) < min_size)))
		{
			u.join(a, b);
		}
		pEdge++;
	}
	free(edges);

	int num_ccs = u.num_sets();
	std::vector<Bgr> colors;
	for (i = 0; i < size; i++)
	{
		Bgr color;
		random_rgb(&color);
		colors.push_back(color);
	}

	out_labels->Reset(in.Width(), in.Height(), in.Depth());
	Volume<Bgr>::Iterator pO = out_labels->Begin();
	Volume<Bgr>::ConstIterator pI = in.Begin();
	i = 0;
	while(pI != in.End()) {
		int segment = u.find(i);
		*pO = colors[segment];
		pI++;
		pO++;
		i++;
	}

}

int GraphSegmentBad(
	PointCloud<Bgr> &in, 
	float sigma_color,
	float sigma_depth,
	float c, 
	float alpha,
	int min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *pseudo_color) 
{

	int width = in.Width();
	int height = in.Height();
	int x, y;

	ImgFloat R(width, height),G(width, height),B(width, height),D(width, height);
	iExtractRGBDColorSpace(in, &B, &G, &R, &D);
	ImgFloat smooth_R(width, height), smooth_G(width, height), smooth_B(width, height), smooth_D(width,height);
	out_labels->Reset(width, height);
	pseudo_color->Reset(width, height);
	iSmooth(B, sigma_color, &smooth_B);
	iSmooth(G, sigma_color, &smooth_G);
	iSmooth(R, sigma_color, &smooth_R);
	iSmooth(D, sigma_depth, &smooth_D);

	std::vector<Edge> edges;
	int num_edges;
	iBuildGraph(smooth_R, smooth_G, smooth_B, smooth_D, alpha, &edges, &num_edges);
	Universe u(width*height);
	iSegment_graph(width*height, num_edges, edges, c, &u);

	int i;
	for (i = 0; i < num_edges; i++)
	{
		int a = u.find(edges[i].a); 
		int b = u.find(edges[i].b);
		if ((a != b) && ((u.size(a) < min_size) || (u.size(b) < min_size)))
		{
			u.join(a, b);
		}
	}

	int num_ccs = u.num_sets();

	std::vector<Bgr> colors;
	for (i = 0; i < width*height; i++)
	{
		Bgr color;
		random_rgb(&color);
		colors.push_back(color);
	}

	PointCloud<int>::Iterator pLabel = out_labels->Begin();
	PointCloud<Bgr>::Iterator pOut = pseudo_color->Begin();
	PointCloud<Bgr>::ConstIterator pOrig = in.Begin();
	for (y = 0; y < height; y++)
	{
		for ( x = 0; x < width; x++)
		{
			int comp = u.find(y * width + x);
			*pLabel = Point3D<int>(pOrig->x,pOrig->y,pOrig->z,comp);
			*pOut = Point3D<Bgr>(pOrig->x,pOrig->y,pOrig->z,colors[comp]);
			pLabel++;
			pOut++;
			pOrig++;
		}
	}  

	return num_ccs;

}


int SHGraphSegment(
	Volume<ColoredPoint> &in, 
	float sigma_depth,
	float sigma_color,
	float c_depth, 
	int depth_min_size,
	float c_color, 
	int color_min_size,
	Volume<Point3D<int>> *out_labels,
	Volume<ColoredPoint> *out_pseudocolors) 
{

	int i, size = in.size();
	Universe u(size), u2(size);
	Edge3D* edges = NULL;
	int num_edges;

	iBuildGraph(in, sigma_depth, sigma_color, edges, &num_edges);
	if(edges == NULL || num_edges == 0) {
		printf("Error, graph has no edges\n");
		return 0;
	}
	iSegment_graph(size, num_edges, edges, c_depth, &u);
	iJoin_graph(edges,num_edges,depth_min_size, &u);
	iSegmentStep2_graph(size,num_edges,edges,c_color,u,&u2);
	iJoin_graph(edges,num_edges,color_min_size,&u2);

	free(edges);

	Bgr *colors = (Bgr *) malloc(size*sizeof(Bgr));
	Bgr *pColor = colors;
	for (i = 0; i < size; i++)
	{
		Bgr color;
		random_rgb(&color);
		*pColor++ = color;
	}

	out_labels->Reset(in.Width(), in.Height(), in.Depth());
	out_pseudocolors->Reset(in.Width(), in.Height(), in.Depth());
	Volume<Point3D<int>>::Iterator pO = out_labels->Begin();
	Volume<ColoredPoint>::Iterator pC = out_pseudocolors->Begin();
	Volume<ColoredPoint>::ConstIterator pI = in.Begin();
	i = 0;
	while(pI != in.End()) {
		int segment = u2.find(i);
		*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,pI->valid);
		*pC = Point3D<Bgr>(pI->x,pI->y,pI->z,colors[segment],pI->valid);
		pC++;
		pI++;
		pO++;
		i++;
	}
	free(colors);
	return u.num_sets();
}

int SHGraphSegment(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *out_pseudocolors) 
{

	int i, size = in.size();
	Universe u(size), u2(size);
	Edge3D* edges = NULL;
	int num_edges;

	iBuildGraph(in, sigma_depth, sigma_color, edges, &num_edges);
	if(edges == NULL || num_edges == 0) {
		printf("Error, graph has no edges\n");
		return 0;
	}
	iSegment_graph(size, num_edges, edges, c_depth, &u);
	iJoin_graph(edges,num_edges,depth_min_size, &u);
	iSegmentStep2_graph(size,num_edges,edges,c_color,u,&u2);
	iJoin_graph(edges,num_edges,color_min_size,&u2);

	free(edges);

	Bgr *colors = (Bgr *) malloc(size*sizeof(Bgr));
	Bgr *pColor = colors;
	for (i = 0; i < size; i++)
	{
		Bgr color;
		random_rgb(&color);
		*pColor++ = color;
	}

	out_labels->Reset(in.Width(), in.Height());
	out_pseudocolors->Reset(in.Width(), in.Height());
	PointCloud<int>::Iterator pO = out_labels->Begin();
	PointCloud<Bgr>::Iterator pC = out_pseudocolors->Begin();
	PointCloud<Bgr>::ConstIterator pI = in.Begin();
	i = 0;
	while(pI != in.End()) {
		int segment = u2.find(i);
		*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,pI->valid);
		*pC = Point3D<Bgr>(pI->x,pI->y,pI->z,colors[segment],pI->valid);
		pC++;
		pI++;
		pO++;
		i++;
	}
	free(colors);
	u.elts.clear();
	u2.elts.clear();
	return u2.num_sets();
}

int SHGraphSegmentDepthOnly(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *out_pseudocolors) 
{

	int i, size = in.size();
	Universe u(size);
	Edge3D* edges = NULL;
	int num_edges;

	iBuildGraph(in, sigma_depth, sigma_color, edges, &num_edges);
	if(edges == NULL || num_edges == 0) {
		printf("Error, graph has no edges\n");
		return 0;
	}
	iSegment_graph(size, num_edges, edges, c_depth, &u);
	iJoin_graph(edges,num_edges,depth_min_size, &u);

	free(edges);

	Bgr *colors = (Bgr *) malloc(size*sizeof(Bgr));
	Bgr *pColor = colors;
	for (i = 0; i < size; i++)
	{
		Bgr color;
		random_rgb(&color);
		*pColor++ = color;
	}

	out_labels->Reset(in.Width(), in.Height());
	out_pseudocolors->Reset(in.Width(), in.Height());
	PointCloud<int>::Iterator pO = out_labels->Begin();
	PointCloud<Bgr>::Iterator pC = out_pseudocolors->Begin();
	PointCloud<Bgr>::ConstIterator pI = in.Begin();
	i = 0;
	while(pI != in.End()) {
		int segment = u.find(i);
		*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,pI->valid);
		*pC = Point3D<Bgr>(pI->x,pI->y,pI->z,colors[segment],pI->valid);
		pC++;
		pI++;
		pO++;
		i++;
	}
	free(colors);
	u.elts.clear();
	return u.num_sets();
}

//currently broken
int SHGraphSegmentComplex(
	deque<PointCloud<Bgr>> in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *out_pseudocolors) 
{

	int i, size = NUM_FRAMES * in[0].size();
	Universe u(size), u2(size);
	Edge3D* edges = NULL;
	int num_edges;

	//iBuildGraph(in, sigma_depth, sigma_color, edges, &num_edges);
	if(edges == NULL || num_edges == 0) {
		printf("Error, graph has no edges\n");
		return 0;
	}
	iSegment_graph(size, num_edges, edges, c_depth, &u);
	iJoin_graph(edges,num_edges,depth_min_size, &u);
	iSegmentStep2_graph(size,num_edges,edges,c_color,u,&u2);
	iJoin_graph(edges,num_edges,color_min_size,&u2);

	free(edges);

	PointCloud<int>::Iterator pO;
	PointCloud<Bgr>::ConstIterator pI;
	//need to do this for all frames
	i = 0;
	for(int c = 0; c < NUM_FRAMES; c++) {
		out_labels[c].Reset(in[c].Width(), in[c].Height());
		out_pseudocolors[c].Reset(in[c].Width(), in[c].Height());
		pO = out_labels[c].Begin();
		pI = in[c].Begin();
		while(pI != in[c].End()) {
			int segment = u2.find(i);
			*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,pI->valid);
			pI++;
			pO++;
			i++;
		}
	}

	//RegionTree3D tree;
	//tree.Create(in,*out_labels,u2.num_sets(),0);

	Bgr *colors = (Bgr *) malloc(size*sizeof(Bgr));
	Bgr *pColor = colors;
	for (i = 0; i < size; i++)
	{
		Bgr color;
		random_rgb(&color);
		*pColor++ = color;
	}

	for(int c = 0; c < NUM_FRAMES; c++) {
		PointCloud<Bgr>::Iterator pC = out_pseudocolors[c].Begin();
		pO = out_labels[c].Begin();
		while(pC != out_pseudocolors[c].End()) {
			*pC++ = Point3D<Bgr>(pO->x,pO->y,pO->z,colors[pO->value],true);
			pO++;
		}
	}

	free(colors);
	return u2.num_sets();
}

void GraphSegmentVideo(
	vector<ImgBgr> &imgs, 
	float sigma, 
	float c, 
	int min_size,
	vector<ImgInt> *out_labels) 
{
	int width = imgs[0].Width();
	int height = imgs[0].Height();
	int i, x, y, frames = imgs.size();
	int num_vertices = width*height*frames;
	Universe u(num_vertices);
	Edge* edges = NULL;
	int num_edges;
	gpuiBuildGraphVideo(imgs,sigma, edges, &num_edges);
	if(edges == NULL || num_edges == 0) {
		printf("Error, graph has no edges\n");
		return;
	}
	iSegment_graph(num_vertices, num_edges, edges, c, &u);
	Edge *pEdge = edges, *edgesEnd = edges + num_edges;
	while(pEdge != edgesEnd)
	{
		int a = u.find(pEdge->a); 
		int b = u.find(pEdge->b);
		if ((a != b) && ((u.size(a) < min_size) || (u.size(b) < min_size)))
		{
			u.join(a, b);
		}
		pEdge++;
	}
	free(edges);

	out_labels->resize(frames);
	vector<ImgInt>::iterator pVI = out_labels->begin();
	int wh = width*height;
	for(i = 0; i < frames; i++) {
		pVI->Reset(width,height);
		ImgInt::Iterator pI = pVI->Begin();
		for (y = 0; y < height; y++)
		{
			for ( x = 0; x < width; x++)
			{
				*pI++ = u.find(wh*i + y * width + x);
			}
		}
		pVI++;
	}

}

int Segment3D::Initialize(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out,
	PointCloud<Bgr> *out_color) {
		if(m_init) return 0;
		else m_init = true;
		int i, size = in.size();
		Universe prevU_C = Universe(size);
		Universe prevU_D = Universe(size);
		Edge3D *prevE = NULL;
		int num_edges;
		iBuildGraph(in, sigma_depth, sigma_color, prevE, &num_edges);
		if(prevE == NULL || num_edges == 0) {
			printf("Error, graph has no edges\n");
			return 0;
		}
		iSegment_graph(size, num_edges, prevE, c_depth, &prevU_D);
		iJoin_graph(prevE,num_edges,depth_min_size, &prevU_D);
		iSegmentStep2_graph(size,num_edges,prevE,c_color,prevU_D,&prevU_C);
		iJoin_graph(prevE,num_edges,color_min_size,&prevU_C);

		free(prevE);

		m_colors = (Bgr *) malloc(size*sizeof(Bgr));
		Bgr *pColor = m_colors;
		for (i = 0; i < size; i++)
		{
			Bgr color;
			random_rgb(&color);
			*pColor++ = color;
		}

		out->Reset(in.Width(), in.Height());
		out_color->Reset(in.Width(), in.Height());
		PointCloud<int>::Iterator pO = out->Begin();
		PointCloud<Bgr>::ConstIterator pI = in.Begin();
		i = 0;
		while(pI != in.End()) {
			int segment = prevU_C.find(i);
			*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,true);
			pI++; pO++; i++;
		}
		//RegionTree3D tree;
		prevTree.Create(in,*out,prevU_C.num_sets(),0);
		//tree.PropagateRegionHierarchy(75);
		//tree.GetRegionList(TREE_LEVEL,&prevList);
		//tree.ImplementSegmentation(TREE_LEVEL);
		//prevTree.Create(in,*out,prevList.size(),0);
		//prevTree.PropagateRegionHierarchy(75);
		//prevTree.GetRegionList(TREE_LEVEL,&prevList);
		//prevTree.UpdateRegionList(prevList);
		//prevTree.ImplementSegmentation(TREE_LEVEL);
		origTree = prevTree;
		m_count++;
		PointCloud<Bgr>::Iterator pPseudo = out_color->Begin();
		pO = out->Begin();
		while(pO != out->End()) {
			*pPseudo = Point3D<Bgr>(pO->x,pO->y,pO->z,m_colors[pO->value],true);
			pO++; pPseudo++;
		}
		return prevU_C.num_sets();
}


int Segment3D::AddSlice(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out,
	PointCloud<Bgr> *out_color) {
		if(!m_init) 
			return Initialize(in,sigma_depth,c_depth,depth_min_size,sigma_color,c_color,color_min_size, out, out_color);
		int i, size = in.size();
		Universe currU_C = Universe(size);
		Universe currU_D = Universe(size);
		Edge3D *currE = NULL;
		int num_edges;
		iBuildGraph(in, sigma_depth, sigma_color, currE, &num_edges);
		if(currE == NULL || num_edges == 0) {
			printf("Error, graph has no edges\n");
			return 0;
		}
		iSegment_graph(size, num_edges, currE, c_depth, &currU_D);
		iJoin_graph(currE,num_edges,depth_min_size, &currU_D);
		iSegmentStep2_graph(size,num_edges,currE,c_color,currU_D,&currU_C);
		iJoin_graph(currE,num_edges,color_min_size,&currU_C);
		free(currE);

		out->Reset(in.Width(), in.Height());
		PointCloud<int>::Iterator pO = out->Begin();
		PointCloud<Bgr>::ConstIterator pI = in.Begin();
		i = 0;
		while(pI != in.End()) {
			int segment = currU_C.find(i);
			*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,true);
			pI++;
			pO++;
			i++;
		}
		//RegionTree3D tree;
		currTree.Create(in,*out,currU_C.num_sets(),m_maxLabel);
		//tree.PropagateRegionHierarchy(75);
		//tree.GetRegionList(TREE_LEVEL,&currList);
		//tree.ImplementSegmentation(TREE_LEVEL);
		//currTree.Create(in,*out,currList.size(),m_maxLabel);
		m_maxLabel += currU_C.num_sets();
		/*if(m_count % 15 == 0) {
		prevTree.Release();
		prevTree = origTree;
		}*/
		m_count++;
		Merge();
		prevTree.Release();
		prevTree = currTree;
		//currTree.ImplementSegmentation((float)TREE_LEVEL / 100);
		//currTree.UpdateRegionList(currList);
		/*prevTree.Release();
		prevTree = currTree;
		prevList.clear();
		prevList = currList;*/
		//set past value to current
		out_color->Reset(in.Width(),in.Height());
		PointCloud<Bgr>::Iterator pPseudo = out_color->Begin();
		pO = out->Begin();
		while(pPseudo != out_color->End()) {
			*pPseudo++ = Point3D<Bgr>(pO->x,pO->y,pO->z,m_colors[pO->value],true);
			pO++;
		}
		return currU_C.num_sets();
}

int Segment4D::Initialize(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out,
	PointCloud<Bgr> *out_color) {
		if(m_init >= 1) return 0;
		else m_init++;
		int i, size = in.size();
		Universe prevU_C = Universe(size);
		Universe prevU_D = Universe(size);
		prevPC = in;
		Edge3D *prevE = NULL;
		int num_edges;
		iBuildGraph(in, sigma_depth, sigma_color, prevE, &num_edges);
		if(prevE == NULL || num_edges == 0) {
			printf("Error, graph has no edges\n");
			return 0;
		}
		iSegment_graph(size, num_edges, prevE, c_depth, &prevU_D);
		iJoin_graph(prevE,num_edges,depth_min_size, &prevU_D);
		iSegmentStep2_graph(size,num_edges,prevE,c_color,prevU_D,&prevU_C);
		iJoin_graph(prevE,num_edges,color_min_size,&prevU_C);

		free(prevE);

		int colorSize = 2 * size;
		m_colors = (Bgr *) malloc(colorSize*sizeof(Bgr));
		Bgr *pColor = m_colors;
		for (i = 0; i < colorSize; i++)
		{
			Bgr color;
			random_rgb(&color);
			*pColor++ = color;
		}

		out->Reset(in.Width(), in.Height());
		out_color->Reset(in.Width(), in.Height());
		PointCloud<int>::Iterator pO = out->Begin();
		PointCloud<Bgr>::ConstIterator pI = in.Begin();
		i = 0;
		while(pI != in.End()) {
			int segment = prevU_C.find(i);
			*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,true);
			pI++; pO++; i++;
		}
		//RegionTree4D tree;
		//prevTree.Create(in,*out,prevU_C.num_sets(),0);
		//tree.PropagateRegionHierarchy(75);
		//tree.GetRegionList(TREE_LEVEL,&prevList);
		//tree.ImplementSegmentation(TREE_LEVEL);
		//prevTree.Create(in,*out,prevList.size(),0);
		prevPC = in;
		prevLabelPC = *out;
		//prevTree.PropagateRegionHierarchy(75);
		//prevTree.GetRegionList(TREE_LEVEL,&prevList);
		//prevTree.UpdateRegionList(prevList);
		//prevTree.ImplementSegmentation(TREE_LEVEL);
		PointCloud<Bgr>::Iterator pPseudo = out_color->Begin();
		pO = out->Begin();
		while(pO != out->End()) {
			*pPseudo = Point3D<Bgr>(pO->x,pO->y,pO->z,m_colors[pO->value],true);
			pO++; pPseudo++;
		}
		return prevU_C.num_sets();
}

int Segment4D::AddSlice(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out,
	PointCloud<Bgr> *out_color) {
		if(!m_init) 
			return Initialize(in,sigma_depth,c_depth,depth_min_size,sigma_color,c_color,color_min_size, out, out_color);
		int i, size = 2*in.size();
		Universe currU_C = Universe(size);
		Universe currU_D = Universe(size);
		Edge3D *currE = NULL;
		int num_edges;
		currPC.ReleaseData();
		currPC = in;
		iBuildGraph(in, prevPC, sigma_depth, sigma_color, &optFlow, currE, &num_edges);
		if(currE == NULL || num_edges == 0) {
			printf("Error, graph has no edges\n");
			return 0;
		}
		iSegment_graph(size, num_edges, currE, c_depth, &currU_D);
		iJoin_graph(currE,num_edges,depth_min_size, &currU_D);
		iSegmentStep2_graph(size,num_edges,currE,c_color,currU_D,&currU_C);
		iJoin_graph(currE,num_edges,color_min_size,&currU_C);
		free(currE);

		out->Reset(in.Width(), in.Height());
		PointCloud<int>::Iterator pO = out->Begin();
		PointCloud<Bgr>::ConstIterator pI = in.Begin();
		i = 0;
		while(pI != in.End()) {
			int segment = currU_C.find(i);
			*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,true);
			pI++;
			pO++;
			i++;
		}
		//RegionTree4D tree;
		if(m_init == 1)
			prevTree.Create(in,*out,prevPC,prevLabelPC,optFlow,currU_C.num_sets(),m_maxLabel);
		else
			currTree.Create(in,*out,prevPC,prevLabelPC,optFlow,currU_C.num_sets(),m_maxLabel);
		//tree.PropagateRegionHierarchy(75);
		//tree.GetRegionList(TREE_LEVEL,&currList);
		//tree.ImplementSegmentation(TREE_LEVEL);
		//currTree.Create(in,*out,currList.size(),m_maxLabel);
		m_maxLabel += currU_C.num_sets();
		if(m_init >= 2) {
			Merge();
			prevTree.Release();
			prevTree = currTree;
		}
		m_init++;
		//currTree.ImplementSegmentation((float)TREE_LEVEL / 100);
		//currTree.UpdateRegionList(currList);
		//currTree.GetRegionList(TREE_LEVEL,&currList);
		//prevList.clear();
		//prevList = currList;
		/*if(m_init % 10 == 0) {

		}*/
		//set past value to current
		out_color->Reset(in.Width(),in.Height());
		PointCloud<Bgr>::Iterator pPseudo = out_color->Begin();
		pO = out->Begin();
		while(pO != out->End()) {
			*pPseudo = Point3D<Bgr>(pO->x,pO->y,pO->z,m_colors[pO->value],true);
			pO++; pPseudo++;
		}
		//prevPC.ReleaseData();
		prevPC = currPC;
		//prevLabelPC.ReleaseData();
		prevLabelPC = *out;
		return currU_C.num_sets();
}

inline void Set(FlowInfo *flow, const Flow &val)
{
	for (FlowInfo::Iterator p = flow->Begin() ; p != flow->End() ; p++)  *p = val;
}

int Segment4DBig::AddSlice(
	PointCloud<Bgr> &in, 
	float alpha,
	float sigma_depth, 
	float sigma_color,
	float c, 
	int min_size) {
		clouds.push_back(in);
		//need to also compute optical flow here
		if(!m_init) {
			FlowInfo tmp_flow;
			tmp_flow.Reset(in.Width(), in.Height());
			Flow zero;
			zero.u = zero.v = zero.w = 0.0f;
			Set(&tmp_flow, zero);
			flows.push_back(tmp_flow);
		} else {
			//printf("Computing flow\n");
			FlowInfo tmp_flow;
			PointCloudBgr tmp_cloud = *(clouds.rbegin() + 1);
			ComputeOpticalFlow(tmp_cloud, in, &tmp_flow);
			flows.push_back(tmp_flow);
		}
		if(clouds.size() > NUM_FRAMES) {
			clouds.pop_front();
			flows.pop_front();
		}
		//	printf("done computing flow\n");
		if(m_init % m_merge_iter == 0 && m_init >= NUM_FRAMES) {
			//printf("I'm merging and such, m_init == %d\n", m_init);
			int i, size = NUM_FRAMES*in.size();
			Universe currU = Universe(size);
			Edge *currE = NULL;
			int num_edges;
			iBuildGraph(clouds,sigma_depth,sigma_color,alpha,currE,&num_edges);
			if(currE == NULL || num_edges == 0) {
				printf("Error, graph has no edges\n");
				return 0;
			}
			iSegment_graph(size, num_edges, currE, c, &currU);
			iJoin_graph(currE,num_edges,min_size, &currU);
			free(currE);

			PointCloud<int>::Iterator pO;
			PointCloud<Bgr>::ConstIterator pI;
			//need to do this for all frames
			i = 0;
			for(int c = 0; c < NUM_FRAMES; c++) {
				labels[c].Reset(clouds[c].Width(), clouds[c].Height());
				labels_colored[c].Reset(clouds[c].Width(), clouds[c].Height());
				pO = labels[c].Begin();
				pI = clouds[c].Begin();
				while(pI != clouds[c].End()) {
					int segment = currU.find(i);
					*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,pI->valid);
					pI++;
					pO++;
					i++;
				}
			}

			//RegionTree4D tree;
			if(m_init <= NUM_FRAMES) {
				int colorSize = size;
				m_colors = (Bgr *) malloc(colorSize*sizeof(Bgr));
				Bgr *pColor = m_colors;
				for (i = 0; i < colorSize; i++)
				{
					Bgr color;
					random_rgb(&color);
					*pColor++ = color;
				}
				prevTree.Create(clouds, flows, labels, currU.num_sets(), m_maxLabel);
			} else {
				currTree.Create(clouds, flows, labels, currU.num_sets(), m_maxLabel);
				Merge();
				prevTree.Release();
				prevTree = currTree;
			}
			m_maxLabel += currU.num_sets();
			//set past value to current
			for(int c = 0; c < NUM_FRAMES; c++) {
				PointCloud<Bgr>::Iterator pC = labels_colored[c].Begin();
				pO = labels[c].Begin();
				while(pC != labels_colored[c].End()) {
					*pC++ = Point3D<Bgr>(pO->x,pO->y,pO->z,m_colors[pO->value],true);
					pO++;
				}
			}
			m_init++;
			return currU.num_sets();
		}
		m_init++;
		return 0;
}

int Segment4DBig::AddSlice(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float max_depth, 
	float sigma_color,
	float c_color, 
	int color_min_size) {
		clouds.push_back(in);
		//need to also compute optical flow here
		if(!m_init) {
			FlowInfo tmp_flow;
			tmp_flow.Reset(in.Width(), in.Height());
			Flow zero;
			zero.u = zero.v = zero.w = 0.0f;
			Set(&tmp_flow, zero);
			flows.push_back(tmp_flow);
		} else {
			//printf("Computing flow\n");
			FlowInfo tmp_flow;
			PointCloudBgr tmp_cloud = *(clouds.rbegin() + 1);
			ComputeOpticalFlow(tmp_cloud, in, &tmp_flow);
			flows.push_back(tmp_flow);
		}
		if(clouds.size() > NUM_FRAMES) {
			clouds.pop_front();
			flows.pop_front();
		}
		//	printf("done computing flow\n");
		if(m_init % m_merge_iter == 0 && m_init >= NUM_FRAMES) {
			//printf("I'm merging and such, m_init == %d\n", m_init);
			int i, size = NUM_FRAMES*in.size();
			Universe currU_C = Universe(size);
			Universe currU_D = Universe(size);
			Edge3D *currE = NULL;
			int num_edges;
			iBuildGraph(clouds,sigma_depth,sigma_color,currE,&num_edges);
			if(currE == NULL || num_edges == 0) {
				printf("Error, graph has no edges\n");
				return 0;
			}
			//prevC_Map.clear();
			//prevD_Map.clear();
			iSegment_graph(size, num_edges, currE, c_depth, &currU_D);
			iJoin_graph(currE,num_edges,depth_min_size, &currU_D);
			iSegmentStep2_graph(size,num_edges,currE,c_color,currU_D,&currU_C);
			iJoin_graph(currE,num_edges,color_min_size,&currU_C);
			free(currE);

			PointCloud<int>::Iterator pO;
			PointCloud<Bgr>::ConstIterator pI;
			//need to do this for all frames
			i = 0;
			for(int c = 0; c < NUM_FRAMES; c++) {
				labels[c].Reset(clouds[c].Width(), clouds[c].Height());
				labels_colored[c].Reset(clouds[c].Width(), clouds[c].Height());
				pO = labels[c].Begin();
				pI = clouds[c].Begin();
				while(pI != clouds[c].End()) {
					int segment = currU_C.find(i);
					*pO = Point3D<int>(pI->x,pI->y,pI->z,segment,pI->valid);
					pI++;
					pO++;
					i++;
				}
			}

			//RegionTree4D tree;
			if(m_init <= NUM_FRAMES) {
				int colorSize = size;
				m_colors = (Bgr *) malloc(colorSize*sizeof(Bgr));
				Bgr *pColor = m_colors;
				for (i = 0; i < colorSize; i++)
				{
					Bgr color;
					random_rgb(&color);
					*pColor++ = color;
				}
				prevTree.Create(clouds, flows, labels, currU_C.num_sets(), m_maxLabel);
				//prevTree.PropagateRegionHierarchy(TREE_LEVEL);
				for(int c = 0; c < NUM_FRAMES; c++) {
					labels_old[c] = labels[c];
				}
			} else {
				currTree.Create(clouds, flows, labels, currU_C.num_sets(), m_maxLabel);
				//currTree.PropagateRegionHierarchy(TREE_LEVEL);
				//Merge2();
				Merge();
				prevTree.Release();
				prevTree = currTree;
				//currTree.ImplementSegmentation(TREE_LEVEL);
			}
			m_maxLabel += currU_C.num_sets();
			//set past value to current
			for(int c = 0; c < NUM_FRAMES; c++) {
				PointCloud<Bgr>::Iterator pC = labels_colored[c].Begin();
				pO = labels[c].Begin();
				while(pC != labels_colored[c].End()) {
					*pC++ = Point3D<Bgr>(pO->x,pO->y,pO->z,m_colors[pO->value],true);
					pO++;
				}
			}
			m_init++;
			return currU_C.num_sets();
		}
		m_init++;
		return 0;
}

void Segment4DBig::Merge2() {
	//I need to go through each label and keep track of what labels are in that one in the past
	const int size = NUM_FRAMES*labels[0].size();
	map<int,int> *corrections = new map<int,int>[size]();
	for(int c = 0; c < m_merge_iter; c++) {
		PointCloud<int>::Iterator pPast = labels_old[c+m_merge_iter].Begin(), pCurr = labels[c].Begin();
		while(pCurr != labels[c].End()) {
			corrections[pCurr->value][pPast->value]++;
			++pCurr; ++pPast;
		}
	}
	//Find the max of each label choice
	int *selection = new int[size]();
	int *pSel = selection;
	map<int,int> *pFix = corrections, *pFixEnd = corrections + size;
	int i = 0;
	while(pFix != pFixEnd) {
		if(!pFix->empty()) {
			map<int,int>::iterator pMap = pFix->begin();
			int max = pMap->second, val = pMap->first;
			++pMap;
			while(pMap != pFix->end()) {
				if(pMap->second > max) {
					max = pMap->second;
					val = pMap->first;
				}
				++pMap;
			}
			*pSel = val;
		}
		++pFix; ++pSel; ++i;
	}
	delete[] corrections;

	//update the labels
	for(int c = 0; c < NUM_FRAMES; c++) {
		PointCloud<int>::Iterator pPast = labels_old[c].Begin(), pCurr = labels[c].Begin();
		while(pCurr != labels[c].End()) {
			pPast = pCurr; //update the past on the fly as well
			pCurr->value = selection[pCurr->value];
			++pCurr; ++pPast;
		}
	}
	delete[] selection;
}

//old merge
/*
void Segment3D::Merge(PointCloud<int> *out) {
//take segments from currPC and find guess from prevPC
out->Reset(currPC.Width(),currPC.Height());
PointCloud<int>::Iterator pCurr = currPC.Begin();
int i = 0;
//look at hierarchical clustering
while(pCurr != currPC.End()) {
if(pCurr->valid) {
//I haven't been seen

//look for the overlapping regions between time
vector<Segments> regions;
PointCloud<int>::Iterator pSeg = pCurr, pPast = prevPC.Begin(i);
while(pSeg != currPC.End()) {
//I'm part of my segment
if(pSeg->value == pCurr->value) {
pSeg->valid = false;
//See if the past region is already in the regions list
vector<Segments>::iterator pRegions = regions.begin();
bool found = false;
while(pRegions != regions.end()) {
if(pPast->value == pRegions->m_loc) {
found = true;
pRegions->m_size++;
break;
}
pRegions++;
}
if(!found) {
regions.push_back(Segments(pPast->value,1));
}
}
pPast++; pSeg++;
}
//I've found the regions, now I should pick the best one (for now just assign the maximum size)
vector<Segments>::iterator pRegions = regions.begin();
int max = pRegions->m_size, maxLoc = pRegions->m_loc;
while(pRegions != regions.end()) {
if(pRegions->m_size > max) {
max = pRegions->m_size;
maxLoc = pRegions->m_loc;
}
pRegions++;
}
//set my segment according to the results
PointCloud<int>::Iterator pO = out->Begin(i);
pSeg = pCurr;
while(pO != out->End()) {
if(pSeg->value == pCurr->value)
*pO = Point3D<int>(pSeg->x, pSeg->y, pSeg->z, maxLoc, true);
pSeg++; pO++;
}
} 
i++; pCurr++;
}
}
*/

void Segment3D::Merge() {
	currTree.TemporalCorrection(prevTree,1);
	//currTree.TemporalCorrection(prevList,currList,1);
}

void Segment4D::Merge() {
	currTree.TemporalCorrection(prevTree,1);
	//currTree.TemporalCorrection(prevList,currList,1);
}

void Segment4DBig::Merge() {
	currTree.TemporalCorrection(prevTree,1);
	//currTree.TemporalCorrection(prevList,currList,1);
}

void GraphSegmentVideoOverlap(
	vector<ImgBgr> &imgs, 
	float sigma, 
	float c, 
	int min_size,
	vector<ImgInt> *out_labels) {
		//split imgs into overlapping 30 frame segments
		int i = 0,j = 0;
		while(i < imgs.size()) {
			while(j < 30) {

			}
		}
}

