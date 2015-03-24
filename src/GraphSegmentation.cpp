#include "GraphSegmentation.h"
//#include "RegionTree.h"

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#define WIDTH 4.0
#define THRESHOLD(size, c) (c/size)

template <class T>
inline T square(const T &x) { return x*x; }; 

/* make filters */
#define MAKE_FILTER(name, fun)                                \
	std::vector<float> make_ ## name (float sigma)       \
{                                                           \
	sigma = max(sigma, 0.01F);			                            \
	int len = (int)ceil(sigma * WIDTH) + 1;                     \
	std::vector<float> mask(len);                               \
	for (int i = 0; i < len; i++)                               \
{                                                           \
	mask[i] = fun;                                              \
}                                                           \
	return mask;                                                \
}

MAKE_FILTER(fgauss, (float) expf(-0.5*square(i/sigma)));

using namespace std;
using namespace concurrency;
using namespace pcl;
using namespace cv;

void normalize(std::vector<float> &mask)
{
	int len = mask.size();
	float sum = 0;
	int i;
	for (i = 1; i < len; i++) 
	{
		sum += fabsf(mask[i]);
	}
	sum = 2*sum + fabsf(mask[0]);
	for (i = 0; i < len; i++)
	{
		mask[i] /= sum;
	}
}

/* convolve src with mask.  dst is flipped! */
void convolve_even(Mat& src, Mat &dst, std::vector<float> &mask)
{
	int width = src.cols;
	int height = src.rows;
	int len = mask.size();
	dst = Mat(src.rows,src.cols,src.type());
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			//cout << x << ", " << y << endl;
			float sum = mask[0] * src.at<float>(y, x);
			for (int i = 1; i < len; i++) {
				sum += mask[i] * (src.at<float>(y,max(x-i,0)) + src.at<float>(y,min(x+i, width-1)));
			}
			dst.at<float>(y,x) = sum;
		}
	}
}

void iExtractRGBDColorSpace(const PointCloud<PointXYZRGBA>& in, Mat &B, Mat &G, Mat &R, Mat &D) {
	B = Mat(in.width,in.height,CV_32F);
	G = Mat(in.width,in.height,CV_32F);
	R = Mat(in.width,in.height,CV_32F);
	D = Mat(in.width,in.height,CV_32F);
	PointCloudBgr::const_iterator pI = in.begin();
	Mat_<float>::iterator pB = B.begin<float>(),pG = G.begin<float>(),pR = R.begin<float>(),pD = D.begin<float>();
	while(pI != in.end()) {
		*pB = (float)pI->b;
		*pG = (float)pI->g;
		*pR = (float)pI->r;
		*pD = 1000.0f*pI->z;
		pI++; pB++; pG++; pR++; pD++;
	}
}

void iExtractNormals(const PointCloud<PointNormal>& in, Mat &X, Mat &Y, Mat &Z) {
	X = Mat(in.width,in.height,CV_32F);
	Y = Mat(in.width,in.height,CV_32F);
	Z = Mat(in.width,in.height,CV_32F);
	PointCloud<PointNormal>::const_iterator pI = in.begin();
	Mat_<float>::iterator pX = X.begin<float>(),pY = Y.begin<float>(),pZ = Z.begin<float>();
	while(pI != in.end()) {
		*pX = pI->normal_x;
		*pY = pI->normal_y;
		*pZ = pI->normal_z;
		++pI; ++pX; ++pY; ++pZ;
	}
}

void iSmooth(Mat &src, float sigma, Mat &out) {
	std::vector<float> mask = make_fgauss(sigma);
	normalize(mask);
	Mat tmp(src.rows, src.cols, src.type());
	convolve_even(src, tmp, mask);
	convolve_even(tmp, out, mask);
}

void iBuildGraph(const PointCloud<PointXYZRGBA> &in,
				 float sigma_depth,
				 float sigma_color,
				 Edge3D *&edges,
				 int *num_edges)
{
	int width = in.width;
	int height = in.height;
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
	Mat R,G,B,D, smooth_r, smooth_g, smooth_b, smooth_d;
	iExtractRGBDColorSpace(in, B, G, R, D);
	iSmooth(B, sigma_color, smooth_b);
	iSmooth(G, sigma_color, smooth_g);
	iSmooth(R, sigma_color, smooth_r);
	iSmooth(D, sigma_depth, smooth_d);

	//Normalize
	//PointCloud<Bgr> norm;
	//Normalize(in,&norm);

	Edge3D *p = edges;
	Mat_<float>::const_iterator pR = smooth_r.begin<float>(), pG = smooth_g.begin<float>(), pB = smooth_b.begin<float>(), pD = smooth_d.begin<float>();
	Mat_<float>::const_iterator pRBegin = pR, pGBegin = pG, pBBegin = pB, pDBegin = pD;
	//PointCloud<Bgr>::ConstIterator pC = in.Begin();
	for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
	{
		for ( x = 0, xp = 1; x < width; x++, xp++)
		{
			//cout << x << ", " << y << endl;
			if (x < safeWidth)
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = y * width + xp;
				edge.w = fabsf(*pD - *(pDBegin + edge.b));
				edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if (y < safeHeight)
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = yp * width + x;
				edge.w = fabsf(*pD - *(pDBegin + edge.b));
				edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y < safeHeight)) 
			{
				Edge3D edge;
				edge.a = y * width + x;
				edge.b = yp * width + xp;
				edge.w = fabsf(*pD - *(pDBegin + edge.b));
				edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y > 0))
			{
				Edge3D edge;
				edge.a  = y * width + x;
				edge.b  = ym * width + xp;
				edge.w = fabsf(*pD - *(pDBegin + edge.b));
				edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			pR++; pG++; pB++; pD++;
		}
	}
	R.release();
	G.release();
	B.release();
	D.release();
	smooth_b.release();
	smooth_g.release();
	smooth_r.release();
	smooth_d.release();
	*num_edges = num;
}


void iBuildGraphNormals(const PointCloud<PointNormal> &in,
						float sigma,
						Edge *&edges,
						int *num_edges)
{
	int width = in.width;
	int height = in.height;
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
	Mat X,Y,Z, smooth_x, smooth_y, smooth_z;
	iExtractNormals(in, X, Y, Z);
	iSmooth(X, sigma, smooth_x);
	iSmooth(Y, sigma, smooth_y);
	iSmooth(Z, sigma, smooth_z);


	Edge *p = edges;
	Mat_<float>::const_iterator pX = smooth_x.begin<float>(), pY = smooth_y.begin<float>(), pZ = smooth_z.begin<float>();
	Mat_<float>::const_iterator pXBegin = pX, pYBegin = pY, pZBegin = pZ;
	//PointCloud<PointNormal>::const_iterator pC = in.begin(), pCBegin = pC;
	for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
	{
		for ( x = 0, xp = 1; x < width; x++, xp++)
		{
			//cout << x << ", " << y << endl;
			if (x < safeWidth)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = y * width + xp;
				edge.w =  sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if (y < safeHeight)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = yp * width + x;
				edge.w =  sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y < safeHeight)) 
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = yp * width + xp;
				edge.w =  sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y > 0))
			{
				Edge edge;
				edge.a  = y * width + x;
				edge.b  = ym * width + xp;
				edge.w =  sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b)));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			++pX; ++pY; ++pZ;
			//++pC;
		}
	}
	X.release();
	Y.release();
	Z.release();
	smooth_x.release();
	smooth_y.release();
	smooth_z.release();

	*num_edges = num;
}

void iBuildGraphColorAndNormals(const PointCloud<PointXYZRGBA> &color,
								const PointCloud<PointNormal> &normal,
								float sigma_normal,
								float sigma_color, 
								Edge *&edges,
								int *num_edges)
{
	int width = color.width;
	int height = color.height;
	int num = 0;
	int x, y, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = color.size()*8;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge*) malloc(reserve_size*sizeof(Edge));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	Mat X,Y,Z, smooth_x, smooth_y, smooth_z;
	iExtractNormals(normal, X, Y, Z);
	iSmooth(X, sigma_normal, smooth_x);
	iSmooth(Y, sigma_normal, smooth_y);
	iSmooth(Z, sigma_normal, smooth_z);

	Mat R,G,B,D, smooth_r, smooth_g, smooth_b;
	iExtractRGBDColorSpace(color, B, G, R, D);
	iSmooth(B, sigma_color, smooth_b);
	iSmooth(G, sigma_color, smooth_g);
	iSmooth(R, sigma_color, smooth_r);

	Edge *p = edges;
	Mat_<float>::const_iterator pX = smooth_x.begin<float>(), pY = smooth_y.begin<float>(), pZ = smooth_z.begin<float>();
	Mat_<float>::const_iterator pXBegin = pX, pYBegin = pY, pZBegin = pZ;
	Mat_<float>::const_iterator pR = smooth_r.begin<float>(), pG = smooth_g.begin<float>(), pB = smooth_b.begin<float>();
	Mat_<float>::const_iterator pRBegin = pR, pGBegin = pG, pBBegin = pB;
	for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
	{
		for ( x = 0, xp = 1; x < width; x++, xp++)
		{
			//cout << x << ", " << y << endl;
			if (x < safeWidth)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = y * width + xp;
				edge.w =  max(sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b))),sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b))));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if (y < safeHeight)
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = yp * width + x;
				edge.w =  max(sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b))),sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b))));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y < safeHeight)) 
			{
				Edge edge;
				edge.a = y * width + x;
				edge.b = yp * width + xp;
				edge.w =  max(sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b))),sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b))));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			if ((x < safeWidth) && (y > 0))
			{
				Edge edge;
				edge.a  = y * width + x;
				edge.b  = ym * width + xp;
				edge.w =  max(sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b))),sqrtf(square(*pX - *(pXBegin + edge.b))+square(*pY - *(pYBegin + edge.b))+square(*pZ - *(pZBegin + edge.b))));
				//edge.valid = true;
				*p++ = edge;
				num++;
			}
			++pX; ++pY; ++pZ; pR++; pG++; pB++;
		}
	}
	X.release();
	Y.release();
	Z.release();
	smooth_x.release();
	smooth_y.release();
	smooth_z.release();
	R.release();
	G.release();
	B.release();
	D.release();
	smooth_b.release();
	smooth_g.release();
	smooth_r.release();
	*num_edges = num;
}

void iBuildGraph(const deque< PointCloud<PointXYZRGBA> > &clouds,
				 float sigma_depth,
				 float sigma_color,
				 Edge3D *&edges,
				 int *num_edges)
{
	int width = clouds[0].width;
	int height = clouds[0].height;
	int num = 0;
	int x, y, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = NUM_FRAMES*clouds[0].size()*9;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	Mat R,G,B,D, smooth_r, smooth_g, smooth_b, smooth_d;

	Edge3D *p = edges;
	Mat_<float> past_r,past_g,past_b,past_d;
	for( int z = 0; z < NUM_FRAMES; z++) {
		iExtractRGBDColorSpace(clouds[z], B, G, R, D);
		iSmooth(B, sigma_color, smooth_b);
		iSmooth(G, sigma_color, smooth_g);
		iSmooth(R, sigma_color, smooth_r);
		if(sigma_depth == 0)
			smooth_d = D;
		else
			iSmooth(D, sigma_depth, smooth_d);

		Mat_<float>::const_iterator pR = smooth_r.begin<float>(), pG = smooth_g.begin<float>(), pB = smooth_b.begin<float>(), pD = smooth_d.begin<float>();
		Mat_<float>::const_iterator pRBegin = pR, pGBegin = pG, pBBegin = pB, pDBegin = pD;
		int currDepthIter = z * height * width, pastDepthIter = (z - 1) * height * width, priorDepthIter = NUM_FRAMES / 2 * height * width;
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				//cout << x << ", " << y << endl;
				if (x < safeWidth)
				{
					Edge3D edge;
					edge.a = y * width + x;
					edge.b = y * width + xp;
					edge.w = fabsf(*pD - *(pDBegin + edge.b));
					edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = fabsf(*pD - *(pDBegin + edge.b));
					edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = fabsf(*pD - *(pDBegin + edge.b));
					edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = fabsf(*pD - *(pDBegin + edge.b));
					edge.w2 =  sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = fabsf(*pD - *(pDBegin + past_d(edge.b)));
					edge.w2 =  sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += pastDepthIter;
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge3D edge;
						edge.a = y * width + x;
						edge.b = y * width + xp;
						edge.w = fabsf(*pD - *(pDBegin + past_d(edge.b)));
						edge.w2 =  sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
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
						edge.w = fabsf(*pD - *(pDBegin + past_d(edge.b)));
						edge.w2 =  sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
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
						edge.w = fabsf(*pD - *(pDBegin + past_d(edge.b)));
						edge.w2 =  sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
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
						edge.w = fabsf(*pD - *(pDBegin + past_d(edge.b)));
						edge.w2 =  sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
				}
				pR++; pG++; pB++; pD++;
			}
		}
		R.release();
		G.release();
		B.release();
		D.release();
		past_r.release();
		past_g.release();
		past_b.release();
		past_d.release();
		past_r = smooth_r;
		past_g = smooth_g;
		past_b = smooth_b;
		past_d = smooth_d;
	}
	*num_edges = num;
}

void iBuildGraph(const deque< PointCloud<PointXYZRGBA> > &clouds,
				 float sigma_depth,
				 float sigma_color,
				 float alpha,
				 Edge *&edges,
				 int *num_edges)
{
	int width = clouds[0].width;
	int height = clouds[0].height;
	int num = 0;
	int x, y, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
	int reserve_size = NUM_FRAMES*clouds[0].size()*9;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) malloc(reserve_size*sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	Mat R,G,B,D, smooth_r, smooth_g, smooth_b, smooth_d;

	Edge *p = edges;
	Mat_<float> past_r,past_g,past_b,past_d;
	for( int z = 0; z < NUM_FRAMES; z++) {
		iExtractRGBDColorSpace(clouds[z], B, G, R, D);
		iSmooth(B, sigma_color, smooth_b);
		iSmooth(G, sigma_color, smooth_g);
		iSmooth(R, sigma_color, smooth_r);
		if(sigma_depth == 0)
			smooth_d = D;
		else
			iSmooth(D, sigma_depth, smooth_d);

		Mat_<float>::const_iterator pR = smooth_r.begin<float>(), pG = smooth_g.begin<float>(), pB = smooth_b.begin<float>(), pD = smooth_d.begin<float>();
		Mat_<float>::const_iterator pRBegin = pR, pGBegin = pG, pBBegin = pB, pDBegin = pD;
		int currDepthIter = z * height * width, pastDepthIter = (z - 1) * height * width, priorDepthIter = NUM_FRAMES / 2 * height * width;
		for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++)
		{
			for ( x = 0, xp = 1; x < width; x++, xp++)
			{
				//cout << x << ", " << y << endl;
				if (x < safeWidth)
				{
					Edge edge;
					edge.a = y * width + x;
					edge.b = y * width + xp;
					edge.w = alpha * fabsf(*pD - *(pDBegin + edge.b)) + (1.0f - alpha) * sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = alpha * fabsf(*pD - *(pDBegin + edge.b)) + (1.0f - alpha) * sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = alpha * fabsf(*pD - *(pDBegin + edge.b)) + (1.0f - alpha) * sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = alpha * fabsf(*pD - *(pDBegin + edge.b)) + (1.0f - alpha) * sqrtf(square(*pR - *(pRBegin + edge.b))+square(*pG - *(pGBegin + edge.b))+square(*pB - *(pBBegin + edge.b)));
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
					edge.w = alpha * fabsf(*pD - *(pDBegin + past_d(edge.b))) + (1.0f - alpha) * sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
					edge.a += currDepthIter;
					edge.b += pastDepthIter;
					*p++ = edge;
					num++;
					if (x < safeWidth)
					{
						Edge edge;
						edge.a = y * width + x;
						edge.b = y * width + xp;
						edge.w = alpha * fabsf(*pD - *(pDBegin + past_d(edge.b))) + (1.0f - alpha) * sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
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
						edge.w = alpha * fabsf(*pD - *(pDBegin + past_d(edge.b))) + (1.0f - alpha) * sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
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
						edge.w = alpha * fabsf(*pD - *(pDBegin + past_d(edge.b))) + (1.0f - alpha) * sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
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
						edge.w = alpha * fabsf(*pD - *(pDBegin + past_d(edge.b))) + (1.0f - alpha) * sqrtf(square(*pR - past_r(edge.b))+square(*pG - past_g(edge.b))+square(*pB - past_b(edge.b)));
						edge.a += currDepthIter;
						edge.b += pastDepthIter;
						*p++ = edge;
						num++;
					}
				}
				pR++; pG++; pB++; pD++;
			}
		}
		R.release();
		G.release();
		B.release();
		D.release();
		past_r.release();
		past_g.release();
		past_b.release();
		past_d.release();
		past_r = smooth_r;
		past_g = smooth_g;
		past_b = smooth_b;
		past_d = smooth_d;
	}
	*num_edges = num;
}


void iBuildGraphCUDA(const PointCloud<PointXYZRGBA>::ConstPtr &in,
					 float sigma_depth,
					 float sigma_color,
					 Edge3D *&edges,
					 int *num_edges)
{
	int reserve_size = in->size()*4;
	//printf("Reserve size = %d\n",reserve_size);
	edges = (Edge3D*) calloc(reserve_size,sizeof(Edge3D));
	if(edges == NULL) {
		printf("Error, could not malloc\n");
		return;
	}
	Mat R,G,B,D, smooth_r, smooth_g, smooth_b, smooth_d;
	iExtractRGBDColorSpace(*in, B, G, R, D);
	iSmooth(B, sigma_color, smooth_b);
	iSmooth(G, sigma_color, smooth_g);
	iSmooth(R, sigma_color, smooth_r);
	iSmooth(D, sigma_depth, smooth_d);

	//Normalize
	//PointCloud<Bgr> norm;
	//Normalize(in,&norm);

	//igpuBuildGraph(smooth_r, smooth_g, smooth_b, smooth_d, edges, reserve_size);
	R.release();
	G.release();
	B.release();
	D.release();
	smooth_b.release();
	smooth_g.release();
	smooth_r.release();
	smooth_d.release();
	//I need to clean up the edges somehow
	*num_edges = reserve_size;
}

bool lessThan (const Edge& a, const Edge& b) {
	return a.w < b.w;
}

bool lessThan3D (const Edge3D& a, const Edge3D& b) {
	return a.w2 < b.w2;
}

void iSegment_graph(int num_vertices, int num_edges, Edge*& edges, float c, Universe *u)
{ 
	Edge* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	concurrency::parallel_sort(pEdge, edgesEnd);
	//thrustsort(pEdge,edgesEnd);

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		//if(pEdge->valid) {
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
		//}
		pEdge++;
	}

	// free up
	delete threshold;
}


void iSegment_graph(int num_vertices, int num_edges, Edge3D*& edges, float c, Universe *u)
{ 
	Edge3D* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	concurrency::parallel_sort(pEdge, edgesEnd);
	//thrustsort(pEdge,edgesEnd);

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		//if(pEdge->valid) {
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
		//}
		pEdge++;
	}

	// free up
	delete threshold;
}

void iSegmentStep2_graph(int num_vertices, int num_edges, Edge3D*& edges, float c, Universe &u1, Universe *u2)
{ 
	Edge3D* pEdge = edges, *edgesEnd = pEdge + num_edges;
	// sort edges by weight
	concurrency::parallel_sort(pEdge, edgesEnd,lessThan3D);
	//thrustsort2(pEdge,edgesEnd);

	// init thresholds
	float *threshold = new float[num_vertices];
	int i;
	float *pThresh = threshold;
	for (i = 0; i < num_vertices; i++)
		*pThresh++ = THRESHOLD(1,c);

	// for each edge, in non-decreasing weight order...
	while(pEdge != edgesEnd)
	{
		//if(pEdge->valid) {
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
		//}
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
		//if(pEdge->valid) {
		int a = u->find(pEdge->a); 
		int b = u->find(pEdge->b);
		if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
		{
			u->join(a, b);
		}
		//}
		pEdge++;
	}
}

void random_rgb(Vec3b &c)
{ 
	c[0] = rand() % 255 + 1;
	c[1] = rand() % 255 + 1;
	c[2] = rand() % 255 + 1;
}

int SHGraphSegment(
	const PointCloudBgr &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<PointXYZI>::Ptr &out,
	PointCloud<PointXYZRGBA>::Ptr &out_color)
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

	Vec3b *m_colors = (Vec3b *) malloc(size*sizeof(Vec3b));
	Vec3b *pColor = m_colors;
	for (i = 0; i < size; i++)
	{
		Vec3b color;
		random_rgb(color);
		*pColor++ = color;
	}

	out->height = in.height;
	out->width = in.width;
	out->is_dense = false;
	out->points.resize (out->height * out->width);
	out_color->height = in.height;
	out_color->width = in.width;
	out_color->is_dense = false;
	out_color->points.resize (out->height * out->width);
	PointCloud<PointXYZI>::iterator pO = out->begin();
	PointCloudBgr::const_iterator pI = in.begin();
	i = 0;
	while(pI != in.end()) {
		int segment = u2.find(i);
		pO->x = pI->x;
		pO->y = pI->y;
		pO->z = pI->z;
		pO->intensity = segment;
		++pI; ++pO; ++i;
	}
	//prevTree.Create(*in,*out,prevU_C.num_sets(),0);
	PointCloudBgr::iterator pPseudo = out_color->begin();
	pO = out->begin();
	while(pO != out->end()) {
		pPseudo->x = pO->x;
		pPseudo->y = pO->y;
		pPseudo->z = pO->z;
		Vec3b tmp = m_colors[int(pO->intensity)];
		pPseudo->r = tmp[2];
		pPseudo->g = tmp[1];
		pPseudo->b = tmp[0];
		pPseudo->a = 255;
		++pO; ++pPseudo;
	}
	free(m_colors);
	u.elts.clear();
	u2.elts.clear();
	return u2.num_sets();
}

int Segment3D::Initialize(
	const PointCloudBgr &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<PointXYZI>::Ptr &out,
	PointCloudBgr::Ptr &out_color) {
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

		m_colors = (Vec3b *) malloc(size*sizeof(Vec3b));
		Vec3b *pColor = m_colors;
		for (i = 0; i < size; i++)
		{
			Vec3b color;
			random_rgb(color);
			*pColor++ = color;
		}

		out->height = in.height;
		out->width = in.width;
		out->is_dense = false;
		out->points.resize (out->height * out->width);
		out_color->height = in.height;
		out_color->width = in.width;
		out_color->is_dense = false;
		out_color->points.resize (out->height * out->width);
		PointCloud<PointXYZI>::iterator pO = out->begin();
		PointCloudBgr::const_iterator pI = in.begin();
		i = 0;
		while(pI != in.end()) {
			int segment = prevU_C.find(i);
			pO->x = pI->x;
			pO->y = pI->y;
			pO->z = pI->z;
			pO->intensity = segment;
			++pI; ++pO; ++i;
		}
		//prevTree.Create(*in,*out,prevU_C.num_sets(),0);
		m_count++;
		PointCloudBgr::iterator pPseudo = out_color->begin();
		pO = out->begin();
		while(pO != out->end()) {
			pPseudo->x = pO->x;
			pPseudo->y = pO->y;
			pPseudo->z = pO->z;
			Vec3b tmp = m_colors[int(pO->intensity)];
			pPseudo->r = tmp[2];
			pPseudo->g = tmp[1];
			pPseudo->b = tmp[0];
			pPseudo->a = 255;
			++pO; ++pPseudo;
		}
		return prevU_C.num_sets();
}


int Segment3D::AddSlice(
	const PointCloudBgr &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<PointXYZI>::Ptr &out,
	PointCloudBgr::Ptr &out_color) {
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

		out->height = in.height;
		out->width = in.width;
		out->is_dense = false;
		out->points.resize (out->height * out->width);
		out_color->height = in.height;
		out_color->width = in.width;
		out_color->is_dense = false;
		out_color->points.resize (out->height * out->width);
		PointCloud<PointXYZI>::iterator pO = out->begin();
		PointCloudBgr::const_iterator pI = in.begin();
		i = 0;
		while(pI != in.end()) {
			int segment = currU_C.find(i);
			pO->x = pI->x;
			pO->y = pI->y;
			pO->z = pI->z;
			pO->intensity = segment;
			++pI; ++pO; ++i;
		}
		//currTree.Create(in,*out,currU_C.num_sets(),m_maxLabel);
		//currTree.PropagateRegionHierarchy(75);
		//currTree.ImplementSegmentation(TREE_LEVEL);
		m_maxLabel += currU_C.num_sets();
		m_count++;
		/*Merge();
		prevTree.Release();
		prevTree = currTree;*/
		//set past value to current
		PointCloudBgr::iterator pPseudo = out_color->begin();
		pO = out->begin();
		while(pO != out->end()) {
			pPseudo->x = pO->x;
			pPseudo->y = pO->y;
			pPseudo->z = pO->z;
			Vec3b tmp = m_colors[int(pO->intensity)];
			pPseudo->r = tmp[2];
			pPseudo->g = tmp[1];
			pPseudo->b = tmp[0];
			pPseudo->a = 255;
			++pO; ++pPseudo;
		}
		return currU_C.num_sets();
}

void Segment3D::Merge() {
	currTree.TemporalCorrection(prevTree,1);
	//currTree.TemporalCorrection(prevList,currList,1);
}

inline void Set(PointCloud<Normal> *flow, const Normal &val)
{
	for (PointCloud<Normal>::iterator p = flow->begin() ; p != flow->end() ; p++)  *p = val;
}

int Segment4DBig::AddSlice(
	const PointCloudBgr &in, 
	float alpha,
	float sigma_depth, 
	float sigma_color,
	float c, 
	int min_size) {
		clouds.push_back(in);
		//need to also compute optical flow here
		if(!m_init) {
			PointCloud<Normal> tmp_flow;
			tmp_flow.resize(in.width * in.height);
			tmp_flow.width = in.width;
			tmp_flow.height = in.height;
			Normal zero;
			zero.normal_x = zero.normal_y = zero.normal_z = 0.0f;
			Set(&tmp_flow, zero);
			flows.push_back(tmp_flow);
		} else {
			//printf("Computing flow\n");
			PointCloud<Normal> tmp_flow;
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

			PointCloudInt::iterator pO;
			PointCloudBgr::const_iterator pI;
			//need to do this for all frames
			i = 0;
			for(int c = 0; c < NUM_FRAMES; c++) {
				labels[c].height = in.height;
				labels[c].width = in.width;
				labels[c].is_dense = false;
				labels[c].points.resize (in.height * in.width);
				labels_colored[c].height = in.height;
				labels_colored[c].width = in.width;
				labels_colored[c].is_dense = false;
				labels_colored[c].points.resize (in.height * in.width);
				pO = labels[c].begin();
				pI = clouds[c].begin();
				while(pI != clouds[c].end()) {
					int segment = currU.find(i);
					pO->x = pI->x;
					pO->y = pI->y;
					pO->z = pI->z;
					pO->intensity = segment;
					++pI; ++pO; ++i;
				}
			}

			//RegionTree4D tree;
			if(m_init <= NUM_FRAMES) {
				int colorSize = size;
				m_colors = (Vec3b *) malloc(colorSize*sizeof(Vec3b));
				Vec3b *pColor = m_colors;
				for (i = 0; i < colorSize; i++)
				{
					Vec3b color;
					random_rgb(color);
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
				PointCloudBgr::iterator pC = labels_colored[c].begin();
				pO = labels[c].begin();
				while(pC != labels_colored[c].end()) {
					pC->x = pO->x;
					pC->y = pO->y;
					pC->z = pO->z;
					Vec3b tmp = m_colors[int(pO->intensity)];
					pC->r = tmp[2];
					pC->g = tmp[1];
					pC->b = tmp[0];
					pC->a = 255;
					++pO; ++pC;
				}
			}
			m_init++;
			return currU.num_sets();
		}
		m_init++;
		return 0;
}

int Segment4DBig::AddSlice(
	const PointCloudBgr &in, 
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
			PointCloud<Normal> tmp_flow;
			tmp_flow.resize(in.width * in.height);
			tmp_flow.width = in.width;
			tmp_flow.height = in.height;
			Normal zero;
			zero.normal_x = zero.normal_y = zero.normal_z = 0.0f;
			Set(&tmp_flow, zero);
			flows.push_back(tmp_flow);
		} else {
			//printf("Computing flow\n");
			PointCloud<Normal> tmp_flow;
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

			PointCloudInt::iterator pO;
			PointCloudBgr::const_iterator pI;
			//need to do this for all frames
			i = 0;
			for(int c = 0; c < NUM_FRAMES; c++) {
				labels[c].height = in.height;
				labels[c].width = in.width;
				labels[c].is_dense = false;
				labels[c].points.resize (in.height * in.width);
				labels_colored[c].height = in.height;
				labels_colored[c].width = in.width;
				labels_colored[c].is_dense = false;
				labels_colored[c].points.resize (in.height * in.width);
				pO = labels[c].begin();
				pI = clouds[c].begin();
				while(pI != clouds[c].end()) {
					int segment = currU_C.find(i);
					pO->x = pI->x;
					pO->y = pI->y;
					pO->z = pI->z;
					pO->intensity = segment;
					++pI; ++pO; ++i;
				}
			}

			//RegionTree4D tree;
			if(m_init <= NUM_FRAMES) {
				int colorSize = size;
				m_colors = (Vec3b *) malloc(colorSize*sizeof(Vec3b));
				Vec3b *pColor = m_colors;
				for (i = 0; i < colorSize; i++)
				{
					Vec3b color;
					random_rgb(color);
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
				PointCloudBgr::iterator pC = labels_colored[c].begin();
				pO = labels[c].begin();
				while(pC != labels_colored[c].end()) {
					pC->x = pO->x;
					pC->y = pO->y;
					pC->z = pO->z;
					Vec3b tmp = m_colors[int(pO->intensity)];
					pC->r = tmp[2];
					pC->g = tmp[1];
					pC->b = tmp[0];
					pC->a = 255;
					++pO; ++pC;
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
		PointCloudInt::iterator pPast = labels_old[c+m_merge_iter].begin(), pCurr = labels[c].begin();
		while(pCurr != labels[c].end()) {
			corrections[int(pCurr->intensity)][int(pPast->intensity)]++;
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
		PointCloudInt::iterator pPast = labels_old[c].begin(), pCurr = labels[c].begin();
		while(pCurr != labels[c].end()) {
			pPast = pCurr; //update the past on the fly as well
			pCurr->intensity = selection[int(pCurr->intensity)];
			++pCurr; ++pPast;
		}
	}
	delete[] selection;
}

void Segment4DBig::Merge() {
	currTree.TemporalCorrection(prevTree,1);
	//currTree.TemporalCorrection(prevList,currList,1);
}

int SegmentNormals(const PointCloudBgr &cloud, 
				   const PointCloud<PointNormal>::ConstPtr &in, 
				   float sigma, 
				   float c, 
				   int min_size,
				   PointCloud<PointXYZI>::Ptr &out,
				   PointCloudBgr::Ptr &out_color) {
					   int i, size = in->size();
					   Universe currU = Universe(size);
					   Edge *currE = NULL;
					   int num_edges;
					   iBuildGraphNormals(*in,sigma,currE,&num_edges);
					   if(currE == NULL || num_edges == 0) {
						   printf("Error, graph has no edges\n");
						   return 0;
					   }
					   iSegment_graph(size, num_edges, currE, c, &currU);
					   iJoin_graph(currE,num_edges,min_size,&currU);
					   free(currE);

					   Vec3b *m_colors = (Vec3b *) malloc(size*sizeof(Vec3b));
					   Vec3b *pColor = m_colors;
					   for (i = 0; i < size; i++)
					   {
						   Vec3b color;
						   random_rgb(color);
						   *pColor++ = color;
					   }

					   out->height = in->height;
					   out->width = in->width;
					   out->is_dense = false;
					   out->points.resize (out->height * out->width);
					   out_color->height = in->height;
					   out_color->width = in->width;
					   out_color->is_dense = false;
					   out_color->points.resize (out->height * out->width);
					   PointCloud<PointXYZI>::iterator pO = out->begin();
					   //PointCloud<PointNormal>::const_iterator pI = in->begin();
					   PointCloud<PointXYZRGBA>::const_iterator pC = cloud.begin();
					   i = 0;
					   while(pC != cloud.end()) {
						   int segment = currU.find(i);
						   pO->x = pC->x;
						   pO->y = pC->y;
						   pO->z = pC->z;
						   pO->intensity = segment;
						   ++pO; ++pC; ++i;
					   }

					   PointCloudBgr::iterator pPseudo = out_color->begin();
					   pO = out->begin();
					   while(pO != out->end()) {
						   pPseudo->x = pO->x;
						   pPseudo->y = pO->y;
						   pPseudo->z = pO->z;
						   Vec3b tmp = m_colors[int(pO->intensity)];
						   pPseudo->r = tmp[2];
						   pPseudo->g = tmp[1];
						   pPseudo->b = tmp[0];
						   pPseudo->a = 255;
						   ++pO; ++pPseudo;
					   }
					   free(m_colors);
					   return currU.num_sets();
}

int SegmentColorAndNormals(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, 
						   const pcl::PointCloud<pcl::PointNormal>::ConstPtr &in, 
						   float sigma_normal, 
						   float sigma_color,
						   float c, 
						   int min_size,
						   pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
						   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_color) {
							   int i, size = in->size();
							   Universe currU = Universe(size);
							   Edge *currE = NULL;
							   int num_edges;
							   iBuildGraphColorAndNormals(cloud,*in,sigma_normal,sigma_color,currE,&num_edges);
							   if(currE == NULL || num_edges == 0) {
								   printf("Error, graph has no edges\n");
								   return 0;
							   }
							   iSegment_graph(size, num_edges, currE, c, &currU);
							   iJoin_graph(currE,num_edges,min_size,&currU);
							   free(currE);

							   Vec3b *m_colors = (Vec3b *) malloc(size*sizeof(Vec3b));
							   Vec3b *pColor = m_colors;
							   for (i = 0; i < size; i++)
							   {
								   Vec3b color;
								   random_rgb(color);
								   *pColor++ = color;
							   }

							   out->height = in->height;
							   out->width = in->width;
							   out->is_dense = false;
							   out->points.resize (out->height * out->width);
							   out_color->height = in->height;
							   out_color->width = in->width;
							   out_color->is_dense = false;
							   out_color->points.resize (out->height * out->width);
							   PointCloud<PointXYZI>::iterator pO = out->begin();
							   //PointCloud<PointNormal>::const_iterator pI = in->begin();
							   PointCloud<PointXYZRGBA>::const_iterator pC = cloud.begin();
							   i = 0;
							   while(pC != cloud.end()) {
								   int segment = currU.find(i);
								   pO->x = pC->x;
								   pO->y = pC->y;
								   pO->z = pC->z;
								   pO->intensity = segment;
								   ++pO; ++pC; ++i;
							   }

							   PointCloudBgr::iterator pPseudo = out_color->begin();
							   pO = out->begin();
							   while(pO != out->end()) {
								   pPseudo->x = pO->x;
								   pPseudo->y = pO->y;
								   pPseudo->z = pO->z;
								   Vec3b tmp = m_colors[int(pO->intensity)];
								   pPseudo->r = tmp[2];
								   pPseudo->g = tmp[1];
								   pPseudo->b = tmp[0];
								   pPseudo->a = 255;
								   ++pO; ++pPseudo;
							   }
							   free(m_colors);
							   return currU.num_sets();
}