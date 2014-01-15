#include "BuildGraph.h"

using namespace cv;
using namespace gpu;
using namespace device;

//template <typename T>
//__global__ void blendLinearKernel(int rows, int cols, int cn, const PtrStep<T> img1, const PtrStep<T> img2,
//								  const PtrStepf weights1, const PtrStepf weights2, PtrStep<T> result)
//{
//	int x = blockIdx.x * blockDim.x + threadIdx.x;
//	int y = blockIdx.y * blockDim.y + threadIdx.y;
//
//	if (y < rows && x < cols)
//	{
//		int x_ = x / cn;
//		float w1 = weights1.ptr(y)[x_];
//		float w2 = weights2.ptr(y)[x_];
//		T p1 = img1.ptr(y)[x];
//		T p2 = img2.ptr(y)[x];
//		result.ptr(y)[x] = (p1 * w1 + p2 * w2) / (w1 + w2 + 1e-5f);
//	}
//}
//
//template <typename T>
//void blendLinearCaller(int rows, int cols, int cn, PtrStep<T> img1, PtrStep<T> img2, PtrStepf weights1, PtrStepf weights2, PtrStep<T> result, cudaStream_t stream)
//{
//	dim3 threads(16, 16);
//	dim3 grid(divUp(cols * cn, threads.x), divUp(rows, threads.y));
//
//	blendLinearKernel<<<grid, threads, 0, stream>>>(rows, cols * cn, cn, img1, img2, weights1, weights2, result);
//	cudaSafeCall( cudaGetLastError() );
//
//	if (stream == 0)
//		cudaSafeCall(cudaDeviceSynchronize());
//}
//
//template void blendLinearCaller<float>(int, int, int, PtrStep<float>, PtrStep<float>, PtrStepf, PtrStepf, PtrStep<float>, cudaStream_t stream);

__global__ void igpuBuildGraphKernel(int safeWidth, int safeHeight, int width, PtrStep<float> R, PtrStep<float> G, PtrStep<float> B, PtrStep<float> D, Edge3D *edges) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int a = (y * width + x), iter = 4*a;
	if (x < safeWidth) {
		edges[iter].a = a;
		edges[iter].b = a + 1;
		edges[iter].w = fabsf(D.ptr(y)[x] - D.ptr(y)[x+1]);
		float rdiff = R.ptr(y)[x] - R.ptr(y)[x+1], gdiff = G.ptr(y)[x] - G.ptr(y)[x+1], bdiff = B.ptr(y)[x] - B.ptr(y)[x+1];
		edges[iter].w2 =  sqrtf(rdiff * rdiff + gdiff * gdiff + bdiff * bdiff);
		edges[iter].valid = true;
		iter++;
	}
	if (y < safeHeight) {
		edges[iter].a = a;
		edges[iter].b = a + width;
		edges[iter].w = fabsf(D.ptr(y)[x] - D.ptr(y+1)[x]);
		float rdiff = R.ptr(y)[x] - R.ptr(y+1)[x], gdiff = G.ptr(y)[x] - G.ptr(y+1)[x], bdiff = B.ptr(y)[x] - B.ptr(y+1)[x];
		edges[iter].w2 =  sqrtf(rdiff * rdiff + gdiff * gdiff + bdiff * bdiff);
		edges[iter].valid = true;
		iter++;
	}
	if (x < safeWidth && y < safeHeight) {
		edges[iter].a = a;
		edges[iter].b = a + width + 1;
		edges[iter].w = fabsf(D.ptr(y)[x] - D.ptr(y+1)[x+1]);
		float rdiff = R.ptr(y)[x] - R.ptr(y+1)[x+1], gdiff = G.ptr(y)[x] - G.ptr(y+1)[x+1], bdiff = B.ptr(y)[x] - B.ptr(y+1)[x+1];
		edges[iter].w2 =  sqrtf(rdiff * rdiff + gdiff * gdiff + bdiff * bdiff);
		edges[iter].valid = true;
		iter++;
	}
	if (x < safeWidth && y > 0) {
		edges[iter].a = a;
		edges[iter].b = a - width + 1;
		edges[iter].w = fabsf(D.ptr(y)[x] - D.ptr(y-1)[x+1]);
		float rdiff = R.ptr(y)[x] - R.ptr(y-1)[x+1], gdiff = G.ptr(y)[x] - G.ptr(y-1)[x+1], bdiff = B.ptr(y)[x] - B.ptr(y-1)[x+1];
		edges[iter].w2 =  sqrtf(rdiff * rdiff + gdiff * gdiff + bdiff * bdiff);
		edges[iter].valid = true;
	}
}

void igpuBuildGraph(Mat &R, Mat &G, Mat &B, Mat &D, Edge3D *edges, int numEdges) {
	//cudaSetDevice(0);
	dim3 threads(16,16);
	int cols = R.cols, rows = R.rows;
	dim3 grid(divUp(cols, threads.x), divUp(rows, threads.y));

	int widthMinus = cols - 1, heightMinus = rows - 1;
	GpuMat gpuR(R), gpuG(G), gpuB(B), gpuD(D);
	Edge3D *gpuEdges;
	size_t edge_size = numEdges*sizeof(Edge3D);
	cudaMalloc(&gpuEdges, edge_size);
	cudaMemset(gpuEdges,0,edge_size);
	cudaMemcpy(gpuEdges,edges,edge_size,cudaMemcpyHostToDevice);
	igpuBuildGraphKernel<<<grid, threads>>>(widthMinus, heightMinus, cols, gpuR, gpuG, gpuB, gpuD, gpuEdges);
	cudaSafeCall( cudaGetLastError() );
	cudaSafeCall(cudaDeviceSynchronize());
	cudaMemcpy(edges,gpuEdges,edge_size,cudaMemcpyDeviceToHost);
	cudaFree(gpuEdges);
}

void thrustsort(Edge3D *pEdge, Edge3D *edgesEnd) {
	thrust::device_vector<Edge3D> d_vec;
	thrust::copy(pEdge,edgesEnd,d_vec.begin());
	thrust::sort(d_vec.begin(),d_vec.end());
	thrust::copy(d_vec.begin(),d_vec.end(),pEdge);
}

__device__ bool lessThan3DGPU(const Edge3D& a, const Edge3D& b) {
	return a.w2 < b.w2;
}

void thrustsort2(Edge3D *pEdge, Edge3D *edgesEnd) {
	thrust::device_vector<Edge3D> d_vec;
	thrust::copy(pEdge,edgesEnd,d_vec.begin());
	thrust::sort(d_vec.begin(),d_vec.end(), lessThan3DGPU);
	thrust::copy(d_vec.begin(),d_vec.end(),pEdge);
}