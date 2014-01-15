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

#ifndef BUILDGRAPH_H
#define BUILDGRAPH_H

#include <cuda_runtime.h>
#include <npp.h>
#include "opencv2/gpu/devmem2d.hpp"
#include "opencv2/gpu/device/common.hpp"
#include <cuda_runtime_api.h>
#include <cufft.h>
#include <cublas.h>
#include "opencv2/gpu/gpu.hpp"
#include "Edges.h"
#include <thrust/version.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
 #include <thrust/sort.h>

void igpuBuildGraph(cv::Mat &R, cv::Mat &G, cv::Mat &B, cv::Mat &D, Edge3D *edges, int numEdges);
void thrustsort(Edge3D *pEdge, Edge3D *edgesEnd);
void thrustsort2(Edge3D *pEdge, Edge3D *edgesEnd);

#endif