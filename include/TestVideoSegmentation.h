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
#pragma once

#include "resource.h"
#include "blepo.h"
#include "RegionTree.h"
#include "GraphSegmentation.h"
#include <amp.h>
#include <amp_math.h>
#include <string>
/*#include "thrust/host_vector.h"
#include "thrust/device_vector.h"
#include "thrust/generate.h"
#include "thrust/sort.h"
#include "thrust/copy.h"
#include <cstdlib>*/

using namespace blepo;
using namespace concurrency;

void CreateVideoFrames(string inFolder, string orderFile, string cloudFolder,string outFolder, int start, int end);
void CreateVideoFrames(string inFolder, int start, int end);
void FixVideoFrames(string inFolder, string orderFile, string vidFolder, int start, int end);
void FixVideoFrames(string depthFolder, string vidFolder, int start, int end);
void VideoSegment(string input, string output);
void VideoSegmentRealTime(string output1, string output2);
void CloudSegment(string input, string output);
void CloudSegmentImproved(string img, string in, string out);
void CloudSegmentImprovedNYU(string img, string depth, string truth, string out);
void RunOnNYUDataSet(string direc);
void CloudSegmentTime();
void CloudSegmentTime(string inFolder, string orderFile);
void CloudSegmentTime(string inFolder, int start, int end);
void CloudSegment();
void DisplayFigure(void* param);
void GetData(string out);
void CreateCloud(string folder, int start, int end);
void CreateCloudNew(string inFolder, string orderFile, string outFolder);
void CreatePointCloudFromRegisteredData(const ImgBgr &img, const ImgInt &depth, PointCloudBgr *cloud);
void CreatePointCloudFromRegisteredNYUData(const ImgBgr &img, const ImgInt &depth, PointCloudBgr *cloud);
void DisplayData();
void BadSegment(string file, float sigma_color, float sigma_depth, float c, float alpha, int min_size);
void Annotation(string imgFile, string cloudFile, string boundFile);
void FindBestCase(string imgFile, string cloudFile, string boundFile, string results, string outSeg, int method);
void FindBestSegmentation(string cloudFile, string boundFile, string results, string outSeg);
void ExtrapolateZeroData(string in, string out);
void DisplayOF(string in1, string in2, string out);
void CreateImageFromCloud(string in, string out);
void CreateImagesFromCloud(string in, string out1, string out2);
void SaveDepthData(string in, string out1, string out2);

void iSegment_graph(int num_vertices, int num_edges, Edge3D*& edges, float c, Universe *u);

int GraphSegmentation(
	const ImgBgr& img, 
	float sigma, 
	float c, 
	int min_size,
	ImgInt *out_labels, 
	ImgBgr *out_pseudocolors);
int GraphSegmentDepth(
	const ImgBgr& img, 
	const ImgGray& depth,
	float sigma, 
	float c, 
	int min_size,
	ImgInt *out_labels, 
	ImgBgr *out_pseudocolors);
void GraphSegmentVideo(
	vector<ImgBgr> &imgs, 
	float sigma, 
	float c, 
	int min_size,
	vector<ImgInt> *out_labels);
void GraphSegmentVideoOverlap(
	vector<ImgBgr> &imgs, 
	float sigma, 
	float c, 
	int min_size,
	vector<ImgInt> *out_labels, 
	vector<ImgBgr> *out_pseudocolors);
void GraphSegment(
	Volume<Bgr> &in, 
	float sigma, 
	float c, 
	int min_size,
	Volume<Bgr> *out_labels);
int SHGraphSegment(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *out_pseudocolors);
int SHGraphSegment(
	Volume<ColoredPoint> &in, 
	float sigma_depth,
	float sigma_color,
	float c_depth, 
	int depth_min_size,
	float c_color, 
	int color_min_size,
	Volume<Point3D<int>> *out_labels,
	Volume<ColoredPoint> *out_pseudocolors);
int GraphSegmentBad(
	PointCloud<Bgr> &in, 
	float sigma_color, 
	float sigma_depth,
	float c, 
	float alpha,
	int min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *pseudo_colors);
int SHGraphSegmentDepthOnly(
	PointCloud<Bgr> &in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *out_pseudocolors);
int SHGraphSegmentComplex(
	PointCloud<Bgr> *in, 
	float sigma_depth, 
	float c_depth, 
	int depth_min_size,
	float sigma_color,
	float c_color, 
	int color_min_size,
	PointCloud<int> *out_labels,
	PointCloud<Bgr> *out_pseudocolors);

void DisplayStuff(string file);

#define EPSILON 0.0001f
