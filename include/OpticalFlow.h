/*
Copyright (C) 2013 Steven Hickson

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

#ifndef FLOW_H
#define FLOW_H

#include "blepo.h"

struct Flow {
public:
	//the pixel change in i and j
	int di,dj;
	//the meter change in x,y, and z
	float u,v,w;
};

typedef blepo::Image<Flow> FlowInfo;

void ComputeOpticalFlow(blepo::PointCloud<blepo::Bgr> &past, blepo::PointCloud<blepo::Bgr> &current, FlowInfo *flow);
void Downsample2x2(const blepo::PointCloud<blepo::Bgr> &in, blepo::PointCloud<blepo::Bgr> *out);

#endif //FLOW_H