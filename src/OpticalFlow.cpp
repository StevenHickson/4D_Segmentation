#include "stdafx.h"
#include "OpticalFlow.h"

using namespace blepo;

void ComputeOpticalFlow(PointCloud<Bgr> &past, PointCloud<Bgr> &current, FlowInfo *flow) {
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
		++pi1; ++pi2; ++pPast; ++pCurr;
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
			pO->di = *pU;
			pO->dj = *pV;
			if(pCurr->z == 0) {
				//let's assign all zeros
				pO->u = 0;
				pO->v = 0;
				pO->w = 0;
			} else {
				pO->u = *pU * pCurr->z * KINECT_FX_D;
				pO->v = *pV * pCurr->z * KINECT_FY_D;
				pO->w = (pCurr->z - past(blepo_ex::Clamp(int(i - pO->u),0,safeWidth), blepo_ex::Clamp(int(j - pO->v),0,safeHeight)).z);
			}
			++pO; ++pCurr; ++pU; ++pV;
		}
	}
}

//will not work inplace
template<typename T>
void iDownsample2x2(const PointCloud<T> &in, PointCloud<T> *out) {
//  iDownsample(img, 2, 2, out);
  out->Reset((in.Width()+1)/2, (in.Height()+1)/2);
  PointCloud<T>::ConstIterator p = in.Begin();
  PointCloud<T>::Iterator q = out->Begin();
  PointCloud<T>::Iterator rowend = q + out->Width();
  int skip = in.Width() % 2;
  while (q != out->End())
  {
    while (q != rowend)
    {
      *q++ = *p++;
      p++;
    }
    p += in.Width() - skip;
    rowend += out->Width();
  }
}

void Downsample2x2(const PointCloudBgr&    in, PointCloudBgr*    out) { iDownsample2x2(in, out); }