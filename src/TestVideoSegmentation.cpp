
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
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "TestVideoSegmentation.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// The one and only application object

CWinApp theApp;

using namespace std;

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	HMODULE hModule = ::GetModuleHandle(NULL);

	if (hModule != NULL)
	{
		// initialize MFC and print and error on failure
		if (!AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
		{
			// TODO: change error code to suit your needs
			_tprintf(_T("Fatal Error: MFC initialization failed\n"));
			nRetCode = 1;
		}
		else
		{
			// TODO: code your application's behavior here.
			try {
				//These are all the different functions for test and paper purposes

				//CreateVideoFrames is used to create the supplementary video
				//CreateVideoFrames(argv[1],argv[2],argv[3],argv[4],atoi(argv[5]),atoi(argv[6]));
				//CreateVideoFrames(argv[1],atoi(argv[2]),atoi(argv[3]));

				/*if(argc > 5)
					FixVideoFrames(argv[1],argv[2],argv[3],atoi(argv[4]),atoi(argv[5]));
				else
					FixVideoFrames(argv[1],argv[2],atoi(argv[3]),atoi(argv[4]));*/

				/*ImgBgr img, colors;
				ImgInt labels;
				Load(argv[1],&img,true);
				Downsample2x2(img,&img);
				Downsample2x2(img,&img);
				FHGraphSegmentation(img,0.5f,500,50,&labels,&colors);
				Figure fig;
				fig.Draw(colors);
				EventLoop();*/

				//CreateImageFromCloud takes a 2d image of a pointcloud
				//CreateImageFromCloud(argv[1],argv[2]);
				//CreateImagesFromCloud(argv[1],argv[2],argv[3]);
				//DisplayOF(argv[1],argv[2],argv[3]);

				//Here are all the different segmentation variants, we will be using CloudSegmentImproved for the most part
				//VideoSegmentRealTime(argv[1],argv[2]);
				//CloudSegment(argv[1],argv[2]);
				//CloudSegmentTime(argv[1],argv[2]);
				//CloudSegmentTime(argv[1],atoi(argv[2]),atoi(argv[3]));
				//SaveDepthData(argv[1],argv[2],argv[3]);
				CloudSegmentTime();
				//CloudSegment();
				//VideoSegment(argv[1],argv[2]);
				//BadSegment(argv[1],atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atoi(argv[6]));
				//CloudSegmentImproved(image file, PointCloud ply file, output folder);
				//CloudSegmentImprovedNYU(argv[1],argv[2],argv[3],argv[4]);
				//RunOnNYUDataSet(argv[1]);

				//The DisplayData, GetData, and CreateCloud functions are used to grab frames
				//if(argc == 1)
				//DisplayData();
				//else if(argc == 2)
				//	GetData(argv[1]);
				//else {
				//	//CreateCloudNew(argv[1],argv[2],argv[3]);
				//	CreateCloud(argv[1], atoi(argv[2]), atoi(argv[3]));
				//}

				//This is the function that annotates and other helper functions for testing
				//Annotation(argv[1],argv[2],argv[3]);
				//FindBestSegmentation(argv[1],argv[2],argv[3],argv[4]);
				//FindBestCase(argv[1],argv[2],argv[3],argv[4],argv[5],atoi(argv[6]));
				//ExtrapolateZeroData(argv[1],argv[2]);

			} catch(blepo::Exception &e) {
				printf("ERROR: %s\n",e.m_message);
				cin.get();
			} catch(std::exception &e) {
				printf("ERROR: %s\n",e.what());
				cin.get();
			}
			printf("Done\n");
			cin.get();
		}
	}
	else
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: GetModuleHandle failed\n"));
		nRetCode = 1;
	}

	return nRetCode;
}

inline void CreateImageFromCloud(const PointCloudBgr &in, ImgBgr *out) {
	out->Reset(in.Width(),in.Height());
	PointCloudBgr::ConstIterator pIn = in.Begin();
	ImgBgr::Iterator pOut = out->Begin();
	while(pIn != in.End()) {
		*pOut = pIn->value;
		++pIn; ++pOut;
	}
}

inline void CreateImagesFromCloud(const PointCloudBgr &in, ImgBgr *out1, ImgFloat *out2) {
	out1->Reset(in.Width(),in.Height());
	out2->Reset(in.Width(),in.Height());
	PointCloudBgr::ConstIterator pIn = in.Begin();
	ImgBgr::Iterator pOut1 = out1->Begin();
	ImgFloat::Iterator pOut2 = out2->Begin();
	while(pIn != in.End()) {
		*pOut1 = pIn->value;
		*pOut2 = pIn->z;
		++pIn; ++pOut1; ++pOut2;
	}
}

inline void ColorizeImage(const ImgFloat &in, ImgBgr *out) {
	float min, max, center;
	MinMax(in,&min,&max);
	out->Reset(in.Width(),in.Height());
	//get the center of each one and make them red/blue
	center = (max - min) / 2.0f + min;
	ImgFloat::ConstIterator pI = in.Begin();
	ImgBgr::Iterator pO = out->Begin();
	while(pI != in.End()) {
		if(*pI == 0)
			*pO = Bgr::BLACK;
		else if(*pI < center)
			*pO = Bgr(255 * (*pI - min) / (center - min),0,0);
		else
			*pO = Bgr(0,0,255 * (*pI - min) / (center - min));
		++pI; ++pO;
	}
}

inline void ColorizeImage(const ImgInt &in, ImgBgr *out) {
	//int min, max, center;
	//MinMax(in,&min,&max);
	//out->Reset(in.Width(),in.Height());
	////get the center of each one and make them red/blue
	//center = (max - min) / 2 + min;
	//ImgInt::ConstIterator pI = in.Begin();
	//ImgBgr::Iterator pO = out->Begin();
	//while(pI != in.End()) {
	//	if(*pI == 0)
	//		*pO = Bgr::BLACK;
	//	else if(*pI < center)
	//		*pO = Bgr(255 * (*pI - min) / (center - min),0,0);
	//	else
	//		*pO = Bgr(0,0,255 * (*pI - min) / (center - min));
	//	++pI; ++pO;
	//}
	ImgInt img2;
	ImgGray gray;
	LinearlyScale(in, 0, 255, &img2);
	Convert(img2, &gray);
	Convert(gray, out);
}

void CreateImageFromCloud(string in, string out) {
	PointCloudBgr pc;
	ImgBgr img;
	pc.LoadCloudFromPly(in.c_str());
	CreateImageFromCloud(pc,&img);
	Save(img,out.c_str());
}

void CreateImagesFromCloud(string in, string out1, string out2) {
	PointCloudBgr pc;
	ImgBgr img, depthC;
	ImgFloat depth;
	pc.LoadCloudFromPly(in.c_str());
	CreateImagesFromCloud(pc,&img,&depth);
	ColorizeImage(depth,&depthC);
	Save(img,out1.c_str());
	Save(depthC,out2.c_str());
}

void SaveDepthData(string in, string out1, string out2) {
	ImgInt depth, depth2;
	ImgBgr color, gray;
	ImgGray tmp;
	Figure fig;
	LoadImgInt(in.c_str(),&depth);
	LinearlyScale(depth, 0, 255, &depth2);
	Convert(depth2,&tmp);
	Convert(tmp,&gray);
	ColorizeImage(depth,&color);
	Save(gray,out1.c_str());
	Save(color,out2.c_str());
	fig.Draw(depth);
	EventLoop();
}

inline void FillOutCloud(ImgBgr &img, PointCloudBgr &cloud) {
	PointCloudBgr::Iterator pCloud = cloud.Begin();
	ImgBgr::ConstIterator pColor = img.Begin();
	while(pCloud != cloud.End()) {
		if(pCloud->z == 0)
			pCloud->value = *pColor;
		++pCloud; ++pColor;
	}
}

inline void FillCloudHoles(PointCloudBgr &cloud) {
	PointCloudBgr::Iterator p = cloud.Begin();
	float cx_d = KINECT_CX_C, cy_d = KINECT_CY_D;
	int safeHeight = cloud.Height() - 1;
	for(int j = 0; j < cloud.Height(); j++) {
		for(int i = 0; i < cloud.Width(); i++) {
			if(p->z == 0 || p->z > 6.0f) {
				p->z = 6.0f;
				p->x = float(((float)i - cx_d) * p->z * KINECT_FX_D);
				p->y = float(((float)(safeHeight - j) - cy_d) * p->z * KINECT_FY_D);
			}
			++p;
		}
	}
}

inline void FillOutCloudAndFillHoles(ImgBgr &img, PointCloudBgr &cloud, float max = 6.0f) {
	PointCloudBgr::Iterator p = cloud.Begin();
	ImgBgr::ConstIterator pColor = img.Begin();
	float cx_d = KINECT_CX_C, cy_d = KINECT_CY_D;
	int safeHeight = cloud.Height() - 1;
	for(int j = 0; j < cloud.Height(); j++) {
		for(int i = 0; i < cloud.Width(); i++) {
			if(p->z == 0 || p->z > max) {
				p->z = max;
				p->x = float(((float)i - cx_d) * p->z / 525.0f);
				//should be safeHeight for my kinect data
				p->y = float((float)(j - cy_d) * p->z / 525.0f);
				p->value = *pColor;
			}
			++p; ++pColor;
		}
	}
}

void CreateVideoFrames(string inFolder, string orderFile, string cloudFolder,string outFolder, int start, int end) {
	Figure3D fig = Figure3D("Cloud",656,496);
	ImgBgr img, imgDepth, imgCloud, imgCloud2, imgFinal;
	ImgInt depth;
	PointCloudBgr pc;
	/*pc.LoadCloudFromPly(string(cloudFolder + "0.ply").c_str());
	Vector3d a,p,u;
	while(1) {
	fig.DrawCloud(pc);
	if(!(fig.camera->Aim == a && fig.camera->Pos == p && fig.camera->Up == u)) {
	printf("\nAim: ");
	fig.camera->Aim.print();
	printf("\nPos: ");
	fig.camera->Pos.print();
	printf("\nUp: ");
	fig.camera->Up.print();
	}
	}*/
	fig.camera->SetAim(Vector3d(-0.663973f,-0.196307f,-0.377329f));
	fig.camera->SetPos(Vector3d(-0.775641f,-0.53013f,0.765389f));
	fig.camera->SetUp(Vector3d(-0.0262554f,-0.960294f,0.277753f));
	int c = 0;
	ifstream file(string(inFolder + orderFile));
	string line, it1, rgbFile, it2, depthFile;
	while(getline(file, line) && c <= end) {
		if(c >= start) {
			stringstream linestream(line);
			stringstream num;
			linestream >> it1 >> rgbFile >> it2 >> depthFile;
			num << c;
			Load(string(inFolder + rgbFile).c_str(),&img,true);
			if(img.IsNull()) {
				BLEPO_ERROR("Can not open image");
			}
			LoadImgInt(string(inFolder + depthFile).c_str(),&depth);
			if(depth.IsNull()) {
				BLEPO_ERROR("Can not open depth");
			}
			pc.LoadCloudFromPly(string(cloudFolder + num.str() + ".ply").c_str());
			if(pc.IsNull()) {
				BLEPO_ERROR("Can not open cloud");
			}
			//FillCloudHoles(pc);
			ColorizeImage(depth,&imgDepth);
			fig.DrawCloud(pc);
			fig.Redraw();


			fig.GetImage(&imgCloud);
			FlipHorizontal(imgCloud,&imgCloud);
			//printf("W: %d, H: %d\n",imgCloud.Width(),imgCloud.Height());
			CreateImageFromCloud(pc,&imgCloud2);

			imgFinal.Reset(img.Width()*2,img.Height()*2);
			for(int j = 0; j < imgFinal.Height(); j++) {
				for(int i = 0; i < imgFinal.Width(); i++) {
					if(i < img.Width() && j < img.Height())
						imgFinal(i,j) = img(i,j);
					else if(i < img.Width())
						imgFinal(i,j) = imgCloud(i,j+16-img.Height());
					else if(j < img.Height())
						imgFinal(i,j) = imgDepth(i - img.Width(),j);
					else
						imgFinal(i,j) = imgCloud2(i-img.Width(),j-img.Height());
				}
			}
			Save(imgFinal,string(outFolder + num.str() + ".bmp").c_str());
		}
		c++;
	}

}

void CreateVideoFrames(string inFolder, int start, int end) {
	Figure3D fig = Figure3D("Cloud",656,496);
	ImgBgr img, imgDepth, imgCloud, imgCloud2, imgFinal;
	ImgInt depth;
	PointCloudBgr pc;
	/*pc.LoadCloudFromPly(string(inFolder + "cloud/" + "5.ply").c_str());
	Vector3d a,p,u;
	while(1) {
	fig.DrawCloud(pc);
	if(!(fig.camera->Aim == a && fig.camera->Pos == p && fig.camera->Up == u)) {
	printf("\nAim: ");
	fig.camera->Aim.print();
	printf("\nPos: ");
	fig.camera->Pos.print();
	printf("\nUp: ");
	fig.camera->Up.print();
	}
	}*/
	fig.camera->SetAim(Vector3d(0,0,0));
	fig.camera->SetPos(Vector3d(-0.126841f,-0.0795516f,1.13985f));
	fig.camera->SetUp(Vector3d(-0.0087428f,0.997564f,0.0692064f));
	int c = start;
	while(c <= end) {
		stringstream num;
		num << c;
		Load(string(inFolder +"rgb/" + num.str() + ".bmp").c_str(),&img,true);
		if(img.IsNull()) {
			BLEPO_ERROR("Can not open image");
		}
		LoadImgInt(string(inFolder +"depth/" + num.str() + ".dep").c_str(),&depth);
		if(depth.IsNull()) {
			BLEPO_ERROR("Can not open depth");
		}
		pc.LoadCloudFromPly(string(inFolder +"segments/" + num.str() + ".ply").c_str());
		if(pc.IsNull()) {
			BLEPO_ERROR("Can not open cloud");
		}
		//FillCloudHoles(pc);
		ColorizeImage(depth,&imgDepth);
		if(imgDepth.Width() == 320)
			Upsample(imgDepth,2,2,&imgDepth);
		fig.DrawCloud(pc);
		fig.Redraw();


		fig.GetImage(&imgCloud);
		//FlipHorizontal(imgCloud,&imgCloud);
		//printf("W: %d, H: %d\n",imgCloud.Width(),imgCloud.Height());
		CreateImageFromCloud(pc,&imgCloud2);
		if(imgCloud2.Width() == 320)
			Upsample(imgCloud2,2,2,&imgCloud2);

		imgFinal.Reset(img.Width()*2,img.Height()*2);
		for(int j = 0; j < imgFinal.Height(); j++) {
			for(int i = 0; i < imgFinal.Width(); i++) {
				if(i < img.Width() && j < img.Height())
					imgFinal(i,j) = img(i,j);
				else if(i < img.Width())
					imgFinal(i,j) = imgCloud(i,j+16-img.Height());
				else if(j < img.Height())
					imgFinal(i,j) = imgDepth(i - img.Width(),j);
				else
					imgFinal(i,j) = imgCloud2(i-img.Width(),j-img.Height());
			}
		}
		Save(imgFinal,string(inFolder + "video/" + num.str() + ".bmp").c_str());
		c++;
	}

}

void FixVideoFrames(string inFolder, string orderFile, string vidFolder, int start, int end) {
	ImgBgr imgDepth, imgFinal;
	ImgInt depth;

	int c = 0;
	ifstream file(string(inFolder + orderFile));
	string line, it1, rgbFile, it2, depthFile;
	while(getline(file, line) && c <= end) {
		if(c >= start) {
			stringstream linestream(line);
			stringstream num;
			linestream >> it1 >> rgbFile >> it2 >> depthFile;
			num << c;
			LoadImgInt(string(inFolder + depthFile).c_str(),&depth);
			if(depth.IsNull()) {
				BLEPO_ERROR("Can not open depth");
			}
			Load(string(vidFolder + num.str() + ".bmp").c_str(),&imgFinal);
			if(imgFinal.IsNull()) {
				BLEPO_ERROR("Can not open cloud");
			}
			//FillCloudHoles(pc);
			ColorizeImage(depth,&imgDepth);

			for(int j = 0; j < imgFinal.Height(); j++) {
				for(int i = 0; i < imgFinal.Width(); i++) {
					if(i >= imgDepth.Width() && j < imgDepth.Height())
						imgFinal(i,j) = imgDepth(i - imgDepth.Width(),j);
				}
			}
			Save(imgFinal,string(vidFolder + num.str() + ".bmp").c_str());
		}
		c++;
	}

}

void FixVideoFrames(string depthFolder, string vidFolder, int start, int end) {
	ImgBgr imgDepth, imgFinal;
	ImgInt depth;

	int c = start;
	while(c <= end) {
		stringstream num;
		num << c;
		LoadImgInt(string(depthFolder + num.str() + ".dep").c_str(),&depth);
		if(depth.IsNull()) {
			BLEPO_ERROR("Can not open depth");
		}
		Load(string(vidFolder + num.str() + ".bmp").c_str(),&imgFinal);
		if(imgFinal.IsNull()) {
			BLEPO_ERROR("Can not open cloud");
		}
		//FillCloudHoles(pc);
		ColorizeImage(depth,&imgDepth);

		for(int j = 0; j < imgFinal.Height(); j++) {
			for(int i = 0; i < imgFinal.Width(); i++) {
				if(i >= imgDepth.Width() && j < imgDepth.Height())
					imgFinal(i,j) = imgDepth(i - imgDepth.Width(),j);
			}
		}
		Save(imgFinal,string(vidFolder + num.str() + ".bmp").c_str());
		c++;
	}

}

void VideoSegment(string input, string output) {
	AVIReaderOpenCv cap1;
	Volume<Bgr> in,out;
	vector<ImgBgr> imgs;
	vector<ImgInt> labels;
	//get sequence of images
	int i;
	if(cap1.OpenFile(input.c_str())) {
		string direc(output.c_str());
		i = 0;
		while(i < 20) {
			ImgBgr img, tmp;
			if(!cap1.GrabNextFrame(&img)) {
				printf("Video Ended\n");
				break;
			}
			//Downsample(img,2,2,&img);
			imgs.push_back(img);
			i++;
		}
		//create temporal volume
		printf("%d frames\n",imgs.size());
		PerformanceTimer watch;
		srand(5);
		watch.Start();
		GraphSegmentVideo(imgs,0.5f,300,500*imgs.size(),&labels);
		watch.Stop();
		printf("Time = %lf seconds for %d frames\n",watch.Duration(),imgs.size());
		/*p = out.Begin();
		ImgBgr tmp = ImgBgr(out.Width(),out.Height());
		ImgBgr::Iterator pImg;*/
		for(int k = 0; k < out.Depth(); k++) {
			ImgBgr tmp;
			PseudoColor(labels[k],&tmp);
			stringstream num;
			num << k;
			Save(tmp,string(direc + num.str() + ".bmp").c_str());
		}
	} else {
		printf("ERROR: Could not open video\n");
		cin.get();
	}
}

void VideoSegmentRealTime(string output1, string output2) {
	KinectCapture kc;
	Volume<Bgr> color, world = Volume<Bgr>(640,480,10);
	//Volume<int> label;
	ImgBgr in;
	Volume<Bgr>::Iterator pW = world.Begin();
	ImgBgr::ConstIterator pC;
	int i;
	while(i < 10) {
		while(!kc.GetFrame(&in));
		pC = in.Begin();
		while(pC != in.End())
			*pW++ = *pC++;
		i++;
		//GraphSegment(in,0.25f,25,200,1.5f,500,&label,&color);
	}
	srand(8);
	PerformanceTimer watch;
	watch.Start();
	GraphSegment(world,0.5f,300,300,&color);
	watch.Stop();
	printf("Time = %lf seconds\n",watch.Duration());
	for(i = 0; i < 10; i++) {
		ImgBgr orig, segment;
		ExtractSlice(world,&orig,i);
		ExtractSlice(color,&segment,i);
		stringstream num;
		num << i;
		Save(orig,string(output1 + "orig" + num.str() + ".bmp").c_str());
		Save(segment,string(output2 + "segment" + num.str() + ".bmp").c_str());
	}
}

void CloudSegment(string input, string output) {
	vector<PointCloud<Bgr>> in;
	int i = 0, size = 0;
	try {
		WIN32_FIND_DATA FindFileData;
		HANDLE hFind;
		string first("\\*");
		hFind = FindFirstFile((input+first).c_str(), &FindFileData);
		bool downSample = true;
		if (hFind == INVALID_HANDLE_VALUE) 
		{
			printf_s("Invalid Directory, FindFirstFile failed (%d)\n", GetLastError());
			return;
		}
		do {
			try {
				PointCloud<Bgr> tmp;
				string file(FindFileData.cFileName);
				if(file != "." && file != "..") {
					tmp.LoadCloudFromPly((input + file).c_str());
					if(tmp.IsNull())
						throw blepo::Exception();
					in.push_back(tmp);
					size++;
				}
			} catch(blepo::Exception) {
				printf_s("Error: Could not Load Image: %s\n",FindFileData.cFileName);
				return;
			} catch(...) {
				printf_s("Error: Could not Load Image: %s\n",FindFileData.cFileName);
				return;
			}
		} while(FindNextFile(hFind,&FindFileData) && size < 10);
		FindClose(hFind);
	} catch(blepo::Exception) {
		printf_s("Error: Could not Load An Image\n");
		return;
	} catch(...) {
		printf_s("Error: Could not Load An Image\n");
		return;
	}
	Volume<ColoredPoint> color, world = Volume<ColoredPoint>(640,480,size);
	Volume<Point3D<int>> label;
	Volume<ColoredPoint>::Iterator pW = world.Begin();
	vector<PointCloud<Bgr>>::iterator pVector = in.begin();
	while(i < size) {
		PointCloud<Bgr>::Iterator pC = pVector->Begin();
		while(pC != pVector->End())
			*pW++ = *pC++;
		i++;
		pVector++;
	}
	srand(8);
	PerformanceTimer watch;
	watch.Start();
	SHGraphSegment(world,0.2f,0.5f,25,50*size,0.4f,60*size,&label,&color);
	watch.Stop();
	printf("Time = %lf seconds\n",watch.Duration());

	for(i = 0; i < size; i++) {
		PointCloud<Bgr> segment;
		//ExtractSlice(world,&orig,i);
		ExtractSlice(color,&segment,i);
		stringstream num;
		num << i;
		//orig.SaveCloudDataToPly(string(output1 + "orig" + num.str() + ".ply").c_str());
		segment.SaveCloudDataToPly(string(output + "segment" + num.str() + ".ply").c_str());
	}
}

void CloudSegmentTime() {
	srand(8);
	KinectCapture kc = KinectCapture(0,NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_320x240);
	kc.TiltKinect(0);
	Sleep(30);
	Figure3D fig1 = Figure3D("Cloud",800,600), fig2 = Figure3D("Segment",800,600);
	PointCloud<Bgr> cloud, segment;
	PointCloud<int> label;
	Segment3D stseg;
	PerformanceTimer time;
	ImgBgr img;
	ImgInt depth;
	while(1) {
		kc.GetFrame(&img,1000);
		kc.GetDepth(&depth,1000);
		if(!img.IsNull() && !depth.IsNull()) {
			kc.GetPointCloudFromData(img,depth,&cloud,true,false,false);
			time.Start();
			stseg.AddSlice(cloud,2.5f,700,800,0.8f,700,800,&label,&segment);
			time.Stop();
			printf("Time (ms): %lf\n", time.Duration() * 1000);
			//PseudoColor(label,&segment);
			//segment.SaveCloudDataToPly("test.ply");
		}
		fig1.SetPoints(cloud);
		fig1.Redraw();
		fig2.SetPoints(segment);
		fig2.Redraw();
		label.ReleaseData();
		segment.ReleaseData();
	}
}

void CloudSegmentTime(string inFolder, string orderFile) {
	int i = 0;
	Segment4DBig stseg;
	PerformanceTimer time;
	float timeTot = 0.0f;
	try {
		ifstream file(string(inFolder + orderFile));
		string line, it1, rgbFile, it2, depthFile;
		while(getline(file, line)) {
			PointCloud<Bgr> cloud;
			ImgBgr img;
			stringstream linestream(line);
			stringstream num;
			num << i;
			linestream >> it1 >> rgbFile >> it2 >> depthFile;
			printf("Opening cloud, ");
			cloud.LoadCloudFromPly((inFolder + "cloud/" + num.str() + ".ply").c_str());
			if(cloud.IsNull()) {
				BLEPO_ERROR("Can not open cloud");
			}
			//printf("opening image\n");
			Load(string(inFolder + rgbFile).c_str(),&img,true);
			if(img.IsNull()) {
				BLEPO_ERROR("Can not open image");
			}
			printf("filling out cloud, ");
			ColoredPoint min, max;
			MinMax(cloud,&min,&max);
			float newMax = max.z * 1.25f;
			FillOutCloudAndFillHoles(img,cloud, newMax);
			time.Start();
			//printf("adding slice\n");
			//These values are super important. Need to figure out how to specify each one via SCIENCE!
			//int ret = stseg.AddSlice(cloud,2.5f,1600, 4500, 0.8f, 600, 4500);
			int ret = stseg.AddSlice(cloud, 2.5f, 800, 6000, newMax, 0.8f, 800, 6000);
			//stseg.AddSlice(cloud,2.5f,1200.0f,1500,0.8f,900.0f,1000,&label,&segment);
			time.Stop();
			timeTot += time.Duration();
			printf("Iter: %d\tTime (s): %lf\n", i, time.Duration());
			if(ret > 0) {
				for(int c = 0; c < NUM_FRAMES; c++) {
					stringstream num2;
					num2 << (i - NUM_FRAMES + c);
					printf("Saving %s\t", num2.str().c_str());
					stseg.labels_colored[c].SaveCloudDataToPly(string(inFolder + "segments/" + num2.str() + ".ply").c_str());
					ImgBgr tmp;
					CreateImageFromCloud(stseg.labels_colored[c],&tmp);
					Save(tmp,string(inFolder + "segments/" + num2.str() + ".bmp").c_str());
				}
				printf("\n");
			}
			/*if(i == 1)
			cin.get();
			else if(i == 2)
			cin.get();*/
			i++;
		}
		file.close();
	} catch(blepo::Exception &e) {
		printf_s("Error: %s\n",e.m_message);
		return;
	} catch(exception &e) {
		printf_s("Error: %s\n",e.what());
		return;
	} catch(...) {
		printf_s("Error: Unknown\n");
		return;
	}
	printf("Total time (s): %lf\n",timeTot);
}

inline void ExtractDepthAndColor(const PointCloud<Bgr> &cloud, ImgInt *depth, ImgBgr *color) {
	depth->Reset(cloud.Width(), cloud.Height());
	color->Reset(cloud.Width(), cloud.Height());
	PointCloudBgr::ConstIterator pI = cloud.Begin();
	ImgInt::Iterator pD = depth->Begin();
	ImgBgr::Iterator pC = color->Begin();
	while(pI != cloud.End()) {
		*pD++ = blepo_ex::Round(1000*pI->z);
		*pC++ = pI->value;
		++pI;
	}
}

void CloudSegmentTime(string inFolder, int start, int end) {
	srand(time(NULL));
	int i = start;
	Segment4DBig stseg;
	PerformanceTimer time;
	//float timeTot = 0.0f;
	try {
		PointCloud<Bgr> cloud;
		while(i <= end) {
			//ImgBgr img;
			stringstream num;
			num << i;
			printf("Opening cloud, ");
			cloud.LoadCloudFromPly((inFolder + "cloud/" + num.str() + ".ply").c_str());
			if(cloud.IsNull()) {
				BLEPO_ERROR("Can not open cloud");
			}
			//printf("opening image\n");
			/*Load(string(inFolder + "rgb/" + num.str() + ".bmp").c_str(),&img,true);
			if(img.IsNull()) {
			BLEPO_ERROR("Can not open image");
			}*/
			//printf("filling out cloud\n");
			ColoredPoint min, max;
			MinMax(cloud,&min,&max);
			float newMax = max.z * 1.25f;
			/*ImgInt depth; ImgBgr color;
			ExtractDepthAndColor(cloud,&depth,&color);
			Figure colorFig, depthFig;
			depthFig.Draw(depth);
			colorFig.Draw(color);
			EventLoop();
			while(!depthFig.IsClosed());*/
			//FillOutCloudAndFillHoles(img,cloud, newMax);
			time.Start();
			//printf("adding slice\n");
			//These values are super important. Need to figure out how to specify each one via SCIENCE!
			//int ret = stseg.AddSlice(cloud,2.5f,700,800, newMax ,0.8f,700,800);
			int ret = stseg.AddSlice(cloud,1.5f, 2500, 6000, newMax, 0.8f, 900, 6000);
			//int ret = stseg.AddSlice(cloud,2.5f, 1500, 4000, newMax, 0.8f, 900, 4000);
			//stseg.AddSlice(cloud,2.5f,1200.0f,1500,0.8f,900.0f,1000,&label,&segment);
			time.Stop();
			printf("Iter: %d\tTime (s): %lf\n", i, time.Duration());
			if(ret > 0) {
				for(int c = 0; c < NUM_FRAMES; c++) {
					stringstream num2;
					num2 << (i - NUM_FRAMES + c);
					printf("Saving %s\t", num2.str().c_str());
					stseg.labels_colored[c].SaveCloudDataToPly(string(inFolder + "segments/" + num2.str() + ".ply").c_str());
					ImgBgr tmp;
					CreateImageFromCloud(stseg.labels_colored[c],&tmp);
					Save(tmp,string(inFolder + "segments/" + num2.str() + ".bmp").c_str());
				}
				printf("\n");
			}
			/*if(i == 1)
			cin.get();
			else if(i == 2)
			cin.get();*/
			i++;
			cloud.ReleaseData();
		}
	} catch(blepo::Exception &e) {
		printf_s("Error: %s\n",e.m_message);
		return;
	} catch(...) {
		printf_s("Error: Unknown\n");
		return;
	}
	//printf("Total time (s): %lf\n",timeTot);
}


void CloudSegment() {
	srand(time(NULL));
	//Figure3D fig = MyFigureGlut("Cloud",900,600);
	//HANDLE h1 = (HANDLE)_beginthread(&DisplayFigure,0,&in);
	//Mutex m;
	//HANDLE h2 = (HANDLE)_beginthread(&DisplayFigure,0,&color);
	KinectCapture kc = KinectCapture(0,NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_320x240);
	kc.TiltKinect(10);
	PerformanceTimer watch;
	Figure3D fig2 = Figure3D("Segment",800,600);
	Figure3D fig1 = Figure3D("Cloud",900,600);
	PointCloudBgr orig,segment;
	PointCloudInt label;
	ImgBgr img;
	ImgInt depth;
	int segments = 0;
	Segment3D seg;
	while(1) {
		//kc.GetData(&img,&depth,&orig);
		kc.GetFrame(&img,1000);
		kc.GetDepth(&depth,1000);
		if(!img.IsNull() && !depth.IsNull()) {
			kc.GetPointCloudFromData(img,depth,&orig,true,false,false);
			watch.Start();
			segments = SHGraphSegment(orig,0.25f,30,300,0.8f,1.5f,300,&label,&segment);
			//segments = SHGraphSegment(orig,0.25f,25,50,0.5f,0.4f,60,&label,&segment);
			RegionTree3D tree;
			tree.Create(orig,label,segments,0);
			//tree.PropagateRegionHierarchy(75);
			//tree.ImplementSegmentation(0.7f);
			watch.Stop();
			printf("Graph Seg Time: %lf ms\n", watch.Duration() * 1000.0f);
			segment.ReleaseData();
			PseudoColor(label,&segment);
			fig1.SetPoints(orig);
			fig1.Redraw();
			fig2.SetPoints(segment);
			fig2.Redraw();
			segment.ReleaseData();
			label.ReleaseData();
			orig.ReleaseData();
			tree.Release();
		}
	}
}

void CloudSegmentImprovedNYU(string img, string depth, string truth, string out) {
	srand(time(NULL));
	PointCloudBgr orig,segment;
	PointCloudInt label;
	ImgBgr origImg;
	ImgInt depthImg;
	int segments;
	Load(img.c_str(),&origImg,true);
	LoadImgInt(depth.c_str(),&depthImg);
	CreatePointCloudFromRegisteredNYUData(origImg, depthImg, &orig);
	segments = SHGraphSegment(orig,2.5f,500.0f,200,0.8f,200.0f,200,&label,&segment);
	segment.SaveCloudDataToPly(string(out + "orig.ply").c_str());
	RegionTree3D tree;
	tree.Create(orig,label,segments,0);
	tree.PropagateRegionHierarchy(100);
	float i = 0.90f;
	while(i > 0.00f) {
		stringstream num;
		num << blepo_ex::Round(i * 100);
		tree.ImplementSegmentation(i);
		segment.ReleaseData();
		PseudoColor(label,&segment);
		segment.SaveCloudDataToPly(string(out + "seg" + num.str() + ".ply").c_str());
		i -= 0.05f;
	}
}

void RunOnNYUDataSet(string direc) {
	srand(time(NULL));
	PointCloudBgr orig,segment;
	PointCloudInt label;
	ImgBgr origImg, segmentImg;
	ImgInt depthImg;
	int segments;
	for(int i = 1; i < 1450; i++) {
		stringstream num;
		num << i;
		Load(string(direc + "rgb\\" + num.str() + ".bmp").c_str(),&origImg,true);
		LoadImgInt(string(direc + "depth\\" + num.str() + ".dep").c_str(),&depthImg);
		CreatePointCloudFromRegisteredNYUData(origImg, depthImg, &orig);
		segments = SHGraphSegment(orig,2.5f,500.0f,200,0.8f,200.0f,200,&label,&segment);
		RegionTree3D tree;
		tree.Create(orig,label,segments,0);
		tree.PropagateRegionHierarchy(100);
		tree.ImplementSegmentation(0.45f);
		segment.ReleaseData();
		PseudoColor(label,&segment);
		segment.SaveCloudDataToPly(string(direc + "results\\" + num.str() + ".ply").c_str());
		CreateImageFromCloud(segment,&segmentImg);
		Save(segmentImg,string(direc + "results\\" + num.str() + ".bmp").c_str());
		orig.ReleaseData();
		segment.ReleaseData();
		label.ReleaseData();
		origImg.Reset();
		depthImg.Reset();
		segmentImg.Reset();
		segments = 0;
	}
}

void CloudSegmentImproved(string img, string in, string out) {
	srand(time(NULL));
	PointCloudBgr orig,segment;
	PointCloudInt label;
	PerformanceTimer watch;
	ImgBgr origImg;
	int segments;
	cin.get();
	orig.LoadCloudFromPly(in.c_str());
	Load(img.c_str(),&origImg,true);
	FillOutCloudAndFillHoles(origImg,orig);
	watch.Start();
	segments = SHGraphSegment(orig,1.5f,500.0f,100,0.8f,50.0f,25,&label,&segment);
	watch.Stop();
	printf("Graph Seg Time: %lf s\n", watch.Duration());
	segment.SaveCloudDataToPly(string(out + "orig.ply").c_str());
	watch.Start();
	RegionTree3D tree;
	tree.Create(orig,label,segments,0);
	watch.Stop();
	printf("Tree Creation Time: %lf s\n", watch.Duration());
	watch.Start();
	tree.PropagateRegionHierarchy(75);
	watch.Stop();
	printf("Propagation time: %lf s\n", watch.Duration());
	cin.get();
	float i = 0.90f;
	while(i > 0.00f) {
		stringstream num;
		num << blepo_ex::Round(i * 100);
		watch.Start();
		tree.ImplementSegmentation(i);
		watch.Stop();
		printf("Seg %s% - time: %lf s\n", num.str(), watch.Duration());
		segment.ReleaseData();
		PseudoColor(label,&segment);
		segment.SaveCloudDataToPly(string(out + "seg" + num.str() + ".ply").c_str());
		i -= 0.05f;
	}
}

void DisplayFigure(void* param) {
	stringstream name;
	name << "Cloud";
	name << GetCurrentThreadId();
	Figure3D fig = Figure3D(name.str().c_str(),900,600);
	Mutex m;
	while(1) {
		PointCloud<Bgr> *cloud = (PointCloud<Bgr> *) param;
		if(!cloud->IsNull()) {
			m.Lock();
			fig.SetPoints(*cloud);
			m.Unlock();
			fig.Redraw();
		}
		SleepEx(30,true);
	}
}

void GetData(string out) {
	KinectCapture kc = KinectCapture(0,NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_320x240);
	SaveKinectData save(&kc,out);
	save.Start();
	cin.get();
	save.Stop();
}

void CreateCloud(string folder, int start, int end) {
	int i = start;
	ImgBgr img;
	ImgInt depth;
	PointCloudBgr cloud;
	KinectCapture *kc = NULL;
	while(i <= end) {
		stringstream num;
		num << i;
		Load(string(folder + "rgb/" + num.str() + ".bmp").c_str(),&img);
		LoadImgInt(string(folder + "depth/" + num.str() + ".dep").c_str(),&depth);
		if(depth.Width() == 320)
			kc = new KinectCapture(0,NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_320x240);
		else
			kc = new KinectCapture();
		kc->GetPointCloudFromData(img,depth,&cloud,true,false,false);
		cloud.SaveCloudDataToPly(string(folder + "cloud/" + num.str() + ".ply").c_str());
		cloud.ReleaseData();
		i++;
	}
}

void CreatePointCloudFromRegisteredData(const ImgBgr &img, const ImgInt &depth, PointCloudBgr *cloud) {
	assert(!img.IsNull() && !depth.IsNull());
	//take care of old cloud to prevent memory leak/corruption
	if (cloud != NULL && cloud->size() > 0) {
		cloud->ReleaseData();
	}
	cloud->Reset(img.Width(),img.Height());
	Set(cloud, Point3D<Bgr>(0.0f,0.0f,0.0f,Bgr::BLACK,false));
	PointCloud<Bgr>::Iterator pCloud = cloud->Begin();
	ImgInt::ConstIterator pDepth = depth.Begin();
	ImgBgr::ConstIterator pImg = img.Begin();
	for(int j = 0; j < img.Height(); j++) {
		for(int i = 0; i < img.Width(); i++) {
			Point3D<bool> loc = Point3D<bool>(0,0,0,false,false);
			loc.z = *pDepth / 1000.0f;
			loc.x = float((i - 319.5f) * loc.z / 525.0f);
			loc.y = float((j - 239.5f) * loc.z / 525.0f);
			*pCloud = Point3D<Bgr>(loc.x,loc.y,loc.z,*pImg,*pDepth!=0);
			pImg++; pDepth++; pCloud++;
		}
	}
}

void CreatePointCloudFromRegisteredNYUData(const ImgBgr &img, const ImgInt &depth, PointCloudBgr *cloud) {
	assert(!img.IsNull() && !depth.IsNull());
	//take care of old cloud to prevent memory leak/corruption
	if (cloud != NULL && cloud->size() > 0) {
		cloud->ReleaseData();
	}
	cloud->Reset(img.Width(),img.Height());
	Set(cloud, Point3D<Bgr>(0.0f,0.0f,0.0f,Bgr::BLACK,false));
	PointCloud<Bgr>::Iterator pCloud = cloud->Begin();
	ImgInt::ConstIterator pDepth = depth.Begin();
	ImgBgr::ConstIterator pImg = img.Begin();
	for(int j = 0; j < img.Height(); j++) {
		for(int i = 0; i < img.Width(); i++) {
			Point3D<bool> loc = Point3D<bool>(0,0,0,false,false);
			loc.z = *pDepth / 1000.0f;
			loc.x = float((i - 3.1304475870804731e+02) * loc.z / 5.8262448167737955e+02);
			loc.y = float((j - 2.3844389626620386e+02) * loc.z / 5.8269103270988637e+02);
			*pCloud = Point3D<Bgr>(loc.x,loc.y,loc.z,*pImg,*pDepth!=0);
			pImg++; pDepth++; pCloud++;
		}
	}
}

void CreateCloudNew(string inFolder, string orderFile, string outFolder) {
	int i = 0;
	ImgBgr img;
	ImgInt depth;
	PointCloudBgr cloud;
	//KinectCapture kc;
	ifstream file(string(inFolder + orderFile));
	string line, it1, rgbFile, it2, depthFile;
	while(getline(file, line)) {
		stringstream linestream(line);
		stringstream num;
		num << i;
		linestream >> it1 >> rgbFile >> it2 >> depthFile;
		Load(string(inFolder + rgbFile).c_str(),&img,true);
		LoadImgInt(string(inFolder + depthFile).c_str(),&depth);
		CreatePointCloudFromRegisteredData(img,depth,&cloud);
		cloud.SaveCloudDataToPly(string(outFolder + num.str() + ".ply").c_str());
		cloud.ReleaseData();
		i++;
	}
}

void DisplayData() {
	KinectCapture kc = KinectCapture(0,NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_320x240);
	Figure depthFig, imgFig;
	Figure3D cloudFig = Figure3D("Cloud",400,300);
	while(1) {
		ImgBgr img;
		ImgInt depth;
		PointCloudBgr cloud;
		kc.GetData(&img,&depth,&cloud,1000);
		kc.GetDepth(&depth,1000);
		kc.GetFrame(&img,1000);
		kc.GetPointCloudFromData(img,depth,&cloud,true,false,false);
		imgFig.Draw(img);
		depthFig.Draw(depth);
		cloudFig.DrawCloud(cloud);
	}
}

void LoadCloudAndMeshFromPly(const char *fileName, PointCloudBgr &cloud, PointCloudBgr &mesh) 
{
	ifstream data;
	data.open(fileName);
	if(!data.is_open()) 
	{
		BLEPO_ERROR("Error, cannot open file");
	} 
	else 
	{
		string dont_care;
		int num_vertices = 0, num_faces = 0;

		// read header
		int num_properties = 0;
		string property_list;
		do {
			getline(data, dont_care);
			if(data.eof())
			{
				BLEPO_ERROR("Error processing PointCloudBgr");
			}

			// read number of vertices
			const char str_ev[] = "element vertex ", str_ev2[] = "element face ";
			const int ev_len = strlen(str_ev), ev_len2 = strlen(str_ev2);
			if (dont_care.compare(0, ev_len, "element vertex ") == 0) 
			{
				const char *tmp = dont_care.c_str();
				num_vertices = atoi(tmp + ev_len);
			}
			if (dont_care.compare(0, ev_len2, "element face ") == 0) 
			{
				const char *tmp = dont_care.c_str();
				num_faces = atoi(tmp + ev_len2);
			}	

			// read number of vertices
			if (dont_care.compare(0, strlen("property"), "property") == 0)
			{
				char s1[1000], s2[1000], s3[1000];
				sscanf(dont_care.c_str(), "%s %s %s", s1, s2, s3);
				property_list += s3;
				property_list += ' ';
				num_properties++;
			}	

			//          //printf_s("dont_care: %s\n",dont_care);
		} while (dont_care != "end_header");

		//AfxMessageBox(property_list);

		if (num_vertices == 0 || num_faces == 0)
		{
			BLEPO_ERROR("Error, empty PointCloudBgr");
		}
		if (property_list == "x y z diffuse_red diffuse_green diffuse_blue list uchar int vertex_index ")
		{ 
			// If your PLY format was not recognized, should be easy to add another 'else if' clause to handle it
			BLEPO_ERROR("Unrecognized PLY format");
		}

		// load rest of file
		float x,y,z;
		Bgr color;
		int i = 0;
		const int max_no_chars_rgb = 3;
		const int max_no_chars_xyz = 9;
		cloud.Resize(num_vertices);
		PointCloudBgr::Iterator p = cloud.Begin();
		do 
		{ // this code is dumb; it only works for certain xyzrgb orders
			string str;
			getline(data, str);
			int r, g, b;
			sscanf(str.c_str(), "%f %f %f %d %d %d", &x, &y, &z, &r, &g, &b); 
			color.r = r;
			color.g = g;
			color.b = b;

			*p++ = ColoredPoint(x, y, z, color);
			i++;
		} while (!data.eof() && i < num_vertices);
		i = 0;
		mesh.Resize(3*num_faces);
		p = mesh.Begin();
		do
		{
			string str;
			getline(data, str);
			int dc, p1, p2, p3;
			sscanf(str.c_str(), "%d %d %d %d", &dc, &p1, &p2, &p3); 
			//Add to mesh
			*p++ = cloud[p1];
			*p++ = cloud[p2];
			*p++ = cloud[p3];
			i++;
		} while(!data.eof() && i < num_faces);

		data.close();
	}
}

void DisplayStuff(string file) {
	PointCloudBgr pc,mesh;
	LoadCloudAndMeshFromPly(file.c_str(),pc,mesh);
	Figure3D fig = Figure3D("Cloud",800,600);
	//fig.DrawCloud(pc);
	while(1) {
		//fig.DrawCloud(pc);
		fig.DrawMesh(mesh);
		fig.Redraw();
	}
	cin.get();
}

void BadSegment(string file, float sigma_color, float sigma_depth, float c, float alpha, int min_size) {
	try {
		Figure3D fig = Figure3D("Cloud",800,600);
		Figure3D fig2 = Figure3D("Segment",800,600);
		PointCloudBgr pc,segment;
		PointCloudInt junk;
		pc.LoadCloudFromPly(file.c_str());
		GraphSegmentBad(pc,sigma_color,sigma_depth,c,alpha,min_size,&junk,&segment);
		while(1) {
			fig.DrawCloud(pc);
			fig2.DrawCloud(segment);
		}
	} catch(blepo::Exception &e) {
		printf("Error: %s\n",e.m_message);
	}
}

void Annotation(string imgFile, string cloudFile, string boundFile) {
	ImgBgr img;
	ImgBinary bound;
	PointCloudBgr cloud;
	Load(imgFile.c_str(),&img,true);
	cloud.LoadCloudFromPly(cloudFile.c_str());
	bound.Reset(img.Width(),img.Height());
	Set(&bound,false);
	Figure fig;
	fig.Draw(img);
	Point click1 = Point(-1,-1), click2 = Point(-1,-1);
	Figure::WhichButton button = fig.TestMouseClick(&click2);
	bool buttonPressed = false;
	while(button != Figure::MC_RIGHT) {
		if(button == Figure::MC_LEFT) {
			printf("Button pressed\n");
			if(buttonPressed) {
				printf("Drawing from %d,%d to %d,%d\n",click1.x,click1.y,click2.x,click2.y);
				DrawLine(click1,click2,&img,Bgr::RED,1);
				DrawLine(click1,click2,&bound,true,1);
				click1 = click2 = Point(-1,-1);
			}
			buttonPressed = !buttonPressed;
			click1 = click2;
		}
		fig.Draw(img);
		button = fig.TestMouseClick(&click2);
	}
	Save(bound,boundFile.c_str());
}

void FindBoundaries(PointCloudBgr &in, PointCloudBgr &mask, ImgBinary &out) {
	out.Reset(in.Width(),in.Height());
	PointCloudBgr::ConstIterator pIn = in.Begin();
	PointCloudBgr::ConstIterator pMask = mask.Begin();
	ImgBinary::Iterator pOut = out.Begin();
	int width = in.Width() - 2, height = in.Height() - 2;
	for(int j = 0; j < in.Height(); j++) {
		for(int i = 0; i < in.Width(); i++) {
			Bgr current = pMask->value;
			if(i > 1 && j > 1 && i < width && j < height && (current != (pMask+1)->value || current != (pMask+in.Width())->value || current != (pMask+in.Width()+1)->value || current != (pMask-in.Width()+1)->value))
				*pOut++ = true;
			else
				*pOut++ = false;
			pIn++;
			pMask++;
		}
	}
}

inline float FindError(ImgInt img1, ImgInt img2) {
	float ret = 0;
	ImgInt::ConstIterator p1 = img1.Begin(), p2 = img2.Begin();
	while(p1 != img1.End()) {
		ret += blepo_ex::Abs(*p1 - *p2);
		++p1; ++p2;
	}
	ret /= float(img1.Width() * img1.Height());
	return ret;
}

//Find Best case has editable bounds that the algorithm compares within with the annotated data until it finds the best match
void FindBestCase(string imgFile, string cloudFile, string boundFile, string results, string outSeg, int method) {
	PointCloudBgr cloud, labels, saveLabels;
	PointCloudInt junk;
	ImgBinary segment, boundSave;
	ImgGray tmp;
	ImgInt cham, truthCham;
	ImgBgr orig,segDisp;
	Load(boundFile.c_str(),&tmp);
	Chamfer(tmp,&truthCham);
	cloud.LoadCloudFromPly(cloudFile.c_str());
	Load(imgFile.c_str(),&orig,true);
	//FillOutCloud(orig,cloud);
	//FillOutCloudAndFillHoles(orig,cloud);
	FILE *fp;
	fopen_s(&fp,results.c_str(),"w");
	int min_error = -1;
	Figure fig1, fig2;
	Figure3D fig3 = Figure3D("Cloud",800,600);
	fig1.Draw(truthCham);
	RegionTree3D tree;
	float start, end, iter, save, saveIter;
	if(method == 0) {
		start = 0.0f; 
		end = 1.0f; 
		iter = 0.1f; //iter = 3.0f;
	} else if(method == 1) {
		start = 0.0f;
		end = 1000.0f;
		iter = 25.0f;
	} else if(method == 2) {
		start = 0.1f;
		end = 0.9f;
		iter = 0.05f;
		int numsegments = SHGraphSegment(cloud,1.0f,2000,2000,0.8f,200.0f,200,&junk,&labels);
		tree.Create(cloud,junk,numsegments,0);
		tree.PropagateRegionHierarchy(75);
	} else if(method == 3) {
		start = 0.1f;
		end = 0.9f;
		iter = 0.05f;
		int numsegments = GraphSegmentBad(cloud,0.4f,0.5f,10,0.15f,60,&junk,&labels);
		tree.Create(cloud,junk,numsegments,0);
		tree.PropagateRegionHierarchy(100);
	} else if(method == 4) {
		start = 0.0f;
		end = 1000.0f;
		iter = 25.0f;
	} else if(method == 5) {
		start = 0.0f;
		end = 500.0f;
		iter = 25.0f;
	} else if(method == 6) {
		start = 0.25f;
		end = 4.0f;
		iter = 0.25f;
	}
	save = start;
	saveIter = end / 10.0f;
	int saveNum = 0;
	for(float alpha = start; alpha < end; alpha += iter) {
		//do segmentation
		if(method == 0) 
			GraphSegmentBad(cloud,0.8f,1.5f,450,alpha,450,&junk,&labels);
		else if(method == 1)
			SHGraphSegment(cloud,2.5f,450,450,0.8f,alpha,450,&junk,&labels);
		else if(method == 4)
			SHGraphSegmentDepthOnly(cloud,2.5f,alpha,200,2.5f,alpha,100,&junk,&labels);
		else if(method == 5)
			GraphSegmentBad(cloud,0.5f,0.5f,alpha,0.15f,100,&junk,&labels);
		else if(method == 6)
			SHGraphSegmentDepthOnly(cloud,alpha,450,450,alpha,600,600,&junk,&labels);
		else {
			tree.ImplementSegmentation(alpha);
			PseudoColor(junk,&labels);
		}
		if(alpha > save) {
			char buff[500];
			sprintf(buff,"%s_%d.bmp",outSeg.c_str(),saveNum);
			CreateImageFromCloud(labels,&segDisp);
			Save(segDisp,buff);
			saveNum++;
			save += saveIter;
		}
		//compare against the ground truth
		FindBoundaries(cloud,labels,segment);
		Convert(segment,&tmp);
		Chamfer(tmp,&cham);
		float error = FindError(cham,truthCham);
		fprintf(fp,"%f: %f\n",alpha,error);
		if(error > 0 && (error < min_error || min_error == -1)) {
			saveLabels = labels;
			Convert(tmp,&boundSave);
			min_error = error;
		}
		fig2.Draw(cham);
		fig3.DrawCloud(labels);
		labels.ReleaseData();
		//cin.get();
	}
	fclose(fp);
	char buff[500];
	sprintf(buff,"%s.ply",outSeg.c_str());
	saveLabels.SaveCloudDataToPly(buff);
	sprintf(buff,"%s_%d.bmp",outSeg.c_str(),saveNum);
	CreateImageFromCloud(saveLabels,&segDisp);
	Save(segDisp,buff);
	sprintf(buff,"%s_outline.bmp",outSeg.c_str());
	Save(boundSave,buff);
}

void FindBestSegmentation(string cloudFile, string boundFile, string results, string outSeg) {
	PointCloudBgr cloud, labels, saveLabels;
	PointCloudInt junk;
	ImgBinary segment, boundSave;
	ImgGray tmp;
	ImgInt cham, truthCham;
	ImgBgr segDisp;
	Load(boundFile.c_str(),&tmp);
	Chamfer(tmp,&truthCham);
	cloud.LoadCloudFromPly(cloudFile.c_str());
	FILE *fp;
	fopen_s(&fp,results.c_str(),"w");
	int min_error = -1;
	//Figure3D fig = Figure3D("cloud",800,600);
	for(float i = 0.0f; i < 700.0f; i += 10.0f) {
		for(float j = 0.0f; j < 700.0f; j += 10.0f) {
			int numsegments = SHGraphSegment(cloud,2.5f,i,100,0.8f,j,60,&junk,&labels);
			RegionTree3D tree;
			tree.Create(cloud,junk,numsegments,0);
			tree.PropagateRegionHierarchy(75);
			for(float k = 0.0f; k < 0.75f; k += 0.05f) {
				tree.ImplementSegmentation(k);
				PseudoColor(junk,&labels);
				//compare against the ground truth
				FindBoundaries(cloud,labels,segment);
				Convert(segment,&tmp);
				Chamfer(tmp,&cham);
				int error = FindError(cham,truthCham);
				fprintf(fp,"%f,%f,%f: %d\n",i,j,k,error);
				if(error > 0 && (error < min_error || min_error == -1)) {
					saveLabels.ReleaseData();
					saveLabels = labels;
					min_error = error;
					Convert(tmp,&boundSave);
				}
				//fig.DrawCloud(labels);
				//labels.ReleaseData();
				//segment.Reset();
				//tmp.Reset();
				//junk.ReleaseData();
			}
			junk.ReleaseData();
			tree.Release();
		}
	}
	fclose(fp);
	char buff[500];
	sprintf(buff,"%s.ply",outSeg.c_str());
	saveLabels.SaveCloudDataToPly(buff);
	sprintf(buff,"%s_img.bmp",outSeg.c_str());
	CreateImageFromCloud(saveLabels,&segDisp);
	Save(segDisp,buff);
	sprintf(buff,"%s_outline.bmp",outSeg.c_str());
	Save(boundSave,buff);
}

void ExtrapolateZeroData(string in, string out) {
	PointCloudBgr pcIn, pcOut;
	pcIn.LoadCloudFromPly(in.c_str());
	pcOut.Reset(pcIn.Width(),pcIn.Height());
	PointCloudBgr::Iterator p1 = pcIn.Begin(), p2 = pcOut.Begin();
	int safeDepthHeight = pcIn.Height() - 1;
	for(int j = 0; j < pcIn.Height(); j++) {
		for(int i = 0; i < pcIn.Width(); i++) {
			if(p1->z == 0) {
				p2->x = float(((float)i - KINECT_CX_D) * KINECT_FX_D);
				p2->y = float(((float)(safeDepthHeight - j) - KINECT_CY_D) * KINECT_FY_D);
				p2->z = 5.0f;
				p2->value = p1->value;
			} else
				*p2 = *p1;
			++p1; ++p2;
		}
	}
	pcOut.SaveCloudDataToPly(out.c_str());
}

void DisplayOF(string in1, string in2, string out) {
	PointCloudBgr past, curr;
	FlowInfo flow;
	past.LoadCloudFromPly(in1.c_str());
	curr.LoadCloudFromPly(in2.c_str());
	ComputeOpticalFlow(past,curr,&flow);
	ImgFloat u,v,w;
	u.Reset(past.Width(),past.Height());
	v.Reset(past.Width(),past.Height());
	w.Reset(past.Width(),past.Height());
	ImgFloat::Iterator pU = u.Begin(), pV = v.Begin(), pW = w.Begin();
	FlowInfo::Iterator pF = flow.Begin();
	while(pU != u.End()) {
		*pU = pF->u;
		*pV = pF->v;
		*pW = pF->w;
		++pU; ++pV; ++pW; ++pF;
	}
	ImgBgr uS, vS, wS;
	ColorizeImage(u,&uS);
	ColorizeImage(v,&vS);
	ColorizeImage(w,&wS);
	Save(uS,string(out + "u.bmp").c_str());
	Save(vS,string(out + "v.bmp").c_str());
	Save(wS,string(out + "w.bmp").c_str());
	Figure fig1, fig2, fig3;
	fig1.Draw(u);
	fig2.Draw(v);
	fig3.Draw(w);
	EventLoop();
	while(!fig1.IsClosed());
	//fig1.GrabMouseClick();
}

void DisplayFloatImage(string filename) {
	/*ImgFloat r,g,b;
	FILE *fp = fopen(argv[1],"rb");
	int width = 640, height = 480, elements = 3, size = 3; //If messed up, just assume

	fread(&width,sizeof(int),1,fp);
	fread(&height,sizeof(int),1,fp);
	fread(&size,sizeof(int),1,fp);
	r.Reset(width,height);
	g.Reset(width,height);
	b.Reset(width,height);
	ImgFloat::Iterator pR = r.Begin(), pG = g.Begin(), pB = b.Begin();
	while(pR != r.End()) {
	float tmp[3];
	fread(tmp,sizeof(float),3,fp);
	*pR++ = tmp[0];
	*pG++ = tmp[1];
	*pB++ = tmp[2];
	}*/

	ImgFloat img;
	FILE *fp = fopen(filename.c_str(),"rb");
	int width = 640, height = 480, elements = 3, size = 3; //If messed up, just assume

	fread(&width,sizeof(int),1,fp);
	fread(&height,sizeof(int),1,fp);
	fread(&size,sizeof(int),1,fp);
	img.Reset(width,height);
	ImgFloat::Iterator p = img.Begin();
	fread(p,sizeof(int),width*height,fp);
	fclose(fp);

	Figure fig;
	fig.Draw(img);
	EventLoop();
	cin.get();

	/*Figure figR, figG, figB;
	figR.Draw(r);
	figG.Draw(g);
	figB.Draw(b);
	EventLoop();
	cin.get();

	ImgFloat *img = (ImgFloat *)malloc(size*sizeof(ImgFloat));
	for(int i = 0; i < size; i++)
	img[i].Reset(width,height);
	ImgFloat::Iterator *p = (ImgFloat::Iterator *)malloc(size*sizeof(ImgFloat::Iterator));
	for(int i = 0; i < size; i++)
	p[i] = img[i].Begin();
	float *tmp = (float *)malloc(size*sizeof(float));
	while(p[0] != img[0].End()) {
	fread(tmp,sizeof(float),size,fp);
	for(int i = 0; i < size; i++)
	*p[i]++ = tmp[i];
	}*/
}