/*
Copyright (C) 2014 Steven Hickson

MIT License
*/
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "TestVideoSegmentation.h"

using namespace std;
using namespace pcl;
using namespace cv;

const char* PathFindExtension(std::string filename) {
    std::string::size_type idx;

    idx = filename.rfind('.');

    if(idx != std::string::npos)
        return filename.substr(idx+1).c_str();
    else
        return "";
}

Mat imread_depth(const char* fname, bool binary) {
    const char* ext = PathFindExtension(fname);
    const char char_dep[] = "dep";
    const char char_png[] = "png";
    Mat out;
    if(strncmp(ext,char_dep,strlen(char_dep))==0) {
        FILE *fp;
        if(binary)
            fp = fopen(fname,"rb");
        else
            fp = fopen(fname,"r");
        int width = 640, height = 480; //If messed up, just assume
        if(binary) {
            fread(&width,sizeof(int),1,fp);
            fread(&height,sizeof(int),1,fp);
            out = Mat(height,width,CV_32S);
            int *p = (int*)out.data;
            fread(p,sizeof(int),width*height,fp);
        } else {
            //fscanf(fp,"%i,%i,",&width,&height);
            out = Mat(height,width,CV_32S);
            int *p = (int*)out.data, *end = ((int*)out.data) + out.rows*out.cols;
            while(p != end) {
                fscanf(fp,"%i",p);
                p++;
            }
        }
        fclose(fp);
    } else if(strncmp(ext,char_png,strlen(char_png))==0) {
        out = imread(fname,cv::IMREAD_ANYDEPTH);
        out.convertTo(out, CV_32S);
        int* pi = (int*)out.data;
        for (int y=0; y < out.rows; y++) {
            for (int x=0; x < out.cols; x++) {
                *pi = Round(*pi * 0.2f);
                pi++;
            }
        }
    } else {
        cout << "Not the write filetype! " << ext << endl;
        throw "Filetype not supported";
    }
    return out;
}

void imwrite_depth(const char* fname, Mat &img, bool binary) {
    const char* ext = PathFindExtension(fname);
    const char char_dep[] = "dep";
    Mat out;
    if(strncmp(ext,char_dep,strlen(char_dep))==0) {
        FILE *fp;
        if(binary)
            fp = fopen(fname,"wb");
        else
            fp = fopen(fname,"w");
        int width = img.cols, height = img.rows; //If messed up, just assume
        if(binary) {
            fwrite(&width,sizeof(int),1,fp);
            fwrite(&height,sizeof(int),1,fp);
            int *p = (int*)img.data;
            fwrite(p,sizeof(int),width*height,fp);
        } else {
            //fscanf(fp,"%i,%i,",&width,&height);
            int *p = (int*)img.data, *end = ((int*)img.data) + width*height;
            while(p != end) {
                fprintf(fp,"%i",*p);
                p++;
            }
        }
        fclose(fp);
    } else {
        throw "Filetype not supported";
    }
}

void imwrite_float(const char* fname, Mat &img, bool binary) {
    const char* ext = PathFindExtension(fname);
    const char char_dep[] = "flt";
    Mat out;
    if(strncmp(ext,char_dep,strlen(char_dep))==0) {
        FILE *fp;
        if(binary)
            fp = fopen(fname,"wb");
        else
            fp = fopen(fname,"w");
        int width = img.cols, height = img.rows; //If messed up, just assume
        if(binary) {
            fwrite(&width,sizeof(int),1,fp);
            fwrite(&height,sizeof(int),1,fp);
            float *p = (float*)img.data;
            fwrite(p,sizeof(float),width*height,fp);
        } else {
            //fscanf(fp,"%i,%i,",&width,&height);
            float *p = (float*)img.data, *end = ((float*)img.data) + width*height;
            while(p != end) {
                fscanf(fp,"%f",p);
                p++;
            }
        }
        fclose(fp);
    } else {
        throw "Filetype not supported";
    }
}

inline void GetMatFromCloud(const PointCloudInt &cloud, Mat &img) {
    img = Mat(cloud.height,cloud.width,CV_32S);
    Mat_<int>::iterator pI = img.begin<int>();
    PointCloudInt::const_iterator pC = cloud.begin();
    while(pC != cloud.end()) {
        *pI = pC->intensity;
        ++pI; ++pC;
    }
}

inline void GetMatFromCloud(const PointCloud<PointXYZRGBA> &cloud, Mat &img) {
    img = Mat(cloud.height,cloud.width,CV_8UC3);
    Mat_<Vec3b>::iterator pI = img.begin<Vec3b>();
    PointCloud<PointXYZRGBA>::const_iterator pC = cloud.begin();
    while(pC != cloud.end()) {
        *pI = Vec3b(pC->b,pC->g,pC->r);
        ++pI; ++pC;
    }
}

void LoadData(string direc, int i, Mat &img, Mat &depth, Mat &label) {
    stringstream num;
    num << i;
    //I should crop everything some number of pixels
    int crop = 7;
    img = imread(string(direc + "rgb/" + num.str() + ".bmp"));
    cout << "Loading image: " << string(direc + "rgb/" + num.str() + ".bmp") << endl;
    Rect roi = Rect(crop,crop,img.cols - 2*crop, img.rows - 2*crop);
    img = img(roi);
    depth = imread_depth(string(direc + "depth/" + num.str() + ".dep").c_str(),true);
    cout << "Loading depth: " << string(direc + "rgb/" + num.str() + ".bmp") << endl;
    depth = depth(roi);
    label = imread_depth(string(direc + "labels/" + num.str() + ".dep").c_str(),true);
    cout << "Loading labels: " << string(direc + "rgb/" + num.str() + ".bmp") << endl;
    label = label(roi);
}

void CreatePointCloudFromRegisteredNYUData(const Mat &img, const Mat &depth, PointCloudBgr *cloud) {
    //assert(!img.IsNull() && !depth.IsNull());
    //take care of old cloud to prevent memory leak/corruption
    if (cloud != NULL && cloud->size() > 0) {
        cloud->clear();
    }
    cloud->header.frame_id =  "/microsoft_rgb_optical_frame";
    cloud->height = img.rows;
    cloud->width = img.cols;
    cloud->is_dense = true;
    cloud->points.resize (cloud->height * cloud->width);
    PointCloud<PointXYZRGBA>::iterator pCloud = cloud->begin();
    Mat_<int>::const_iterator pDepth = depth.begin<int>();
    Mat_<Vec3b>::const_iterator pImg = img.begin<Vec3b>();
    for(int j = 0; j < img.rows; j++) {
        for(int i = 0; i < img.cols; i++) {
            pCloud->z = *pDepth / 1000.0f;
            pCloud->x = float((i - 3.1304475870804731e+02) * pCloud->z / 5.8262448167737955e+02);
            pCloud->y = float((j - 2.3844389626620386e+02) * pCloud->z / 5.8269103270988637e+02);
            pCloud->b = (*pImg)[0];
            pCloud->g = (*pImg)[1];
            pCloud->r = (*pImg)[2];
            pCloud->a = 255;
            pImg++; pDepth++; pCloud++;
        }
    }
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.w () = 1.0;
    cloud->sensor_orientation_.x () = 0.0;
    cloud->sensor_orientation_.y () = 0.0;
    cloud->sensor_orientation_.z () = 0.0;
}

void CreatePointCloudWithZeroDepth(const Mat &img,PointCloudBgr *cloud) {
    //assert(!img.IsNull() && !depth.IsNull());
    //take care of old cloud to prevent memory leak/corruption
    if (cloud != NULL && cloud->size() > 0) {
        cloud->clear();
    }
    cloud->header.frame_id =  "/microsoft_rgb_optical_frame";
    cloud->height = img.rows;
    cloud->width = img.cols;
    cloud->is_dense = true;
    cloud->points.resize (cloud->height * cloud->width);
    PointCloud<PointXYZRGBA>::iterator pCloud = cloud->begin();
    Mat_<Vec3b>::const_iterator pImg = img.begin<Vec3b>();
    for(int j = 0; j < img.rows; j++) {
        for(int i = 0; i < img.cols; i++) {
            pCloud->z = 0.0f;
            pCloud->y = 0.0f;
            pCloud->x = 0.0f;
            pCloud->b = (*pImg)[0];
            pCloud->g = (*pImg)[1];
            pCloud->r = (*pImg)[2];
            pCloud->a = 255;
            pImg++; pCloud++;
        }
    }
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.w () = 1.0;
    cloud->sensor_orientation_.x () = 0.0;
    cloud->sensor_orientation_.y () = 0.0;
    cloud->sensor_orientation_.z () = 0.0;
}

void Seg4DExample(string data_folder, bool fast=false) {
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBA> > cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > label(new pcl::PointCloud<pcl::PointXYZI>);
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBA> > label_color(new pcl::PointCloud<pcl::PointXYZRGBA>);
	SegmentationOptions options;

    Mat img, depth, label_mat;
	//Set up options here
    if(fast) {
	    options.use_fast_method = true;
        options.c_depth = 500;
        options.depth_min_size = 500;
        options.c_color = 400;
        options.color_min_size = 400;
        options.number_of_frames = 1;
        options.use_time = false;
    } 


	RGBDTSegmentation segments(options);

  /*
	PCDReader reader;
	int i = 1;
	while(i < 1450) {
		//read the pointcloud file
		//stringstream fileName;
		//fileName << data_folder;
		//fileName << i;
		//fileName << ".pcd";
		//reader.read<pcl::PointXYZRGBA> (fileName.str(), *cloud);
		//cloud->width = 640;
		//cloud->height = 480;
        LoadData(data_folder,i,img,depth,label_mat);
        CreatePointCloudFromRegisteredNYUData(img,depth,&*cloud);

		//run the segmentation
		segments.AddSlice(cloud, label, label_color);

        //save the results
        stringstream fileName2;
        fileName2 << data_folder;
        fileName2 << "out_";
        fileName2 << i;
		//fileName2 << ".pcd";
        fileName2 << ".dep";
        if(label->size() > 0 ) {
            Mat label_out;
            GetMatFromCloud(*label, label_out);
            cout << "Saving: " << fileName2.str() << endl;
            imwrite_depth(fileName2.str().c_str(), label_out, true);
            //pcl::io::savePCDFileASCII (fileName2.str(), *label_color);
        } else
            cout << "Empty cloud" << endl;

		++i;
	}*/

  img = imread(data_folder);
  CreatePointCloudWithZeroDepth(img,&*cloud);
	segments.AddSlice(cloud, label, label_color);
  Mat output;
  GetMatFromCloud(*label_color, output);
  imwrite(data_folder + "_seg.png", output);
}

int main (int argc, char** argv) {
	try {
        Seg4DExample(string(argv[1]),true);
		cout << "Done" << endl;
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
	return 0;
}
