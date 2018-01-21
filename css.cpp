/* \author Clayder Gonzalez */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/distances.h>


#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unistd.h>
#include <set>

#include <fstream>

#include <cmath>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"

#include "Util.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include "persistence1d.hpp"

using namespace p1d;

const unsigned int MIN_CLUSTER_POINTS = 70;
const unsigned int MAX_CLUSTER_POINTS = 300;

boost::shared_ptr<visualization::PCLVisualizer>
simpleVis (PointCloud<PointXYZ>::ConstPtr cloud, PointCloud<PointXYZ>::ConstPtr cloud2)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------

  PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
  cloud1->push_back((*cloud)[0]);
  cloud1->push_back((*cloud)[1]);

  boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");

  visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_color_handler (cloud2, 0, 255, 0);
  viewer->addPointCloud<PointXYZ> (cloud2, cloud_color_handler, "cloud2");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud2");

  //visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_color_handler2 (cloud1, 255, 0, 0);
  //viewer->addPointCloud<PointXYZ> (cloud1, cloud_color_handler2, "cloud1");
  //viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud1");
  
  //viewer->addCoordinateSystem (1.0, "global");
  //viewer->initCameraParameters ();
  return (viewer);
}

bool compare (pair<float, float> i, pair<float, float> j) { return (i.second<j.second); }

void parametrizeCurve(PointCloud<PointXYZ> in, vector<float> &s)
{
	s = vector<float>(in.size());
	vector<float> d(in.size());

	float Dp = 0.0;
	float sumDistances = 0.0;

	d[0] = 0.0;
	s[0] = 0.0;	

	for(int i=1; i<in.size(); ++i)
	{
		d[i] = pcl::euclideanDistance(in[i], in[i-1]);
		Dp += d[i];
	}

	for(int i=1; i<in.size(); ++i)
	{
		for(int j=1; j<=i; ++j)
			sumDistances += d[j];

		s[i] = sumDistances/Dp;
		sumDistances = 0.0;
	}
}

typedef struct curvatureTriplet
{
	int index;
	float sIndex;
	float curvature;
}CurvatureTriplet;


int findCurvatureTriplet (vector<CurvatureTriplet> vec, CurvatureTriplet c)
{
  for(int i=0; i<vec.size(); ++i)
  	if(vec[i].sIndex==c.sIndex)
  		return i;

  return -1;
}

bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j) { return (i.sIndex<j.sIndex); }

void getCurvatureExtrema(vector<float> curvature, vector<float> s, vector<CurvatureTriplet>& keypoints)
{

  float threshold = *max_element(curvature.begin(), curvature.end());
  threshold = (threshold<0.0)?(threshold+(threshold/2.0)):(threshold/2.0);

  vector<CurvaturePair> cv(curvature.size());

  for(int i=0; i<cv.size(); ++i)
  {
  	CurvaturePair cp;
  	cp.s = s[i];
  	cp.curvature = curvature[i];
  	cv[i] = cp;
  }

  CurvatureTriplet c;
  vector<int> peaksInds;
  //findPeaks(curvature, peaksInds);
  findPeaks(cv, threshold, peaksInds);

  for(int i=0; i<peaksInds.size(); ++i)
  {
  	c.index = peaksInds[i];
  	c.sIndex = s[peaksInds[i]];
    c.curvature = curvature[peaksInds[i]];
    keypoints.push_back(c);
  }

  //for(int i=0; i<curvature.size(); ++i)
  	//curvature[i] = -curvature[i];

  for(int i=0; i<cv.size(); ++i)
  {
  	curvature[i] = -curvature[i];
  	CurvaturePair cp;
  	cp.s = s[i];
  	cp.curvature = curvature[i];
  	cv[i] = cp;

  }

  threshold = *max_element(curvature.begin(), curvature.end());
  threshold = (threshold<0.0)?(threshold+(threshold/2.0)):(threshold/2.0);

  peaksInds.clear();
  //findPeaks(curvature, peaksInds);
  findPeaks(cv, threshold, peaksInds);

  for(int i=0; i<peaksInds.size(); ++i)
  {
  	c.index = peaksInds[i];
  	c.sIndex = s[peaksInds[i]];
    c.curvature = curvature[peaksInds[i]];
    keypoints.push_back(c);
  }

}

//void curvature(PointCloud<PointXYZ> in, int kernelWidth, float sigma, vector<float>& curvature, PointCloud<PointXYZ>& out, PointCloud<PointXYZ>& keypoints)
void computeCurvature(PointCloud<PointXYZ> in, int kernelWidth, float sigma, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints,
					  PointCloud<PointXYZ>& keypointsCloud, PointCloud<PointXYZ>& convolvedCurve)
{

  float gaussian1[kernelWidth];
  float gaussian2[kernelWidth];
  float gaussian3[kernelWidth];
  float sum = 0.0;

  int extension = kernelWidth/2;
  PointCloud<PointXYZ> tmp;
  float x=0;
  float y=0;
  float x1=0;
  float y1=0;
  float x2=0;
  float y2=0;
  float a,b;

  int i, j, k;

  curvature = vector<float>(in.points.size());

  for(int i=0; i<extension; ++i)
    tmp.push_back(PointXYZ(in.points[0]));

  for(int i=0; i<in.points.size(); ++i)
    tmp.push_back(PointXYZ(in.points[i]));

  for(int i=in.points.size(); i<in.points.size()+extension; ++i)
    tmp.push_back(PointXYZ(in.points[in.points.size()-1]));

  VectorXd linspace = VectorXd::LinSpaced(kernelWidth,-(float)kernelWidth/2.0, (float)kernelWidth/2.0);
  vector<float> sTmp(linspace.data(), linspace.data() + linspace.rows());

  //for(int i=0; i<kernelWidth; ++i)
  	//cout<<sTmp[i]<<endl;

  //int sizeTmp = in.points.size()+extension*2;

  for(j=0; j<kernelWidth; ++j)
  {
    gaussian1[kernelWidth-1-j] = (-1/(sigma*sigma*sigma*sqrt(2*M_PI)))*sTmp[j]*exp((-(sTmp[j]*sTmp[j])) / (2 * sigma*sigma)); //-sTmp[j+i]*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma))/(sigma*sigma*sigma*sqrt(2*M_PI));
    gaussian2[kernelWidth-1-j] = (1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(((sTmp[j]/sigma)*(sTmp[j]/sigma))-1)*(exp((-(sTmp[j]*sTmp[j])) / (2 * sigma *sigma)));//((sTmp[j+i]/sigma)*(sTmp[j+i]/sigma)-1)/(sigma*sigma*sigma*sqrt(2*M_PI))*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma));
    gaussian3[kernelWidth-1-j] = exp(-(sTmp[j]*sTmp[j])/(2*sigma*sigma))/(sigma*sqrt(2*M_PI));
  }


  for(i = 0; i < in.size(); ++i)
  {

    for(j = 0; j<kernelWidth; ++j)
    {
    	//cout<< inTmp[i+j] <<"x"<<kernelTmp[j]<<"+ =";
        //sum += tmp[i+j] * kernelTmp[j];
        x1 += tmp.points[i+j].x * gaussian1[j];
    	y1 += tmp.points[i+j].y * gaussian1[j];
    	x2 += tmp.points[i+j].x * gaussian2[j];
    	y2 += tmp.points[i+j].y * gaussian2[j];

    	x += tmp.points[i+j].x * gaussian3[j];
    	y += tmp.points[i+j].y * gaussian3[j];
    }

    convolvedCurve.push_back(PointXYZ(x, y, 0));
    curvature[i] = (x1*y2-y1*x2);

    x = 0.0;
    y = 0.0;
    x1 = 0.0;
    y1 = 0.0;
    x2 = 0.0;
    y2 = 0.0;
  }

  for(i=0; i<extension; ++i)
  	curvature[i] = curvature[extension];

  for(i=curvature.size()-1; i>curvature.size()-1-extension; --i)
  	curvature[i] = curvature[curvature.size()-1-extension];


  parametrizeCurve(convolvedCurve, s);
  getCurvatureExtrema(curvature, s, keypoints);

}

typedef struct curvatureCounter
{
	float index;
	int counter;
	float curvature;
}CurvatureCounter;


typedef struct sCounter
{
	int index;
	int counter;
	float curvature;
}SCounter;

bool compareCurvatureCounter (CurvatureCounter i, CurvatureCounter j) { return (i.counter>j.counter); }
bool compareCounter3 (SCounter i, SCounter j) { return (i.counter>j.counter); }

struct Compare {
  bool operator() (const float& lhs, const float& rhs) const
  {return lhs<rhs;}
};

//Busca en todos los S de los diferentes grados de smooth
int findIndex(vector<float> s[][7], int sigmaSize, int widthSize, float index, float threshold)
{
	vector<SCounter> sCounter(s[0][0].size());

	for(int i=0; i<sigmaSize; ++i)
		for(int j=0; j<widthSize; ++j)
			for(int k=0; k<s[i][j].size(); ++k)
				if( (index-s[i][j][k])<threshold )//3.5e-3
				{
					sCounter[k].index=k;
					sCounter[k].counter++;
					break;
				}

	//for(int k=0; k<s[0][0].size(); ++k)
		//cout<<k<<"->"<<sCounter[k].counter<<endl;

	sort(sCounter.begin(), sCounter.end(), compareCounter3);

	return sCounter[0].index;
}

float findKeypointInScaleSpace(float index, vector<float> keypoints, float threshold)
{
	for(int k=0; k<keypoints.size(); ++k)
		if( abs(index-keypoints[k])<threshold )//3.5e-3
			return keypoints[k];

	return -1.0;
}

int findIndexInS(float index, vector<float> s, float threshold)
{
	for(int k=0; k<s.size(); ++k)
		if( abs(index-s[k])<threshold )//3.5e-3
			return k;

	return -1;
}

int findByThreshold(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold)
{
	for(int i=0; i<vec.size(); ++i)
		if(abs(a.sIndex-vec[i].sIndex)<threshold)
			return vec[i].index;
	return -1;
}

int main (int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloud<PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloud<PointXYZ>::Ptr filteredCloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloud<PointXYZ>::Ptr keypointsCloud (new pcl::PointCloud<pcl::PointXYZ>);
	//vector<int> keypoints;
	
	
	pcl::io::loadPCDFile (argv[1], *cloud);
	
	const int sigmaSize = 7;
	const int widthSize = 7;
	//const int sigmaSize = 5;
	//const int widthSize = 5;
	//vector<int> **scaleSpace;
	int sigma, width;

	vector<float> curvature;
	vector<float> s[sigmaSize][widthSize];
	vector<float> sVec;
	vector<CurvatureTriplet> keypoints[sigmaSize][widthSize];
	vector<CurvatureTriplet> keypointsVec;

	vector<float> sInicial;
	parametrizeCurve(*cloud, sInicial);

	vector<float> final;

	for(int i=0; i<sigmaSize; ++i)
	{
		sigma = i+4;

		cout<<"sigma:"<<sigma<<endl;

		for(int j=0; j<widthSize; ++j)
		{
			curvature.clear();
			filteredCloud->clear();
			//keypoints.clear();
			//s[i][j].clear();

			//if(i!=6)
			keypointsCloud->clear();

			width = ((sigma*(j+2))%2==0)?sigma*(j+2)+1:sigma*(j+2);

			cout<<"width:"<<width<<endl;


			//computeCurvature(*cloud, width, sigma, curvature, curvatureAcum, *filteredCloud, s[i][j], keypoints[i][j], *keypointsCloud);
			computeCurvature(*cloud, width, sigma, curvature, s[i][j], keypoints[i][j], *keypointsCloud, *filteredCloud);
			//cout<<"kp size:"<<keypoints[i][j].size()<<endl;

			sort(keypoints[i][j].begin(), keypoints[i][j].end(), compareCurvatureTriplet);

			sVec.insert(sVec.end(), s[i][j].begin(), s[i][j].end());
			
			for(int k=0; k<keypoints[i][j].size(); ++k)
			{
				//cout<<keypoints[i][j][k].sIndex<<":"<<abs(curvature[keypoints[i][j][k].index])<<", ";
				cout<<keypoints[i][j][k].sIndex<<":"<<abs(keypoints[i][j][k].curvature)<<", ";
			}

			cout<<endl<<endl;

			if(i==0)
				keypointsVec.insert(keypointsVec.end(), keypoints[i][j].begin(), keypoints[i][j].end());


			if(i==6 && j==5)
			{

				cout<<endl<<"[";

				for(int k=0; k<curvature.size(); ++k)
					cout<<curvature[k]<<" ";

				cout<<"]"<<endl;

				/*vector<int> inds;
				vector<float> mags;
				findPeaks(curvature, inds, mags);

				cout<<"ind size: "<<inds.size()<<endl;

				for(int k=0; k<inds.size(); ++k)
					//cout<<s[i][j][inds[k]]<<" ";
					cout<<inds[k]<<" ";

				cout<<endl;*/
				*filteredCloud2 = *filteredCloud;
				
				plt::plot(s[i][j], curvature);
			}


		}

		cout<<endl;
	}
	plt::show();

	//return 0;

	set<float, Compare> sSet(sVec.begin(), sVec.end());
	vector<int> scaleSpace[sigmaSize][widthSize];

	for(int i=0; i<sigmaSize; ++i)
	{
		for(int j=0; j<widthSize; ++j)
		{
			scaleSpace[i][j] = vector<int>(sSet.size());

			for(int k=0; k<keypoints[i][j].size(); ++k)
			{
				set<float, Compare>::iterator it = sSet.find(keypoints[i][j][k].sIndex);
				scaleSpace[i][j][ distance(sSet.begin(),it) ]++;
				//scaleSpace[i][j][ distance(sSet.begin(),it) ] = 1;
			}
		}
	}

	//Keypoints at finest scale
	//buscar menor dada una distancia
	sort(keypointsVec.begin(), keypointsVec.end(), compareCurvatureTriplet);

	//vector<float> keypointsFinestScale(keypoints[0][widthSize-1].size()); //los que aguantan al maximo width de smoothing son los maximos
	//vector<CurvatureTriplet> keypointsFinestScale(keypoints[0][widthSize-1].size()); //los que aguantan al maximo width de smoothing son los maximos
	vector<CurvatureTriplet> keypointsFinestScale; //los que aguantan al maximo width de smoothing son los maximos
	cout<<"KPFS Size:"<<keypointsVec.size()<<endl;

	for(int k=0; k<keypointsVec.size(); ++k)
		cout<<keypointsVec[k].sIndex<<",";

	cout<<endl;

	for(int k=0; k<keypoints[0][widthSize-1].size(); ++k)
	{
		//int idx = keypoints[0][widthSize-1][k].index;
		//vector<CurvatureTriplet>::iterator it = find (keypointsVec.begin(), keypointsVec.end(), keypoints[0][widthSize-1][k]);
		int idx = findCurvatureTriplet(keypointsVec, keypoints[0][widthSize-1][k]);
		int j=idx;
		int counter = 0;

		cout<<"Find:"<<keypoints[0][widthSize-1][k].sIndex<<"->"<<idx<<endl;

		//while((keypointsVec[j]-keypointsVec[j-1])<0.0246 && j>-1)
		//while((keypointsVec[j].sIndex-keypointsVec[j-1].sIndex)<1e-3 && j>-1)

		vector<int> indices;
		indices.push_back(keypointsVec[j].index);
		while((keypointsVec[j].sIndex-keypointsVec[j-1].sIndex)<4.29e-3 && j>-1)
		//while((keypointsVec[j].sIndex-keypointsVec[j-1].sIndex)<0.018 && j>-1)
		{
			j--;
			indices.push_back(keypointsVec[j].index);
			counter++;
			//cout<<"hola111"<<endl;
		}

		int i = idx;

		//int minIdx = keypointsVec[j].index - j;

		//while((keypointsVec[i+1].sIndex-keypointsVec[i].sIndex)<9.72e-3)
		//while((keypointsVec[i+1].sIndex-keypointsVec[i].sIndex)<0.01773)
		while((keypointsVec[i+1].sIndex-keypointsVec[i].sIndex)<0.033 && i<keypointsVec.size()-1)
		{
			i++;
			indices.push_back(keypointsVec[i].index);
			counter++;
			//cout<<"hola112"<<endl;
		}

		//toma la mediana de los keypoints
		sort(indices.begin(), indices.end());

		//int maxIdx = keypointsVec[i].index + i;
		//cout<<"idx:"<<indices[indices.size()/2]<<endl;
		keypointsVec[j].index = indices[indices.size()/2];

		//cout<<"keypointsVec[j].index size: "<<indices.size()<<endl;
		//cout<<"keypointsVec[j].index: "<<keypointsVec[j].index<<endl;
		cout<<"Counter: "<<counter<<endl;

		//cout<<"idx:"<<minIdx+((maxIdx-minIdx)/2)<<endl;

		//if(counter>4)
		//{
			//keypointsFinestScale[k] = keypointsVec[j];
			//keypointsFinestScale.push_back(keypointsVec[j]);
			//cout<<"FSKP: "<<keypointsFinestScale[k].sIndex<<" Counter: "<<counter<<endl;
		//}
		if(counter>=1)
		{
			//if(keypointsVec[j].curvature>3.11e-5)
			{
				//keypointsFinestScale[k] = keypointsVec[j];
				keypointsFinestScale.push_back(keypointsVec[j]);
				//cout<<"FSKP: "<<keypointsFinestScale[k].sIndex<<" Counter: "<<counter<<endl;
			}
		}
	}


	cout<<"hola"<<endl;
	
	//vector<CurvatureCounter> curvatureCounter(sVec.size());
	//sVec = vector<float>(sSet.begin(), sSet.end());
	//vector<int> counterPlot(sVec.size());
	vector<int> counterPlot(cloud->size(), 0);
	vector<CurvatureCounter> curvatureCounter(cloud->size());
	
	
	//Aqui


	//for(int k=0; k<scaleSpace[0][0].size(); ++k)
		//counterPlot[k] = counterPlot[k] + inc[k];
		//cout<<band[k]<<endl;

	
	

	/*int i=3;
	//for(int i=0; i<sigmaSize; ++i)
	{

		bool band[scaleSpace[0][0].size()] = {false};
		int inc[scaleSpace[0][0].size()] = {0};
		//for(int j=0; j<widthSize; ++j)
		for(int j=widthSize-1; j>-1; --j)
		//int j=widthSize-1;
		{


			for(int k=0; k<scaleSpace[0][0].size(); ++k)			
			{
				curvatureCounter[k].index = sVec[k];
				curvatureCounter[k].curvature = curvatureAcum[k];

				if(scaleSpace[i][j][k]==1)
				{
					//cout<<"hola"<<endl;
					band[k] = true;
				}

				if(band[k] == true && scaleSpace[i][j][k]==0)
					inc[k]++;

				counterPlot[k]+=scaleSpace[i][j][k];
				curvatureCounter[k].counter += scaleSpace[i][j][k];
			}

			for(int k=0; k<scaleSpace[0][0].size(); ++k)
				counterPlot[k] = (counterPlot[k] + inc[k])*3;
		}
	}*/

	//Contador final para sacar maximos: counterPlot

	//for(int i=0; i<sInicial.size(); ++i)
		//cout<<sInicial[i]<<", ";

	//cout<<endl<<"**********************"<<endl;

	//cout<<"hola2: "<<keypointsFinestScale.size()<<endl;

	for(int i=1; i<7; ++i)
	//int i=1;
	{
		for(int j=0; j<7; ++j)
		{
			for(int k=0; k<keypointsFinestScale.size(); ++k)
			{
				//if(findByThreshold(keypointsFinestScale[k], keypoints[i][j], 0.05)!=-1)
				int idx = findByThreshold(keypointsFinestScale[k], keypoints[i][j], 0.025);

				if(idx!=-1)
				{
					//cout<<"hola: "<<keypointsFinestScale[k].index<<endl;
					counterPlot[keypointsFinestScale[k].index] = counterPlot[keypointsFinestScale[k].index]+1;
					curvatureCounter[keypointsFinestScale[k].index].index = keypointsFinestScale[k].index;
					curvatureCounter[keypointsFinestScale[k].index].counter = curvatureCounter[keypointsFinestScale[k].index].counter+1;
					curvatureCounter[keypointsFinestScale[k].index].curvature = keypoints[i][j][idx].curvature;
					
					if(i==6 && j==6)
						cout<<"curvature: "<<keypoints[i][j][idx].curvature<<endl;
				}
			}

			//cout<<"---------------------------------------------------------------------"<<endl;
		}
	}

    vector<float> x(cloud->size());

    for (int i = 0; i < x.size(); ++i)
    	x[i] = i;

    cout<<"holagggg"<<endl;


    plt::plot(x, counterPlot);
    plt::show();

    //sort(counterPlot.begin(), counterPlot.end());

	sort(curvatureCounter.begin(), curvatureCounter.end(), compareCurvatureCounter);    

    //cout<<"KP:"<<counterPlot[71]<<endl;

    //return 0;

    
    keypointsCloud->clear();
    for(int i=0; i<3; ++i)
    	if(curvatureCounter[i].counter>0)
    	{
    		cout<<"curvature final: "<<curvatureCounter[i].curvature<<endl;
    		keypointsCloud->push_back(cloud->points[curvatureCounter[i].index]);
    	}



	pcl::io::savePCDFile<pcl::PointXYZ>("filteredCloud.pcd", *filteredCloud);

	boost::shared_ptr<visualization::PCLVisualizer> viewer;
  	//viewer = simpleVis(filteredCloud2, keypointsCloud);
  	viewer = simpleVis(cloud, keypointsCloud);

  	while (!viewer->wasStopped ())
  	{
    	viewer->spinOnce (100);
    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}

  	
	return 0;
}
