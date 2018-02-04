/* \author Clayder Gonzalez */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <set>
#include <fstream>
#include <cmath>

#include "util.h"
#include "css.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

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
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud2");

  visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_color_handler2 (cloud1, 255, 0, 0);
  viewer->addPointCloud<PointXYZ> (cloud1, cloud_color_handler2, "cloud1");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud1");
  
  //viewer->addCoordinateSystem (1.0, "global");
  //viewer->initCameraParameters ();
  return (viewer);
}

/*struct CompareSet {
  bool operator() (const float& lhs, const float& rhs) const
  {
  	return lhs<rhs;
  }
};*/


int main (int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloud<PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloud<PointXYZ>::Ptr filteredCloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloud<PointXYZ>::Ptr keypointsCloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::io::loadPCDFile (argv[1], *cloud);

	int numScales = 16;
	float scales[numScales] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};
	//int numScales = 15;
	//float scales[numScales] = {0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};
	//int numScales = 13;
	//float scales[numScales] = {0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};
	vector<CurvatureTriplet> keypoints[numScales];
	vector<float> curvature;
	vector<float> s;

	vector<CurvatureTriplet> keypointsMinScale;
	vector<CurvatureTriplet> keypointsMaxScale;
	
  	clock_t begin = clock();
 	
 	parametrizeCurve(*cloud, s);

 	vector<float> gauss;
	vector<float> kernel0;

 	//recorrido normal
 	for(int i=0; i<numScales; ++i)
	{
		curvature.clear();

		if(i!=numScales-1)
			computeCurvature(*cloud, scales[i], curvature, s, keypoints[i], gauss, kernel0, false); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
		else
			computeCurvature(*cloud, scales[i], curvature, s, keypoints[i], gauss, kernel0, true); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
		
		//removeConstantCurvature(keypoints[i]);
		printKeypoints(keypoints[i]);

		/*if(i==0)
		{
			keypointsMinScale = keypoints[i];
			//for(int k=0; k<keypoints[i].size(); ++k)
				//keypointsMinScale.push_back(keypoints[i][k]);
		}*/


		if(i==numScales-1)
		{
			removeConstantCurvature(keypoints[i]);
			keypointsMaxScale = keypoints[i];
			//for(int k=0; k<keypoints[i].size(); ++k)
				//keypointsMaxScale.push_back(keypoints[i][k]);
		}



		if(i==4)
		{
			plt::plot(s, curvature);
			//plt::plot(kernel0, gauss);
			plt::show();
		}

		cout<<"*******************************************"<<endl;
	}

	/*plt::plot(kernel0, gauss);
	plt::figure();*/
	/*vector<float> x0(cloud->size());

	for(int i=0; i<x0.size(); ++i)
		x0[i] = i;
	plt::plot(x0, curvature);
	plt::show();*/

	vector<CurvatureTriplet> keypointsTmp;
	keypointsTmp = keypointsMaxScale;
	//keypointsTmp = keypointsMinScale;

	//printKeypoints(keypointsTmp);


	vector<CurvatureCounter> keypointsFinalCounter(keypointsMaxScale.size());
	for(int k=0; k<keypointsFinalCounter.size(); ++k)
	{
		CurvatureCounter cc;
		cc.index = keypointsMaxScale[k].index;
		cc.curvature = keypointsMaxScale[k].curvature;
		cc.counter = 0;

		keypointsFinalCounter[k] = cc;
	}

	/*vector<CurvatureCounter> keypointsFinalCounter(keypointsMinScale.size());
	for(int k=0; k<keypointsFinalCounter.size(); ++k)
	{
		CurvatureCounter cc;
		cc.index = keypointsMinScale[k].index;
		cc.curvature = keypointsMinScale[k].curvature;
		cc.counter = 0;

		keypointsFinalCounter[k] = cc;
	}*/


	//recorrido para llegar a la posicion del keypoint del finest scale
	int idx1, idx2 = -1;
	
	for(int i=numScales-1; i>0; --i)
	{
		for(int j=0; j<keypointsTmp.size(); ++j)
		{
			//idx1 = findByThreshold(keypointsTmp[j], keypoints[i-1], 0.029, idx2);
			idx1 = findByThreshold(keypointsTmp[j], keypoints[i-1], 0.0298, idx2);
			//idx1 = findByThreshold(keypointsTmp[j], keypoints[i-1], 0.121, idx2);
			//idx1 = findByThreshold(keypointsTmp[j], keypoints[i-1], 0.05, idx2);

			if(idx1!=-1)
			{
				keypointsTmp[j].index = keypoints[i-1][idx2].index;
				keypointsTmp[j].sIndex = keypoints[i-1][idx2].sIndex;

				keypointsFinalCounter[j].index = keypoints[i-1][idx2].index;
				keypointsFinalCounter[j].counter = keypointsFinalCounter[j].counter + 1;
			}
				
			cout<<keypointsTmp[j].index<<"->"<<keypointsTmp[j].sIndex<<", ";
		}

		cout<<endl;
	}

	/*cout<<"hola"<<endl;

	for(int i=0; i<numScales-1; ++i)
	{
		for(int j=0; j<keypointsTmp.size(); ++j)
		{
			idx1 = findByThreshold(keypointsTmp[j], keypoints[i+1], 0.01, idx2);
			//idx1 = findByThreshold(keypointsTmp[j], keypoints[i-1], 0.121, idx2);
			//idx1 = findByThreshold(keypointsTmp[j], keypoints[i-1], 0.05, idx2);

			if(idx1!=-1)
			{
				keypointsTmp[j].index = keypoints[i+1][idx2].index;
				keypointsTmp[j].sIndex = keypoints[i+1][idx2].sIndex;

				keypointsFinalCounter[j].index = keypoints[i+1][idx2].index;
				keypointsFinalCounter[j].counter = keypointsFinalCounter[j].counter + 1;
			}
				
			cout<<keypointsTmp[j].index<<"->"<<keypointsTmp[j].sIndex<<", ";
		}

		cout<<endl;
	}*/

	vector<int> counterPlot(cloud->size());

	vector<float> x(cloud->size());

	for(int i=0; i<x.size(); ++i)
		x[i] = i;


	for(int k=0; k<keypointsFinalCounter.size(); ++k)
	{
		keypointsCloud->push_back((*cloud)[keypointsFinalCounter[k].index]);
		counterPlot[keypointsFinalCounter[k].index] = keypointsFinalCounter[k].counter;
	}

	plt::figure();
	plt::plot(x, counterPlot);
	plt::show();


	//keypointsCloud->clear();
	//keypointsCloud->push_back((*cloud)[202]);
	//keypointsCloud->push_back((*cloud)[234]);
    /*for(int i=0; i<3; ++i)
    	if(curvatureCounter[i].counter>0)
    	{
    		//cout<<"curvature final: "<<curvatureCounter[i].curvature<<endl;
    		cout<<"curvature index!: "<<curvatureCounter[i].index<<endl;
    		keypointsCloud->push_back(cloud->points[curvatureCounter[i].index]);
    	}*/

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
