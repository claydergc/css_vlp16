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

  //visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_color_handler2 (cloud1, 255, 0, 0);
  //viewer->addPointCloud<PointXYZ> (cloud1, cloud_color_handler2, "cloud1");
  //viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud1");
  
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
	
	vector<CurvatureTriplet> keypoints[SIGMA_SIZE][WIDTH_SIZE];
	vector<CurvatureTriplet> keypointsFinestScale;
	vector<CurvatureTriplet> selectedKeypointsFinestScale;
	vector<CurvatureCounter> curvatureCounter;

	//Agregado por clayder para ver el tiempo de divison por capas
  	clock_t begin = clock();
 	//Fin Agregado por clayder para ver el tiempo de divison por capas

	computeScaleSpace(*cloud, keypoints, keypointsFinestScale);

	printKeypoints(keypointsFinestScale);

	getKeypointsAtFinestScale(keypointsFinestScale, keypoints[0][WIDTH_SIZE-1], selectedKeypointsFinestScale);
	getFinalKeypoints(*cloud, selectedKeypointsFinestScale, keypoints, *keypointsCloud, curvatureCounter);


	//Agregado por clayder para ver si es dividido por capas o rings
  	clock_t end = clock();  
  	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  	cout<<"************************************"<<endl<<endl;
  	cout<<"Ring extraction time: "<<elapsed_secs<<endl;
  	cout<<"************************************"<<endl<<endl;

	
	/*vector<int> counterPlot(cloud->size(), 0);
	vector<CurvatureCounter> curvatureCounter(cloud->size());
	
	
	for(int i=1; i<7; ++i)
	//int i=1;
	{
		for(int j=0; j<7; ++j)
		{
			for(int k=0; k<selectedKeypointsFinestScale.size(); ++k)
			{
				//if(findByThreshold(keypointsFinestScale[k], keypoints[i][j], 0.05)!=-1)
				int idx = findByThreshold(selectedKeypointsFinestScale[k], keypoints[i][j], 0.025);

				if(idx!=-1)
				{
					//cout<<"hola: "<<keypointsFinestScale[k].index<<endl;
					counterPlot[selectedKeypointsFinestScale[k].index] = counterPlot[selectedKeypointsFinestScale[k].index]+1;
					curvatureCounter[selectedKeypointsFinestScale[k].index].index = selectedKeypointsFinestScale[k].index;
					curvatureCounter[selectedKeypointsFinestScale[k].index].counter = curvatureCounter[selectedKeypointsFinestScale[k].index].counter+1;
					curvatureCounter[selectedKeypointsFinestScale[k].index].curvature = keypoints[i][j][idx].curvature;
					
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
    		//cout<<"curvature final: "<<curvatureCounter[i].curvature<<endl;
    		cout<<"curvature index!: "<<curvatureCounter[i].index<<endl;
    		keypointsCloud->push_back(cloud->points[curvatureCounter[i].index]);
    	}
	*/


	//pcl::io::savePCDFile<pcl::PointXYZ>("filteredCloud.pcd", *filteredCloud);

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
