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

	/*cloud->push_back(PointXYZ(0,0,0));
	cloud->push_back(PointXYZ(1,1,0));
	cloud->push_back(PointXYZ(2,2,0));
	cloud->push_back(PointXYZ(3,3,0));
	cloud->push_back(PointXYZ(4,4,0));
	cloud->push_back(PointXYZ(5,5,0));
	cloud->push_back(PointXYZ(6,6,0));
	cloud->push_back(PointXYZ(7,7,0));
	cloud->push_back(PointXYZ(8,8,0));
	cloud->push_back(PointXYZ(9,9,0));
	cloud->push_back(PointXYZ(10,10,0));
	/*cloud->push_back(PointXYZ(11,11,0));
	cloud->push_back(PointXYZ(12,12,0));
	cloud->push_back(PointXYZ(13,13,0));
	cloud->push_back(PointXYZ(14,14,0));
	cloud->push_back(PointXYZ(15,15,0));
	cloud->push_back(PointXYZ(16,16,0));
	cloud->push_back(PointXYZ(17,17,0));
	cloud->push_back(PointXYZ(18,18,0));
	cloud->push_back(PointXYZ(19,19,0));
	cloud->push_back(PointXYZ(20,20,0));*/
	/*cloud->push_back(PointXYZ(11,9,0));
	cloud->push_back(PointXYZ(12,8,0));
	cloud->push_back(PointXYZ(13,7,0));
	cloud->push_back(PointXYZ(14,6,0));
	cloud->push_back(PointXYZ(15,5,0));
	cloud->push_back(PointXYZ(16,4,0));
	cloud->push_back(PointXYZ(17,3,0));
	cloud->push_back(PointXYZ(18,2,0));
	cloud->push_back(PointXYZ(19,1,0));
	cloud->push_back(PointXYZ(20,0,0));*/
	/*cloud->push_back(PointXYZ(11,9,0));
	cloud->push_back(PointXYZ(12,8,0));
	cloud->push_back(PointXYZ(13,7,0));
	cloud->push_back(PointXYZ(14,6,0));
	cloud->push_back(PointXYZ(15,5,0));
	cloud->push_back(PointXYZ(16,6,0));
	cloud->push_back(PointXYZ(17,7,0));
	cloud->push_back(PointXYZ(18,8,0));
	cloud->push_back(PointXYZ(19,9,0));
	cloud->push_back(PointXYZ(20,10,0));*/

	vector<float> curvature;
	vector<float> s;
	vector<CurvatureTriplet> keypoints;
	//vector<CurvatureTriplet> keypoints[9][7];

	vector<float> x0(cloud->size());

	for(int i=0; i<x0.size(); ++i)
		x0[i] = i;


	vector<float> gauss;
	vector<float> kernel0;

	//object5.pcd
	/*computeCurvature(*cloud, 0.1, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.2, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.3, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.4, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.5, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.6, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();*/



	//object4.pcd
	computeCurvature(*cloud, 0.1, curvature, s, keypoints, gauss, kernel0); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	/*computeCurvature(*cloud, 0.15, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.2, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.25, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.3, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.35, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.4, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.45, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.5, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.55, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.6, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.65, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.7, curvature, s, keypoints); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();*/
	computeCurvature(*cloud, 0.75, curvature, s, keypoints, gauss, kernel0); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
	printKeypoints(keypoints);
	keypoints.clear();


	

	/*computeCurvature(*cloud, 0.0096, 21, curvature, s, keypoints, gauss, kernel0); //10% puntos = 21 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.0096 = (max-min/2)/3.4
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.02, 43, curvature, s, keypoints, gauss, kernel0); //20% puntos = 43 ptos = distancia de 0.2 en x para la gaussiana y dev standar de 0.03
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.036, 65, curvature, s, keypoints, gauss, kernel0); //30% puntos = 65 ptos = distancia de 0.3 en x para la gaussiana y dev standar de 0.045
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.0448, 87, curvature, s, keypoints, gauss, kernel0); //40% puntos = 87 ptos = distancia de 0.4 en x para la gaussiana y dev standar de 0.055
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.0448, 103, curvature, s, keypoints, gauss, kernel0); //50% puntos = 103 ptos = distancia de 0.4 en x para la gaussiana y dev standar de 0.055
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.0448, 125, curvature, s, keypoints, gauss, kernel0); //50% puntos = 123 ptos = distancia de 0.4 en x para la gaussiana y dev standar de 0.055
	printKeypoints(keypoints);
	keypoints.clear();
	computeCurvature(*cloud, 0.0448, 147, curvature, s, keypoints, gauss, kernel0); //50% puntos = 123 ptos = distancia de 0.4 en x para la gaussiana y dev standar de 0.055
	printKeypoints(keypoints);
	keypoints.clear();*/

	

	plt::plot(kernel0, gauss);
	plt::figure();
	plt::plot(x0, curvature);
	plt::show();

	return 0;
}
