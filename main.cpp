/* \author Clayder Gonzalez */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <set>
#include <fstream>
#include <cmath>

#include "css.h"
#include "gaussians.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

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
	PointCloud<PointXYZ>::Ptr keypointsCloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::io::loadPCDFile (argv[1], *cloud);

	vector<float> gaussian1[NUM_GAUSSIANS];
	vector<float> gaussian2[NUM_GAUSSIANS];
	vector<CurvatureTriplet> keypoints[NUM_SCALES];
	vector<float> curvature[NUM_SCALES];
	vector<float> s;

	initKernels(gaussian1, gaussian2);

	clock_t begin = clock();
	computeScaleSpace(*cloud, gaussian1, gaussian2, keypoints, s);
	//computeScaleSpace(*cloud, keypoints, s);
	getFinalKeypointsAtMinScale(*cloud, keypoints, s, *keypointsCloud);
	clock_t end = clock();  
  	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  	cout<<"************************************"<<endl<<endl;
  	cout<<"Ring extraction time: "<<elapsed_secs<<endl;
  	cout<<"************************************"<<endl<<endl;


	boost::shared_ptr<visualization::PCLVisualizer> viewer;
  	//viewer = simpleVis(filteredCloud2, keypointsCloud);
  	viewer = simpleVis(cloud, keypointsCloud);

  	while (!viewer->wasStopped ())
  	{
    	viewer->spinOnce (100);
    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}


	/*ofstream myfile;
  	myfile.open ("gaussian_kernels.h");

  	ofstream myfile2;
  	myfile2.open ("gaussian_kernels.cpp");

  	int k = 0;


  	//vector<float> gaussian1[numKernels];
  	//vector<float> gaussian2[numKernels];

	//for(int kernelWidth=3; kernelWidth<=241; kernelWidth+=2)
	for(int kernelWidth=3; kernelWidth<=321; kernelWidth+=2)
	{
	   	VectorXd linspace = VectorXd::LinSpaced(kernelWidth,-0.2,0.2);
		vector<float> sKernel(linspace.data(), linspace.data() + linspace.rows());
		float u = 0.0;
	   	float sigma = 0.05;
	   	float gaussian1[kernelWidth];
		float gaussian2[kernelWidth];

	   	stringstream ss1, ss2, ss3, ss4;

	   	ss1<< "const float g1_"<<k<<"[] = {";
	   	ss2<< "const float g2_"<<k<<"[] = {";
	   	

		for(int j=0; j<kernelWidth; ++j)
		{

			gaussian1[j] = (-1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(sKernel[j]-u)*exp((-( (sKernel[j]-u)*(sKernel[j]-u)) ) / (2 * sigma*sigma));
			gaussian2[j] = (1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(( ((sKernel[j]-u)/sigma )*( (sKernel[j]-u)/sigma))-1)*(exp((-( (sKernel[j]-u)*(sKernel[j]-u) )) / (2 * sigma *sigma)));
			if (j!=kernelWidth-1)
			{
				ss1<<gaussian1[j]<<",";
				ss2<<gaussian2[j]<<",";
			}
			else
			{
				ss1<<gaussian1[j]<<"};";
				ss2<<gaussian2[j]<<"};";
			}
			
		}

		
		ss3 << "gaussian1["<<k<<"] = vector<float>(g1_"<<k<<", g1_"<<k<<" + sizeof(g1_"<<k<<") / sizeof(float) );";
		ss4 << "gaussian2["<<k<<"] = vector<float>(g2_"<<k<<", g2_"<<k<<" + sizeof(g2_"<<k<<") / sizeof(float) );";

	   	myfile << ss1.str() <<"\n";
	   	myfile << ss2.str() <<"\n";
	   	
	   	myfile2 <<"\t" << ss3.str() <<"\n";
	   	myfile2 <<"\t" << ss4.str() <<"\n\n";

		k++;
	}

	myfile.close();
	myfile2.close();*/

	return 0;
}
