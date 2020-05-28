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
	cloud->push_back(PointXYZ(10,10,0));*/
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


	
	//vector<CurvatureTriplet> keypoints[SIGMA_SIZE][WIDTH_SIZE];
	//vector<CurvatureTriplet> keypointsFinestScale;
	//vector<CurvatureTriplet> selectedKeypointsFinestScale;
	vector<CurvatureCounter> curvatureCounter(cloud->size());

	//Agregado por clayder para ver el tiempo de divison por capas
  	clock_t begin = clock();
 	//Fin Agregado por clayder para ver el tiempo de divison por capas


  	vector<float> curvature;
	vector<float> s;
	//vector<CurvatureTriplet> keypoints;
	vector<CurvatureTriplet> keypoints[9][7];

	//computeCurvature(*cloud, 0.005, 19, curvature, s, keypoints);

	//computeCurvature(*cloud, 0.03, 41, curvature, s, keypoints);

	//computeCurvature(*cloud, 0.07, 51, curvature, s, keypoints);

	//computeCurvature(*cloud, 0.04, 7, curvature, s, keypoints);
	

	//computeCurvature(*cloud, 0.01, 21, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.01, 25, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.01, 29, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.01, 33, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.01, 37, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.01, 51, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.01, 61, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.01, 91, curvature, s, keypoints);

	//computeCurvature(*cloud, 0.05, 89, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.11, 185, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.09, 171, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.09, 185, curvature, s, keypoints);

	//printKeypoints(keypoints);

	//plt::plot(s, curvature);
	//plt::show();

	//return 0;


	float scales[9] = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09};
	//int width[7] = {21, 25, 29, 33, 37, 41, 45};

	vector<int> counterMinScale(cloud->size());
	vector<int> counterMaxScale(cloud->size());
	vector<int> counterPlot(cloud->size());
	vector<CurvatureTriplet> keypointsMinScale;
	vector<CurvatureTriplet> keypointsMaxScale;

	vector<CurvatureTriplet> keypointsPrev;
	/*for(int j=0; j<7; ++j)
	{
		curvature.clear();
		s.clear();
		keypoints.clear();
		computeCurvature(*cloud, scales[0], width[j], curvature, s, keypoints);

		for(int i=0; i<keypoints.size(); ++i)
		{
			counterFinesScale[keypoints[i].index]++;

			if(counterFinesScale[keypoints[i].index]==7)
			{
				keypointsFinestScale.push_back(keypoints[i]);				
			}
		}
	}*/

	int width = 23;


	for(int i=0; i<9; ++i)
	{
		cout<<"sigma: "<<scales[i]<<endl;
		for(int j=0; j<7; ++j)
		{
			//cout<<"width:"<<width<<endl;
			curvature.clear();
			s.clear();
			//keypoints.clear();
			
			//computeCurvature(*cloud, scales[i], j, curvature, s, keypoints[i][j]);
			computeCurvature(*cloud, scales[i], width, curvature, s, keypoints[i][j]);
			//computeCurvature(*cloud, scales[i], width, curvature, s, keypoints);

			printKeypoints(keypoints[i][j]);
			//printKeypoints(keypoints);

			if(i==0)
			{
				for(int k=0; k<keypoints[i][j].size(); ++k)
				{
					counterMinScale[keypoints[i][j][k].index]++;

					if(counterMinScale[keypoints[i][j][k].index]==7)
						keypointsMinScale.push_back(keypoints[i][j][k]);
				}
				//keypointsPrev = keypointsFinestScale;
			}
			else if(i==8)
			{
				for(int k=0; k<keypoints[i][j].size(); ++k)
				{
					counterMaxScale[keypoints[i][j][k].index]++;

					if(counterMaxScale[keypoints[i][j][k].index]==7)
						keypointsMaxScale.push_back(keypoints[i][j][k]);
				}
			}


			/*else
			{
				for(int k=0; k<keypoints.size(); ++k)
				{
					int idx2 = -1;
					//int idx = findByThreshold(keypoints[k], keypointsFinestScale, 0.033, idx2);	
					//int idx = findByThreshold(keypoints[k], keypointsFinestScale, 0.04, idx2);
					//int idx = findByThreshold(keypoints[k], keypointsFinestScale, 0.012, idx2);	
					int idx = findByThreshold(keypoints[k], keypointsPrev, 0.011, idx2);	

					if(idx!=-1)
					{
						counterPlot[idx]++;
						curvatureCounter[idx].index = idx;
						curvatureCounter[idx].counter = curvatureCounter[idx].counter+1;
						curvatureCounter[idx].curvature = keypoints[k].curvature;


						if(i==1)
							cout<<"idx:"<<idx<<endl;
						
						//CurvatureTriplet c;
						//c.index = idx;
						//c.sIndex = keypointsFinestScale[k].sIndex;
						//c.curvature = keypoints[k].sIndex;
						//c.sIndex = keypoints[k].sIndex;
						//c.curvature = keypoints[k].sIndex;
						//keypointsPrev.push_back(c);
					}
				}

				//keypointsPrev = keypoints;
				//for(int k=0; k<keypointsPrev.size)
			}
			width += 4;*/

			/*if(i==4 && j==0)
			{
				plt::plot(s, curvature);
			}*/

			width += 4;
		}

		//keypointsPrev = keypoints;

		cout<<"*******************************************"<<endl;
	}

	//plt::show();

	//return 0;

	/*cout<<endl<<"kp size:"<<keypointsMaxScale.size()<<endl;

	for(int i=0; i<keypointsMaxScale.size(); ++i)
	{
		cout<<keypointsMaxScale[i].sIndex<<",";
	}*/

	cout<<endl<<"kp size:"<<keypointsMinScale.size()<<endl;

	for(int i=0; i<keypointsMinScale.size(); ++i)
	{
		cout<<keypointsMinScale[i].index<<"->"<<keypointsMinScale[i].sIndex<<",";
	}


	cout<<endl<<"hola"<<endl;

	vector<CurvatureTriplet> keypointsTmp;
	keypointsTmp = keypointsMaxScale;
	vector<CurvatureCounter> keypointsFinalCounter(keypointsMaxScale.size());

	for(int k=0; k<keypointsFinalCounter.size(); ++k)
	{
		CurvatureCounter cc;
		cc.index = keypointsMaxScale[k].index;
		cc.curvature = keypointsMaxScale[k].curvature;
		cc.counter = 0;

		keypointsFinalCounter[k] = cc;
	}

	cout<<endl<<"hola2"<<endl;

	bool isMinScale = false;

	for(int i=8; i>0; --i)
	{
		for(int j=6; j>-1 && !isMinScale; --j)
		{
			cout<<i<<","<<j<<endl;

			for(int k=0; k<keypointsTmp.size(); ++k)
			{
				int idx1, idx2 = -1;

				if(i==1)
				{
					/*idx1 = findByThreshold(keypointsTmp[k], keypointsMinScale, 0.0145, idx2);

					if(idx1!=-1)
					{
						keypointsTmp[k].index = keypointsMinScale[idx2].index;
						keypointsTmp[k].sIndex = keypointsMinScale[idx2].sIndex;
					}*/
					isMinScale = true;
					break;
				}
				else
				{
					idx1 = findByThreshold(keypointsTmp[k], keypoints[i-1][j], 0.0145, idx2);

					if(idx1!=-1)
					{
						keypointsTmp[k].index = keypoints[i-1][j][idx2].index;
						keypointsTmp[k].sIndex = keypoints[i-1][j][idx2].sIndex;

						keypointsFinalCounter[k].index = keypoints[i-1][j][idx2].index;
						keypointsFinalCounter[k].counter = keypointsFinalCounter[k].counter + 1;
					}
				}
					

				/*if(idx1!=-1)
				{
					keypointsTmp[k].index = keypoints[i-1][j][idx2].index;
					keypointsTmp[k].sIndex = keypoints[i-1][j][idx2].sIndex;
				}*/

				cout<<keypointsTmp[k].index<<"->"<<keypointsTmp[k].sIndex<<", ";
			}

			cout<<endl;
		}

		if(i==1)
		{
			for(int k=0; k<keypointsTmp.size(); ++k)
			{
				int idx1, idx2 = -1;				
				idx1 = findByThreshold(keypointsTmp[k], keypointsMinScale, 0.0145, idx2);

				if(idx1!=-1)
				{
					keypointsTmp[k].index = keypointsMinScale[idx2].index;
					keypointsTmp[k].sIndex = keypointsMinScale[idx2].sIndex;

					keypointsFinalCounter[k].index = keypointsMinScale[idx2].index;
					keypointsFinalCounter[k].counter = keypointsFinalCounter[k].counter + 1;
				}
				cout<<keypointsTmp[k].index<<"->"<<keypointsTmp[k].sIndex<<", ";
			}
		}
	}


	vector<float> x(cloud->size());

	for(int i=0; i<x.size(); ++i)
		x[i] = i;


	for(int k=0; k<keypointsFinalCounter.size(); ++k)
	{
		keypointsCloud->push_back((*cloud)[keypointsFinalCounter[k].index]);
		counterPlot[keypointsFinalCounter[k].index] = keypointsFinalCounter[k].counter;
	}
	


	//plt::figure();
	plt::plot(x, counterPlot);
	
	//plt::plot(s, counterPlot);
	//plt::plot(s, counterFinesScale);
  	
  	plt::show();


	//computeCurvature(*cloud, 0.02, 41, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 45, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 49, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 53, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 57, curvature, s, keypoints);

	//computeCurvature(*cloud, 0.03, 61, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 45, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 49, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 53, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.02, 57, curvature, s, keypoints);

	//computeCurvature(*cloud, 0.05, 81, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.05, 89, curvature, s, keypoints);

	//computeCurvature(*cloud, 0.08, 111, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.08, 121, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.08, 131, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.08, 139, curvature, s, keypoints);
	

	//computeCurvature(*cloud, 0.09, 171, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.09, 179, curvature, s, keypoints);
	//computeCurvature(*cloud, 0.09, 185, curvature, s, keypoints);

	
	
  	//plt::plot(s, curvature);
  	//plt::show();


	//computeScaleSpace(*cloud, keypoints, keypointsFinestScale);
	
	//printKeypoints(keypoints[6][6]);
	//CurvatureTriplet c;
	//c.sIndex = 0.174573;
	//cout<<"KPS:"<<keypoints[6][6].size()<<endl;

	//int idx2 = -1;
	//int idx = findByThreshold(c, keypoints[6][6], 0.035, idx2); //devuelve el indice del global por eso no me devuelve bien la curvatura. jaja saludos

	//cout<<"idx:"<<idx<<", idx2:"<<idx2<<endl;

	//printKeypoints(keypointsFinestScale);

	//getKeypointsAtFinestScale(keypointsFinestScale, keypoints[0][WIDTH_SIZE-1], selectedKeypointsFinestScale);
	//getFinalKeypoints(*cloud, selectedKeypointsFinestScale, keypoints, *keypointsCloud, curvatureCounter);


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
