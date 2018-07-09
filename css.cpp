// css.cpp
#include "css.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace std;
using namespace pcl;
using namespace Eigen;


bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j)
{
	return (i.sIndex<j.sIndex);
}

bool compareCurvatureCounter (CurvatureCounter i, CurvatureCounter j)
{ 
	return (i.counter>j.counter);
}

int findByThreshold(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold)
{
	int idx1 = -1;
	float diff;
	float minDiff = FLOAT_MAX;

	int sumIdx = 0;
	int counter = 0;

	for(int i=0; i<vec.size(); ++i)
	{
		diff = abs(a.sIndex-vec[i].sIndex);
		
		if(diff<=threshold && diff<minDiff)
		//if(diff<threshold && diff<minDiff)
		{
			idx1 = vec[i].index;
			minDiff = diff;//store the minimum difference
		}
	}

	return idx1;
}

int findByThresholdAtMinScale(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold)
{
	int idx1 = -1;
	float diff;

	int sumIdx = 0;
	int counter = 0;

	for(int i=0; i<vec.size(); ++i)
	{
		diff = abs(a.sIndex-vec[i].sIndex);
		
		if(diff<0.0000001)
		{
			//cout<<"hola1"<<endl;
			return vec[i].index;
		}
		//else if(diff<threshold)
		else if(diff<=threshold)
		{
			if(i<2 || i>vec.size()-2)
				return vec[i].index;

			//cout<<"hola2"<<endl;
			sumIdx += vec[i].index;
			counter++;
		}
	}

	idx1 = (counter==0)?-1:(sumIdx/counter);
	
	return idx1;
}

/*int findByThreshold(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold, float& idx2)
{
	for(int i=0; i<vec.size(); ++i)
		if(abs(a.sIndex-vec[i].sIndex)<threshold)
		{
			idx2 = ;
			return vec[i].index;
		}
	return -1;
}*/

int findCurvatureTriplet (vector<CurvatureTriplet> vec, CurvatureTriplet c)
{
  for(int i=0; i<vec.size(); ++i)
  	if(vec[i].sIndex==c.sIndex)
  		return i;
  return -1;
}

void parametrizeCurve(PointCloud<PointXYZ> in, vector<float> &s)
{
	s = vector<float>(in.size());
	vector<float> d(in.size());

	float Dp = 0.0;
	float sumDistances = 0.0;

	d[0] = 0.0;
	s[0] = 0.0;	

	PointXYZ a, b;

	for(int i=1; i<in.size(); ++i)
	{
		/*a.x = in[i].x; b.x = in[i-1].x;
		a.y = in[i].y; b.y = in[i-1].y;
		a.z = 0.0   ;  b.z = 0.0;*/
		a.x = in[i].x; b.x = in[i-1].x;
		a.z = in[i].z; b.z = in[i-1].z;
		a.y = 0.0   ;  b.y = 0.0;
		d[i] = pcl::euclideanDistance(a, b);
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

void printKeypoints(vector<CurvatureTriplet> keypoints)
{
	for(int k=0; k<keypoints.size(); ++k)
		cout<<keypoints[k].index<<"->"<<keypoints[k].sIndex<<":"<<keypoints[k].curvature<<", ";

	cout<<endl<<endl;
}

void printVector(vector<float> v)
{
	for(int i=0; i<v.size(); ++i)
		cout<<v[i]<<endl;

}

void printArray(float v[], int size)
{
	for(int i=0; i<size; ++i)
		cout<<v[i]<<endl;
}


void getCurvatureExtrema(vector<float> curvature, vector<float> s, vector<CurvatureTriplet>& keypoints, float min, float max, bool isMaxScale)
{
	for(int i=2; i<curvature.size()-2; ++i)
	{
		if( abs(curvature[i])>abs(curvature[i-1]) && abs(curvature[i])>abs(curvature[i+1]) &&
			abs(curvature[i])>abs(curvature[i-2]) && abs(curvature[i])>abs(curvature[i+2])
		  )
		{
			//if(curvature[i]>0 && curvature[i]>(max/4.2) ||
			//   curvature[i]<0 && curvature[i]<(min/4.0)
			//  )
			/*if(curvature[i]>0 && curvature[i]>(max/4.3) ||
			   curvature[i]<0 && curvature[i]<(min/4.5)
			  )*/

			CurvatureTriplet c;
			
			
			if(isMaxScale)
			{
				//if(curvature[i]>0 && curvature[i]>(max/2.3) ||
				//   curvature[i]<0 && curvature[i]<(min/2.3)
				//  )
				if( (curvature[i]>0 && curvature[i]>(max/2.3) && curvature[i]>abs(min/4.0)) ||
				    (curvature[i]<0 && curvature[i]<(min/2.3) && abs(curvature[i])>(max/4.0))
				  )
				{
					if( abs(curvature[i])>4e-4 )
					{
						c.index = i;
						c.sIndex = s[i];
						c.curvature = curvature[i];
						keypoints.push_back(c);
					}
				}
			}
			else
			{
				c.index = i;
				c.sIndex = s[i];
				c.curvature = curvature[i];
				keypoints.push_back(c);
			}

		}
	}
}

int sign(float num)
{
	return (int)((float)(num)/(float)abs(num));
}

void removeConstantCurvature(vector<CurvatureTriplet>& keypoints)
{
	CurvatureTriplet keypointsArray[keypoints.size()];
	CurvatureTriplet keypointTmp;

	//int counter = 0;
	int counter = -1;
	//int i=1;

	for(int i=1; i<keypoints.size(); ++i)
	//while(i<keypoints.size())
	{
		if(sign(keypoints[i].curvature)==sign(keypoints[i-1].curvature))
		{
			if( (keypoints[i].index-keypoints[i-1].index)<5)
			{
				keypointTmp=(abs(keypoints[i].curvature)>abs(keypoints[i-1].curvature))?keypoints[i]:keypoints[i-1];
				//keypointsArray[counter] = keypointTmp;
				if(counter==-1)
					keypointsArray[++counter] = keypointTmp;
				else
					keypointsArray[counter] = keypointTmp;
				//counter++;
				//cout<<"keypointTmp: "<<keypointTmp.index<<endl;
			}
			else
			{
				//keypointsArray[++counter] = keypoints[i-1];
				//keypointsArray[++counter] = keypoints[i];
				if(counter==-1)
				{
					keypointsArray[++counter] = keypoints[i-1];
					keypointsArray[++counter] = keypoints[i];
				}
				else
				{
					keypointsArray[++counter] = keypoints[i];	
				}
			}
		}
		else if(abs(keypoints[i].curvature-keypoints[i-1].curvature)>35000)
		{
			if(counter==-1)
			{
				keypointsArray[++counter] = keypoints[i-1];
				keypointsArray[++counter] = keypoints[i];
			}
			else
			{
				keypointsArray[++counter] = keypoints[i];
			}
			//i++;
		}
		/*else if( (abs(keypoints[i].curvature-keypoints[i-1].curvature)>35000) && counter!=-1)
		{
			keypointsArray[++counter] = keypoints[i];
		}*/
		//cout<<"counter:"<<counter<<endl;

	}

	if(keypoints.size()==1)
		keypointsArray[++counter] = keypoints[0];

	//keypoints = vector<CurvatureTriplet>(keypointsArray, keypointsArray + counter + 1 );
	if(counter>-1)
		keypoints = vector<CurvatureTriplet>(keypointsArray, keypointsArray + counter + 1 );
}

void computeCurvature(PointCloud<PointXYZ> in, vector<float> gaussian1[NUM_GAUSSIANS], vector<float> gaussian2[NUM_GAUSSIANS], float kernelFactor, vector<float>& curvature, 
	vector<float>& s, vector<CurvatureTriplet>& keypoints, vector<float>& gauss, vector<float>& kernel0, bool isMaxScale)
{
	//kernel factor = percentage of the points to define the width and std deviation of gaussian kernel
	int m = in.size();
	int kernelWidth = ((int)((float)(m)*kernelFactor)%2==0)?(m*kernelFactor+1):(m*kernelFactor);
	float sigma;

	cout<<"width: "<<kernelWidth<<endl;

	float sum = 0.0;

	int extension = kernelWidth/2;
	PointCloud<PointXYZ> tmp;
	tmp.width = in.size() + extension*2;
	tmp.height = 1;
	tmp.resize(tmp.width * tmp.height);

	float x0=0;
	float y0=0;
	float x1=0;
	float y1=0;
	float x2=0;
	float y2=0;
	float a,b;

	int i, j, k;
	curvature = vector<float>(in.points.size());

	for(int i=0; i<extension; ++i)
	{
		tmp[i] = PointXYZ(in.points[0]);
	}

	for(int i=0; i<in.points.size(); ++i)
	{
		tmp[i+extension] = PointXYZ(in.points[i]);
	}

	for(int i=0; i<extension; ++i)
	{
		tmp[i+m+extension] = PointXYZ(in.points[in.points.size()-1]);
	}


	float min = FLOAT_MAX;
	float max = FLOAT_MIN;

	bool containsZeroCurvature = false;

	vector<float> convolvedCurveX(in.size());
	vector<float> convolvedCurveY(in.size());
  	
	for(i = 0; i < in.size(); ++i)
	{
	   	VectorXd linspace = VectorXd::LinSpaced(kernelWidth,-kernelWidth/1000.0,kernelWidth/1000.0);
		vector<float> sKernel(linspace.data(), linspace.data() + linspace.rows());
	   	sigma = kernelWidth/4000.0;

	   	float gaussian0[kernelWidth];
	   	float gaussian1[kernelWidth];
	   	float gaussian2[kernelWidth];

	   	for(j=0; j<kernelWidth; ++j)
	   	{
	   		gaussian0[j] = exp(-( (sKernel[j]*sKernel[j]) )/(2*sigma*sigma))/(sigma*sqrt(2*M_PI));
	   		gaussian1[j] = (-1/(sigma*sigma*sigma*sqrt(2*M_PI))) * sKernel[j] * exp( -(sKernel[j]*sKernel[j]) / (2 * sigma*sigma));
			gaussian2[j] = (1/(sigma*sigma*sigma*sqrt(2*M_PI))) * ( ((sKernel[j]/sigma )*( sKernel[j]/sigma))-1 ) * exp( -(sKernel[j]*sKernel[j]) / (2 * sigma*sigma));;
	   		//gaussian1[j] = (-1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(sKernel[j]-u)*exp((-( (sKernel[j]-u)*(sKernel[j]-u)) ) / (2 * sigma*sigma));
			//gaussian2[j] = (1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(( ((sKernel[j]-u)/sigma )*( (sKernel[j]-u)/sigma))-1)*(exp((-( (sKernel[j]-u)*(sKernel[j]-u) )) / (2 * sigma *sigma)));
	   	}

		//Convolution
		for(j = 0; j<kernelWidth; ++j)
		{
			/*x0 += tmp.points[i+j].x * gaussian0[j];
			y0 += tmp.points[i+j].y * gaussian0[j];
			x1 += tmp.points[i+j].x * gaussian1[j];
			y1 += tmp.points[i+j].y * gaussian1[j];
			x2 += tmp.points[i+j].x * gaussian2[j];
			y2 += tmp.points[i+j].y * gaussian2[j];*/

			x0 += tmp.points[i+j].x * gaussian0[j];
			y0 += tmp.points[i+j].z * gaussian0[j];
			x1 += tmp.points[i+j].x * gaussian1[j];
			y1 += tmp.points[i+j].z * gaussian1[j];
			x2 += tmp.points[i+j].x * gaussian2[j];
			y2 += tmp.points[i+j].z * gaussian2[j];
			
			/*x1 += tmp.points[i+j].x * gaussian1[kernelWidth/2-1][j];
			y1 += tmp.points[i+j].y * gaussian1[kernelWidth/2-1][j];
			x2 += tmp.points[i+j].x * gaussian2[kernelWidth/2-1][j];
			y2 += tmp.points[i+j].y * gaussian2[kernelWidth/2-1][j];*/
			
			/*x1 += tmp.points[i+j].x * gaussian1[kernelWidth/2-1][j];
			y1 += tmp.points[i+j].z * gaussian1[kernelWidth/2-1][j];
			x2 += tmp.points[i+j].x * gaussian2[kernelWidth/2-1][j];
			y2 += tmp.points[i+j].z * gaussian2[kernelWidth/2-1][j];*/
		}

		convolvedCurveX[i] = x0;
		convolvedCurveY[i] = y0;
		//curvature[i] = (x1*y2-y1*x2);
		curvature[i] = (x1*y2-y1*x2) / pow((x1*x1+y1*y1), 1.5);	

		x0 = 0.0;
		y0 = 0.0;
		x1 = 0.0;
		y1 = 0.0;
		x2 = 0.0;
		y2 = 0.0;

		//if(isMaxScale && abs(curvature[i])<1e6)
			//containsZeroCurvature=true;

		if(curvature[i]<min)
			min = curvature[i];
		if(curvature[i]>max)
			max = curvature[i];

	}

	if((kernelFactor-0.8)<0.01 && (kernelFactor-0.8)>0)
	{
		//plt::plot(convolvedCurveX, convolvedCurveY);
		//plt::plot(curvature);
		plt::plot(s, curvature);
		plt::xlabel("t");
		plt::ylabel("k");
		plt::show();
		//cout<<"Diff:"<<(max-min)<<endl;
	}

	getCurvatureExtrema(curvature, s, keypoints, min, max, isMaxScale);
}

//void computeScaleSpace(vector<CurvatureTriplet> keypoints[SIGMA_SIZE][WIDTH_SIZE]) //, vector<CurvatureTriplet>& keypointsFinestScale)
void computeScaleSpace(PointCloud<PointXYZ> in, vector<float> gaussian1[NUM_GAUSSIANS], vector<float> gaussian2[NUM_GAUSSIANS], vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float>& s)
//void computeScaleSpace(PointCloud<PointXYZ> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float>& s)
{
	float scales[NUM_SCALES] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};
	//float scales[NUM_SCALES] = {0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85};
	//float scales[NUM_SCALES] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75};

	//float scales[NUM_SCALES] = {0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};

	vector<float> curvature;
	//vector<float> s;
 	
 	parametrizeCurve(in, s);

 	vector<float> gauss;
	vector<float> kernel0;

 	//iterate all scales
 	for(int i=0; i<NUM_SCALES; ++i)
	{
		curvature.clear();

		if(i==0 && in.size()<40)
			continue;

		//compute curvatures and store keypoints of current scale in keypoints[i]
		if(i!=NUM_SCALES-1)
			computeCurvature(in, gaussian1, gaussian2, scales[i], curvature, s, keypoints[i], gauss, kernel0, false);
		else
			computeCurvature(in, gaussian1, gaussian2, scales[i], curvature, s, keypoints[i], gauss, kernel0, true);
		
		printKeypoints(keypoints[i]);
	}
}

void getFinalKeypointsAtMinScale(PointCloud<PointXYZ> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float> s, PointCloud<PointXYZ>& keypointsCloud)
{
	vector<CurvatureTriplet> keypointsScaleIterator;
	keypointsScaleIterator = keypoints[NUM_SCALES-1];//initialize the keypointsScaleIterator = keypoints at highest scale, which are the final keypoints
	
	vector<CurvatureCounter> keypointsFinalCounter(keypointsScaleIterator.size());

	//initialize keypointsFinalCounter
	for(int i=0; i<keypointsFinalCounter.size(); ++i)
	{
		CurvatureCounter cc;
		cc.index = keypointsScaleIterator[i].index;
		cc.curvature = keypointsScaleIterator[i].curvature;
		cc.counter = 0;

		keypointsFinalCounter[i] = cc;
	}

	int idx;
	//int idxAux = -1;
	float threshold = 0.0;
	
	//Serch from the highest scale to the finest to find the final keypoints
	for(int i=NUM_SCALES-1; i>0; --i)
	{
		for(int j=0; j<keypointsScaleIterator.size(); ++j)
		{
			if(keypointsScaleIterator[j].index>1 && keypointsScaleIterator[j].index<in.size()-2)
			{
				//see if we choose as threshold, the difference to the left or to the right
				threshold = abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index-2])>abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index+2])?
						abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index-2]):abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index+2]);
			}

			//cout<<"threshold final: "<<threshold<<endl;

			if(i==1)
				idx = findByThresholdAtMinScale(keypointsScaleIterator[j], keypoints[i-1], threshold);
			else
				idx = findByThreshold(keypointsScaleIterator[j], keypoints[i-1], threshold);
	
			if(idx!=-1)
			{
				keypointsScaleIterator[j].index = idx; //overwriteScaleIterator
				keypointsScaleIterator[j].sIndex = s[idx]; //overwriteScaleIterator

				keypointsFinalCounter[j].index = idx;
				keypointsFinalCounter[j].counter = keypointsFinalCounter[j].counter + 1;
				
			}
			
			cout<<keypointsFinalCounter[j].index<<"->"<<keypointsFinalCounter[j].counter<<", ";	
		}

		cout<<endl;
	}

	keypointsCloud.clear();

	for(int j=0; j<keypointsFinalCounter.size(); ++j)
	{
		//if(keypointsFinalCounter[j].counter>6)
		if(keypointsFinalCounter[j].counter>10) //keypoints must have been found in at least 10 scales out of 16
			keypointsCloud.push_back(in[keypointsFinalCounter[j].index]);
	}
}