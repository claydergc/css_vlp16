// css.cpp
#include "css.h"


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
	for(int i=0; i<vec.size(); ++i)
		if(abs(a.sIndex-vec[i].sIndex)<threshold)
			return vec[i].index;
	return -1;
}

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

void computeCurvature(PointCloud<PointXYZ> in, int kernelWidth, float sigma, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints)
{

	PointCloud<PointXYZ> convolvedCurve;
	convolvedCurve.width = in.size();
	convolvedCurve.height = 1;
	convolvedCurve.resize(in.size());

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
		    x1 += tmp.points[i+j].x * gaussian1[j];
			y1 += tmp.points[i+j].y * gaussian1[j];
			x2 += tmp.points[i+j].x * gaussian2[j];
			y2 += tmp.points[i+j].y * gaussian2[j];

			x += tmp.points[i+j].x * gaussian3[j];
			y += tmp.points[i+j].y * gaussian3[j];
		}

		curvature[i] = (x1*y2-y1*x2);
		convolvedCurve[i] = PointXYZ(x, y, 0);


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