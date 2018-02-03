// css.cpp
#include "css.h"

//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;


bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j)
{
	return (i.sIndex<j.sIndex);
}

bool compareCurvatureCounter (CurvatureCounter i, CurvatureCounter j)
{ 
	return (i.counter>j.counter);
}

int findByThreshold(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold, int& idx2)
{
	int idx1 = -1;
	idx2 = -1;
	float diff;
	float minDiff = FLOAT_MAX;
	//float maxCurvature = FLOAT_MIN;

	int sumIdx = 0;
	int counter = 0;

	for(int i=0; i<vec.size(); ++i)
	{
		diff = abs(a.sIndex-vec[i].sIndex);
		//if(diff<threshold)
		if(diff<threshold && diff<minDiff)
		//if( (diff<threshold && abs(vec[i].curvature)>maxCurvature) )
		{
			//sumIdx += vec[i].index;
			//counter++;

			//cout<<a.sIndex<<"<->"<<vec[i].sIndex<<endl;

			idx2 = i;
			//return vec[i].index;
			
			idx1 = vec[i].index;
			minDiff = diff;
			//maxCurvature = abs(vec[i].curvature);
		}
	}

	//idx1 = sumIdx/counter;

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
		a.x = in[i].x; b.x = in[i-1].x;
		a.y = in[i].y; b.y = in[i-1].y;
		a.z = 0.0   ;  b.z = 0.0;
		//d[i] = pcl::euclideanDistance(in[i], in[i-1]);
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

/*void getCurvatureExtrema(vector<float> curvature, vector<float> s, vector<CurvatureTriplet>& keypoints)
{

  float threshold = *max_element(curvature.begin(), curvature.end());
  //threshold = (threshold<0.0)?(threshold+(threshold/2.0)):(threshold/2.0);
  //threshold = (threshold<0.0)?(threshold+(threshold/10.0)):(threshold-(threshold/6.0));
  threshold = (threshold<0.0)?(threshold+(threshold/6.0)):(threshold-(threshold/6.0));

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

  //Agregar keypoints
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
  	curvature[i] = -curvature[i];//cambio de signo a la curvatura para hallar los minimos
  	CurvaturePair cp;
  	cp.s = s[i];
  	cp.curvature = curvature[i];
  	cv[i] = cp;

  }

  threshold = *max_element(curvature.begin(), curvature.end());
  //threshold = (threshold<0.0)?(threshold+(threshold/2.0)):(threshold/2.0);
  //threshold = (threshold<0.0)?(threshold+(threshold/10.0)):(threshold-(threshold/6.0));
  threshold = (threshold<0.0)?(threshold+(threshold/6.0)):(threshold-(threshold/6.0));

  peaksInds.clear();
  //findPeaks(curvature, peaksInds);
  findPeaks(cv, threshold, peaksInds);

  //Agregar keypoints
  for(int i=0; i<peaksInds.size(); ++i)
  {
  	c.index = peaksInds[i];
  	c.sIndex = s[peaksInds[i]];
    c.curvature = -curvature[peaksInds[i]];//lo devuelvo al signo
    keypoints.push_back(c);
  }
}*/

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

void getCurvatureExtrema(vector<float> curvature, vector<float> s, vector<CurvatureTriplet>& keypoints, float min, float max)
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
			if(curvature[i]>0 && curvature[i]>(max/4.3) ||
			   curvature[i]<0 && curvature[i]<(min/4.5)
			  )
			{
				CurvatureTriplet c;
				c.index = i;
				c.sIndex = s[i];
				c.curvature = curvature[i];
				keypoints.push_back(c);
			}
		}
	}
}

//void computeCurvature(PointCloud<PointXYZ> in, float sigma, int kernelWidth, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints)
//void computeCurvature(PointCloud<PointXYZ> in, float sigma, int width, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints)
//void computeCurvature(PointCloud<PointXYZ> in, float sigma, const int width, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints, vector<float>& gauss, vector<float>& kernel0)
void computeCurvature(PointCloud<PointXYZ> in, float kernelFactor, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints, vector<float>& gauss, vector<float>& kernel0)
{
	//kernel factor = percentage of the points to define the width and std deviation of gaussian kernel
	int m = in.size();
	int kernelWidth = ((int)((float)(m)*kernelFactor)%2==0)?(m*kernelFactor+1):(m*kernelFactor);
	float sigma;
	//sigmaCentro, sigmaIzquierda, sigmaDerecha;

	cout<<"width: "<<kernelWidth<<endl;

	PointCloud<PointXYZ> convolvedCurve;
	convolvedCurve.width = in.size();
	convolvedCurve.height = 1;
	convolvedCurve.resize(convolvedCurve.width * convolvedCurve.height);

	float gaussian0[kernelWidth];
	float gaussian1[kernelWidth];
	float gaussian2[kernelWidth];
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

	//parametrizeCurve(in, s);

	float sAux = s[0];
	vector<float> sTmp(m+2*extension);


	for(int i=0; i<extension; ++i)
	{
		tmp[i] = PointXYZ(in.points[0]);
		//sAux = sAux - (s[(extension-1)-(i+1)]-s[(extension-1)-i]);
		sAux = sAux - (s[i+1]-s[i]);
		sTmp[extension-1-i] = sAux;

		//yTmp[i] = in.points[0].y;
	}



	for(int i=0; i<in.points.size(); ++i)
	{
		tmp[i+extension] = PointXYZ(in.points[i]);
		sTmp[i+extension] = s[i];

		//yTmp[i+extension] = in.points[i].y;
	}

	sAux = s[m-1];

	//for(int i=in.points.size(); i<in.points.size()+extension; ++i)
	for(int i=0; i<extension; ++i)
	{
		//tmp[i+extension] = PointXYZ(in.points[in.points.size()-1]);
		tmp[i+m+extension] = PointXYZ(in.points[in.points.size()-1]);
		sAux = sAux + (s[m-i-1]-s[m-i-2]);
		//sTmp[m+i] = sAux;
		sTmp[i+m+extension] = sAux;

		//yTmp[i+m+extension] = in.points[in.points.size()-1].y;
	}


	//float sKernel[kernelWidth];

	float min = FLOAT_MAX;
	float max = FLOAT_MIN;

	//cout<<"gauss diff: "<<sTmp[kernelWidth-1]-sTmp[0]<<endl;
	//cout<<"gauss diff: "<<sTmp[(sTmp.size()/2)+(kernelWidth-1)]-sTmp[sTmp.size()/2]<<endl;

	for(i = 0; i < in.size(); ++i)
	{
		//float u = s[i];

		//From u to the left
		/*for(j=0; j<extension; ++j )
       		sKernel[extension-j-1] = sTmp[i+extension-j-1];

       	sKernel[extension] = u;

       	//From u to the right
       	for(j=0; j<extension; ++j )
       		sKernel[extension+j+1] = sTmp[i+extension+j+1];*/



       	float leftLimit = sTmp[i];
       	float rightLimit = sTmp[i+2*extension];
       	float u;
       	
       	VectorXd linspace = VectorXd::LinSpaced(kernelWidth,leftLimit,rightLimit);
  		vector<float> sKernel(linspace.data(), linspace.data() + linspace.rows());
  		u = sKernel[kernelWidth/2];

  		if(i==0)
  		{
  			cout<<"LL: "<<leftLimit<<", RL:"<<rightLimit<<"->Diff:"<<(rightLimit-leftLimit)<<endl;
  		}

       	//sigmaIzquierda = (u-sKernel[0])/3.5;
       	//sigmaCentro = u;
       	//sigmaDerecha = (sKernel[extension*2]-u)/3.5;
       	//sigma = (sigmaIzquierda<sigmaDerecha)?sigmaIzquierda:sigmaDerecha;

       	sigma = (u-sKernel[0])/3.5;

		for(j=0; j<kernelWidth; ++j)
		{
			gaussian0[j] = exp(-( (sKernel[j]-u)*(sKernel[j]-u) )/(2*sigma*sigma))/(sigma*sqrt(2*M_PI));
			gaussian1[j] = (-1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(sKernel[j]-u)*exp((-( (sKernel[j]-u)*(sKernel[j]-u)) ) / (2 * sigma*sigma)); //-sTmp[j+i]*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma))/(sigma*sigma*sigma*sqrt(2*M_PI));
			gaussian2[j] = (1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(( ((sKernel[j]-u)/sigma )*( (sKernel[j]-u)/sigma))-1)*(exp((-( (sKernel[j]-u)*(sKernel[j]-u) )) / (2 * sigma *sigma)));//((sTmp[j+i]/sigma)*(sTmp[j+i]/sigma)-1)/(sigma*sigma*sigma*sqrt(2*M_PI))*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma));
		}

		if(i==m/2)
		{
			gauss = vector<float>(gaussian1, gaussian1 + sizeof(gaussian1) / sizeof(float) );
			kernel0 = sKernel;
		}

		for(j = 0; j<kernelWidth; ++j)
		{
			x0 += tmp.points[i+j].x * gaussian0[j];
			y0 += tmp.points[i+j].y * gaussian0[j];
		    x1 += tmp.points[i+j].x * gaussian1[j];
			y1 += tmp.points[i+j].y * gaussian1[j];
			x2 += tmp.points[i+j].x * gaussian2[j];
			y2 += tmp.points[i+j].y * gaussian2[j];

			if(i==0)
			{
				cout<<"sTmp: "<<sTmp[i+j]<<endl;
				cout<<"sKernel: "<<sKernel[j]<<endl;
			}
		}

		curvature[i] = (x1*y2-y1*x2);
		convolvedCurve[i] = PointXYZ(x0, y0, 0);

		x0 = 0.0;
		y0 = 0.0;
		x1 = 0.0;
		y1 = 0.0;
		x2 = 0.0;
		y2 = 0.0;

		if(curvature[i]<min)
			min = curvature[i];
		if(curvature[i]>max)
			max = curvature[i];

	}
	
	getCurvatureExtrema(curvature, s, keypoints, min, max);
}

void computeCurvature(PointCloud<PointXYZ> in, float sigma, int width, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints)
{
	float factorWidth = 0.1 * (sigma*100);

	int widthAux = ceil((float)(in.size())*(factorWidth+(float)(width)*0.02));
	//int kernelWidth = (widthAux%2==0)?widthAux+1:widthAux;

	int kernelWidth = width;

	cout<<"width: "<<kernelWidth<<endl;

	PointCloud<PointXYZ> convolvedCurve;
	convolvedCurve.width = in.size();
	convolvedCurve.height = 1;
	convolvedCurve.resize(convolvedCurve.width * convolvedCurve.height);

	float gaussian0[kernelWidth];
	float gaussian1[kernelWidth];
	float gaussian2[kernelWidth];
	float sum = 0.0;

	int extension = kernelWidth/2;
	int m = in.size();
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

	//vector<float> yTmp(m+2*extension);


	int i, j, k;
	curvature = vector<float>(in.points.size());

	parametrizeCurve(in, s);

	float sAux = s[0];
	vector<float> sTmp(m+2*extension);

	for(int i=0; i<extension; ++i)
	{
		tmp[i] = PointXYZ(in.points[0]);
		//sAux = sAux - (s[(extension-1)-(i+1)]-s[(extension-1)-i]);
		sAux = sAux - (s[i+1]-s[i]);
		sTmp[extension-1-i] = sAux;

		//yTmp[i] = in.points[0].y;
	}



	for(int i=0; i<in.points.size(); ++i)
	{
		tmp[i+extension] = PointXYZ(in.points[i]);
		sTmp[i+extension] = s[i];

		//yTmp[i+extension] = in.points[i].y;
	}

	sAux = s[m-1];

	//for(int i=in.points.size(); i<in.points.size()+extension; ++i)
	for(int i=0; i<extension; ++i)
	{
		//tmp[i+extension] = PointXYZ(in.points[in.points.size()-1]);
		tmp[i+m+extension] = PointXYZ(in.points[in.points.size()-1]);
		sAux = sAux + (s[m-i-1]-s[m-i-2]);
		//sTmp[m+i] = sAux;
		sTmp[i+m+extension] = sAux;

		//yTmp[i+m+extension] = in.points[in.points.size()-1].y;
	}


	//plt::plot(sTmp, yTmp);
	//plt::show();



	//VectorXd linspace = VectorXd::LinSpaced(kernelWidth,-(float)kernelWidth/2.0, (float)kernelWidth/2.0);
	//vector<float> sTmp(linspace.data(), linspace.data() + linspace.rows());

	//for(int i=0; i<kernelWidth; ++i)
		//cout<<sTmp[i]<<endl;

	//int sizeTmp = in.points.size()+extension*2;

	/*for(j=0; j<kernelWidth; ++j)
	{
		gaussian1[kernelWidth-1-j] = (-1/(sigma*sigma*sigma*sqrt(2*M_PI)))*sTmp[j]*exp((-(sTmp[j]*sTmp[j])) / (2 * sigma*sigma)); //-sTmp[j+i]*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma))/(sigma*sigma*sigma*sqrt(2*M_PI));
		gaussian2[kernelWidth-1-j] = (1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(((sTmp[j]/sigma)*(sTmp[j]/sigma))-1)*(exp((-(sTmp[j]*sTmp[j])) / (2 * sigma *sigma)));//((sTmp[j+i]/sigma)*(sTmp[j+i]/sigma)-1)/(sigma*sigma*sigma*sqrt(2*M_PI))*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma));
		gaussian3[kernelWidth-1-j] = exp(-(sTmp[j]*sTmp[j])/(2*sigma*sigma))/(sigma*sqrt(2*M_PI));
	}*/

	vector<float> sKernel(kernelWidth);

	float min = FLOAT_MAX;
	float max = FLOAT_MIN;

	for(i = 0; i < in.size(); ++i)
	{

		float u = s[i];

		//From u to the left
		for(j=0; j<extension; ++j )
       		sKernel[extension-j-1] = sTmp[i+extension-j-1];

       	sKernel[extension] = u;

       	//From u to the right
       	for(j=0; j<extension; ++j )
       		sKernel[extension+j+1] = sTmp[i+extension+j+1];
    

		for(j=0; j<kernelWidth; ++j)
		{
			gaussian0[j] = exp(-( (sKernel[j]-u)*(sKernel[j]-u) )/(2*sigma*sigma))/(sigma*sqrt(2*M_PI));
			gaussian1[j] = (-1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(sKernel[j]-u)*exp((-( (sKernel[j]-u)*(sKernel[j]-u)) ) / (2 * sigma*sigma)); //-sTmp[j+i]*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma))/(sigma*sigma*sigma*sqrt(2*M_PI));
			gaussian2[j] = (1/(sigma*sigma*sigma*sqrt(2*M_PI)))*(( ((sKernel[j]-u)/sigma )*( (sKernel[j]-u)/sigma))-1)*(exp((-( (sKernel[j]-u)*(sKernel[j]-u) )) / (2 * sigma *sigma)));//((sTmp[j+i]/sigma)*(sTmp[j+i]/sigma)-1)/(sigma*sigma*sigma*sqrt(2*M_PI))*exp(-(sTmp[j+i]*sTmp[j+i])/(2*sigma*sigma));
		}


		if(i==0)
		{
			//printVector(sKernel);
			//std::vector<float> y(gaussian0, gaussian0 + sizeof(gaussian0) / sizeof(float) );
			//std::vector<float> y(gaussian1, gaussian1 + sizeof(gaussian1) / sizeof(float) );
			//std::vector<float> y(gaussian2, gaussian2 + sizeof(gaussian2) / sizeof(float) );
			//plt::plot(sKernel, y);
		}

		for(j = 0; j<kernelWidth; ++j)
		{
			x0 += tmp.points[i+j].x * gaussian0[j];
			y0 += tmp.points[i+j].y * gaussian0[j];
		    x1 += tmp.points[i+j].x * gaussian1[j];
			y1 += tmp.points[i+j].y * gaussian1[j];
			x2 += tmp.points[i+j].x * gaussian2[j];
			y2 += tmp.points[i+j].y * gaussian2[j];
		}

		curvature[i] = (x1*y2-y1*x2);
		convolvedCurve[i] = PointXYZ(x0, y0, 0);

		x0 = 0.0;
		y0 = 0.0;
		x1 = 0.0;
		y1 = 0.0;
		x2 = 0.0;
		y2 = 0.0;

		if(curvature[i]<min)
			min = curvature[i];
		if(curvature[i]>max)
			max = curvature[i];

	}


	vector<float> x(m);

	for(int i=0; i<m; ++i)
		x[i] = i;

	//plt::figure();

	//vector<float> sFinal;
	//parametrizeCurve(convolvedCurve, sFinal);
	//getCurvatureExtrema(curvature, sFinal, keypoints);
	getCurvatureExtrema(curvature, s, keypoints, min, max);

	//printKeypoints(keypoints);

	//plt::plot(x, curvature);
	//plt::plot(sFinal, curvature);
	//plt::plot(s, curvature);
	//plt::show();

	//for(i=0; i<extension; ++i)
	//	curvature[i] = curvature[extension];

	//for(i=curvature.size()-1; i>curvature.size()-1-extension; --i)
	//	curvature[i] = curvature[curvature.size()-1-extension];


	//parametrizeCurve(convolvedCurve, s);
	//getCurvatureExtrema(curvature, s, keypoints);
	//printKeypoints(keypoints);

}

//void computeScaleSpace(vector<CurvatureTriplet> keypoints[SIGMA_SIZE][WIDTH_SIZE]) //, vector<CurvatureTriplet>& keypointsFinestScale)
void computeScaleSpace(PointCloud<PointXYZ> in, vector<CurvatureTriplet> keypoints[SIGMA_SIZE][WIDTH_SIZE], vector<CurvatureTriplet>& keypointsFinestScale) //, vector<CurvatureTriplet>& keypointsFinestScale)
{
	int sigma, width;

	vector<float> curvature;
	vector<float> s;

	for(int i=0; i<SIGMA_SIZE; ++i)
	{
		sigma = i+4;

		//cout<<"sigma:"<<sigma<<endl;

		for(int j=0; j<WIDTH_SIZE; ++j)
		{
			curvature.clear();
			s.clear();

			width = ((sigma*(j+2))%2==0)?sigma*(j+2)+1:sigma*(j+2);

			//cout<<"width:"<<width<<endl;

			computeCurvature(in, width, sigma, curvature, s, keypoints[i][j]);

			sort(keypoints[i][j].begin(), keypoints[i][j].end(), compareCurvatureTriplet);

			//printKeypoints(keypoints[i][j]);
			
			if(i==0)
				keypointsFinestScale.insert(keypointsFinestScale.end(), keypoints[i][j].begin(), keypoints[i][j].end());

			if(i==0 && j==0)
			{
				//plt::plot(s, curvature);
			}
		}
	}

	sort(keypointsFinestScale.begin(), keypointsFinestScale.end(), compareCurvatureTriplet);

	//plt::show();
}

void getKeypointsAtFinestScale(vector<CurvatureTriplet> keypointsFinestScale, vector<CurvatureTriplet> keypointsAtLastWidth, vector<CurvatureTriplet>& selectedKeypointsFinestScale)
{
	//for(int k=0; k<keypoints[0][WIDTH_SIZE-1].size(); ++k)
	for(int k=0; k<keypointsAtLastWidth.size(); ++k)
	{
		int idx = findCurvatureTriplet(keypointsFinestScale, keypointsAtLastWidth[k]);
		
		if(idx==0)
			continue;

		int j=idx;
		int i=idx;
		int counter = 0;

		//cout<<"Find:"<<keypointsAtLastWidth[k].sIndex<<"->"<<idx<<endl;

		vector<int> indices;//para tomar el valor intermedio de los keypointsFinestScale
		indices.push_back(keypointsFinestScale[j].index);//para tomar el valor intermedio de los keypointsFinestScale
		
		//while((keypointsFinestScale[j].sIndex-keypointsFinestScale[j-1].sIndex)<4.29e-3 && j>-1)
		//while((keypointsFinestScale[j].sIndex-keypointsFinestScale[j-1].sIndex)<0.01 && j>-1)
		while((keypointsFinestScale[j].sIndex-keypointsFinestScale[j-1].sIndex)<0.033 && j>-1)//hacia la izquierda de los keypoints
		//while((keypointsFinestScale[j].sIndex-keypointsFinestScale[j-1].sIndex)<4.29e-3 && j>-1)
		{
			//cout<<j<<","<<j-1<<endl;
			j--;
			indices.push_back(keypointsFinestScale[j].index);//para tomar el valor intermedio de los keypointsFinestScale
			counter++;
		}

		//while((keypointsFinestScale[i+1].sIndex-keypointsFinestScale[i].sIndex)<0.033 && i<keypointsFinestScale.size()-1)
		while((keypointsFinestScale[i+1].sIndex-keypointsFinestScale[i].sIndex)<0.033 && i<keypointsFinestScale.size()-1)//hacia la derecha de los keypoints
		{
			//cout<<j<<";"<<j-1<<endl;
			i++;
			indices.push_back(keypointsFinestScale[i].index);//para tomar el valor intermedio de los keypointsFinestScale
			counter++;
		}

		//tomar el valor intermedio de los keypointsFinestScale
		sort(indices.begin(), indices.end());
		keypointsFinestScale[j].index = indices[indices.size()/2];
		//tomar el valor intermedio de los keypointsFinestScale

		//cout<<"Counter: "<<counter<<endl;

		//if(counter>=3)
		if(counter>=1)
			selectedKeypointsFinestScale.push_back(keypointsFinestScale[j]);
	}
}

void getFinalKeypoints(PointCloud<PointXYZ> in, vector<CurvatureTriplet> selectedKeypointsFinestScale, vector<CurvatureTriplet> keypoints[SIGMA_SIZE][WIDTH_SIZE],
					   PointCloud<PointXYZ>& keypointsCloud, vector<CurvatureCounter>& curvatureCounter)
{

	curvatureCounter = vector<CurvatureCounter>(in.size());
	
	for(int i=1; i<SIGMA_SIZE; ++i)
	{
		for(int j=0; j<WIDTH_SIZE; ++j)
		{
			for(int k=0; k<selectedKeypointsFinestScale.size(); ++k)
			{
				//if(findByThreshold(keypointsFinestScale[k], keypoints[i][j], 0.05)!=-1)
				//int idx = findByThreshold(selectedKeypointsFinestScale[k], keypoints[i][j], 0.025);
				int idx2 = -1;
				int idx = findByThreshold(selectedKeypointsFinestScale[k], keypoints[i][j], 0.033, idx2);
				//int idx = findByThreshold(selectedKeypointsFinestScale[k], keypoints[i][j], 0.025, idx2);

				if(idx!=-1 && idx2!=-1)
				{
					//cout<<"i:"<<i<<"j:"<<j<<"->"<<keypoints[i][j][idx].curvature<<endl; 
					//counterPlot[selectedKeypointsFinestScale[k].index] = counterPlot[selectedKeypointsFinestScale[k].index]+1;
					curvatureCounter[selectedKeypointsFinestScale[k].index].index = selectedKeypointsFinestScale[k].index;
					curvatureCounter[selectedKeypointsFinestScale[k].index].counter = curvatureCounter[selectedKeypointsFinestScale[k].index].counter+1;
					//curvatureCounter[selectedKeypointsFinestScale[k].index].curvature = keypoints[i][j][idx].curvature;
					curvatureCounter[selectedKeypointsFinestScale[k].index].curvature = keypoints[i][j][idx2].curvature;
					//cout<<"i:"<<i<<endl;
				}
			}
		}
	}

	sort(curvatureCounter.begin(), curvatureCounter.end(), compareCurvatureCounter);

	/*for(int i=0; i<3; ++i)
	{
		//cout<<"index: "<<curvatureCounter[i].index<<endl;
    	cout<<"counter: "<<curvatureCounter[i].counter<<endl;
    	//cout<<"curvature: "<<curvatureCounter[i].curvature<<endl;
    	
    	//if(curvatureCounter[i].counter>0)
    		//keypointsCloud.push_back(in.points[curvatureCounter[i].index]);
    }

    cout<<"counterddd: "<<curvatureCounter[0].counter<<endl;*/

    if(curvatureCounter[0].counter>23)
		keypointsCloud.push_back(in.points[curvatureCounter[0].index]);
	if(curvatureCounter[1].counter>10)
		keypointsCloud.push_back(in.points[curvatureCounter[1].index]);
	if(curvatureCounter[2].counter>1)
		keypointsCloud.push_back(in.points[curvatureCounter[2].index]);

}
