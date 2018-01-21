// util.h
#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#define EPS 2.2204e-16

using namespace std;

typedef struct curvaturePair
{
     float s; //parametrized s
     float curvature; //curvature
}CurvaturePair;

//void findPeaks(vector<CurvaturePair> x0, vector<int>& peakInds);
void findPeaks(vector<CurvaturePair> x0, float threshold, vector<int>& peakInds);

#endif
