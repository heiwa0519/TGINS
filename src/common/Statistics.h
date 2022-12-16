//
// Created by hw on 12/10/22.
//

#ifndef FUSING_STATISTICS_H
#define FUSING_STATISTICS_H

#include <cmath>
#include <algorithm>
#include <cstring>

extern double calcSum(const double *data,int n,bool abs);
extern double calcMax(const double *data,int n);
extern double calcAve(const double *data,int n,bool abs);
extern double calcRMSE(const double *data,int n,bool abs);
extern double calcSTD(const double *data,int n,bool abs);
extern double calcCEP(const double *data,int n,int nSigma);

#endif //FUSING_STATISTICS_H
