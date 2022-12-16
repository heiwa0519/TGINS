//
// Created by hw on 12/10/22.
//

#include "Logger.h"
#include "Statistics.h"

using namespace std;

extern double calcSum(const double *data,int n,bool abs)
{
    double sum=0.0;
    if(abs){
        for(int i=0;i<n;i++) sum+=fabs(data[i]);
    }
    else{
        for(int i=0;i<n;i++) sum+=data[i];
    }

    return sum;
}

extern double calcMax(const double *data,int n)
{
    double max=0.0;
    for(int i=0;i<n;i++){
        if(fabs(data[i])>max) max=fabs(data[i]);
    }
    return max;
}

extern double calcAve(const double *data,int n,bool abs)
{
    double sum= calcSum(data,n,abs);
    return sum/n;
}


extern double calcRMSE(const double *data,int n,bool abs)
{
    double sum= calcSum(data,n,abs);
    return sqrt(fabs(sum)/n);
}

extern double calcSTD(const double *data,int n,bool abs)
{
    double ave= calcAve(data,n,abs);

    double sum=0.0;
    for(int i=0;i<n;i++){
        sum+=(data[i]-ave)*(data[i]-ave);
    }

    return sqrt(fabs(sum)/n);
}

static int cmp(const void *p1, const void *p2)
{
    double *q1=(double *)p1,*q2=(double *)p2;
    return fabs(*q1)>fabs(*q2);
}

extern double calcCEP(const double *data,int n,int nSigma)
{
    double *new_data;

    if(nSigma<1||nSigma>3) return 0.0;

    if (!(new_data=(double *)malloc(sizeof(double)*n))) {
        LOG(ERROR)<<"matrix memory allocation error\n";
    }

    memcpy(new_data,data,sizeof(double)*n);

    for(int i=0;i<n;i++){
        new_data[i]= fabs(new_data[i]);
    }

    qsort(new_data,n,sizeof(double),cmp);

    int nn;
    if(nSigma==1){
        nn=n*0.68;
    }
    else if(nSigma==2){
        nn=n*0.95;
    }
    else if(nSigma==3){
        nn=n*0.99;
    }
    double cep=new_data[nn];

    free(new_data);
    return cep;
}