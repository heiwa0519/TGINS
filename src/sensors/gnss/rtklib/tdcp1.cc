//
// Created by hw on 11/17/22.
//

#include "rtklib.h"
#include "Fusing.h"

extern int calTdcp(const obsd_t *obs,int n,ssat_t *ssat)
{
    int i,j,sat,isat[MAXSAT]={0},k=0;
    double tt,tdcp,dop,sec;
    gtime_t t0={0};

    sec= time2gpst(obs->time, nullptr);
    for(i=0;i<n;i++) for(j=0;j<NFREQ;j++){
        sat=obs[i].sat;
        if (obs[i].L[j]==0.0||obs[i].D[j]==0.0||ssat[sat-1].tdcp.pre_L[0][j]==0.0) continue;
        if ((tt=timediff(obs[i].time,ssat[sat-1].tdcp.pre_t[0][j]))<DTTOL) continue;
            tdcp=(obs[i].L[j]-ssat[sat-1].tdcp.pre_L[0][j])/tt;
            dop=-obs[i].D[j];
            ssat[sat-1].td_dif[j]=tdcp-dop;
            ssat[sat-1].tdcp.val[j]=tdcp;
            isat[sat-1]=1;
            k++;
    }

    for(i=0;i<MAXSAT;i++){
        if(!isat[i]){
            for(j=0;j<NFREQ;j++){
                ssat[i].tdcp.pre_L[0][j]=0.0;
                ssat[i].tdcp.pre_t[0][j]=t0;
                ssat[i].tdcp.val[j]=0.0;
            }
        }
    }

    /* save phase measurements */
    for (i=0;i<n;i++) for (j=0;j<2;j++) {
            if (obs[i].L[j]==0.0) continue;
            ssat[obs[i].sat-1].tdcp.pre_t[obs[i].rcv-1][j]=obs[i].time;
            ssat[obs[i].sat-1].tdcp.pre_L[obs[i].rcv-1][j]=obs[i].L[j];
    }

    return k;
}

extern int calRange(const prcopt_t *opt,const obsd_t *obs,int n,const nav_t *nav,const double *rr,ssat_t *ssat)
{
    int i,j=0,sat,sys;
    double r,e[3],*rs,*dts,*var;
    int svh[MAXOBS];

    rs=mat(6,n); dts=mat(2,n); var=mat(1,n);

    satposs(obs[0].time,obs,n,nav,opt->sateph,rs,dts,var,svh);

    for(i=0;i<n&&i<MAXOBS;i++){

        sat=obs[i].sat;
//        if(!ssat[sat-1].vs) continue;

        /* geometric distance and elevation mask*/
        if ((r=geodist(rs+i*6,rr,e))<=0.0) continue;

        ssat[sat-1].tdcp.pre_rho=r;

        j++;
    }

    free(rs),free(dts),free(var);
    return j;
}