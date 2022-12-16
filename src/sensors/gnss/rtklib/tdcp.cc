//
// Created by hw on 11/17/22.
//

#include "rtklib.h"
#include "Fusing.h"

extern void calTdcp1(const obsd_t *obs,int n,ssat_t *ssat)
{
    int i,j,sat,isat[MAXSAT]={0};
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
    }

    for(i=0;i<MAXSAT;i++){
        if(!isat[i]){
            for(j=0;j<NFREQ;j++){
                ssat[i].tdcp.pre_L[0][j]=0.0;
                ssat[i].tdcp.pre_t[0][j]=t0;
            }
        }
    }
    /* save phase measurements */
    for (i=0;i<n;i++) for (j=0;j<2;j++) {
            if (obs[i].L[j]==0.0) continue;
            ssat[obs[i].sat-1].tdcp.pre_t[obs[i].rcv-1][j]=obs[i].time;
            ssat[obs[i].sat-1].tdcp.pre_L[obs[i].rcv-1][j]=obs[i].L[j];
    }
}

/* range rate residuals ------------------------------------------------------*/
extern int restdcp1(const prcopt_t *opt,const obsd_t *obs, int n, const double *rs, const double *dts,
                  const nav_t *nav, const double *rr, const double *x,
                  const double *azel, const int *vsat, double err, double *v,
                  double *H,ssat_t *ssat)
{
    double freq,rate,pos[3],E[9],a[3],e[3],vs[3],cosel,sig;
    int i,j,nv=0,sat,sys;

    trace(3,"restdcp  : n=%d\n",n);

    ecef2pos(rr,pos); xyz2enu(pos,E);

    for (i=0;i<n&&i<MAXOBS;i++) {
        sat=obs[i].sat;
        sys=satsys(sat, nullptr);
        freq=sat2freq(sat,obs[i].code[0],nav);

        if (ssat[sat-1].tdcp.val[0]==0.0||freq==0.0||!vsat[i]||norm(rs+3+i*6,3)<=0.0) {
            continue;
        }
        /* LOS (line-of-sight) vector in ECEF */
        cosel=cos(azel[1+i*2]);
        a[0]=sin(azel[i*2])*cosel;
        a[1]=cos(azel[i*2])*cosel;
        a[2]=sin(azel[1+i*2]);
        matmul("TN",3,1,3,1.0,E,a,0.0,e);

        /* satellite velocity relative to receiver in ECEF */
        for (j=0;j<3;j++) {
            vs[j]=rs[j+3+i*6]-x[j];
        }
        /* range rate with earth rotation correction */
        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*x[0]-
                                      rs[3+i*6]*rr[1]-rs[  i*6]*x[1]);

        /* Std of range rate error (m/s) */
        sig=((err<=0.0)?1.0:err*CLIGHT/freq)/sin(azel[1+i*2]);
//        sig=sqrt(varerr(opt,&ssat[i],&obs[i],azel[1+i*2],sys)/10000.0*2)*CLIGHT/freq;

        /* range rate residual (m/s) */
        v[nv]=(ssat[sat-1].tdcp.val[0]*CLIGHT/freq-(rate+x[3]-CLIGHT*dts[1+i*2]))/sig;
        trace(2,"sat=%2d v=%.3f T=%.3f rate=%.3f ddtr=%.3f ddts=%.3f\n",obs[i].sat,v[nv],ssat[sat-1].tdcp.val[0]*CLIGHT/freq,rate,x[3],dts[1+i*2]*CLIGHT);

        /* design matrix */
        for (j=0;j<4;j++) {
            H[j+nv*4]=((j<3)?-e[j]:1.0)/sig;
        }
        nv++;
    }
    return nv;
}
