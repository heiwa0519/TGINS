//
// Created by hw on 10/30/22.
//

#include "TightCouple.h"
#include "rtklib.h"

#define SQR(x) ((x)*(x))

#define MAX_GDOP 30.0
#define MAX_ITER 5

static int sdHVR(int nm,int nx,const double *SD_mat,double *H,double *v,double *R,int tra)
{
    double *H_,*v_,*R_;

    v_= zeros(nm-1,1);
    R_= zeros(nm-1,nm-1);
    H_= zeros(nx,nm-1);

    if(v) matmul("NN",nm-1,1,nm,1.0,SD_mat,v,0.0,v_);
    if(R) matmul33("NNT",SD_mat,R,SD_mat,nm-1,nm,nm,nm-1,R_);
    if(H) matmul("NT",nx,nm-1,nm,1.0,H,SD_mat,0.0,H_);

    if(tra){
        matprint(0,SD_mat,nm-1,nm,15,6,"SD_mat:");

        matprint(1,v_,nm-1,1,15,6,"v_:");
        matprint(1,v,nm,1,15,6,"v:");

        matprint(1,R_,nm-1,nm-1,15,6,"R_:");
        matprint(1,R,nm,nm,15,6,"R:");

        matprint(1,H_,nx,nm-1,15,6,"H_:");
        matprint(1,H,nx,nm,15,6,"H:");
    }

    if(v) matcpy(v,v_,nm-1,1);
    if(R) matcpy(R,R_,nm-1,nm-1);
    if(H) matcpy(H,H_,nx,nm-1);

    free(H_);free(v_);free(R_);

    return nm-1;
}
/* pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, const obsd_t *obs, int n, const double *rs,
                      const double *dts, const double *vare, const int *svh,
                      const nav_t *nav,const double *rr, const double *x, const prcopt_t *opt,
                      const ssat_t *ssat, double *v, double *H, double *var,
                      double *azel, int *vsat, double *resp, int *ns,rtk_t *rtk,double *SD_mat)
{
    gtime_t time;
    double r,freq,dion=0.0,dtrp=0.0,vmeas,vion=0.0,vtrp=0.0,pos[3],dtr,e[3],P,max_el=0.0;
    int i,j,k=0,nv=0,nm=0,sat,sys,mask[6]={0};
    int iclk= xiClk(rtk->fusing->opts),nx=rtk->fusing->ep_kf.nx,max_el_i=0;

    dtr=SD_mat?0.0:x[iclk];

    ecef2pos(rr,pos);
    trace(3,"rescode: rr=%.3f %.3f %.3f\n",rr[0], rr[1], rr[2]);

    for (i=*ns=0;i<n&&i<MAXOBS;i++) {
        vsat[i]=0; azel[i*2]=azel[1+i*2]=resp[i]=0.0;
        time=obs[i].time;
        sat=obs[i].sat;
        if (!(sys=satsys(sat, nullptr))) continue;

        /* reject duplicated observation data */
        if (i<n-1&&i<MAXOBS-1&&sat==obs[i+1].sat) {
            trace(2,"duplicated obs data %s sat=%d\n",time_str(time,3),sat);
            i++;
            continue;
        }
        /* excluded satellite? */
        if (satexclude(sat,vare[i],svh[i],opt)) continue;

        /* geometric distance and elevation mask*/
        if ((r=geodist(rs+i*6,rr,e))<=0.0) continue;
        if (satazel(pos,e,azel+i*2)<opt->elmin) continue;

        /* test SNR mask */
        if (!snrmask(obs+i,azel+i*2,opt)) continue;

        /* ionospheric correction */
        if (!ionocorr(time,nav,sat,pos,azel+i*2,opt->ionoopt,&dion,&vion)) {
            continue;
        }
        if ((freq=sat2freq(sat,obs[i].code[0],nav))==0.0) continue;
        dion*=SQR(FREQL1/freq);
        vion*=SQR(FREQL1/freq);

        /* tropospheric correction */
        if (!tropcorr(time,nav,pos,azel+i*2,opt->tropopt,&dtrp,&vtrp)) {
            continue;
        }
        /* psendorange with code bias correction */
        if ((P=prange(obs+i,nav,opt,&vmeas))==0.0) continue;

        /* pseudorange residual */
        v[nv]=P-(r+dtr-CLIGHT*dts[i*2]+dion+dtrp);

        k++;
        if(azel[i*2+1]>max_el){
            max_el=azel[i*2+1];max_el_i=k;
        }

        trace(4,"sat=%d: v=%.3f P=%.3f r=%.3f dtr=%.6f dts=%.6f dion=%.3f dtrp=%.3f\n",
              sat,v[nv],P,r,dtr,dts[i*2],dion,dtrp);

        /* design matrix for position */
        if(rtk->fusing->opts.imu.nav_coord==+E_InsNavCoord::LLH){
            Vector3d mu;
            for(int k=0;k<3;k++) mu[k]=e[k];
            mu=mu.transpose()*rtk->fusing->ins.earth.DPne;
            for(j=xiP();j<xiP()+xnP();j++){
                H[j+nv*nx]=mu[j-xiP()];                     /* translation of innovation to position states */
            }
        }
        else{
            for(j=xiP();j<xiP()+xnP();j++){
                H[j+nv*nx]=e[j-xiP()];
            }
        }

        /*design matrix for attitude*/

        /*design matrix for receiver clock*/
        if(!SD_mat){
            for(j=iclk;j<iclk+1;j++){
                H[j+nv*nx]=1.0;
            }
            /* time system offset and receiver bias correction */
            if      (sys==SYS_GLO) {v[nv]-=x[iclk+1]; H[iclk+1+nv*nx]=1.0; mask[1]=1;}
            else if (sys==SYS_GAL) {v[nv]-=x[iclk+2]; H[iclk+2+nv*nx]=1.0; mask[2]=1;}
            else if (sys==SYS_CMP) {v[nv]-=x[iclk+3]; H[iclk+3+nv*nx]=1.0; mask[3]=1;}
            else if (sys==SYS_IRN) {v[nv]-=x[iclk+4]; H[iclk+4+nv*nx]=1.0; mask[4]=1;}
#if 0 /* enable QZS-GPS time offset estimation */
                else if (sys==SYS_QZS) {v[nv]-=x[8]; H[8+nv*NX]=1.0; mask[5]=1;}
#endif
            else mask[0]=1;
        }
        vsat[i]=obs[i].sat; resp[i]=v[nv]; (*ns)++;

        /* variance of pseudorange error */
        var[nv++]=varerr(opt,&ssat[i],&obs[i],azel[1+i*2],sys)+vare[i]+vmeas+vion+vtrp;
        trace(4,"sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n",obs[i].sat,
              azel[i*2]*R2D,azel[1+i*2]*R2D,resp[i],sqrt(var[nv-1]));
    }

    /* constraint to avoid rank-deficient */
    nm=nv;
    if(!SD_mat){
        for (i=0;i<6;i++) {
            if (mask[i]) continue;
            v[nv]=0.0;
            for (j=0;j<nx;j++) H[j+nv*nx]=j==i+iclk?1.0:0.0;
            var[nv++]=0.01;
        }
    }
    else{
        for(i=0;i<nm-1;i++){
            for(j=0;j<nm;j++){
                if(j==max_el_i-1) SD_mat[i+j*(nm-1)]=1.0;
                else SD_mat[i+i*(nm-1)]=-1.0;
            }
        }
        rtk->ssat[obs[max_el_i].sat-1].ref=1;
    }

    return (SD_mat)?nm:nv;
}

/* pseudorange residuals -----------------------------------------------------*/
static int resPR(int iter,const prcopt_t *opt, const obsd_t *obs, int n, const nav_t *nav,
                 const double *rs,const double *dts, const double *vare, const int *svh,
                 const double *rr, const double *x,
                 rtk_t *rtk, double *v, double *H, double *R,double *SD_mat,int *ns)
{
    gtime_t time;
    double r,freq,dion=0.0,dtrp=0.0,vmeas,vion=0.0,vtrp=0.0,pos[3],dtr,e[3],P,max_el=0.0;
    int i,j,k=0,nv=0,nm=0,sat,sys,mask[6]={0},isat[MAXOBS]={0},tra=4;
    int iclk=xiClk(rtk->fusing->opts),nx=rtk->fusing->ep_kf.nx,max_el_i=0;
    double azel[MAXOBS*2],var[MAXOBS],sec;

    ecef2pos(rr,pos);
    trace(3,"rescode: rr=%.3f %.3f %.3f\n",rr[0], rr[1], rr[2]);
    sec= time2gpst(obs[0].time, nullptr);

    dtr=SD_mat?0.0:x[iclk];
    for (i=0;i<n&&i<MAXOBS;i++) {
        azel[i*2]=azel[1+i*2]=0.0;
        time=obs[i].time;
        sat=obs[i].sat;
        if (!(sys=satsys(sat, nullptr))) continue;

        /* reject duplicated observation data */
        if (i<n-1&&i<MAXOBS-1&&sat==obs[i+1].sat) {
            trace(2,"duplicated obs data %s sat=%d\n",time_str(time,3),sat);
            i++;
            continue;
        }
        /* excluded satellite? */
        if (satexclude(sat,vare[i],svh[i],opt)) continue;

        /* geometric distance and elevation mask*/
        if ((r=geodist(rs+i*6,rr,e))<=0.0) continue;
        if (satazel(pos,e,azel+i*2)<opt->elmin) continue;

        /* test SNR mask */
        if (!snrmask(obs+i,azel+i*2,opt)) continue;

        /* ionospheric correction */
        if (!ionocorr(time,nav,sat,pos,azel+i*2,opt->ionoopt,&dion,&vion)) {
            continue;
        }
        if ((freq=sat2freq(sat,obs[i].code[0],nav))==0.0) continue;
        dion*=SQR(FREQL1/freq);
        vion*=SQR(FREQL1/freq);

        /* tropospheric correction */
        if (!tropcorr(time,nav,pos,azel+i*2,opt->tropopt,&dtrp,&vtrp)) {
            continue;
        }
        /* psendorange with code bias correction */
        if ((P=prange(obs+i,nav,opt,&vmeas))==0.0) continue;

        /* pseudorange residual */
        v[nv]=P-(r+dtr-CLIGHT*dts[i*2]+dion+dtrp);

        k++;
        if(azel[i*2+1]>max_el){
            max_el=azel[i*2+1];max_el_i=k;
        }
        rtk->ssat[sat-1].azel[0]=azel[2*i];
        rtk->ssat[sat-1].azel[1]=azel[2*i+1];

        /* design matrix for position */
        if(H){
            if(rtk->fusing->opts.imu.nav_coord==+E_InsNavCoord::LLH){
                Vector3d mu;
                for(int k=0;k<3;k++) mu[k]=e[k];
                mu=mu.transpose()*rtk->fusing->ins.earth.DPne;
                for(j=xiP();j<xiP()+xnP();j++){
                    H[j+nv*nx]=mu[j-xiP()];                     /* translation of innovation to position states */
                }
            }
            else{
                for(j=xiP();j<xiP()+xnP();j++){
                    H[j+nv*nx]=e[j-xiP()];
                }
            }

            /*design matrix for receiver clock*/
            if(!SD_mat){
                for(j=iclk;j<iclk+1;j++){
                    H[j+nv*nx]=1.0;
                }
                /* time system offset and receiver bias correction */
                if      (sys==SYS_GLO) {v[nv]-=x[iclk+1]; H[iclk+1+nv*nx]=1.0; mask[1]=1;}
                else if (sys==SYS_GAL) {v[nv]-=x[iclk+2]; H[iclk+2+nv*nx]=1.0; mask[2]=1;}
                else if (sys==SYS_CMP) {v[nv]-=x[iclk+3]; H[iclk+3+nv*nx]=1.0; mask[3]=1;}
                else if (sys==SYS_IRN) {v[nv]-=x[iclk+4]; H[iclk+4+nv*nx]=1.0; mask[4]=1;}
                else mask[0]=1;
            }
        }

        /* variance of pseudorange error */
        vion=0.0;
        var[nv]=varerr(opt,&rtk->ssat[obs[i].sat-1],&obs[i],azel[1+i*2],sys)+vare[i]+vmeas+vion+vtrp;
//        if(rtk->epoch>=1142) tra=4;
        trace(tra,"%05d(%d): sec=%7.2f sat=%2d, v=%6.3f P=%10.3f r=%10.3f dtr=%9.3f dts=%12.3f dion=%5.3f dtrp=%5.3f var=%5.2f el=%3.1f\n",
              rtk->epoch,iter,sec,sat,v[nv],P,r,dtr,dts[i*2]*CLIGHT,dion,dtrp,var[nv],azel[i*2+1]*R2D);
        isat[nv]=sat;
        nv++;
    }

    /* constraint to avoid rank-deficient */
    nm=nv;*ns=k;
    if(H){
        if(!SD_mat){
            for (i=0;i<6;i++) {
                if (mask[i]) continue;
                v[nv]=0.0;
                for (j=0;j<nx;j++) H[j+nv*nx]=j==i+iclk?1.0:0.0;
                var[nv++]=0.01;
            }
        }
        else{
            for(i=0;i<nm-1;i++){
                for(j=0;j<nm;j++){
                    if(j==max_el_i-1) SD_mat[i+j*(nm-1)]=1.0;
                    if(j==i){
                        if(j<max_el_i-1) SD_mat[i+j*(nm-1)]=-1.0;
                        else{
                            SD_mat[i+(j+1)*(nm-1)]=-1.0;
                        }
                    }

//                    if(j==i){
//                        SD_mat[i+j*(nm-1)]=-1.0;
//                    }
                }
            }
//            matprint(0,SD_mat,nm-1,nm,15,6,"SD:");
            rtk->ssat[obs[max_el_i].sat-1].ref=1;
        }

        for(i=0;i<nv;i++){
            for(j=0;j<nv;j++) R[i+j*nv]=i==j?var[i]:0.0;
        }

        if(SD_mat){
            nv=sdHVR(nv,nx,SD_mat,H,v,R,0);
        }
    }

    if(iter==0||iter==MAX_ITER){
        iter==0?j=1:j=0;
        for(i=0;i<nm;i++){
            sat=isat[i];
            if(SD_mat&&sat==obs[max_el_i].sat){
                rtk->ssat[sat-1].vs=1;
                continue;
            }
            rtk->ssat[sat-1].meas_res.code[j][0]=v[i];
            rtk->ssat[sat-1].vs=1;
            rtk->ssat[sat-1].vsat[0]=1;
        }
    }
    return nv;
}

static void updateClk(const fusingopt_t &opts,MatrixXd &P,VectorXd &x)
{
    for(int i= xiClk(opts);i< xiClk(opts)+xnClk(opts.gnss.mode,opts.gnss.sd_gnss);i++) {
        P(i,i)=3600.0;
        if(x[i]==0) x[i]=NO_ZERO_FLAG;
    }
}

static void updateClkDrift(const fusingopt_t &opts,MatrixXd &P,VectorXd &x)
{
    bool clk_drift=opts.imu.dop_aid==+E_SwitchOpt::ON||opts.imu.tdcp_aid==+E_SwitchOpt::ON;
    for(int i= xiClkDrift(opts);i< xiClkDrift(opts)+xnClkDrift(opts.gnss.mode,opts.gnss.sd_gnss,clk_drift);i++) {
        P(i,i)=3600.0;
        if(x[i]==0.0) x[i]=NO_ZERO_FLAG;
    }
}

/* validate solution ---------------------------------------------------------*/
static int valsolins(const double *azel, const int *vsat, int n,
                  const prcopt_t *opt, const double *v, int nv, int nx,
                  char *msg)
{
//    double azels[MAXOBS*2],dop[4],vv;
//    int i,ns;
//
//    trace(3,"valsol  : n=%d nv=%d\n",n,nv);
//
//    /* Chi-square validation of residuals */
//    vv=dot(v,v,nv);
//    if (nv>nx&&vv>chisqr[nv-nx-1]) {
//        sprintf(msg,"Warning: large chi-square error nv=%d vv=%.1f cs=%.1f",nv,vv,chisqr[nv-nx-1]);
//        /* return 0; */ /* threshold too strict for all use cases, report error but continue on */
//    }
//    /* large GDOP check */
//    for (i=ns=0;i<n;i++) {
//        if (!vsat[i]) continue;
//        azels[  ns*2]=azel[  i*2];
//        azels[1+ns*2]=azel[1+i*2];
//        ns++;
//    }
//    dops(ns,azels,opt->elmin,dop);
//    if (dop[0]<=0.0||dop[0]>MAX_GDOP) {
//        sprintf(msg,"gdop error nv=%d gdop=%.1f",nv,dop[0]);
//        return 0;
//    }
    return 1;
}

/* estimate receiver position ------------------------------------------------*/
static int tcFilterPR(const prcopt_t *opt, const obsd_t *obs, int n,const nav_t *nav,
                        const double *rs, const double *dts,const double *vare, const int *svh,
                        rtk_t *rtk, char *msg)
{
    bool sd=rtk->fusing->opts.gnss.sd_gnss;
    int nx=rtk->fusing->ep_kf.nx,i,j,k,nv,ns,info,tra=0;
    int nclk=xnClk(rtk->fusing->opts.gnss.mode,sd),iclk= xiClk(rtk->fusing->opts);
    int nm=sd?n:(n+nclk-1),stat=SOLQ_SINGLE;
    double *xp,*Pp,re[3],*v,*H,*var,*R,*SD_mat;
    ins_t ins=rtk->fusing->ins;

    v=zeros(nm,1); H=zeros(nx,nm); var=zeros(nm,1); SD_mat= mat(nm,nm);R=mat(nm,nm);
    xp= mat(nx,1);Pp= mat(nx,nx);

    matcpy(re,rtk->ins_pred.re,3,1);

    if(!sd){
        updateClk(rtk->fusing->opts,rtk->fusing->ep_kf.P,rtk->fusing->ep_kf.x);
    }

    matcpy(xp,rtk->fusing->ep_kf.x.data(),nx,1);

    for(i=0;i<MAX_ITER;i++){

        matcpy(Pp,rtk->fusing->ep_kf.P.data(),nx,nx);
        for(j=0;j<nm;j++) for(k=0;k<nm;k++) SD_mat[j+k*nm]=0.0;

        /* pseudorange residuals (m) */
        nv=resPR(i+1,opt,obs,n,nav,rs,dts,vare,svh,re,xp,rtk,v,H,R,sd?SD_mat:nullptr,&ns);

        if(nv<=0) return -1;

        if(rtk->epoch>=1142){
            tra=4;
        }
        if ((info=filter(xp,Pp,H,v,R,nx,nv,tra))) {
            stat=SOLQ_NONE;
            LOG(WARNING)<<rtk->epoch<<" "<<" filter error";
            break;
        }

        if(!insFeedback(rtk->fusing->opts.imu,rtk->fusing->opts.imu.nav_coord,rtk->fusing->opts.imu.att_def,ins,xp,nx)){
            LOG(WARNING)<<"ins feedback error";
            stat=SOLQ_NONE;
            break;
        }

        removeIGArmLever(ins, rtk->fusing->ins.ig_lever, reinterpret_cast<Vector3d &>(re), nullptr, true, rtk->fusing->opts.imu.nav_coord);
        if(rtk->fusing->opts.imu.nav_coord==+E_InsNavCoord::LLH){
            pos2ecef(re,re);
        }
    }

    if(stat){
        /*post-fit residuals*/
        resPR(0,opt,obs,n,nav,rs,dts,vare,svh,re,xp,rtk,v,nullptr,R,sd?SD_mat:nullptr,&ns);

        rtk->sol.stat=ins.gstat=opt->sateph==EPHOPT_SBAS?SOLQ_SBAS:SOLQ_SINGLE;
        ins.ns=ns;
        rtk->fusing->ig_ins=rtk->fusing->ins=ins;

        matcpy(rtk->fusing->ep_kf.x.data(),xp,nx,1);
        matcpy(rtk->fusing->ep_kf.P.data(),Pp,nx,nx);
        getStateCov(rtk->fusing->ep_kf.P,ins.state);
    }

    free(xp);free(Pp);
    free(v);free(H);free(var);free(R);
    free(SD_mat);
    return stat;
}

static int valdoppler(const double *x,const double *R,const double *v,int nv,double thres)
{
    double fact=SQR(thres);
    int nba,nbg,i;
    int iba,ibg;

    nba=xnBa();nbg=xnBg();
    iba=xnBa();ibg=xnBg();

    if(norm(x+xiA(),xnA())>15.0*D2R||(nba? norm(x+iba,nba)>1E5*MG2MPS2:false)
        ||(nbg? norm(x+ibg,nbg)>30.0*D2R:false)){
        LOG(ERROR)<<"too large estimated state error";
        return 0;
    }

    for(i=0;i<nv;i++){
        if(v[i]*v[i]<fact*R[i+i*nv]) continue;
        LOG(ERROR)<<" large doppler residual";
        return 0;
    }

    return 1;
}

/* range rate residuals ------------------------------------------------------*/
static int resdop(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const nav_t *nav, const double *rr,const double *ve, const double *x,
                  const double *azel, const int *vsat, double err, double *v,
                  double *H,double *var,int *vdop,rtk_t *rtk,double *SD_mat,int nx)
{
    int i,j,nv=0,max_el_i,k=0,iclk=xiClkDrift(rtk->fusing->opts),mask[6]={0},sys,nm;
    double freq,rate,pos[3],E[9],a[3],e[3],vs[3],cosel,sig;
    double max_el=0.0;
    double ddtr=SD_mat?0.0:x[iclk];

    trace(3,"resdop  : n=%d\n",n);

    ecef2pos(rr,pos); xyz2enu(pos,E);

    for (i=0;i<n&&i<MAXOBS;i++) {

        freq=sat2freq(obs[i].sat,obs[i].code[0],nav);
        sys=satsys(obs[i].sat, nullptr);

        if (obs[i].D[0]==0.0||freq==0.0||!vsat[i]||norm(rs+3+i*6,3)<=0.0) {
            continue;
        }
        /* LOS (line-of-sight) vector in ECEF */
        cosel=cos(azel[1+i*2]);
        a[0]=sin(azel[i*2])*cosel;
        a[1]=cos(azel[i*2])*cosel;
        a[2]=sin(azel[1+i*2]);
        matmul("TN",3,1,3,1.0,E,a,0.0,e);

        k++;
        if(azel[1+i*2]>max_el) max_el=azel[1+i*2],max_el_i=k;

        /* satellite velocity relative to receiver in ECEF */
        for (j=0;j<3;j++) {
            vs[j]=rs[j+3+i*6]-ve[j];
        }
        /* range rate with earth rotation correction */
        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*ve[0]-
                                      rs[3+i*6]*rr[1]-rs[  i*6]*ve[1]);

        /* Std of range rate error (m/s) */
        sig=(err<=0.0)?1.0:err*CLIGHT/freq;

        /* range rate residual (m/s) */
        v[nv]=-obs[i].D[0]*CLIGHT/freq-(rate+ddtr-CLIGHT*dts[1+i*2]);
        var[nv]=sig/sin(azel[1+2*i]);
        trace(4,"sat=%2d v=%.3f D=%.3f rate=%.3f ddtr=%.3f ddts=%.3f\n",obs[i].sat,v[nv],-obs[i].D[0]*CLIGHT/freq,rate,ddtr,dts[1+i*2]*CLIGHT);

        for (j=xiV();j<xiV()+xnV();j++) {
            H[j+nv*nx]=e[j-xiV()];
        }

        /* design matrix */
        if(!SD_mat){
            for(j=iclk;j<iclk+1;j++){
                H[j+nv*nx]=1.0;
            }
            /* time system offset and receiver bias correction */
            if      (sys==SYS_GLO) {v[nv]-=x[iclk+1]; H[iclk+1+nv*nx]=1.0; mask[1]=1;}
            else if (sys==SYS_GAL) {v[nv]-=x[iclk+2]; H[iclk+2+nv*nx]=1.0; mask[2]=1;}
            else if (sys==SYS_CMP) {v[nv]-=x[iclk+3]; H[iclk+3+nv*nx]=1.0; mask[3]=1;}
            else if (sys==SYS_IRN) {v[nv]-=x[iclk+4]; H[iclk+4+nv*nx]=1.0; mask[4]=1;}
#if 0 /* enable QZS-GPS time offset estimation */
                else if (sys==SYS_QZS) {v[nv]-=x[8]; H[8+nv*NX]=1.0; mask[5]=1;}
#endif
            else mask[0]=1;
        }
        vdop[nv]=obs[i].sat;
        nv++;
    }

    nm=nv;
    if(!SD_mat){
        for (i=0;i<6;i++) {
            if (mask[i]) continue;
            v[nv]=0.0;
            for (j=0;j<nx;j++) H[j+nv*nx]=j==i+iclk?1.0:0.0;
            var[nv++]=0.001;
        }
    }
    else{
        for(i=0;i<nm-1;i++){
            for(j=0;j<nm;j++){
                if(j==max_el_i-1) SD_mat[i+j*(nm-1)]=1.0;
                else SD_mat[i+i*(nm-1)]=-1.0;
            }
        }
        for(i=0;i<nm;i++){
            if(vdop[i]==obs[max_el_i].sat){
                for(j=i;j<nm-1;j++){
                    vdop[j]=vdop[j+1];
                }
            }
        }
    }


    return SD_mat?nm:nv;
}

static int resDoppler(int iter,const obsd_t *obs, int n,const nav_t *nav, const double *rs, const double *dts,
                      const double *rr,const double *vel, const double *x,
                      double *v,double *H,double *R,rtk_t *rtk,double *SD_mat)
{
    int i,j,nv=0,max_el_i,k=0,iclk=xiClkDrift(rtk->fusing->opts);
    int mask[6]={0},sys,nm,sat,isat[MAXOBS]={0},nx=rtk->fusing->ep_kf.nx;
    double freq,rate,pos[3],E[9],a[3],e[3],vs[3],cosel,sig;
    double max_el=0.0,ddtr=SD_mat?0.0:x[iclk];
    double el,az,var[MAXOBS]={0};

    trace(3,"resDoppler  : n=%d\n",n);

    ecef2pos(rr,pos); xyz2enu(pos,E);

    for (i=0;i<n&&i<MAXOBS;i++) {

        sat=obs[i].sat;
        freq=sat2freq(sat,obs[i].code[0],nav);
        sys=satsys(sat, nullptr);

        if (obs[i].D[0]==0.0||freq==0.0||!rtk->ssat[sat-1].vs||norm(rs+3+i*6,3)<=0.0) {
            continue;
        }
        el=rtk->ssat[sat-1].azel[1];
        az=rtk->ssat[sat-1].azel[0];

        /* LOS (line-of-sight) vector in ECEF */
        cosel=cos(el);
        a[0]=sin(az)*cosel;
        a[1]=cos(az)*cosel;
        a[2]=sin(el);
        matmul("TN",3,1,3,1.0,E,a,0.0,e);

        k++;
        if(el>max_el) max_el=el,max_el_i=k;

        /* satellite velocity relative to receiver in ECEF */
        for (j=0;j<3;j++) {
            vs[j]=rs[j+3+i*6]-vel[j];
        }

        /* range rate with earth rotation correction */
        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*vel[0]-
                                      rs[3+i*6]*rr[1]-rs[  i*6]*vel[1]);

        /* Std of range rate error (m/s) */
        sig=1.0*CLIGHT/freq;

        /* range rate residual (m/s) */
        v[nv]=-obs[i].D[0]*CLIGHT/freq-(rate+ddtr-CLIGHT*dts[1+i*2]);
        var[nv]=SQR(sig/sin(el));

        trace(4,"%05d(%d): sat=%2d, v=%6.3f D=%10.3f rate=%10.3f ddtr=%9.3f ddts=%12.3f var=%5.2f el=%3.1f\n",
              rtk->epoch,iter,sat,v[nv],-obs[i].D[0]*CLIGHT/freq,rate,ddtr,CLIGHT*dts[1+i*2],var[nv],el*R2D);

        if(H){
            for(j=xiV();j<xiV()+xnV();j++) {
                H[j+nv*nx]=e[j-xiV()];
            }
            if(!SD_mat){
                for(j=iclk;j<iclk+1;j++){
                    H[j+nv*nx]=1.0;
                }
                /* time system offset and receiver bias correction */
                if      (sys==SYS_GLO) {v[nv]-=x[iclk+1]; H[iclk+1+nv*nx]=1.0; mask[1]=1;}
                else if (sys==SYS_GAL) {v[nv]-=x[iclk+2]; H[iclk+2+nv*nx]=1.0; mask[2]=1;}
                else if (sys==SYS_CMP) {v[nv]-=x[iclk+3]; H[iclk+3+nv*nx]=1.0; mask[3]=1;}
                else if (sys==SYS_IRN) {v[nv]-=x[iclk+4]; H[iclk+4+nv*nx]=1.0; mask[4]=1;}
                else mask[0]=1;
            }
        }
        isat[nv]=sat;
        nv++;
    }

    nm=nv;
    if(H){
        if(!SD_mat){
            for (i=0;i<6;i++) {
                if (mask[i]) continue;
                v[nv]=0.0;
                for (j=0;j<nx;j++) H[j+nv*nx]=j==i+iclk?1.0:0.0;
                var[nv++]=0.01;
            }
        }
        else{
            for(i=0;i<nm-1;i++){
                for(j=0;j<nm;j++){
                    if(j==max_el_i-1) SD_mat[i+j*(nm-1)]=1.0;
                    else SD_mat[i+i*(nm-1)]=-1.0;
                }
            }
        }
        for(i=0;i<nv;i++){
            for(j=0;j<nv;j++){
                R[i+j*nv]=i==j?var[i]:0.0;
            }
        }
        if(SD_mat){
            nv=sdHVR(nv,nx,SD_mat,H,v,R,0);
        }
    }

    if(iter==0||iter==MAX_ITER){
        iter==0?j=1:j=0;
        for(i=0;i<nm;i++){
            sat=isat[i];
            if(SD_mat&&sat==obs[max_el_i].sat) continue;
            rtk->ssat[sat-1].meas_res.doppler[j][0]=v[i];
        }
    }

    return nv;
}

/* pseudorange residuals -----------------------------------------------------*/
static int resTDCP(int iter,const prcopt_t *opt, const obsd_t *obs, int n, const nav_t *nav,
                 const double *rs,const double *dts,const double *vare, const int *svh,
                 const double *rr, const double *x,
                 rtk_t *rtk, double *v, double *H, double *R,double *SD_mat,int *ns)
{
    gtime_t time;
    double r,freq,dion=0.0,dtrp=0.0,vmeas,vion=0.0,vtrp=0.0,pos[3],ddtr,e[3],P,max_el=0.0;
    int i,j,k=0,nv=0,nm=0,sat,sys,mask[6]={0},isat[MAXOBS]={0};
    int iclk=xiClkDrift(rtk->fusing->opts),nx=rtk->fusing->ep_kf.nx,max_el_i=0;
    double azel[MAXOBS*2],var[MAXOBS],tdcp,pre_range;

    ecef2pos(rr,pos);
    trace(3,"rescode: rr=%.3f %.3f %.3f\n",rr[0], rr[1], rr[2]);

    ddtr=SD_mat?0.0:x[iclk];
    for (i=0;i<n&&i<MAXOBS;i++) {
        azel[i*2]=azel[1+i*2]=0.0;
        time=obs[i].time;
        sat=obs[i].sat;
        if (!(sys=satsys(sat, nullptr))) continue;

        /* reject duplicated observation data */
        if (i<n-1&&i<MAXOBS-1&&sat==obs[i+1].sat) {
            trace(2,"duplicated obs data %s sat=%d\n",time_str(time,3),sat);
            i++;
            continue;
        }
        /* excluded satellite? */
        if (satexclude(sat,vare[i],svh[i],opt)) continue;

        /* geometric distance and elevation mask*/
        if ((r=geodist(rs+i*6,rr,e))<=0.0) continue;
        if (satazel(pos,e,azel+i*2)<opt->elmin) continue;

        /* test SNR mask */
        if (!snrmask(obs+i,azel+i*2,opt)) continue;

        /* ionospheric correction */
        if (!ionocorr(time,nav,sat,pos,azel+i*2,opt->ionoopt,&dion,&vion)) {
            continue;
        }
        if ((freq=sat2freq(sat,obs[i].code[0],nav))==0.0) continue;
        dion*=SQR(FREQL1/freq);
        vion*=SQR(FREQL1/freq);

        /* tropospheric correction */
        if (!tropcorr(time,nav,pos,azel+i*2,opt->tropopt,&dtrp,&vtrp)) {
            continue;
        }
        /* psendorange with code bias correction */
        tdcp=rtk->ssat[sat-1].tdcp.val[0];
        pre_range=rtk->ssat[sat-1].tdcp.pre_rho;
        if (tdcp==0.0||pre_range==0.0) continue;

        /* pseudorange residual */
        double a=tdcp;
        double b=a*CLIGHT/freq;
        double c=r-CLIGHT*dts[1+i*2]+ddtr-rtk->ssat[sat-1].tdcp.pre_rho;
        v[nv]=tdcp*CLIGHT/freq-(r-CLIGHT*dts[1+i*2]+ddtr-pre_range);

        k++;
        if(azel[i*2+1]>max_el){
            max_el=azel[i*2+1];max_el_i=k;
        }
        rtk->ssat[sat-1].azel[0]=azel[2*i];
        rtk->ssat[sat-1].azel[1]=azel[2*i+1];

        /* design matrix for position */
        if(H){
            if(rtk->fusing->opts.imu.nav_coord==+E_InsNavCoord::LLH){
                Vector3d mu;
                for(int k=0;k<3;k++) mu[k]=e[k];
                mu=mu.transpose()*rtk->fusing->ins.earth.DPne;
                for(j=xiV();j<xiV()+xnV();j++){
                    H[j+nv*nx]=mu[j-xiV()];                     /* translation of innovation to position states */
                }
            }
            else{
                for(j=xiV();j<xiV()+xnV();j++){
                    H[j+nv*nx]=e[j-xiV()];
                }
            }

            /*design matrix for receiver clock*/
            if(!SD_mat){
                for(j=iclk;j<iclk+1;j++){
                    H[j+nv*nx]=1.0;
                }
                /* time system offset and receiver bias correction */
                if      (sys==SYS_GLO) {v[nv]-=x[iclk+1]; H[iclk+1+nv*nx]=1.0; mask[1]=1;}
                else if (sys==SYS_GAL) {v[nv]-=x[iclk+2]; H[iclk+2+nv*nx]=1.0; mask[2]=1;}
                else if (sys==SYS_CMP) {v[nv]-=x[iclk+3]; H[iclk+3+nv*nx]=1.0; mask[3]=1;}
                else if (sys==SYS_IRN) {v[nv]-=x[iclk+4]; H[iclk+4+nv*nx]=1.0; mask[4]=1;}
                else mask[0]=1;
            }
        }

        /* variance of pseudorange error */
        var[nv]=varerr(opt,&rtk->ssat[obs[i].sat-1],&obs[i],azel[1+i*2],sys);
        trace(2,"%05d(%d): sat=%2d, v=%6.3f tdcp=%10.3f r=%10.3f pre_r=%10.3f ddtr=%9.3f dts=%12.3f var=%5.2f el=%3.1f\n",
              rtk->epoch,iter,sat,v[nv],tdcp,r, pre_range,ddtr,dts[1+i*2]*CLIGHT,var[nv],azel[i*2+1]*R2D);
        isat[nv]=sat;
        nv++;
    }

    /* constraint to avoid rank-deficient */
    nm=nv;
    if(ns) *ns=k;
    if(H){
        if(!SD_mat){
            for (i=0;i<6;i++) {
                if (mask[i]) continue;
                v[nv]=0.0;
                for (j=0;j<nx;j++) H[j+nv*nx]=j==i+iclk?1.0:0.0;
                var[nv++]=0.01;
            }
        }
        else{
            for(i=0;i<nm-1;i++){
                for(j=0;j<nm;j++){
                    if(j==max_el_i-1) SD_mat[i+j*(nm-1)]=1.0;
                    else SD_mat[i+i*(nm-1)]=-1.0;
                }
            }
            rtk->ssat[obs[max_el_i].sat-1].ref=1;
        }

        for(i=0;i<nv;i++){
            for(j=0;j<nv;j++) R[i+j*nv]=i==j?var[i]:0.0;
        }

        if(SD_mat){
            nv=sdHVR(nv,nx,SD_mat,H,v,R,0);
        }
    }

    if(iter==0||iter==MAX_ITER-2
    ){
        iter==0?j=1:j=0;
        for(i=0;i<nm;i++){
            sat=isat[i];
            if(SD_mat&&sat==obs[max_el_i].sat){
                rtk->ssat[sat-1].vs=1;
                continue;
            }
            rtk->ssat[sat-1].meas_res.tdcp[j][0]=v[i];
            rtk->ssat[sat-1].vs=1;
            rtk->ssat[sat-1].vsat[0]=1;
        }
    }
    return nv;
}


static int resTdcp(int iter,const obsd_t *obs, int n,const nav_t *nav, const double *rs, const double *dts,
                      const double *rr,const double *vel, const double *x,
                      double *v,double *H,double *R,rtk_t *rtk,double *SD_mat)
{
    int i,j,nv=0,max_el_i,k=0,iclk=xiClkDrift(rtk->fusing->opts);
    int mask[6]={0},sys,nm,sat,isat[MAXOBS]={0},nx=rtk->fusing->ep_kf.nx;
    double freq,rate,pos[3],E[9],a[3],e[3],vs[3],cosel,sig;
    double max_el=0.0,ddtr=SD_mat?0.0:x[iclk];
    double el,az,var[MAXOBS]={0},tdcp;
    char time_str[100];

    trace(3,"resDoppler  : n=%d\n",n);

    ecef2pos(rr,pos); xyz2enu(pos,E);

    for (i=0;i<n&&i<MAXOBS;i++) {

        sat=obs[i].sat;
        freq=sat2freq(sat,obs[i].code[0],nav);
        sys=satsys(sat, nullptr);
        tdcp=rtk->ssat[sat-1].tdcp.val[0];

        if (tdcp==0.0||freq==0.0||!rtk->ssat[sat-1].vs||norm(rs+3+i*6,3)<=0.0) {
            continue;
        }
        el=rtk->ssat[sat-1].azel[1];
        az=rtk->ssat[sat-1].azel[0];

        /* LOS (line-of-sight) vector in ECEF */
        cosel=cos(el);
        a[0]=sin(az)*cosel;
        a[1]=cos(az)*cosel;
        a[2]=sin(el);
        matmul("TN",3,1,3,1.0,E,a,0.0,e);

        k++;
        if(el>max_el) max_el=el,max_el_i=k;

        /* satellite velocity relative to receiver in ECEF */
        for (j=0;j<3;j++) {
            vs[j]=rs[j+3+i*6]-vel[j];
        }

        /* range rate with earth rotation correction */
        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*vel[0]-
                                      rs[3+i*6]*rr[1]-rs[  i*6]*vel[1]);

        /* Std of range rate error (m/s) */
        sig=5.0*CLIGHT/freq;

        /* range rate residual (m/s) */
        v[nv]=tdcp*CLIGHT/freq-(rate+ddtr-CLIGHT*dts[1+i*2]);
        var[nv]=sig/sin(el);

        trace(4,"%05d(%d): sat=%2d, v=%6.3f T=%10.3f rate=%10.3f ddtr=%9.3f ddts=%12.3f var=%5.2f el=%3.1f\n",
              rtk->epoch,iter,sat,v[nv],tdcp*CLIGHT/freq,rate,ddtr,CLIGHT*dts[1+i*2],var[nv],el*R2D);

        if(H){
            for(j=xiV();j<xiV()+xnV();j++) {
                H[j+nv*nx]=e[j-xiV()];
            }
            if(!SD_mat){
                for(j=iclk;j<iclk+1;j++){
                    H[j+nv*nx]=1.0;
                }
                /* time system offset and receiver bias correction */
                if      (sys==SYS_GLO) {v[nv]-=x[iclk+1]; H[iclk+1+nv*nx]=1.0; mask[1]=1;}
                else if (sys==SYS_GAL) {v[nv]-=x[iclk+2]; H[iclk+2+nv*nx]=1.0; mask[2]=1;}
                else if (sys==SYS_CMP) {v[nv]-=x[iclk+3]; H[iclk+3+nv*nx]=1.0; mask[3]=1;}
                else if (sys==SYS_IRN) {v[nv]-=x[iclk+4]; H[iclk+4+nv*nx]=1.0; mask[4]=1;}
                else mask[0]=1;
            }
        }
        isat[nv]=sat;
        nv++;
    }

    nm=nv;
    if(H&&nv){
        if(!SD_mat){
            for (i=0;i<6;i++) {
                if (mask[i]) continue;
                v[nv]=0.0;
                for (j=0;j<nx;j++) H[j+nv*nx]=j==i+iclk?1.0:0.0;
                var[nv++]=0.01;
            }
        }
        else{
            for(i=0;i<nm-1;i++){
                for(j=0;j<nm;j++){
                    if(j==max_el_i-1) SD_mat[i+j*(nm-1)]=1.0;
                    else SD_mat[i+i*(nm-1)]=-1.0;
                }
            }
        }
        for(i=0;i<nv;i++){
            for(j=0;j<nv;j++){
                R[i+j*nv]=i==j?var[i]:0.0;
            }
        }
        if(SD_mat){
            nv=sdHVR(nv,nx,SD_mat,H,v,R,0);
        }
    }

    if(iter==0||iter==MAX_ITER-2){
        iter==0?j=1:j=0;
        for(i=0;i<nm;i++){
            sat=isat[i];
            if(SD_mat&&sat==obs[max_el_i].sat) continue;
            rtk->ssat[sat-1].meas_res.tdcp[j][0]=v[i];
        }
    }

    return nv;
}

static int tcFilterDop(const prcopt_t *opt, const obsd_t *obs, int n,const nav_t *nav,
                      const double *rs, const double *dts,rtk_t *rtk)
{
    int i,j,k,nv,nx=rtk->fusing->ep_kf.nx,info,stat=1,nm,nclk;
    bool sd=rtk->fusing->opts.gnss.sd_gnss;
    double *xp,*Pp,*H,*v,*R,*SD_mat,err=opt->err[4];
    Vector3d pos,vel;
    ins_t ins=rtk->fusing->ins;
    nclk=xnClkDrift(rtk->fusing->opts.gnss.mode,sd,rtk->fusing->opts.imu.dop_aid);
    nm=sd?n:n+nclk-1;

    v=mat(nm,1); H=zeros(nx,nm);SD_mat=zeros(nm,nm);
    xp=mat(nx,1);Pp=mat(nx,nx);
    R=zeros(nm,nm);

    removeIGArmLever(ins, rtk->fusing->ins.ig_lever, pos, &vel, true, rtk->fusing->opts.imu.nav_coord);
    matcpy(xp,rtk->fusing->ep_kf.x.data(),nx,1);

    if(!sd){
        updateClkDrift(rtk->fusing->opts,rtk->fusing->ep_kf.P,rtk->fusing->ep_kf.x);
    }

    for(i=0;i<MAX_ITER;i++){

        matcpy(Pp,rtk->fusing->ep_kf.P.data(),nx,nx);
        if(sd) for(j=0;j<nm;j++) for(k=0;k<nm;k++) SD_mat[j+k*nm]=0;

        /* range rate residuals (m/s) */
        nv=resDoppler(i+1,obs,n,nav,rs,dts,pos.data(),vel.data(),xp,v,H,R,rtk,sd?SD_mat:nullptr);

        if(nv<=0){
            stat=SOLQ_NONE;
            break;
        }

        if ((info=filter(xp,Pp,H,v,R,nx,nv,0))) {
            stat=SOLQ_NONE;
            break;
        }

        if(!insFeedback(rtk->fusing->opts.imu,rtk->fusing->opts.imu.nav_coord,rtk->fusing->opts.imu.att_def,ins,xp,nx)){
            LOG(WARNING)<<"ins feedback error";
            stat=SOLQ_NONE;
            break;
        }
        removeIGArmLever(ins, rtk->fusing->ins.ig_lever, pos, &vel, true, rtk->fusing->opts.imu.nav_coord);
    }

    if(stat&&valdoppler(xp,R,v,nv,4.0)){
        resDoppler(0,obs,n,nav,rs,dts,pos.data(),vel.data(),xp,v,nullptr,R,rtk,sd?SD_mat:nullptr);
        rtk->fusing->ig_ins=rtk->fusing->ins=ins;
        matcpy(rtk->fusing->ep_kf.x.data(),xp,nx,1);
        matcpy(rtk->fusing->ep_kf.P.data(),Pp,nx,nx);
        getStateCov(rtk->fusing->ep_kf.P,ins.state);
    }

    free(xp);free(Pp);
    free(v);free(R);free(H);free(SD_mat);
    return stat;
}

static int tcFilterTdcp(const prcopt_t *opt, const obsd_t *obs, int n,const nav_t *nav,
                       const double *rs, const double *dts,const double *vare,const int *svh,rtk_t *rtk)
{
    int i,j,k,nv,nx=rtk->fusing->ep_kf.nx,info,stat=1,nm,nclk;
    bool sd=rtk->fusing->opts.gnss.sd_gnss;
    double *xp,*Pp,*H,*v,*R,*SD_mat,err=opt->err[4];
    Vector3d pos,vel;
    ins_t ins=rtk->fusing->ins;
    nclk=xnClkDrift(rtk->fusing->opts.gnss.mode,sd,rtk->fusing->opts.imu.tdcp_aid);
    nm=sd?n:n+nclk-1;

    v=mat(nm,1); H=zeros(nx,nm);SD_mat=zeros(nm,nm);
    xp=mat(nx,1);Pp=mat(nx,nx);
    R=zeros(nm,nm);

    removeIGArmLever(ins, rtk->fusing->ins.ig_lever, pos, &vel, true, rtk->fusing->opts.imu.nav_coord);

    if(!sd){
        updateClkDrift(rtk->fusing->opts,rtk->fusing->ep_kf.P,rtk->fusing->ep_kf.x);
    }
    matcpy(xp,rtk->fusing->ep_kf.x.data(),nx,1);

    for(i=0;i<MAX_ITER-2;i++){
        matcpy(Pp,rtk->fusing->ep_kf.P.data(),nx,nx);
        if(sd) for(j=0;j<nm;j++) for(k=0;k<nm;k++) SD_mat[j+k*nm]=0;

        /* range rate residuals (m/s) */
        nv= resTDCP(i+1,opt,obs,n,nav,rs,dts,vare,svh,pos.data(),xp,rtk,v,H,R,sd?SD_mat:nullptr, nullptr);
        if(nv<=0){
            stat=SOLQ_NONE;
            break;
        }

        if ((info=filter(xp,Pp,H,v,R,nx,nv,0))) {
            stat=SOLQ_NONE;
            break;
        }

        if(!insFeedback(rtk->fusing->opts.imu,rtk->fusing->opts.imu.nav_coord,rtk->fusing->opts.imu.att_def,ins,xp,nx)){
            LOG(WARNING)<<"ins feedback error";
            stat=SOLQ_NONE;
            break;
        }
        removeIGArmLever(ins, rtk->fusing->ins.ig_lever, pos, &vel, true, rtk->fusing->opts.imu.nav_coord);
    }

    if(stat){
//        resTdcp(0,obs,n,nav,rs,dts,pos.data(),vel.data(),xp,v,nullptr,R,rtk,sd?SD_mat:nullptr);
//        Vector3d delta_pos=rtk->fusing->ins.state.pos-ins.state.pos;
        rtk->fusing->ig_ins=rtk->fusing->ins=ins;
        matcpy(rtk->fusing->ep_kf.x.data(),xp,nx,1);
        matcpy(rtk->fusing->ep_kf.P.data(),Pp,nx,nx);
        getStateCov(rtk->fusing->ep_kf.P,ins.state);
    }

    free(xp);free(Pp);
    free(v);free(R);free(H);free(SD_mat);
    return stat;
}

extern bool igSensorTc1(fusing_t &fusing)
{
    bool stat=false;
    int nobs=(int)fusing.ep_sensors_meas.gnss.size();
    double sow= time2gpst(fusing.ep_sensors_meas.gnss[0].time, nullptr);
    if(rtkpos(&fusing.rtk,fusing.ep_sensors_meas.gnss.data(),nobs,&fusing.nav)){
        LOG(DEBUG)<<logTime(sow,fusing.ins.epoch,fusing.rtk.epoch)<<" ig tigtly coupled, gnss state = "<<(int)fusing.ins.gstat;
        stat=true;
    }
    else{
        LOG(WARNING)<<logTime(sow,fusing.ins.epoch,fusing.rtk.epoch)<<" tightly coupled error,nobs = "<<nobs;
        stat=false;
    }

    return stat;
}

static int tightCouplePR(const prcopt_t *opt,const obsd_t *obs, int n, const nav_t *nav,rtk_t *rtk,char *msg)
{
    prcopt_t opt_=*opt;
    double *rs,*dts,*var;
    int i,j,svh[MAXOBS],stat=0;

    if(opt_.mode!=PMODE_SINGLE){
        opt_.ionoopt=IONOOPT_BRDC;
        opt_.tropopt=TROPOPT_SAAS;
    }

    rtk->sol.stat=SOLQ_NONE;

    if (n<=0) {
        strcpy(msg,"no observation data");
        return 0;
    }
    rtk->sol.time=obs[0].time;
    msg[0]='\0';

    rs=mat(6,n); dts=mat(2,n); var=mat(1,n);

    for (i=0;i<MAXSAT;i++) {
        rtk->ssat[i].snr_rover[0]=0;rtk->ssat[i].snr_base[0]=0;
        rtk->ssat[i].meas_res.code[0][0]=rtk->ssat[i].meas_res.code[1][0]=0.0;
        rtk->ssat[i].meas_res.phase[0][0]=rtk->ssat[i].meas_res.phase[1][0]=0.0;
        rtk->ssat[i].meas_res.tdcp[0][0]=rtk->ssat[i].meas_res.tdcp[1][0]=0.0;
        rtk->ssat[i].meas_res.doppler[0][0]=rtk->ssat[i].meas_res.doppler[1][0]=0.0;
        rtk->ssat[i].ref=0;
        rtk->ssat[i].vs=0;
        rtk->ssat[i].azel[0]=rtk->ssat[i].azel[1]=0.0;
        for(j=0;j<NFREQ;j++) rtk->ssat[i].vsat[j]=0;
    }
    for (i=0;i<n;i++)
        rtk->ssat[obs[i].sat-1].snr_rover[0]=obs[i].SNR[0];

    /* satellite positons, velocities and clocks */
    satposs(rtk->sol.time,obs,n,nav,opt_.sateph,rs,dts,var,svh);

    rtk->epoch++;
    /* estimate receiver position and time with pseudorange */
    stat=tcFilterPR(opt,obs,n,nav,rs,dts,var,svh,rtk,msg);

    free(rs); free(dts); free(var);
    return stat;
}

static bool tightCoupleCP(fusing_t& fusing)
{
    bool stat=false;
    int nobs=(int)fusing.ep_sensors_meas.gnss.size();
    double sow= time2gpst(fusing.ep_sensors_meas.gnss[0].time, nullptr);
    if(rtkpos(&fusing.rtk,fusing.ep_sensors_meas.gnss.data(),nobs,&fusing.nav)){
        LOG(DEBUG)<<logTime(sow,fusing.ins.epoch,fusing.rtk.epoch)<<" ig tigtly coupled, gnss state = "<<(int)fusing.ins.gstat;
        stat=true;
    }
    else{
        LOG(WARNING)<<logTime(sow,fusing.ins.epoch,fusing.rtk.epoch)<<" tightly coupled error,nobs = "<<nobs;
        stat=false;
    }

    return stat;
}

static int tightCoupleDoppler(const prcopt_t *opt,const obsd_t *obs, int n, const nav_t *nav,rtk_t *rtk,char *msg)
{
    prcopt_t opt_=*opt;
    double *rs,*dts,*var;
    int i,svh[MAXOBS],stat=0;

    if (n<=0) {
        strcpy(msg,"no observation data");
        return 0;
    }

    msg[0]='\0';
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n);

    for (i=0;i<MAXSAT;i++) {
        rtk->ssat[i].meas_res.doppler[0][0]=rtk->ssat[i].meas_res.doppler[1][0]=0.0;
    }

    /* satellite positons, velocities and clocks */
    satposs(rtk->sol.time,obs,n,nav,opt_.sateph,rs,dts,var,svh);

    /* estimate receiver position and time with pseudorange */
    stat=tcFilterDop(opt,obs,n,nav,rs,dts,rtk);

    free(rs); free(dts); free(var);
    return stat;
}

static int tightCoupleTdcp(const prcopt_t *opt,const obsd_t *obs, int n, const nav_t *nav,rtk_t *rtk,char *msg){
    prcopt_t opt_=*opt;
    double *rs,*dts,*var;
    int i,svh[MAXOBS],stat=0;

    if (n<=0) {
        strcpy(msg,"no observation data");
        return 0;
    }

    msg[0]='\0';
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n);

    for (i=0;i<MAXSAT;i++) {
        rtk->ssat[i].meas_res.tdcp[0][0]=rtk->ssat[i].meas_res.tdcp[1][0]=0.0;
    }

    /* satellite positons, velocities and clocks */
    satposs(obs[0].time,obs,n,nav,opt_.sateph,rs,dts,var,svh);

    /* estimate receiver position and time with pseudorange */
    stat=tcFilterTdcp(opt,obs,n,nav,rs,dts,var,svh,rtk);

    free(rs); free(dts); free(var);
    return stat;
}


extern bool igSensorTC(fusing_t &fusing)
{
    bool stat=false;
    int nobs=(int)fusing.ep_sensors_meas.gnss.size();
    double sow= time2gpst(fusing.ep_sensors_meas.gnss[0].time, nullptr);
    E_GnssMode mode=fusing.opts.gnss.mode;
    char msg[MAXERRMSG];

    if(mode==+E_GnssMode::SINGLE){ /* SPP/INS tc */
        stat=tightCouplePR(&fusing.opts.rtklib.prc_opt,fusing.ep_sensors_meas.gnss.data(),nobs,
                           &fusing.nav,&fusing.rtk, msg);
    }
    else if(mode>=+E_GnssMode::DGPS&&mode<=+E_GnssMode::PPP_KINEMA){ /* DGPS\PPK\PPP/INS tc */
        stat=tightCoupleCP(fusing);
    }

    if(fusing.opts.imu.dop_aid){ /* Doppler/INS tc */
        if(fusing.rtk.epoch!=1){
            stat=tightCoupleDoppler(&fusing.opts.rtklib.prc_opt,fusing.ep_sensors_meas.gnss.data(),nobs,
                                    &fusing.nav,&fusing.rtk, msg);
        }
    }

    if(fusing.opts.imu.tdcp_aid){ /* TDCP/INS tc */
#if 0
        if(calTdcp(fusing.ep_sensors_meas.gnss.data(),nobs,fusing.rtk.ssat)){
            stat=tightCoupleTdcp(&fusing.opts.rtklib.prc_opt,fusing.ep_sensors_meas.gnss.data(),nobs,
                                 &fusing.nav,&fusing.rtk, msg);
        }

        Vector3d pos;
        removeIGArmLever(fusing.ins,fusing.ins.ig_lever,pos, nullptr,true,fusing.opts.imu.nav_coord);
        calRange(&fusing.rtk.fusing->opts.rtklib.prc_opt,fusing.ep_sensors_meas.gnss.data(),nobs,&fusing.nav, pos.data(),fusing.rtk.ssat);
#endif
    }

    outsolstat(&fusing.rtk,&fusing.nav);

    return stat;
}