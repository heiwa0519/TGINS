//
// Created by hw on 10/11/22.
//

#include "RefSol.h"
#include "Rotation.h"
#include "Coordinate.h"

static int buff2num(char *buff, const char *sep, double *v)
{
    int n,len=(int)strlen(sep);
    char *p,*q;
    char bu[MAXSOLMSG];

    strcpy(bu,buff);

    for (p=bu,n=0;n<1024;p=q+len) {
        if ((q=strstr(p,sep))) *q='\0';
        if (*p) v[n++]=atof(p);
        if (!q) break;
    }
    return n;
}

static bool loadRefGins(FILE *fp,solbuf_t *ref_sol)
{
    int week,n;
    double sow;
    Vector3d rn,vn,re,ve,att,ba,bg;
    Matrix3d Cne;
    sol_t data;
    char line[512];

    while(fgets(line,512,fp)){
        if(line[0]=='%') continue;
        n=sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &week,&sow,&rn.x(),&rn.y(),&rn.z(),&vn.x(),&vn.y(),&vn.z(),&att.x(),&att.y(),&att.z(),
               &bg.x(),&bg.y(),&bg.z(),&ba.x(),&ba.y(),&ba.z());

        data.time= gpst2time(week,sow);
        rn[0]*=D2R,rn[1]*=D2R;
        re=Coordinate::llh2ecef(rn);
        Cne=Rotation::getCne(rn,E_AttDefination::NED_FRD);
        ve=Cne*vn;
        data.rr[0]=re[0],data.rr[1]=re[1],data.rr[2]=re[2];
        data.rr[3]=ve[0],data.rr[4]=ve[1],data.rr[5]=ve[2];
        data.rpy[0]=att[0]*D2R,data.rpy[1]=att[1]*D2R,data.rpy[2]=att[2]*D2R;
        if(n>11){
            data.bg[0]=bg[0],data.bg[1]=bg[1],data.bg[2]=bg[2];
            data.ba[0]=ba[0],data.ba[1]=ba[1],data.ba[2]=ba[2];
        }
        if(!addsol(ref_sol,&data)){
            return false;
        }
    }

    return true;
}

static bool loadRefIE(FILE *fp,solbuf_t *ref_sol)
{
    int week,n,i=2;
    double sow;
    Vector3d rn,vn,att,pos;
    sol_t data;
    char line[512];
    double v[50];

    while(fgets(line,512,fp)){
        if(48>=line[1]||line[1]>=57) {
            continue;
        }
        if((n= buff2num(line,",",v))<3){
            return false;
        }
        data.time= gpst2time(v[0],v[1]);
        if(i+3<n){
            pos[0]=v[i++]*D2R;
            pos[1]=v[i++]*D2R;
            pos[2]=v[i++];
            pos2ecef(pos.data(),data.rr);
        }
        if(i+3<n){
            vn[1]=v[i++];
            vn[0]=v[i++];
            vn[2]=-v[i++];
            enu2ecef(pos.data(),vn.data(),data.rr+3);
        }
        if(i+3<n){
            data.rpy[0]=v[i++]*D2R;
            data.rpy[1]=v[i++]*D2R;
            data.rpy[2]=v[i++]*D2R;
            if(data.rpy[2]<0) data.rpy[2]+=2*PI;
        }
        i+=9;
        if(i+3<n){
            data.ba[1]=v[i++]*MPS22MG;
            data.ba[0]=v[i++]*MPS22MG;
            data.ba[2]=-v[i++]*MPS22MG;
        }
        if(i+3<=n){
            data.bg[1]=v[i++]*DPS2DPH;
            data.bg[0]=v[i++]*DPS2DPH;
            data.bg[2]=-v[i++]*DPS2DPH;
        }

        if(!addsol(ref_sol,&data)){
            return false;
        }
        i=2;
    }

    return true;
}

static int loadRtklibSol(FILE *fp,solbuf_t *solbuf){
    solopt_t opt=solopt_default;
    gtime_t t0={0};

    initsolbuf(solbuf,0,0);
    readsolopt(fp,&opt);
    rewind(fp);

    if(!readsoldata(fp,t0,t0,0.0,0,&opt,solbuf)){
        fprintf(stderr,"no solution\n");
        return 0;
    }

    return solbuf->n>0;
}

extern bool loadRef(string ref_file,E_RefSolFmt fmt,solbuf_t *ref_sol)
{
    if(ref_file.empty()){
        LOG(ERROR)<<"ref file error";
        return false;
    }

    bool stat=false;
    FILE *fp;
    if(!(fp=fopen(ref_file.c_str(),"r"))){
        LOG(ERROR)<<"GNSS solution file open error";
        return false;
    }

    switch (fmt) {
        case +E_RefSolFmt::IE:
            stat = loadRefIE(fp,ref_sol);
            break;
        case +E_RefSolFmt::RTKLIB:
            stat = loadRtklibSol(fp,ref_sol);
            break;
        case +E_RefSolFmt::PSINS:
            break;
        case +E_RefSolFmt::GINS:
            stat = loadRefGins(fp,ref_sol);
            break;
        case +E_RefSolFmt::FUSING:
            break;
    }
    return stat;
}

static void avpinterp(Vector3d delta,double dt,double all_dt,Vector3d& avp,bool att)
{
    if(att){
        for(int j=0;j<3;j++){
            if(delta[j]>PI*R2D){
                delta[j]-=2*PI*R2D;
            }
            if(delta[j]<-PI*R2D){
                delta[j]+=2*PI*R2D;
            }
        }
    }
    avp+=delta*(dt/all_dt);

}

extern bool matchRefSol(double sow,int *idx,const track_t *ref_sols,state_t *ref_data,bool back,int hz)
{
    double ref_time,ref_time1,dt0,dt1,dd,delta_t=1.0/hz;
    Vector3d delta;
    double dt_ref= ref_sols->data[2].state.sow-ref_sols->data[1].state.sow;

    if(back){
        for(int i=*idx;i>1;i--){
            ref_time=ref_sols->data[i].state.sow;
            ref_time1=ref_sols->data[i-1].state.sow;
            if(sow<=ref_time&&sow>ref_time1){
                dt0=fabs(sow-ref_time);
                dt1=fabs(ref_time1-sow);
                dt0<dt1?*idx=i:*idx=i-1;
                *ref_data=ref_sols->data[*idx].state;
                dt0<dt1?dd=dt0:dd=dt1;
                LOG(TRACE)<<setprecision(9)<<"match BKF ref sol, sol_sow: "<<sow<<" "<<"ref0_sow: "<<ref_time<<" dt0: "<<dt0<<" ref1_sow: "<<ref_time1<<" dt1: "<<dt1<<" dd: "<<dd;
//                delta=ref_sols->data[i].pos-ref_sols->data[i-1].pos;
//                avpinterp(delta,-dd, fabs(ref_time-ref_time1),ref_data->pos,false);
//                delta=ref_sols->data[i].vel-ref_sols->data[i-1].vel;
//                avpinterp(delta,-dd, fabs(ref_time-ref_time1),ref_data->vel,false);
//                delta=ref_sols->data[i].att-ref_sols->data[i-1].att;
//                avpinterp(delta,dd, fabs(ref_time-ref_time1),ref_data->att,true);
                return true;
            }
            else if(ref_time-sow<-1.0) break;
        }
    }
    else{
        for(int i=*idx>5?*idx-3:*idx;i<ref_sols->n;i++){
            ref_time=ref_sols->data[i].state.sow;

            if(ref_time-sow<-1.0){
                i++;
            }
            if(ref_time-sow>60.0){
                break;
            }
            if(fabs(sow-ref_time)<dt_ref/2.0){
                *idx=i;
                *ref_data=ref_sols->data[i].state;
                return true;
            }
        }
    }

    return false;
}

static int matchRefSolIdx(gtime_t solTime,const solbuf_t *refSol,int *refIdx,int dir){
    int i,stat=0;
    double dtRef=timediff(refSol->data[2].time,refSol->data[1].time);

    double secSol=time2gpst(solTime,NULL);
    double secRef;

    if(dir==0){
        for(i=*refIdx>100?*refIdx-100:*refIdx;i<refSol->n;i++){
            secRef=time2gpst(refSol->data[i].time,NULL);
            if(secRef-secSol<-1.0){
                i++;
            }
            if(secRef-secSol>60.0){
                break;
            }
            if(secSol>=secRef&&secSol-secRef<dtRef/2){
#if 0
                fprintf(stdout,"solTime=%.3f refTime=%.3f dt=%.3f\n",secSol,secRef,secSol-secRef);
                fflush(stdout);
#endif
                *refIdx=i;
                stat=1;
                break;
            }
        }
    }
    else if(dir==1){
        for(i=((*refIdx+100)<refSol->n)?*refIdx-100:*refIdx;i>0;i--){
            secRef=time2gpst(refSol->data[i].time,NULL);
            if(secRef-secSol<-1.0){
                i--;
            }
            if(secRef-secSol>60.0){
//                break;
                continue;
            }
            if(fabs(secSol-secRef)<dtRef/2){
#if 0
                fprintf(stdout,"solTime=%.3f refTime=%.3f dt=%.3f idx=%d\n",secSol,secRef,secSol-secRef,i);
                fflush(stdout);
#endif
                *refIdx=i;
                stat=1;
                break;
            }
        }
    }


    return stat;
}

extern int makeSolDiff(const solbuf_t *sol,const solbuf_t *refSol,solbuf_t *errSol)
{
    int i,refIdx=0,dir=0;
    sol_t solData={0};
    double sow;

    if(timediff(sol->data[0].time,sol->data[1].time)>0.0) dir=1,refIdx=refSol->n-1;

    errSol->err_fmt=1;
    for(i=0;i<sol->n;i++){
        if(matchRefSolIdx(sol->data[i].time,refSol,&refIdx,dir)){
            solData=sol->data[i];
            for(int j=0;j<6;j++){
                solData.rr[j]=sol->data[i].rr[j]-refSol->data[refIdx].rr[j];
            }
            double pos[3];
            ecef2pos(refSol->data[refIdx].rr,pos);
            ecef2enu(pos,solData.rr,solData.rr);
            ecef2enu(pos,solData.rr+3,solData.rr+3);  /*rr enu format*/
            sow= time2gpst(solData.time,NULL);
#if 0
            quat_t q1,q2,dq;
            rpy2quat(sol->data[i].att,&q1);
            rpy2quat(refSol->data[refIdx].att,&q2);
            quat_conj(&q2);
            dq=quatMul(&q1,&q2);
            quat2rpy(&dq,solData.att);
#else
            solData.rpy[0]=(sol->data[i].rpy[0])-(refSol->data[refIdx].rpy[0]);
            solData.rpy[1]=(sol->data[i].rpy[1])-(refSol->data[refIdx].rpy[1]);
            solData.rpy[2]=sol->data[i].rpy[2]-refSol->data[refIdx].rpy[2];
            if(solData.rpy[2]>350*D2R) solData.rpy[2]-=2*PI;
            if(solData.rpy[2]<-350*D2R) solData.rpy[2]+=2*PI;
            for(int j=0;j<6;j++){
                solData.qr[j]=sol->data[i].qr[j];
                solData.qv[j]=sol->data[i].qv[j];
                solData.qa[j]=sol->data[i].qa[j];
            }
#endif
            if(!addsol(errSol,&solData)){
                return 0;
            }
        }
    }
    if(refIdx==0) return 0;

    return 1;
}