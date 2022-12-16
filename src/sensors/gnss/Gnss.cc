//
// Created by hw on 10/2/22.
//

#include "Coordinate.h"
#include "Rotation.h"
#include "Gnss.h"
#include "rtklib.h"

/* safe to call malloc */
#define MALLOC(pointer, type, sz)                                               \
    if(!((pointer) = (type *)malloc(((unsigned long)sz)*sizeof(type)))){        \
        LOG(ERROR)<<" memory allocation failed";                                  \
    }

/* safe to free pointer */
#define FREE(pointer) {free(pointer);(pointer) = NULL;}

static bool addPvData(const pvData_t *pv_data,pv_t *pvs) {
    pvData_t *data_tmp;

    if(pvs->n==0&&pvs->nmax==0){
        pvs->nmax=10000;
        MALLOC(pvs->data,pvData_t ,pvs->nmax);
    }
    else if(pvs->n>=pvs->nmax){
        pvs->nmax*=2;
        if(!(data_tmp=(pvData_t *)realloc(pvs->data, sizeof(pvData_t)*pvs->nmax))){
            FREE(pvs->data);
            pvs->n=pvs->nmax=0;
            return false;
        }
        pvs->data=data_tmp;
    }
    pvs->data[pvs->n++]=*pv_data;
    return true;
}

extern void sol2pvData(sol_t &sol,pvData_t &pv,E_InsNavCoord nav_coord,E_AttDefination att_def)
{
    pv.state=sol.stat;
    pv.sow= time2gpst(sol.time, nullptr);
    pv.pos=(Eigen::Map<Eigen::Vector3d>)(sol.rr);
    pv.vel=(Eigen::Map<Eigen::Vector3d>)(sol.rr+3);
    for(int j=0;j<3;j++){
        pv.cov_pos[j]=(double)sol.qr[j];
        pv.cov_vel[j]=(double)sol.qv[j];
    }
    if(nav_coord==+E_InsNavCoord::LLH){
        Vector3d llh=Coordinate::ecef2llh(pv.pos);
        pv.pos=llh;
        Matrix3d Cne=Rotation::getCne(llh,att_def);
        Matrix3d R=pv.cov_pos.asDiagonal();
        R=Cne.transpose()*R*Cne;
        pv.cov_pos[0]=R(0,0)/(RE_WGS84*RE_WGS84);
        pv.cov_pos[1]=R(1,1)/(RE_WGS84*RE_WGS84);
        pv.cov_pos[2]=R(2,2);
        pv.vel=Cne.transpose()*pv.vel;
    }
}

static bool readPvPos(FILE *fp,pv_t *pvs,E_InsNavCoord nav_coord,E_AttDefination att_def)
{
    solopt_t opt=solopt_default;
    solbuf_t solbuf={0};
    gtime_t t0={0};

    initsolbuf(&solbuf,0,0);
    readsolopt(fp,&opt);
    rewind(fp);

    if(!readsoldata(fp,t0,t0,0.0,0,&opt,&solbuf)){
        LOG(ERROR)<<"read rtklib pos file error";
        return false;
    }

    pvData_t data;
    for(int i=0;i<solbuf.n;i++){
        sol2pvData(solbuf.data[i],data,nav_coord,att_def);
        if(!addPvData(&data,pvs)){
            return false;
        }
    }

    return true;
}

static bool readPvPsins(FILE *fp,pv_t *pvs,E_InsNavCoord nav_coord,E_AttDefination att_def)
{
    double sow,x,y,z;
    pvData_t pv_data;
    char line[512];

    while(fgets(line,512,fp)){
        if(line[0]=='%') continue;
        sscanf(line,"%lf,%lf,%lf,%lf",&x,&y,&z,&sow);

        pv_data.sow=sow;
        pv_data.state=SOLQ_FIX;
        Vector3d llh=Vector3d(x,y,z);
        if(nav_coord==+E_InsNavCoord::ECEF){
            pv_data.pos=Coordinate::llh2ecef(llh);
            Matrix3d Cne,Q=M30;
            Q(0,0)=0.003,Q(1,1)=0.003,Q(2,2)=0.005;
            Cne=Rotation::getCne(llh,att_def);
            Q=Cne*Q*Cne.transpose();
            pv_data.cov_pos[0]=fabs(Q(0,0)),pv_data.cov_pos[1]=fabs(Q(1,1)),pv_data.cov_pos[2]=fabs(Q(2,2));
        }
        else{
            pv_data.pos=llh;
            pv_data.cov_pos=Vector3d(1.0/(RE_WGS84*RE_WGS84),1.0/(RE_WGS84*RE_WGS84),9.0);
        }
        if(!addPvData(&pv_data,pvs)){
            return false;
        }
    }

    return true;
}

static bool readPvGins(FILE *fp,pv_t *pvs,E_InsNavCoord nav_coord,E_AttDefination att_def)
{
    double sow,x,y,z,qx,qy,qz;
    pvData_t pv_data;
    char line[512];

    while(fgets(line,512,fp)){
        if(line[0]=='%') continue;
        sscanf(line,"%lf %lf %lf %lf %lf %lf %lf",&sow,&x,&y,&z,&qx,&qy,&qz);

        pv_data.sow=sow;
        pv_data.state=SOLQ_FIX;
        pv_data.pos=Vector3d(x*D2R,y*D2R,z);
        if(nav_coord==+E_InsNavCoord::ECEF){
            Vector3d ecef=Coordinate::llh2ecef(pv_data.pos);
            Matrix3d Cne,Q=M30;
            Q(0,0)=(qx*qx),Q(1,1)=(qy*qy),Q(2,2)=(qz*qz);
            Cne=Rotation::getCne(pv_data.pos,att_def);
            Q=Cne*Q*Cne.transpose();
            pv_data.pos=ecef;
            pv_data.cov_pos[0]=fabs(Q(0,0)),pv_data.cov_pos[1]=fabs(Q(1,1)),pv_data.cov_pos[2]=fabs(Q(2,2));
        }
        else{
            pv_data.cov_pos=Vector3d(qx*qx/(RE_WGS84*RE_WGS84),qy*qy/(RE_WGS84*RE_WGS84),qz*qz);
        }
        if(!addPvData(&pv_data,pvs)){
            return false;
        }
    }

    return true;
}

extern bool loadPVs(string pv_file,E_GnssPvFmt fmt,pv_t *pvs,E_InsNavCoord nav_coord,E_AttDefination att_def)
{
    if(pv_file.empty()){
        LOG(ERROR)<<"pv file error";
        return false;
    }

    bool stat=false;
    FILE *fp;
    if(!(fp=fopen(pv_file.c_str(),"r"))){
        LOG(ERROR)<<"GNSS solution file open error";
        return false;
    }

    switch (fmt) {
        case E_GnssPvFmt::PSINS:
            stat= readPvPsins(fp,pvs,nav_coord,att_def);
            break;
        case E_GnssPvFmt::RTKLIB:
            stat= readPvPos(fp,pvs,nav_coord,att_def);
            break;
        case E_GnssPvFmt::GINS:
            stat= readPvGins(fp,pvs,nav_coord,att_def);
            break;
    }

    return stat;
}