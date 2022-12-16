//
// Created by hw on 10/30/22.
//

#include "SyncSensors.h"

extern bool igSyncPv(double pre_imu,double cur_imu,const pv_t &pvs,int *pv_idx,bool back)
{
    bool sync=false;
    int i=0;
    double dt1,dt2;

    if(back){
        for(i=*pv_idx;i>0;i--){
            double gpst=pvs.data[i].sow;
            if(gpst>=cur_imu&&gpst<=pre_imu){
                sync=true;
                *pv_idx=i;
                LOG(DEBUG)<<setprecision(12)<<"BKF PV synchronized imu_tag0: "<<pre_imu<<" imu_tag1: "<<cur_imu<<" pv tag: "<<gpst;
                break;
            }
            else if(gpst-cur_imu<-1.0){
                sync=false;
                break;
            }
        }
    }
    else{
        for(i=*pv_idx>5?*pv_idx-5:0;i<pvs.n;i++){
            double gpst=pvs.data[i].sow;
            dt1=fabs(gpst-cur_imu);
            if(pre_imu<=gpst&&gpst<=cur_imu){
                sync=true;
                *pv_idx=i;
                LOG(DEBUG)<<setprecision(12)<<"FKF PV synchronized imu_tag0: "<<pre_imu<<" imu_tag1: "<<cur_imu<<" pv tag: "<<gpst<<" dt: "<<dt1;
                break;
            }
            else if(gpst-cur_imu>1.0){
                sync=false;
                break;
            }
        }
    }
    return sync;
}

extern bool igSyncObs(double pre_imu,double cur_imu,const obs_t &obs,measIdx_t *idx,bool back)
{
    bool sync=false;
    int i=0;
    double dt1,dt2;
    double gpst;

    if(!back){
        for(i=idx->rover>100?idx->rover-100:0;i<obs.n;i++){
            if(obs.data[i].rcv!=1) continue;
            gpst= time2gpst(obs.data[i].time, nullptr);
            if(gpst>=pre_imu&&gpst<cur_imu){
                sync=true;
                idx->rover=i;
                LOG(TRACE)<<setprecision(12)<<"FKF gnss obs synchronized imu_tag0: "<<pre_imu<<" obs tag: "<<gpst<<" imu_tag1: "<<cur_imu;
                break;
            }
            else if(gpst-cur_imu>1.0){
                sync=false;
                break;
            }
        }
    }
    else{
        for(i=idx->rover>(idx->rover-100)?idx->rover:idx->rover-100;i>=0;i--){
            gpst= time2gpst(obs.data[i].time, nullptr);
            if(gpst>cur_imu&&gpst<=pre_imu){
                sync=true;
                idx->rover=i;
                LOG(TRACE)<<setprecision(12)<<"BKF gnss obs synchronized imu_tag0: "<<pre_imu<<" imu_tag1: "<<cur_imu<<" obs tag: "<<gpst;
                break;
            }
            else if(gpst-pre_imu<-1.0){
                sync=false;
                break;
            }
        }
    }
    return sync;
}

extern int inputEpGnssObs(const fusing_t &fusing,const obs_t& obs,measIdx_t &idx,vector<obsd_t>& gnss,bool back)
{
    int i,nu,nr,k=0;

    gnss.clear();
    if(idx.rover<0||idx.rover>obs.n) return 0;

    if(!back){
        if((nu=nextobsf(&obs,&idx.rover,1))<=0) return 0;
        if(fusing.opts.rtklib.prc_opt.intpref) {
            for(;(nr=nextobsf(&obs,&idx.base, 2))>0;idx.base+=nr)
                if(timediff(obs.data[idx.base].time,obs.data[idx.rover].time)>-DTTOL) break;
        }else{
            for(i=idx.base;(nr=nextobsf(&obs,&i,2)) > 0;idx.base=i,i+=nr)
                if(timediff(obs.data[i].time,obs.data[idx.rover].time)>DTTOL) break;
        }
        nr=nextobsf(&obs,&idx.base,2);
        if(nr<=0){
            nr=nextobsf(&obs,&idx.base,2);
        }
        for(i=0;i<nu;i++){
            gnss.push_back(obs.data[idx.rover+i]);
        }
        if (fusing.isPPK){
            for(i=0;i<nr;i++){
                gnss.push_back(obs.data[idx.base+i]);
            }
        }
        idx.rover+=nu;
    }
    else{
        if((nu=nextobsb(&obs,&idx.rover,1))<=0) return -1;
        if(fusing.opts.rtklib.prc_opt.intpref){
            for(;(nr=nextobsb(&obs,&idx.base,2))>0;idx.base-=nr)
                if (timediff(obs.data[idx.base].time,obs.data[idx.rover].time)<DTTOL) break;
        }else{
            for(i=idx.base;(nr=nextobsb(&obs,&i,2))>0;idx.base=i,i-=nr)
                if (timediff(obs.data[i].time,obs.data[idx.rover].time)<-DTTOL) break;
        }
        nr=nextobsb(&obs,&idx.base,2);
        for(i=0;i<nu;i++){
            gnss.push_back(obs.data[idx.rover-nu+1+i]);
        }
        if(fusing.isPPK){
            for(i=0;i<nr;i++){
                gnss.push_back(obs.data[idx.base-nr+1+i]);
            }
        }
        idx.rover-=nu;
    }

    /*to do check*/
    for (i=k=0;i<gnss.size();i++) {
        if ((satsys(gnss[i].sat, nullptr)&fusing.opts.rtklib.prc_opt.navsys)&&
            fusing.opts.rtklib.prc_opt.exsats[gnss[i].sat-1]!=1) gnss[k++]=gnss[i];
    }

    return (int)gnss.size();
}