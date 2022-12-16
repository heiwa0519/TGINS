//
// Created by hw on 10/2/22.
//

#include <regex>
#include <iostream>
#include "Fusing.h"

#define MAXIMUDATA 100000
#define MAXSTRLINEIMUDATA 512

/* safe to call malloc */
#define MALLOC(pointer, type, sz)                                               \
    if(!((pointer) = (type *)malloc(((unsigned long)sz)*sizeof(type)))){        \
        LOG(ERROR)<<" memory allocation failed";                                  \
    }

/* safe to free pointer */
#define FREE(pointer) {free(pointer);(pointer) = NULL;}

static void stringSplit(const string& str,const string& split,vector<string>& res)
{
    std::regex reg(split);
    std::sregex_token_iterator pos(str.begin(),str.end(),reg,-1);
    decltype(pos) end;
    for(;pos!=end;++pos){
        res.push_back(pos->str());
    }
}

static bool addImuData(const imuDataUnit_t *imu_data,imu_t *imu)
{
    imuDataUnit_t *data_tmp;

    if(imu->n==0&&imu->nmax==0){
        imu->nmax=MAXIMUDATA;
        MALLOC(imu->data,imuDataUnit_t,imu->nmax);
    }
    else if(imu->n>=imu->nmax){
        imu->nmax*=2;
        if(!(data_tmp=(imuDataUnit_t*) realloc(imu->data, sizeof(imuDataUnit_t)*imu->nmax))){
            FREE(imu->data);
            imu->n=imu->nmax=0;
            return false;
        }
        imu->data=data_tmp;
    }
    imu->data[imu->n++]=*imu_data;
    return true;
}

static bool readImuDataNvt(double ts,double te,fstream &file_fp,imuopt_t imu_opt,imu_t *imu)
{
    double sow,gx,gy,gz,ax,ay,az;
    imuDataUnit_t imu_data;
    string line;
    while(std::getline(file_fp,line)){
        if(line[0]=='%') continue;
        sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf %lf",&sow,&gx,&gy,&gz,&ax,&ay,&az);

        if((ts!=0.0&&sow>=ts)&&(te!=0.0&&sow<=te)){
            imu_data.sow=sow;
            if(imu_opt.att_def==+E_AttDefination::ENU_RFU){
                imu_data.gyr=Vector3d(gx*D2R,gy*D2R,gz*D2R);
                imu_data.acc=Vector3d(ax,ay,az);
            }
            else{
                imu_data.gyr=Vector3d(gy*D2R,gx*D2R,-gz*D2R);
                imu_data.acc=Vector3d(ay,ax,-az);
            }
            addImuData(&imu_data,imu);
        }
    }

    return true;
}

static bool readImuDataPsins(fstream &file_fp,imuopt_t imu_opt,imu_t *imu)
{
    double sow,gx,gy,gz,ax,ay,az;
    imuDataUnit_t imu_data;
    string line;
    while(std::getline(file_fp,line)){
        if(line[0]=='%') continue;
        sscanf(line.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",&gx,&gy,&gz,&ax,&ay,&az,&sow);

        imu_data.sow=sow;
        if(imu_opt.att_def==+E_AttDefination::ENU_RFU){
            imu_data.gyr=Vector3d(gx,gy,gz);
            imu_data.acc=Vector3d(ax,ay,az);
        }
        else{
            imu_data.gyr=Vector3d(gy,gx,-gz);
            imu_data.acc=Vector3d(ay,ax,-az);
        }
        imu_data.acc*=imu->property.rate;
        imu_data.gyr*=imu->property.rate;
        addImuData(&imu_data,imu);
    }

    return true;
}

static bool readImuDataGins(fstream &file_fp,imuopt_t imu_opt,imu_t *imu)
{
    double sow,gx,gy,gz,ax,ay,az;
    imuDataUnit_t imu_data;
    string line;
    while(std::getline(file_fp,line)){
        if(line[0]=='%') continue;
        sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf %lf",&sow,&gx,&gy,&gz,&ax,&ay,&az);

        imu_data.sow=sow;
        if(imu_opt.att_def==+E_AttDefination::ENU_RFU){
            imu_data.gyr=Vector3d(gy,gx,-gz);
            imu_data.acc=Vector3d(ay,ax,-az);
        }
        else{
            imu_data.gyr=Vector3d(gx,gy,gz);
            imu_data.acc=Vector3d(ax,ay,az);
        }
        imu_data.acc*=imu->property.rate;
        imu_data.gyr*=imu->property.rate;
        addImuData(&imu_data,imu);
    }

    return true;
}

static bool readImuDataDefault(std::fstream &file_fp,imu_t *imu)
{
    vector<double> data;
    imuDataUnit_t imu_data;
    while(file_fp.read((char*)data.data(),sizeof(double)*7)){
        imu_data.sow=data[0];
        imu_data.gyr[0]= data[1];
        imu_data.gyr[1]= data[2];
        imu_data.gyr[2]= data[3];
        imu_data.acc[0]=data[4];
        imu_data.acc[1]=data[5];
        imu_data.acc[2]=data[6];
        addImuData(&imu_data,imu);
        data.clear();
    }
    return imu->n>0;
}

extern bool loadImu(double ts,double te,string imu_file,imuopt_t imu_opt,imu_t *imu)
{
    if(imu_file.empty()){
        LOG(ERROR)<<"imu file error";
        return false;
    }

    bool stat=false;
    std::fstream file_fp;
    file_fp.open(imu_file,std::ios_base::in);
    if(!file_fp.is_open()) return false;

    switch (imu->property.fmt) {
        case +E_ImuFmt::PSINS:
            stat = readImuDataPsins(file_fp,imu_opt,imu);
            break;
        case +E_ImuFmt::NOVATEL:
            stat = readImuDataNvt(ts,te,file_fp,imu_opt,imu);
            break;
        case +E_ImuFmt::M39:
            break;
        case +E_ImuFmt::GINS:
            stat = readImuDataGins(file_fp,imu_opt,imu);
            break;
        case +E_ImuFmt::A15:
            break;
        case +E_ImuFmt::DEFAULT:
            stat = readImuDataDefault(file_fp,imu);
            break;
    }
    file_fp.close();
    return stat;
}

extern bool inputImu(const imu_t *imu,int sample,const epImuData_t &pre_imu,epMeas_t &ep_meas,Vector3d &da,Vector3d &dv,bool back)
{
    int sample_=sample;
    if(sample==-1) sample_=1;

    ep_meas.imu.data.clear();
    if(back){
        if(ep_meas.idx.imu>imu->n||ep_meas.idx.imu-sample_<0) return false;
        for(int i=ep_meas.idx.imu;i>ep_meas.idx.imu-sample_;i--){
            ep_meas.imu.data.push_back(imu->data[i+1]);
            ep_meas.imu.data[0].sow=imu->data[i].sow;
        }
        ep_meas.idx.imu-=sample_;
    }
    else{
        if(ep_meas.idx.imu<0||ep_meas.idx.imu+sample_>imu->n) return false;
        for(int i=ep_meas.idx.imu;i<ep_meas.idx.imu+sample_;i++){
            ep_meas.imu.data.push_back(imu->data[i]);
        }
        ep_meas.idx.imu+=sample_;
    }

    ep_meas.imu.sow=ep_meas.imu.data[0].sow;
    ep_meas.imu.dt=back?-1.0/imu->property.rate:1.0/imu->property.rate;
    ep_meas.imu.da=V30;
    ep_meas.imu.dv=V30;

    getIncreMeas(imu->property.rate,back,1,pre_imu,ep_meas.imu);
    imuCompensate(pre_imu,ep_meas.imu,da,dv);

    return true;
}

extern void imuInterpolate(const epImuData_t &imu0,epImuData_t &imu2,double t,epImuData_t &imu1,bool back)
{
    if(!back&&(imu0.sow>t||imu2.sow<t)) return;
    if(back&&(imu2.sow>t||imu0.sow<t)) return;

    double lamda;
    imuDataUnit_t imu_data={0};
    imu1.data.push_back(imu_data);

    lamda=fabs((t-imu0.sow)/fabs((imu2.sow-imu0.sow)));

    imu1.sow=imu1.data[0].sow=t;
    imu1.data[0].dt=t-imu0.sow;
    imu1.data[0].gyr=imu2.data[0].gyr*lamda;
    imu1.data[0].acc=imu2.data[0].acc*lamda;

    imu2.data[0].dt=imu2.sow-t;
    imu2.data[0].gyr=imu2.data[0].gyr*(1.0-lamda);
    imu2.data[0].acc=imu2.data[0].acc*(1.0-lamda);
}

extern void getIncreMeas(double rate,bool back,int epoch,const epImuData_t &pre_imu,epImuData_t &cur_imu)
{
    double dt=0.0;
    for(int i=0;i<cur_imu.data.size();i++){
        if(i==0){
            if(pre_imu.data.empty()){
                dt=1.0/rate*(back?-1.0:1.0);
            }
            else dt=cur_imu.data.begin()->sow-pre_imu.data.back().sow;
            if(dt>(1.0/rate)*1.5){
                LOG(WARNING)<<logTime(cur_imu.sow,epoch,0)<<" Loss imu data, pre_time_tag "<<setprecision(10)<<pre_imu.data.back().sow
                            <<", back_time_tag "<<cur_imu.data[i].sow
                            <<", imu_epoch "<<epoch<<setprecision(4)<<", dt "<<dt<<", rate "<<1.0/rate;
            }
        }
        else{
            dt=cur_imu.data[i].sow-cur_imu.data[i-1].sow;
            if(dt>(1.0/rate)*1.5){
                LOG(WARNING)<<logTime(cur_imu.sow,epoch,0)<<"Loss imu data, pre_time_tag "<<setprecision(10)<<cur_imu.data[i-1].sow
                            <<", back_time_tag "<<cur_imu.data[i].sow
                            <<", imu_epoch "<<epoch<<setprecision(4)<<", dt "<<dt<<" rate "<<1.0/rate;
            }
        }

        if(dt<=0.0) dt=back?-1.0/rate:1.0/rate;
        cur_imu.data[i].vel=cur_imu.data[i].acc*dt;
        cur_imu.data[i].ang=cur_imu.data[i].gyr*dt;
        cur_imu.data[i].dt=dt;
    }
}

extern void imuCompensate(const epImuData_t &pre_imu, epImuData_t &cur_imu,Vector3d& da,Vector3d& dv)
{
    Vector3d dphi=Vector3d::Zero(),rot=Vector3d::Zero(),scull=Vector3d::Zero();

    dphi = (1.0/12.0)*pre_imu.da.cross(cur_imu.data.begin()->ang);
    da = cur_imu.data.begin()->ang + dphi;

    scull = 1.0/12.0*(pre_imu.da.cross(cur_imu.data.begin()->vel)+pre_imu.dv.cross(cur_imu.data.begin()->ang));
    rot = 1.0/2.0*(cur_imu.data.begin()->ang.cross(cur_imu.data.begin()->vel));

    dv = cur_imu.data.begin()->vel + scull+rot;
    dv=cur_imu.data.begin()->vel+scull;
    cur_imu.da=cur_imu.data.begin()->ang;
    cur_imu.dv=cur_imu.data.begin()->vel;
    cur_imu.dt=cur_imu.data.begin()->dt;
}



