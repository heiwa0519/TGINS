//
// Created by hw on 10/2/22.
//

#ifndef FUSING_IMU_H
#define FUSING_IMU_H

#include <vector>
#include "EigenInc.h"
#include "Enums.h"
#include "Logger.h"
#include "Config.h"

using namespace std;

struct epMeas_t;

typedef struct {
    Matrix3d pos,vel,att;
    Matrix3d ba,bg;
}stateCov_t;

typedef struct{
    double sow;
    Vector3d pos,vel,acc,att;
    Matrix3d dcm;
    Quaterniond quat;
    Vector3d ba,bg;
    Vector3d sa,sg;
    stateCov_t cov;
}state_t;

typedef struct{
    Vector3d acc_vrw,gyr_arw;
    Vector3d gb_std,ab_std;
    Vector3d gs_std,as_std;
    double tau_bg,tau_ba;
}imuNoise_t;

typedef struct{
    Vector3d std_pos,std_vel,std_att;
    Vector3d std_ba,std_bg;
    Vector3d std_sa,std_sg;
}imuError_t;

struct imuProperty_t{
    E_ImuFmt fmt;
    int week;
    double rate;
    imuNoise_t imu_noise;
    imuError_t imu_error;
    state_t imu_init;
    Vector3d ig_lever;
};

typedef struct{
    double sow;
    double dt;
    Vector3d acc,vel;
    Vector3d gyr,ang;
}imuDataUnit_t;

typedef struct{
    double sow;
    double dt;
    Vector3d da,dv; /*no correction, eg rot or scull error*/
    vector<imuDataUnit_t> data;
}epImuData_t;

typedef struct{
    int n,nmax;
    imuDataUnit_t *data;
    imuProperty_t property;
}imu_t;

extern bool loadImu(double ts,double te,string imu_file,imuopt_t imu_opt,imu_t *imu);
extern bool inputImu(const imu_t *imu,int sample,const epImuData_t &pre_imu,epMeas_t &ep_meas,Vector3d &da,Vector3d &dv,bool back);
extern void imuInterpolate(const epImuData_t &imu0,epImuData_t &imu2,double t,epImuData_t  &imu1,bool back);
extern void getIncreMeas(double rate,bool back,int epoch,const epImuData_t &pre_imu,epImuData_t &cur_imu);
extern void imuCompensate(const epImuData_t &pre_imu,epImuData_t &cur_imu,Vector3d &da,Vector3d& dv);

#endif //FUSING_IMU_H
