//
// Created by hw on 10/6/22.
//

#ifndef FUSING_INS_H
#define FUSING_INS_H

#include "Imu.h"

struct fusing_t;
struct fusingMeas_t;

typedef struct{
    Vector3d w_nie,w_nen,w_nin,w_nien;
    double tl,sl,cl;
    double RNh,RMh,clRNh;
    Matrix3d Mpv=Matrix3d::Zero(),DPne=Matrix3d::Zero();
    Vector3d gn,gcc;
}earth_t;

typedef struct{
    int epoch=0;
    state_t state;
    Vector3d ig_lever;
    double dt;
    Vector3d da,dv;
    Vector3d fb,wib,fn,web;
    earth_t earth;
    epImuData_t pre_imu={0};
    int gstat=SOLQ_NONE;
    int ns;
}ins_t;

extern void getStateCov(const MatrixXd& P,state_t &state);
extern int processTimeControl(double ts,double te, double sow, bool back);
extern bool insAlign(const fusingopt_t &opt,const fusingMeas_t &meas,fusing_t &fusing,bool back);
extern bool insUpdate(const imuopt_t &opt,epImuData_t &ep_imu,ins_t &ins, bool back);

#endif //FUSING_INS_H
