//
// Created by hw on 10/2/22.
//

#ifndef FUSING_FUSING_H
#define FUSING_FUSING_H

#include "Gnss.h"
#include "Imu.h"
#include "Ins.h"
#include "rtklib.h"

#define NO_ZERO_FLAG 1E-32

struct track_t{
    int n,nmax;
    ins_t *data;
};

struct fusingMeas_t{
    imu_t imu;
    pv_t  pvs;
    obs_t obs;
    nav_t nav;
};

typedef struct{
    int imu;
    int pv;
    int rover;
    int base;
}measIdx_t;

struct epMeas_t{
    epImuData_t  imu;
    pvData_t pv;
    vector<obsd_t> gnss;
    measIdx_t idx;
};

typedef struct{
    int nx,nix;
    int na;
    VectorXd x, xa;
    MatrixXd P, Pa;
    MatrixXd F, Q, G;
}kf_t;

struct fusing_t{
    fusingopt_t opts;
    rtk_t  rtk;
    ins_t  ins,ig_ins;
    nav_t  nav;

    int week;
    epMeas_t ep_sensors_meas;
    kf_t  ep_kf;
    sol_t  ep_sol;
    track_t solf={0},solb={0};

    bool iglc_sol=false,iglc_obs=false,igstc=false,igtc=false;
    bool isSPP=false,isDGPS=false,isPPK=false,isPPP=false,isPPP_AR=false;
};

extern int xnP();
extern int xnV();
extern int xnA();
extern int xnBg();
extern int xnBa();
extern int xnSg(bool est);
extern int xnSa(bool est);
extern int xnClk(E_GnssMode mode,bool sd_gnss);
extern int xnClkDrift(E_GnssMode mode,bool sd_gnss,bool dop_aid);
extern int xnTrp(E_GnssMode mode,E_TropOpt opt);
extern int xnIon(E_GnssMode mode,E_IonoOpt opt);
extern int xnIfcb(E_GnssMode mode,int nf);
extern int xnAmb(const fusingopt_t &opt);
extern int xnX(const fusingopt_t &opt);
extern int xnFrq(const fusingopt_t &opt);
extern int xnIns(const fusingopt_t &opt);

extern int xiP();
extern int xiV();
extern int xiA();
extern int xiBg();
extern int xiBa();
extern int xiSg();
extern int xiSa(const imuopt_t &opt);
extern int xiClk(const fusingopt_t &opts);
extern int xiClkDrift(const fusingopt_t &opts);
extern int xiTrp(const fusingopt_t &opts);
extern int xiIon(const fusingopt_t &opts,int sat);
extern int xiIfcb(const fusingopt_t &opts);
extern int xiAmb(const fusingopt_t &opts,int sat, int f);


extern bool addSolData(const state_t *sol,track_t *ref);
extern void removeIGArmLever(const ins_t &ins, const Eigen::Vector3d& arm, Eigen::Vector3d &pred_pos,
                             Eigen::Vector3d* pred_vel, bool ins2gnss,E_InsNavCoord mech_coord);
extern bool loadMeasurement(fusing_t &fusing, const fusingopt_t &opts,fusingMeas_t &meas);
extern bool sensorsFusion(fusing_t &fusing);
extern bool sensorFusingProcess(FILE *fp_out,const fusingMeas_t &meas,fusing_t& fusing,bool back);
extern bool multiSensorFusing(fusingMeas_t &meas,fusing_t& fusing);
extern void initFusing(fusing_t &fusing, Config &config);
extern bool insFeedback(const imuopt_t &opt,E_InsNavCoord nav_coord,E_AttDefination att_def,ins_t &ins,double *x,int nx);
#endif //FUSING_FUSING_H
