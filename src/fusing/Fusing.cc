//
// Created by hw on 10/2/22.
//

#include <random>
#include "ProcessBar.h"
#include "jprogress_bar.hpp"
#include "Rotation.h"
#include "Coordinate.h"
#include "Fbs.h"
#include "OutSol.h"
#include "SyncSensors.h"
#include "TimeUpdate.h"
#include "LooseCouple.h"
#include "TightCouple.h"
#include "Fusing.h"

using namespace Joger::ProgressBar;

/* safe to call malloc */
#define MALLOC(pointer, type, sz)                                               \
    if(!((pointer) = (type *)malloc(((unsigned long)sz)*sizeof(type)))){        \
        LOG(ERROR)<<" memory allocation failed";                                  \
    }

/* safe to free pointer */
#define FREE(pointer) {free(pointer);(pointer) = NULL;}

#define SQR(x) ((x)*(x))


extern int xnP()
{
    return 3;
}

extern int xnV()
{
    return 3;
}

extern int xnA()
{
    return 3;
}

extern int xnBg()
{
    return 3;
}

extern int xnBa()
{
    return 3;
}

extern int xnSg(bool est)
{
    return est?3:0;
}

extern int xnSa(bool est)
{
    return est?3:0;
}

extern int xnClk(E_GnssMode mode,bool sd_gnss)
{
    if(sd_gnss) return 0;
   if(mode==+E_GnssMode::SINGLE) return 6;
   else if(mode<+E_GnssMode::PPP_KINEMA) return 0;
   else return NSYS;
}

extern int xnClkDrift(E_GnssMode mode,bool sd_gnss,bool clk_drift){
    if(!clk_drift) return 0;
    if(sd_gnss) return 0;
    if(mode==+E_GnssMode::SINGLE) return 6;
    else if(mode<+E_GnssMode::PPP_KINEMA) return 0;
    else return NSYS;
}

extern int xnTrp(E_GnssMode mode,E_TropOpt opt)
{
    if(mode<+E_GnssMode::KINEMATIC) return 0;
    return opt==+E_TropOpt::EST_ZTD?1:(opt==+E_TropOpt::EST_ZTD_GRAD?3:0);
}

extern int xnIon(E_GnssMode mode,E_IonoOpt opt)
{
    if(mode<+E_GnssMode::KINEMATIC) return 0;
    return opt==+E_IonoOpt::UC?MAXSAT:0;
}

extern int xnIfcb(E_GnssMode mode,int nf)
{
    if(mode>=+E_GnssMode::PPP_KINEMA&&mode<=+E_GnssMode::PPP_FIXED){
        return nf>3?1:0;
    }
    return 0;
}

extern int xnAmb(const fusingopt_t &opt)
{
    if(opt.common.prc_mode!=+E_PrcMode::IGTC) return 0;

    if(opt.gnss.mode<+E_GnssMode::DGPS){
        return 0;
    }
    else{
        return MAXSAT*xnFrq(opt);
    }
}

extern int xnX(const fusingopt_t &opt)
{
    E_PrcMode prc_mode=opt.common.prc_mode;
    E_GnssMode mode=opt.gnss.mode;
    E_IonoOpt  ion_opt = opt.gnss.iono_opt;
    E_TropOpt  trp_opt = opt.gnss.trop_opt;
    int a=xnClk(mode,opt.gnss.sd_gnss);
    int b=xnTrp(mode,trp_opt);
    int c=xnIon(mode,ion_opt);
    int d=xnClkDrift(mode,opt.gnss.sd_gnss,opt.imu.dop_aid==+E_SwitchOpt::ON||opt.imu.tdcp_aid==+E_SwitchOpt::ON);
    return xnP()+xnV()+xnA()+xnBg()+xnBa()+xnSg(opt.imu.est_Sg)+xnSa(opt.imu.est_Sg)
           + xnClk(mode,opt.gnss.sd_gnss)+ xnClkDrift(mode,opt.gnss.sd_gnss,opt.imu.dop_aid==+E_SwitchOpt::ON||opt.imu.tdcp_aid==+E_SwitchOpt::ON)
           + xnTrp(mode,trp_opt)+ xnIon(mode,ion_opt) + xnIfcb(mode,opt.gnss.nf)+ xnAmb(opt);
}

extern int xiP()
{
    return 0;
}

extern int xiV()
{
    return xnP();
}

extern int xiA()
{
    return xnP()+ xnV();
}

extern int xiBg()
{
    return xnA()+ xnV()+ xnP();
}

extern int xiBa()
{
    return xnA()+ xnV()+ xnP()+ xnBg();

}

extern int xiSg( )
{
    return xnA()+ xnV()+ xnP()+ xnBg()+ xnBa();
}

extern int xiSa(const imuopt_t &opt)
{
    return xnP()+ xnV()+ xnA()+ xnBg()+ xnBa()+ xnSg(opt.est_Sg);
}

extern int xiClk(const fusingopt_t &opts)
{
    return xnP()+ xnV()+ xnA()+ xnBg()+ xnBa()+ xnSg(opts.imu.est_Sg) + xnSa(opts.imu.est_Sa);
}

extern int xiClkDrift(const fusingopt_t &opts)
{
    return xnP()+ xnV()+ xnA()+ xnBg()+ xnBa()+ xnSg(opts.imu.est_Sg) + xnSa(opts.imu.est_Sa)+ xnClk(opts.gnss.mode,opts.gnss.sd_gnss);
}


extern int xiTrp(const fusingopt_t &opts)
{
    return xnP()+ xnV()+ xnA()+ xnBg()+ xnBa()+ xnSg(opts.imu.est_Sg) + xnSa(opts.imu.est_Sa) + xnClk(opts.gnss.mode,opts.gnss.sd_gnss)
            + xnClkDrift(opts.gnss.mode,opts.gnss.sd_gnss,opts.imu.dop_aid==+E_SwitchOpt::ON||opts.imu.tdcp_aid==+E_SwitchOpt::ON);
}

extern int xiIon(const fusingopt_t &opts,int sat)
{
    return xnP()+ xnV()+ xnA()+ xnBg()+ xnBa()+ xnSg(opts.imu.est_Sg) + xnSa(opts.imu.est_Sa) + xnClk(opts.gnss.mode,opts.gnss.sd_gnss)
           + xnClkDrift(opts.gnss.mode,opts.gnss.sd_gnss,opts.imu.dop_aid==+E_SwitchOpt::ON||opts.imu.tdcp_aid==+E_SwitchOpt::ON)
           + xnTrp(opts.gnss.mode,opts.gnss.trop_opt) + sat - 1;
}

extern int xiIfcb(const fusingopt_t &opts)
{
    return xnP()+ xnV()+ xnA()+ xnBg()+ xnBa()+ xnSg(opts.imu.est_Sg) + xnSa(opts.imu.est_Sa) + xnClk(opts.gnss.mode,opts.gnss.sd_gnss)
           + xnClkDrift(opts.gnss.mode,opts.gnss.sd_gnss,opts.imu.dop_aid==+E_SwitchOpt::ON||opts.imu.tdcp_aid==+E_SwitchOpt::ON)
           + xnTrp(opts.gnss.mode,opts.gnss.trop_opt) + xnIon(opts.gnss.mode,opts.gnss.iono_opt);
}

extern int xiAmb(const fusingopt_t &opts,int sat, int f)
{
    return xnP()+ xnV()+ xnA()+ xnBg()+ xnBa()+ xnSg(opts.imu.est_Sg) + xnSa(opts.imu.est_Sa) + xnClk(opts.gnss.mode,opts.gnss.sd_gnss)
           + xnClkDrift(opts.gnss.mode,opts.gnss.sd_gnss,opts.imu.dop_aid==+E_SwitchOpt::ON||opts.imu.tdcp_aid==+E_SwitchOpt::ON)
           + xnTrp(opts.gnss.mode,opts.gnss.trop_opt)+ xnIon(opts.gnss.mode,opts.gnss.iono_opt) + xnIfcb(opts.gnss.mode,opts.gnss.nf)+
           + MAXSAT*f+sat-1;
}

extern int xnFrq(const fusingopt_t &opt)
{
    return opt.gnss.iono_opt==+E_IonoOpt::IF?1:opt.gnss.nf;
}

extern int xnIns(const fusingopt_t &opt)
{
    return xnP()+xnV()+xnA()+xnBg()+xnBa()+ xnSg(opt.imu.est_Sg==+E_SwitchOpt::ON)+ xnSa(opt.imu.est_Sa==+E_SwitchOpt::ON);
}

extern bool addSolData(const ins_t *sol,track_t *ref)
{
    ins_t *data_tmp;

    if(ref->n==0&&ref->nmax==0){
        ref->nmax=10000;
        MALLOC(ref->data,ins_t ,ref->nmax);
    }
    else if(ref->n>=ref->nmax){
        ref->nmax*=2;
        if(!(data_tmp=(ins_t *)realloc(ref->data, sizeof(ins_t)*ref->nmax))){
            FREE(ref->data);
            ref->n=ref->nmax=0;
            return false;
        }
        ref->data=data_tmp;
    }
    ref->data[ref->n++]=*sol;
    return true;
}


static bool simMeasLoss(const imuopt_t & opt,double sow_meas)
{
    vector<double> ts,te;
    int i;

    if(opt.sim_gps_loss.n<0){
        LOG_N_TIMES(1,WARNING)<<"gps measurement loss simulation error n = "<<opt.sim_gps_loss.n<<" duration.size() = "<<opt.sim_gps_loss.duration.size();
        return false;
    }

    double t1,t2,dt;
    for(i=0;i<opt.sim_gps_loss.n;i++){
        if(i==0) t1=opt.sim_gps_loss.ts;
        else t1=t2+opt.sim_gps_loss.spacing;
        if(opt.sim_gps_loss.duration.size()==1) dt=opt.sim_gps_loss.duration[0];
        else dt=opt.sim_gps_loss.duration[i];
        t2=t1+dt;
        if(t1<=sow_meas&&sow_meas<=t2){
            LOG(DEBUG)<<"simulated gps measurement loss, ts: "<<t1<<" te: "<<t2<<" measurement sow: "<<sow_meas;
            return true;
        }
    }

    return false;
}

static bool stateUpdate(double pre_time,double cur_time,double meas_time,const imuProperty_t &imup,fusing_t &fusing,bool back)
{

    double delta=0.001;
    int stat=0;
    static int count=0;

    if(fabs(pre_time-meas_time)<delta){
        stat=1;
    }
    else if(fabs(cur_time-meas_time)<delta){
        stat=2;
    }
    else if((!back&&(pre_time<meas_time&&meas_time<cur_time))||(back&&(cur_time<meas_time&&meas_time<pre_time))){
        stat=2;
    }

    if(stat==0){
        LOG(TRACE)<<setprecision(12)<<fusing.ins.epoch<<" "<<cur_time<<" INS mechanization";
        /*ins mech*/
        timeUpdate(imup,fusing,back);
    }
    else if(stat==1) {
        LOG(TRACE)<<setprecision(12)<<fusing.ins.epoch<<" "<<pre_time<<" measurement update "<<cur_time<<" INS mechanization";
        /*measurement update*/
        if(!sensorsFusion(fusing)){
            LOG(WARNING)<<"sensor fusion measurement update error";
        }
        /*time update*/
        timeUpdate(imup,fusing,back);
    }
    else if(stat==2){
        LOG(TRACE)<<setprecision(12)<<fusing.ins.epoch<<" "<<pre_time<<" INS mechanization "<<cur_time<<" measurement update";

        /*time update*/
        timeUpdate(imup,fusing,back);
        /*measurement update*/
        if(!sensorsFusion(fusing)){
            LOG(WARNING)<<"sensor fusion measurement update error";
        }
    }
    else{
        epImuData_t mid_imu={0};
        epImuData_t pre_imu=fusing.ins.pre_imu,cur_imu=fusing.ep_sensors_meas.imu;

        imuInterpolate(pre_imu,cur_imu,meas_time,mid_imu,back);
        LOG(TRACE)<<setprecision(12)<<fusing.ins.epoch<<" "<<mid_imu.sow<<" INS mechanization "<<mid_imu.sow<<" measurement update "<<cur_time<<" INS mechanization";

        getIncreMeas(imup.rate,back,fusing.ins.epoch,pre_imu,mid_imu);
        imuCompensate(pre_imu,mid_imu,fusing.ins.da,fusing.ins.dv);
        fusing.ep_sensors_meas.imu=mid_imu;
        timeUpdate(imup,fusing,back);
        if(!sensorsFusion(fusing)){
            LOG(ERROR)<<"sensor fusion measurement update error";
        }

        getIncreMeas(imup.rate,back,fusing.ins.epoch,mid_imu,cur_imu);
        imuCompensate(fusing.ins.pre_imu,cur_imu,fusing.ins.da,fusing.ins.dv);
        fusing.ep_sensors_meas.imu=cur_imu;
        timeUpdate(imup,fusing,back);
    }

    return true;
}

static void insAidedGnssInfo(fusing_t &fusing,const Vector3d& ig_lever)
{
    Vector3d pos;
    Matrix3d cov_pos,Cne,DPne=fusing.ins.earth.DPne;
    removeIGArmLever(fusing.ins,ig_lever,pos, nullptr,true,fusing.opts.imu.nav_coord);
    cov_pos=fusing.ins.state.cov.pos;
    if(fusing.opts.imu.nav_coord==+E_InsNavCoord::LLH){
        pos=Coordinate::llh2ecef(pos);
        cov_pos=DPne*cov_pos*DPne.transpose();
    }
    matcpy(fusing.rtk.ins_pred.re,pos.data(),3,1);
    matcpy(fusing.rtk.ins_pred.var,cov_pos.data(),3,3);
}

static double syncSensors(const fusingMeas_t &meas,fusing_t &fusing,bool back)
{
    bool stat=true;
    double imu_tag0;
    double sow=0.0;

    /*previous imu time tag*/
    imu_tag0=fusing.ep_sensors_meas.imu.sow-fusing.ep_sensors_meas.imu.dt;

    if(fusing.iglc_sol){
        if(stat=igSyncPv(imu_tag0,fusing.ep_sensors_meas.imu.sow,meas.pvs,&fusing.ep_sensors_meas.idx.pv,back)){
            fusing.ep_sensors_meas.pv=meas.pvs.data[fusing.ep_sensors_meas.idx.pv];
            sow=fusing.ep_sensors_meas.pv.sow;
        }
    }
    else if(fusing.iglc_obs||fusing.igstc||fusing.igtc){
        if(stat=igSyncObs(imu_tag0,fusing.ep_sensors_meas.imu.sow,meas.obs,&fusing.ep_sensors_meas.idx,back)){

            insAidedGnssInfo(fusing,fusing.ins.ig_lever);

            /*input gnss obervation*/
            int nobs=0;
            if(!(nobs=inputEpGnssObs(fusing,meas.obs,fusing.ep_sensors_meas.idx,fusing.ep_sensors_meas.gnss,back))){
                LOG(WARNING)<<setprecision(9)<<fusing.ep_sensors_meas.imu.sow<<" number of satellites is insufficient";
                stat=false;
            }

            /*make gnss solution*/
            if(stat&&(fusing.iglc_obs||fusing.igstc)){
                if(!rtkpos(&fusing.rtk,fusing.ep_sensors_meas.gnss.data(),nobs,&fusing.nav)){
                    LOG(WARNING)<<setprecision(9)<<fusing.ep_sensors_meas.imu.sow<<" ig loosely coupled, make gnss solution error";
                    stat=false;
                }
                if(fusing.rtk.sol.stat){
                    fusing.ins.gstat=fusing.rtk.sol.stat;
                    sow=time2gpst(fusing.ep_sensors_meas.gnss[0].time, nullptr);
                    /*convert rtk.sol to pvData_t*/
                    if(fusing.igstc){
                        LOG(DEBUG)<<logTime(sow,fusing.ins.epoch,fusing.rtk.epoch)<<" ig semi-tightly coupled, gnss state = "<<(int)fusing.rtk.sol.stat;
                    }
                    else{
                        LOG(TRACE)<<setprecision(9)<<sow<<" ig loosely coupled, gnss state = "<<(int)fusing.rtk.sol.stat;
                    }
                    sol2pvData(fusing.rtk.sol,fusing.ep_sensors_meas.pv,fusing.opts.imu.nav_coord,fusing.opts.imu.att_def);
                    fusing.ep_sensors_meas.idx.pv=0;
                    sow=fusing.ep_sensors_meas.pv.sow;
                    stat=true;
                }
            }
            else if(fusing.igtc){
                sow=time2gpst(fusing.ep_sensors_meas.gnss[0].time, nullptr);
            }
        }
    }
    if(stat){
        if(simMeasLoss(fusing.opts.imu,sow)){
            stat=false;
            sow=0.0;
        }
    }
    return sow;
}

extern void removeIGArmLever(const ins_t &ins, const Eigen::Vector3d& arm, Eigen::Vector3d &pred_pos,
                               Eigen::Vector3d* pred_vel, bool ins2gnss,E_InsNavCoord mech_coord) {
    Vector3d T;

    if(mech_coord==+E_InsNavCoord::ECEF){
        Vector3d omge={0.0,0.0,WGS84_OMGE};
        T=ins.state.dcm*arm;
        if(ins2gnss){
            pred_pos=ins.state.pos+T;
            if(pred_vel){
                *pred_vel=ins.state.vel+ins.state.dcm*Rotation::skewSymmetric(ins.wib)*arm-Rotation::skewSymmetric(omge)*T;
            }
        }
        else{
            pred_pos=ins.state.pos-T;
            if(pred_vel)  *pred_vel=ins.state.vel-T;
        }
    }
    else if(mech_coord==+E_InsNavCoord::LLH){
        Matrix3d CW=ins.state.dcm*Rotation::skewSymmetric(ins.web);
        Matrix3d MpvCnb=ins.earth.Mpv*ins.state.dcm;
        if(ins2gnss){
            pred_pos=ins.state.pos+MpvCnb*arm;
            if(pred_vel) *pred_vel=ins.state.vel+CW*arm;
        }
        else{
            pred_pos=ins.state.pos-MpvCnb*arm;
            if(pred_vel) *pred_vel=ins.state.vel-CW*arm;
        }
    }
}

extern bool insFeedback(const imuopt_t &opt,E_InsNavCoord nav_coord,E_AttDefination att_def,ins_t &ins,double *x,int nx)
{
    Vector3d tmp,v_zero(NO_ZERO_FLAG,NO_ZERO_FLAG,NO_ZERO_FLAG);
    VectorXd x_tmp(nx);
    matcpy(x_tmp.data(),x,nx,1);

    /*feedback attitude*/
    tmp=x_tmp.segment(xiA(),xnA());

    if(tmp[0]*R2D>10.0||tmp[1]*R2D>10.0||tmp[2]*R2D>10.0){
        LOG(WARNING)<<setprecision(9)<<ins.state.sow<<": abnormal attitude correction";
        return false;
    }

    if(nav_coord==+E_InsNavCoord::ECEF){
        Quaterniond qtmp=Rotation::rv2quat(-tmp);
        ins.state.quat=qtmp*ins.state.quat;
        ins.state.dcm=Rotation::quat2dcm(ins.state.quat);
    }
    else if(nav_coord==+E_InsNavCoord::LLH){
        Quaterniond qtmp=Rotation::rv2quat(tmp);
        ins.state.quat=qtmp*ins.state.quat;
        ins.state.dcm=Rotation::quat2dcm(ins.state.quat);
    }

    if(nav_coord==+E_InsNavCoord::ECEF){
        Vector3d llh=Coordinate::ecef2llh(ins.state.pos);
        ins.state.att= Rotation::qbe2att(ins.state.quat,llh,att_def);
    }
    else if(nav_coord==+E_InsNavCoord::LLH){
        ins.state.att=Rotation::qbn2att(ins.state.quat,att_def);
    }
    x_tmp.segment(xiA(),xnA())=v_zero;

    /*feedback velocity*/
    tmp=x_tmp.segment(xiV(),xnV());
    ins.state.vel-=tmp;
    x_tmp.segment(xiV(),xnV())=v_zero;

    /*feedback position*/
    tmp=x_tmp.segment(xiP(),xnP());
    ins.state.pos-=tmp;
    x_tmp.segment(xiP(),xnP())=v_zero;

    /*feedback bg*/
    tmp=x_tmp.segment(xiBg(),xnBg());
    if(nav_coord==+E_InsNavCoord::LLH){
        ins.state.bg+=tmp;
    }
    else if(nav_coord==+E_InsNavCoord::ECEF){
        ins.state.bg-=tmp;
    }
    x_tmp.segment(xiBg(),xnBg())=v_zero;

    /*feedback ba*/
    tmp=x_tmp.segment(xiBa(),xnBa());
    if(nav_coord==+E_InsNavCoord::LLH){
        ins.state.ba+=tmp;
    }
    else if(nav_coord==+E_InsNavCoord::ECEF){
        ins.state.ba-=tmp;
    }
    x_tmp.segment(xiBa(),xnBa())=v_zero;

    if(opt.est_Sg==+E_SwitchOpt::ON){
        tmp=x_tmp.segment(xiSg(),xnSg(opt.est_Sg));
        if(nav_coord==+E_InsNavCoord::LLH){
            ins.state.sg+=tmp;
        }
        else if(nav_coord==+E_InsNavCoord::ECEF){
            ins.state.sg-=tmp;
        }
        x_tmp.segment(xiSg(),xnSg(opt.est_Sg))=v_zero;
    }

    if(opt.est_Sa==+E_SwitchOpt::ON){
        tmp=x_tmp.segment(xiSa(opt),xnSa(opt.est_Sa));
        if(nav_coord==+E_InsNavCoord::LLH){
            ins.state.sa+=tmp;
        }
        else if(nav_coord==+E_InsNavCoord::ECEF){
            ins.state.sg-=tmp;
        }
        x_tmp.segment(xiSa(opt),xnSa(opt.est_Sa))=v_zero;
    }

    matcpy(x,x_tmp.data(),nx,1);
    return true;
}


extern bool sensorsFusion(fusing_t &fusing)
{
    bool stat=false;
    if(fusing.iglc_sol||fusing.iglc_obs||fusing.igstc){
        if((stat=igSensorLC(fusing))){
            getStateCov(fusing.ep_kf.P,fusing.ins.state);
            fusing.ig_ins=fusing.ins;
        }
    }
    else if(fusing.igtc){
        fusing.ins.state.sow=time2gpst(fusing.ep_sensors_meas.gnss[0].time, nullptr);
        stat=igSensorTC(fusing);
    }

    if(stat) state2sol(fusing.opts.imu,fusing,fusing.ig_ins,fusing.ep_sol,fusing.week,fusing.opts.common.prc_mode);

    return stat;
}

static void initKalmanFilter(const fusingopt_t &fusing_opt,kf_t &kf,const imuProperty_t &imup,const imuopt_t &opt,bool back)
{
    bool comb=(fusing_opt.common.filter_type==+E_Estimator::FBS&&back)?true:false;
    int nx=xnX(fusing_opt);
    Matrix3d tmp;

    kf.nx=nx;
    kf.na=kf.nx-xnAmb(fusing_opt);

    /*initialize x*/
    kf.x=VectorXd::Zero(nx,1);
    kf.xa=VectorXd::Zero(kf.na,1);
    for(int i=0;i< xnIns(fusing_opt);i++) kf.x[i]=NO_ZERO_FLAG;

    /*initialize P*/
    kf.P=MatrixXd::Zero(nx,nx);
    kf.Pa=MatrixXd::Zero(kf.na,kf.na);

    tmp=imup.imu_error.std_att.asDiagonal();kf.P.block(xiA(), xiA(),3,3)  =tmp.array().square();
    tmp=imup.imu_error.std_vel.asDiagonal();kf.P.block(xiV(), xiV(),3,3)  =tmp.array().square();
    if(opt.nav_coord==+E_InsNavCoord::ECEF){
        tmp=imup.imu_error.std_pos.asDiagonal();kf.P.block(xiP(),xiP(),3,3)=tmp.array().square();
    }
    else if(opt.nav_coord==+E_InsNavCoord::LLH){
        tmp=imup.imu_error.std_pos.asDiagonal();
        tmp(0)/=WGS84_RE;tmp(4)/=WGS84_RE;
        kf.P.block(xiP(),xiP(),3,3)=tmp.array().square();
    }
    tmp=imup.imu_error.std_bg.asDiagonal(); kf.P.block(xiBg(),xiBg(),3,3)=tmp.array().square();
    tmp=imup.imu_error.std_ba.asDiagonal(); kf.P.block(xiBa(),xiBa(),3,3)=tmp.array().square();
    if(fusing_opt.imu.est_Sg==+E_SwitchOpt::ON){
        tmp=imup.imu_error.std_sg.asDiagonal();kf.P.block(xiSg(), xiSg(),3,3)=tmp.array().square();
    }
    if(fusing_opt.imu.est_Sa==+E_SwitchOpt::ON){
        tmp=imup.imu_error.std_sa.asDiagonal();kf.P.block(xiSa(fusing_opt.imu), xiSa(fusing_opt.imu),3,3)=tmp.array().square();
    }

    /*initialize Q*/
    kf.Q=MatrixXd::Zero(nx,nx);
    tmp=imup.imu_noise.gyr_arw.asDiagonal();kf.Q.block(xiA(), xiA(),3,3)  =tmp.array().square();
    tmp=imup.imu_noise.acc_vrw.asDiagonal();kf.Q.block(xiV(), xiV(),3,3)  =tmp.array().square();
    tmp=imup.imu_noise.gb_std.asDiagonal();  kf.Q.block(xiBg(),xiBg(),3,3) =tmp.array().square();
    tmp=imup.imu_noise.ab_std.asDiagonal();  kf.Q.block(xiBa(),xiBa(),3,3) =tmp.array().square();
    if(opt.est_Sg){
        tmp=imup.imu_noise.gs_std.asDiagonal();  kf.Q.block(xiSg(),xiSg(),3,3) =tmp.array().square();
    }
    if(opt.est_Sa){
        tmp=imup.imu_noise.as_std.asDiagonal();  kf.Q.block(xiSa(opt),xiSa(opt),3,3) =tmp.array().square();
    }

    /*initialize F*/
    kf.F=MatrixXd::Identity(nx,nx);

    /*initialize G*/
    kf.G=MatrixXd::Identity(nx,nx);
}

static void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;
    for (j=0;j<rtk->nx;j++) {
        rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;
    }
}


static void initRtklib(fusing_t *fusing, rtk_t &rtk, kf_t &kf, const fusingopt_t &opt)
{
    int i,f;
    rtkinit(&rtk,&opt.rtklib.prc_opt);

    if(opt.common.prc_mode==+E_PrcMode::IGSTC){
        rtk.stc=1;
    }
    else if(opt.common.prc_mode==+E_PrcMode::IGTC){
        rtk.nx = kf.nx;
        rtk.na = kf.na;
        rtk.x  = kf.x.data();
        rtk.P  = kf.P.data();
        rtk.xa = kf.xa.data();
        rtk.Pa = kf.Pa.data();
        rtk.fusing = fusing;
        rtk.tc=1;
    }
}

static void initProcess(fusing_t &fusing,const fusingMeas_t& meas,bool back)
{
    fusing.ep_sensors_meas.idx.imu=back?meas.imu.n-2:0;
    fusing.ep_sensors_meas.idx.pv=back?meas.pvs.n-1:0;
    fusing.ep_sensors_meas.idx.base=back?meas.obs.n-1:0;
    fusing.ep_sensors_meas.idx.rover=back?meas.obs.n-1:0;
    fusing.ins.state.ba=V30;
    fusing.ins.state.bg=V30;
    fusing.ins.epoch=0;
    fusing.ins.ig_lever=meas.imu.property.ig_lever;
    fusing.week=meas.imu.property.week;
    initKalmanFilter(fusing.opts,fusing.ep_kf,meas.imu.property,fusing.opts.imu,back);
    if(fusing.iglc_obs||fusing.igstc||fusing.igtc){
        initRtklib(&fusing,fusing.rtk,fusing.ep_kf,fusing.opts);
    }else{
        sol_t sol0={{0}};
        fusing.rtk.sol=sol0;
    }

    char statfile[1024];
    if(fusing.iglc_obs||fusing.igstc||fusing.igtc){
        if(fusing.opts.rtklib.sol_opt.sstat>0){
            strcpy(statfile,fusing.opts.input.sol_file.c_str());
            strcat(statfile,".stat");
            rtkclosestat();
            rtkopenstat(statfile,fusing.opts.rtklib.sol_opt.sstat);
        }
    }

}

extern void initFusing(fusing_t &fusing, Config &config)
{
    fusing.opts=config.fusingopt_;
    config.getRtklibOpts(&fusing.opts.rtklib.prc_opt,&fusing.opts.rtklib.sol_opt);

    if(fusing.opts.common.prc_mode==+E_PrcMode::IGLC){
        if(fusing.opts.gnss.mode==+E_GnssMode::SOL){
            fusing.iglc_sol=true;
        }
        else fusing.iglc_obs=true;
    }
    else if(fusing.opts.common.prc_mode==+E_PrcMode::IGSTC){
        fusing.igstc=true;
    }
    else if(fusing.opts.common.prc_mode==+E_PrcMode::IGTC){
        fusing.igtc=true;
    }

    if(fusing.opts.gnss.mode==+E_GnssMode::SINGLE){
        fusing.isSPP=true;
    }
    else if(fusing.opts.gnss.mode==+E_GnssMode::DGPS){
        fusing.isDGPS=true;
    }
    else if(fusing.opts.gnss.mode==+E_GnssMode::KINEMATIC){
        fusing.isPPK=true;
    }
    else if(fusing.opts.gnss.mode==+E_GnssMode::PPP_KINEMA){
        fusing.isPPP=true;
    }
}

static void freeFusing(fusing_t &fusing,fusingMeas_t& meas)
{
    if(fusing.solf.data) FREE(fusing.solf.data);fusing.solf.n=fusing.solf.nmax=0;
    if(fusing.solb.data) FREE(fusing.solb.data);fusing.solb.n=fusing.solb.nmax=0;
    fusing.ep_sensors_meas.imu.data.clear();
    fusing.ep_sensors_meas.gnss.clear();
    if(meas.imu.data) FREE(meas.imu.data);meas.imu.n=meas.imu.nmax=0;
    if(meas.pvs.data) FREE(meas.pvs.data);meas.pvs.n=meas.pvs.nmax=0;
    if(meas.obs.data) FREE(meas.obs.data);meas.obs.n=meas.obs.nmax=0;
}

extern bool sensorFusingProcess(FILE *fp_out,const fusingMeas_t &meas,fusing_t& fusing,bool back)
{
    LOG(TRACE)<<"sensorFusingProcess() back= "<<back?"true":"false";
    int stat=0,cont=0;
    /*init fusing*/
    initProcess(fusing,meas,back);

    /*ins algin*/
    if(!insAlign(fusing.opts,meas,fusing,back)){
        return false;
    }

    double ts=fusing.opts.common.sow_start,te=fusing.opts.common.sow_end;
    double meas_sow=0.0;

    fusing.ins.pre_imu.da=fusing.ins.pre_imu.data[0].ang;
    fusing.ins.pre_imu.dv=fusing.ins.pre_imu.data[0].vel;
    JProgressBar bar((meas.imu.n)/10000.0,back?"BKF":"FKF");

    while(inputImu(&meas.imu,-1,fusing.ins.pre_imu,fusing.ep_sensors_meas,fusing.ins.da,fusing.ins.dv,back)){

        /*process time control*/
        if(processTimeControl(ts,te,fusing.ep_sensors_meas.imu.sow,back)==-1) break;
        if(processTimeControl(ts,te,fusing.ep_sensors_meas.imu.sow,back)==1)  continue;

#if 1
        timeUpdate(meas.imu.property,fusing,back);

        if(meas_sow=syncSensors(meas,fusing,back)){
            bar.update(fusing.ep_sensors_meas.idx.imu/10000.0);
            if(!sensorsFusion(fusing)){
                LOG(WARNING)<<logTime(fusing.ins.state.sow,fusing.ins.epoch,fusing.rtk.epoch)<<" sensor fusion failed";
            }
        }
#else
        /*sensors synchronous*/
        if(meas_sow=syncSensors(meas,fusing,back)){
            /*measurement update*/
            if(fusing.ins.epoch==0) meas_sow=0.0;
            stateUpdate(fusing.ins.pre_imu.sow,fusing.ep_sensors_meas.imu.sow,meas_sow,meas.imu.property,fusing,back);
        }else{
            /*time update*/
            stateUpdate(fusing.ins.pre_imu.sow,fusing.ep_sensors_meas.imu.sow,0.0,meas.imu.property,fusing,back);
        }
#endif
        /*out solution*/
        if(fp_out&&meas_sow){
            outsol(fp_out,&fusing.ep_sol,fusing.opts.rtklib.prc_opt.rb,&fusing.opts.rtklib.sol_opt,0);
        }
        else{
            back?addSolData(&fusing.ins,&fusing.solb):addSolData(&fusing.ins,&fusing.solf);
        }
    }
    bar.end();

    return true;
}


extern bool loadMeasurement(fusing_t &fusing,const fusingopt_t &opts,fusingMeas_t &meas)
{
    LOG(TRACE)<<"loadMeasurement()";
    if(opts.common.prc_mode>=+E_PrcMode::INS){
        if(!Config::parseImuConf(opts.input.imup_file,meas.imu.property,opts.imu.att_def)){
            return false;
        }

        if(!loadImu(opts.common.sow_start,opts.common.sow_end,opts.input.imu_file,opts.imu,&meas.imu)){
            return false;
        }
    }

    /*load gnss pv solutions*/
    if(fusing.iglc_sol){
        if(!loadPVs(opts.input.pos_file,opts.imu.gnsspv_fmt,&meas.pvs,opts.imu.nav_coord,opts.imu.att_def)){
            return false;
        }
    }
    else if(fusing.iglc_obs||fusing.igstc||fusing.igtc){
        const char *infile[10];
        int n=0,index[10]={0};
        gtime_t ts={0},te={0};

        infile[n]=opts.input.rnx_files[0].c_str(); index[n]=n;n++;
        if(fusing.isDGPS||fusing.isPPK) infile[n]=opts.input.rnx_files[1].c_str(), index[n]=n,n++;
        infile[n]=opts.input.nav_files[0].c_str(), index[n]=n,n++;

        if(!readobsnav(ts, te, 0.0, const_cast<char **>(infile), index, n, &opts.rtklib.prc_opt, &meas.obs, &fusing.nav, nullptr)){
            freeobsnav(&meas.obs,&meas.nav);
            return false;
        }

        if(fusing.isPPP){
           char path[1024];
           pcvs_t pcvss={0};        /* receiver antenna parameters */
           pcvs_t pcvsr={0};        /* satellite antenna parameters */

           for(int i=0;i<opts.input.sp3_files.size();i++) infile[n]=opts.input.sp3_files[i].c_str(),n++;
           for(int i=0;i<opts.input.clk_files.size();i++) infile[n]=opts.input.clk_files[i].c_str(),n++;

            readpreceph(const_cast<char **>(infile), n, &fusing.opts.rtklib.prc_opt, &fusing.nav, nullptr);

            /* read satellite antenna parameters */
            if (!opts.input.atx_file.empty()&&!(readpcv(opts.input.atx_file.c_str(),&pcvss))) {
                LOG(WARNING)<<"miss atx file";
                return false;
            }
            else{
                setpcv(meas.obs.data[0].time,&fusing.opts.rtklib.prc_opt,&fusing.nav,&pcvss,&pcvsr,
                       nullptr);
            }

            /* read erp data */
            if (!opts.input.erp_files[0].empty()) {
                free(fusing.nav.erp.data); fusing.nav.erp.data= nullptr; fusing.nav.erp.n=fusing.nav.erp.nmax=0;
                reppath(opts.input.erp_files[0].c_str(),path,ts,"","");
                if (!readerp(path,&fusing.nav.erp)) {
                    LOG(WARNING)<<"miss erp";
                    return false;
                }
            }

            /* read dcb parameters */
            if (!opts.input.dcb_files[0].empty()) {
                reppath(opts.input.dcb_files[0].c_str(),path,ts,"","");
                readdcb(path,&fusing.nav, nullptr);
            } else {
                for (int i=0;i<3;i++) {
                    for (auto & cbia : fusing.nav.cbias) cbia[i]=0;
                    for (auto & rbia : fusing.nav.rbias) for (int k=0;k<2;k++) rbia[k][i]=0;
                }
            }

            /* read ocean tide loading parameters */
//            if (fusing.opts.rtklib.prc_opt.mode>PMODE_SINGLE&&!fusing.opts.input.blq_file.empty()) {
//                readotl(&fusing.opts.rtklib.prc_opt,fusing.opts.input.blq_file.c_str(), nullptr);
//            }
        }
    }

    return true;
}

extern bool multiSensorFusing(fusingMeas_t &meas,fusing_t& fusing)
{
    LOG(TRACE)<<"multiSensorFusing()";
    bool back=fusing.opts.common.filter_type==+E_Estimator::BKF?true:false;
    bool comb=fusing.opts.common.filter_type==+E_Estimator::FBS?true:false;
    bool stat=false;

    FILE *fp=fopen(fusing.opts.input.sol_file.c_str(),"a+");

    /*out solution header*/
    if(!outhead(fusing.opts.input.sol_file.c_str(),nullptr,0,&fusing.opts.rtklib.prc_opt,&fusing.opts.rtklib.sol_opt)){
        LOG(ERROR)<<"open solution file failed, exit ...";
        return -1;
    }

    fusing.solf.n=fusing.solf.nmax=0;
    if(!(stat=sensorFusingProcess(comb?nullptr:fp,meas,fusing,back))){
        return false;
    }

    /*backward filter for combined*/
    if(fusing.opts.common.filter_type==+E_Estimator::FBS){
        fusing.solb.n=fusing.solb.nmax=0;
        if(!(stat=sensorFusingProcess(nullptr,meas,fusing,true))){
            return false;
        }
        if(!fbSmoother(fp,fusing)){
            LOG(ERROR)<<"forward-backward smoothing process error, solf.n= "<<fusing.solf.n<<" solb.n= "<<fusing.solb.n;
        }
    }

    if(fusing.isPPK||fusing.isPPP_AR){
        LOG(INFO)<<endl<<"total epoch: "<<fusing.rtk.epoch<< "  fixed epoch: "<<fusing.rtk.fix_epoch<<"  fixed rate: "
                 <<setprecision(5)<<fusing.rtk.fix_epoch/(fusing.rtk.epoch+0.01)*100.0<<"%";
    }

    freeFusing(fusing,meas);
    fclose(fp);

    return stat;
}