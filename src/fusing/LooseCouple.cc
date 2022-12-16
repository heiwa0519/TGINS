//
// Created by hw on 10/30/22.
//

#include <random>
#include "LooseCouple.h"

#define SQR(x) ((x)*(x))

static default_random_engine e(time(0));
static normal_distribution<double> n(0,0.5);

static bool getHRV(const fusingopt_t &opts,const pvData_t &pv, const ins_t &ins,int nx, Eigen::MatrixXd& H, Eigen::Vector3d& v, Eigen::Matrix3d& R) {


    H=MatrixXd::Zero(3,nx);
    H.block(0,xiP(),3,3)=Matrix3d::Identity();

    Vector3d pred_pos;
    removeIGArmLever(ins, ins.ig_lever, pred_pos, nullptr,true,opts.imu.nav_coord);

    R=pv.cov_pos.asDiagonal();
    if(opts.imu.gnsspv_fmt==+E_GnssPvFmt::PSINS){
        Vector3d pos_err;
        if(opts.imu.nav_coord==+E_InsNavCoord::ECEF){
            pos_err=Vector3d(0.1,0.1,0.3);
        }
        else if(opts.imu.nav_coord==+E_InsNavCoord::LLH){
            pos_err=Vector3d(1,1,3);
        }
        Vector3d tmp,rn=Coordinate::ecef2llh(ins.state.pos);
        Matrix3d Cne=Rotation::getCne(rn,E_AttDefination::ENU_RFU);
        tmp=Cne*pos_err;
        Vector3d rand_pos_err;
        if(opts.imu.nav_coord==+E_InsNavCoord::ECEF){
            rand_pos_err=Vector3d(pos_err(0)*n(e),pos_err(1)*n(e),pos_err(2)*n(e));
        }
        else if(opts.imu.nav_coord==+E_InsNavCoord::LLH){
            rand_pos_err=Vector3d(pos_err(0)*n(e)/WGS84_RE,pos_err(1)*n(e)/WGS84_RE,pos_err(2)*n(e));
        }
        v=ins.state.pos-pv.pos+rand_pos_err;
    }
    else{
        v=pred_pos-pv.pos;
        R=R*10.0;
    }

    return true;
}

static Matrix3d igg3(const fusing_t &fusing,const MatrixXd H,const MatrixXd P,const Matrix3d R,Vector3d v)
{
    Matrix3d new_Pyy,Pyy;
    Matrix3d new_R=R;
    double norm_v=1.5,k0=1.0,k1=2.5;

    Pyy=H*P*H.transpose()+R;
    for(int i=0;i<3;i++){
//        norm_v=fabs(v(i)/sqrt(Pyy(i,i)));
//        if(fabs(v[i])>0.10){
//            new_R*=1000000.0;
//            break;
//        }
//        if(norm_v>k1){
//            new_R(i,i)*=1000000.0;
//            LOG(TRACE)<<logTime(fusing.ins.state.sow,fusing.ins.epoch,fusing.rtk.epoch)<<" innovation abnormal nv= "<<norm_v;
//        }
//        if(norm_v>k0&&norm_v<k1){
//            new_R(i,i)*=1.0/((k0/norm_v)*SQR((k1-norm_v)/(k1-k0)));
//            LOG(TRACE)<<logTime(fusing.ins.state.sow,fusing.ins.epoch,fusing.rtk.epoch)<<" innovation abnormal nv= "<<norm_v;
//        }
    }

    new_Pyy=H*P*H.transpose()+new_R;

    return new_Pyy;
}

static void measUpdate(fusing_t &fusing,const Eigen::MatrixXd H, const Eigen::Vector3d v, const Eigen::Matrix3d R) {
    Matrix3d Pyy=Matrix3d::Zero();

//    Pyy=igg3(fusing,H,fusing.ep_kf.P,R,v);
    Pyy=H*fusing.ep_kf.P*H.transpose()+R;


    MatrixXd Kk=MatrixXd::Zero(fusing.ep_kf.nx,3);
    Kk=fusing.ep_kf.P*H.transpose()*Pyy.inverse();
    fusing.ep_kf.x+=Kk*v;

    fusing.ep_kf.P=fusing.ep_kf.P-Kk*Pyy*Kk.transpose();
    fusing.ep_kf.P=(fusing.ep_kf.P+fusing.ep_kf.P.transpose())/2;
}


extern bool igSensorLC(fusing_t &fusing)
{
    MatrixXd H;
    Vector3d v;
    Matrix3d R;

    if(fusing.iglc_sol) fusing.rtk.epoch++;

    getHRV(fusing.opts,fusing.ep_sensors_meas.pv,fusing.ins,fusing.ep_kf.nx,H,v,R);

    measUpdate(fusing,H,v,R);
#if 0
//    Vector3d re,ve,rn,vn,vvv,gps_re,gps_rn;
//    Matrix3d Cne;
//    if(fusing.opts.imu.nav_coord==+E_InsNavCoord::ECEF){
//        gps_re=fusing.ep_sensors_meas.pv.pos;gps_rn=Coordinate::ecef2llh(gps_re);
//        re=fusing.ins.state.pos; rn=Coordinate::ecef2llh(re);
//        ve=fusing.ins.state.vel; Cne=Rotation::getCne(rn,fusing.opts.imu.att_def);vn=Cne.transpose()*ve;
//    }
//    else if(fusing.opts.imu.nav_coord==+E_InsNavCoord::LLH){
//        gps_rn=fusing.ep_sensors_meas.pv.pos;gps_re=Coordinate::llh2ecef(gps_rn);
//        rn=fusing.ins.state.pos; re=Coordinate::llh2ecef(rn);
//        vn=fusing.ins.state.vel; Cne=Rotation::getCne(rn,fusing.opts.imu.att_def);ve=Cne*vn;
//    }
//    fprintf(stdout,"ins_xyz: %15.3f %15.3f %15.3f\n",re[0],re[1],re[2]);
//    fprintf(stdout,"gps_xyz: %15.3f %15.3f %15.3f\n",gps_re[0],gps_re[1],gps_re[2]);
//    fprintf(stdout,"ins_llh: %15.9f %15.9f %15.3f\n",rn[0],rn[1],rn[2]);
//    fprintf(stdout,"gps_llh: %15.9f %15.9f %15.3f\n",gps_rn[0],gps_rn[1],gps_rn[2]);
//    fprintf(stdout,"ins_ve : %12.3f %12.3f %12.3f\n",ve[0],ve[1],ve[2]);
//    fprintf(stdout,"ins_vn : %12.3f %12.3f %12.3f\n",vn[0],vn[1],vn[2]);
    fprintf(stdout,"v      : %12.3f %12.3f %12.3f\n",v[0],v[1],v[2]);
    fflush( stdout);
#endif
    /* ins feedback */
    if(!insFeedback(fusing.opts.imu,fusing.opts.imu.nav_coord,fusing.opts.imu.att_def,fusing.ins,fusing.ep_kf.x.data(),fusing.ep_kf.nx)){
        LOG(ERROR)<<fusing.ins.epoch<<" ins feedback abnormal";
        return false;
    }

    fusing.ins.state.sow=fusing.ep_sensors_meas.pv.sow;
    fusing.ins.gstat=fusing.ep_sensors_meas.pv.state;

    return true;
}
