//
// Created by hw on 10/6/22.
//

#include "Coordinate.h"
#include "Rotation.h"
#include "SyncSensors.h"
#include "Fusing.h"

#define SQR(x) ((x)*(x))

extern Vector3d gravity_ecef(const Vector3d &re)
{
    Vector3d ge;
    double mag_r=re.norm();
    if(fabs(mag_r)<1e-32){
        ge[0]=0.0;ge[1]=0.0;ge[2]=0.0;
    }else{
        /* Calculate gravitational acceleration using (2.142) */
        double f1=-WGS84_MU/pow(mag_r,3);
        double f2=1.5*WGS84_J2*pow(WGS84_RE/mag_r,2.0);
        double z_scale=5.0*pow((re[2]/mag_r),2.0);
        double g1=f1*(re[0]+f2*(1.0-z_scale)*re[0]);
        double g2=f1*(re[1]+f2*(1.0-z_scale)*re[1]);
        double g3=f1*(re[2]+f2*(3.0-z_scale)*re[2]);
        /* Add centripetal acceleration using (2.133) */
        ge[0]=g1+SQR(WGS84_OMGE)*re[0];
        ge[1]=g2+SQR(WGS84_OMGE)*re[1];
        ge[2]=g3;
    }
    return ge;
}

extern Vector3d gravity_llh(const Vector3d &rn,E_AttDefination llh_type)
{
    Vector3d g,gn;
    double g0;
    g0=9.7803253359*(1.0+0.001931853*SQR(sin(rn[0])))/sqrt(1.0-SQR(WGS84_ECC*sin(rn[0]))); /* (2.85) */
    /* calculate north gravity */
    g[0]=-8.08E-9*rn[2]*sin(2.0*rn[0]); /* (2.140)-v2.0 */
    /* east gravity is zero */
    g[1]=0.0;
    /* calculate down gravity */
    g[2]=g0*(1.0-2.0/WGS84_RE*(1.0+WGS84_FE*(1.0-2*SQR(sin(rn[0])))+
                               SQR(WGS84_OMGE*WGS84_RE)*WGS84_RP/WGS84_MU)*rn[2]+3.0/WGS84_RE/WGS84_RE*SQR(rn[2])); /* (2.139)-v2.0 */
    if(llh_type==+E_AttDefination::NED_FRD){
        gn=g;
    }
    else if(llh_type==+E_AttDefination::ENU_RFU){
        gn[0]=g[1];gn[1]=g[0];gn[2]=-g[2];
    }
    return gn;
}

static void updateEarthPar(Eigen::Vector3d llh, Eigen::Vector3d vn, E_AttDefination llh_type, earth_t *earth) {
    double sl=sin(llh[0]),cl=cos(llh[0]),tl=sl/cl;
    double sq=1.0-WGS84_ECC2*sl*sl, RN=WGS84_RE/sqrt(sq);
    double sl2=sl*sl,sl4=sl2*sl2;
    earth->tl = tl;
    earth->cl = cl;
    earth->sl = sl;
    earth->RNh = RN+llh[2];
    earth->RMh = RN*(1.0-WGS84_ECC2)/sq+llh[2];
    earth->clRNh=earth->RNh*cl;
    double g=GRAVITY0*(1.0+5.27094E-3*sl2+2.32718E-5*sl4)-3.086E-6*llh[2];
    Vector3d gn;
    switch (llh_type) {
        case E_AttDefination::NED_FRD:
            earth->w_nie=Vector3d(WGS84_OMGE*cl,0.0,-WGS84_OMGE*sl);
            earth->w_nen=Vector3d(vn[1]/earth->RNh,-vn[0]/earth->RMh,-vn[1]/earth->RNh*tl);
            earth->Mpv(0,0)=1.0/earth->RMh;earth->Mpv(1,1)=1.0/earth->clRNh;earth->Mpv(2,2)=-1.0;
            gn<<0,0,g;
            break;
        case E_AttDefination::ENU_RFU:
            earth->w_nie=Vector3d(0.0,WGS84_OMGE*cl,WGS84_OMGE*sl);
            earth->w_nen=Vector3d(-vn[1]/earth->RMh,vn[0]/earth->RNh,vn[0]/earth->RNh*tl);
            earth->Mpv(0,1)=1.0/earth->RMh;earth->Mpv(1,0)=1.0/earth->clRNh;earth->Mpv(2,2)=1.0;
            gn<<0,0,-g;
            break;
    }
    earth->w_nin=earth->w_nie+earth->w_nen;
    earth->w_nien=earth->w_nie+earth->w_nin;
    earth->gcc=gn-earth->w_nien.cross(vn);

    double sb=sin(llh[0]),cb=cos(llh[0]);
    sl=sin(llh[1]),cl=cos(llh[1]);
    earth->DPne(0,0)=-earth->RNh*sb*cl;earth->DPne(0,1)=-earth->RNh*cb*sl;earth->DPne(0,2)=cb*cl;
    earth->DPne(1,0)=-earth->RNh*sb*sl;earth->DPne(1,1)=earth->RNh*cb*cl;earth->DPne(1,2)=cb*sl;
    earth->DPne(2,0)=(earth->RNh-WGS84_ECC2*RN)*cb;earth->DPne(2,1)=0.0;earth->DPne(2,2)=sb;
}

static Quaterniond quatUpdate(Quaterniond qnb0,Vector3d rv_ib,Vector3d rv_in)
{
    double norm=rv_ib.norm();
    double n2=norm*norm;

    double rv_ib0,s,n,n_2;
    if(n2<1.0E-8){
        rv_ib0=1.0-n2*(1/8.0-n2/384.0);
        s=1/2.0-n2*(1/48.0-n2/3840.0);
    }
    else{
        n= sqrt(n2);n_2=n/2.0;
        rv_ib0=cos(n_2);s=sin(n_2)/n;
    }
    rv_ib*=s;
    double qb1,qb2,qb3,qb4;
    qb1=qnb0.w()*rv_ib0         - qnb0.x()*rv_ib(0) - qnb0.y()*rv_ib(1) - qnb0.z()*rv_ib(2);
    qb2=qnb0.w()*rv_ib(0) + qnb0.x()*rv_ib0         + qnb0.y()*rv_ib(2) - qnb0.z()*rv_ib(1);
    qb3=qnb0.w()*rv_ib(1) + qnb0.y()*rv_ib0         + qnb0.z()*rv_ib(0) - qnb0.x()*rv_ib(2);
    qb4=qnb0.w()*rv_ib(2) + qnb0.z()*rv_ib0         + qnb0.x()*rv_ib(1) - qnb0.y()*rv_ib(0);

    norm=rv_in.norm();
    n2=norm*norm;
    double rv_in0;
    if(n2<1.0E-8){
        rv_in0=1-n2*(1/8.0-n2/384.0);s=-1/2.0+n2*(1/48.0-n2/3840.0);
    }
    else{
        n= sqrt(n2);n_2=n/2.0;
        rv_in0=cos(n_2);s=-sin(n_2)/n;
    }
    rv_in*=s;

    Quaterniond qnb1=qnb0;
    qnb1.w()=rv_in0*qb1 - rv_in(0)*qb2 - rv_in(1)*qb3 - rv_in(2)*qb4;
    qnb1.x()=rv_in0*qb2 + rv_in(0)*qb1 + rv_in(1)*qb4 - rv_in(2)*qb3;
    qnb1.y()=rv_in0*qb3 + rv_in(1)*qb1 + rv_in(2)*qb2 - rv_in(0)*qb4;
    qnb1.z()=rv_in0*qb4 + rv_in(2)*qb1 + rv_in(0)*qb3 - rv_in(1)*qb2;
    norm=qnb1.norm();
    n2=norm*norm;
    if(n2>1.000001||n2<0.999999){
        double nq=1/sqrt(n2);
        qnb1.w()*=nq;
        qnb1.x()*=nq;
        qnb1.y()*=nq;
        qnb1.z()*=nq;
    }

    return qnb1;
}

extern int processTimeControl(double ts,double te, double sow, bool back)
{
    int stat=0;

    if(ts==0.0) stat=0;

    if(back){
        if(sow>te){
            LOG_N_TIMES(1,INFO)<<"process start time set "<<te;
            stat=1;
        }
        if(sow<ts) {
            LOG_N_TIMES(1,INFO)<<"process end time set "<<ts;
            stat=-1;
        }
    }
    else{
        if(sow<ts){
            LOG_N_TIMES(1,DEBUG)<<"process start time set "<<ts;
            stat=1;
        }
        if(sow>te){
            LOG_N_TIMES(1,DEBUG)<<"process end time set "<<te;
            stat=-1;
        }
    }
    return stat;
}

extern void getStateCov(const MatrixXd& P,state_t &state)
{
    Matrix3d tmp;
    state.cov.pos=P.block(xiP(),xiP(),3,3);
    state.cov.vel=P.block(xiV(),xiV(),3,3);
    state.cov.att=P.block(xiA(),xiA(),3,3);
    state.cov.ba =P.block(xiBa(),xiBa(),3,3);
    state.cov.bg =P.block(xiBg(),xiBg(),3,3);
}

static void traceInsAlign(const state_t& state,E_InsNavCoord nav_coord,E_AttDefination att_def,bool back)
{
    Vector3d re,rn,ve,vn,rpy;
    Matrix3d Cne;
    double sow=state.sow;

    if(nav_coord==+E_InsNavCoord::ECEF){
        re=state.pos;ve=state.vel;
        rn=Coordinate::ecef2llh(re);
        Cne=Rotation::getCne(rn,att_def);
        vn=Cne.transpose()*ve;
        rpy=state.att;
    }
    else if(nav_coord==+E_InsNavCoord::LLH){
        rn=state.pos,vn=state.vel;
        re=Coordinate::llh2ecef(rn);
        Cne=Rotation::getCne(rn,att_def);
        ve=Cne*vn;
        rpy=state.att;
    }

    fprintf(stdout,"%8.3f(%s): IMU initialization\n",back?"BKF":"FKF",sow);
    fprintf(stdout,"Position(ecef): %15.4f %15.4f %15.4f m\n",
            re[0],re[1],re[2]);
    fprintf(stdout,"Position(llh) : %15.9f %15.9f %15.4f deg m\n",
            rn[0]*R2D,rn[1]*R2D,rn[2]);
    fprintf(stdout,"Velocity(ecef): %15.4f %15.4f %15.4f m/s\n",
            ve[0],ve[1],ve[2]);
    fprintf(stdout,"Velocity(%s) : %15.4f %15.4f %15.4f m/s\n",
            att_def==+E_AttDefination::ENU_RFU?"enu":"ned",vn[0],vn[1],vn[2]);
    fprintf(stdout,"Attitude(rpy) : %15.4f %15.4f %15.4f deg\n",
            rpy[0]*R2D,rpy[1]*R2D,rpy[2]*R2D);
    fflush(stdout);
}

static bool alignTimeSync(double align_time,double imu_tag0,double imu_tag1,bool back)
{
    if(align_time==imu_tag1) return true;
    if(imu_tag0==0.0) return false;

    if(back){
        if(align_time>=imu_tag1&&align_time<imu_tag0) return true;
    }
    else{
        if(align_time<=imu_tag1&&align_time>imu_tag0) {
            return true;
        }
    }
    return false;
}

static bool insAlignManual(const imuopt_t &opt,const state_t &init,double imu_tag0,double imu_tag1,ins_t& ins,bool back)
{
    Matrix3d Cne;
    Vector3d re,rn,ve,vn,att;

    if(alignTimeSync(init.sow,imu_tag0,imu_tag1,back)){
        ins.state.sow=init.sow;
        if(opt.nav_coord==+E_InsNavCoord::ECEF){
            re=ins.state.pos=init.pos;ve=ins.state.vel=init.vel;
            att=init.att;
            if(opt.att_def==+E_AttDefination::ENU_RFU){
                att[0]=init.att[1],att[1]=init.att[0],att[2]=-init.att[2];
            }
            ins.state.att=att;
            rn=Coordinate::ecef2llh(re);
            Cne=Rotation::getCne(rn,opt.att_def);
            vn=Cne.transpose()*ve;
            ins.state.dcm=Rotation::att2Cbe(att,rn,opt.att_def);
            ins.state.quat=Rotation::dcm2quat(ins.state.dcm);
        }
        else if(opt.nav_coord==+E_InsNavCoord::LLH){
            re=init.pos,ve=init.vel;
            rn=ins.state.pos=Coordinate::ecef2llh(re);
            Cne=Rotation::getCne(rn,opt.att_def);
            vn=ins.state.vel=Cne.transpose()*ve;
            att=init.att;
            if(opt.att_def==+E_AttDefination::ENU_RFU){
                att[0]=init.att[1],att[1]=init.att[0],att[2]=-init.att[2];
            }
            ins.state.att=att;
            ins.state.dcm=Rotation::att2Cbn(att,opt.att_def);
            ins.state.quat=Rotation::dcm2quat(ins.state.dcm);
            ins.state.acc=Vector3d::Zero();
        }
        traceInsAlign(ins.state,opt.nav_coord,opt.att_def,back);
        return true;
    }
    return false;
}

static bool insAlignGpsSol(const imuopt_t &opt,const pv_t &pvs,int *idx,const Vector3d& arm,double imu_tag0,double imu_tag1,ins_t& ins,bool back)
{
    bool stat=false;
    static vector<pvData_t> align_data;
    if(imu_tag0==0.0) return stat;

    stat=igSyncPv(imu_tag0,imu_tag1,pvs,idx,back);
    if(stat){
        if(pvs.data[*idx].state!=SOLQ_FIX){
            align_data.clear();
            stat=false;
        }
        else{
            align_data.push_back(pvs.data[*idx]);
        }
    }

    Matrix3d Cne;
    Vector3d vn,ve,rn=V30,re,re1;
    if(align_data.size()==5){
        if(opt.nav_coord==+E_InsNavCoord::ECEF){
            re=align_data[4].pos;
            rn=Coordinate::ecef2llh(align_data[4].pos);
            Cne=Rotation::getCne(rn,opt.att_def);
            ve=(align_data[4].pos-align_data[3].pos)/(align_data[4].sow-align_data[3].sow);
            vn=Cne.transpose()*ve;
        }
        else if(opt.nav_coord==+E_InsNavCoord::LLH){
            rn=align_data[4].pos;
            re=Coordinate::llh2ecef(rn);
            re1=Coordinate::llh2ecef(align_data[3].pos);
            ve=(re-re1)/(align_data[4].sow-align_data[3].sow);
            Cne=Rotation::getCne(rn,opt.att_def);
            vn=Cne.transpose()*ve;
        }

        if(fabs(vn[0])>3.0&&fabs(vn[1])>3.0){
            ins.state.sow=imu_tag1;
            ins.state.att[0]=ins.state.att[1]=0.0;
            if(opt.att_def==+E_AttDefination::NED_FRD){
                ins.state.att[2]= atan2(vn[1], fabs(vn[0])<1E-4?1E-4:vn[0]);
            }
            else if(opt.att_def==+E_AttDefination::ENU_RFU){
                ins.state.att[2]=-(atan2(vn[0], fabs(vn[1])<1E-4?1E-4:vn[1]));
            }

            if(opt.nav_coord==+E_InsNavCoord::ECEF){
                ins.state.pos=re;ins.state.vel=ve;
                ins.state.dcm=Rotation::att2Cbe(ins.state.att,rn,opt.att_def);
                ins.state.quat=Rotation::dcm2quat(ins.state.dcm);
                removeIGArmLever(ins,arm, re, nullptr,false,opt.nav_coord);
                ins.state.pos=re;
            }
            else if(opt.nav_coord==+E_InsNavCoord::LLH){
                ins.state.pos=rn;ins.state.vel=vn;
                ins.state.dcm=Rotation::att2Cbn(ins.state.att,opt.att_def);
                ins.state.quat=Rotation::dcm2quat(ins.state.dcm);
                removeIGArmLever(ins,arm, rn, nullptr,false,opt.nav_coord);
                ins.state.pos=rn;
            }

            stat=true;
            traceInsAlign(ins.state,opt.nav_coord,opt.att_def,back);
            align_data.clear();
        }
        else{
            stat=false;
            align_data.clear();
        }

    }else stat=false;

    return stat;
}

extern bool insAlign(const fusingopt_t &opt,const fusingMeas_t &meas,fusing_t &fusing,bool back)
{
    bool stat=false;

    if(back&&fusing.opts.common.filter_type==+E_Estimator::FBS){
        LOG(DEBUG)<<setprecision(9)<<fusing.ins.state.sow<<" bkf ins alignment using fbk ins state";
        traceInsAlign(fusing.ins.state,opt.imu.nav_coord,opt.imu.att_def,back);
        return true;
    }

    double ts=fusing.opts.common.sow_start,te=fusing.opts.common.sow_end;
    while(!stat&&inputImu(&meas.imu,1,fusing.ins.pre_imu,fusing.ep_sensors_meas,fusing.ins.da,fusing.ins.dv,back)){

        /*process time control*/
        if(processTimeControl(ts,te,fusing.ep_sensors_meas.imu.sow,back)==-1) break;
        if(processTimeControl(ts,te,fusing.ep_sensors_meas.imu.sow,back)==1)  continue;

        switch (opt.imu.align_method) {
            case E_InsAlign::MANUAL:
                stat= insAlignManual(opt.imu,meas.imu.property.imu_init,fusing.ins.pre_imu.sow,fusing.ep_sensors_meas.imu.sow,fusing.ins,back);
                break;
            case E_InsAlign::MOVING:
                break;
            case E_InsAlign::GPSSOL:
                stat= insAlignGpsSol(opt.imu,meas.pvs,&fusing.ep_sensors_meas.idx.pv,meas.imu.property.ig_lever,
                                     fusing.ins.pre_imu.sow,fusing.ep_sensors_meas.imu.sow,fusing.ins,back);
                break;
            case E_InsAlign::GPSOBS:
                break;
        }
        if(!stat) fusing.ins.pre_imu=fusing.ep_sensors_meas.imu;
    }

    if(stat&&opt.imu.nav_coord==+E_InsNavCoord::LLH){
        updateEarthPar(fusing.ins.state.pos,fusing.ins.state.vel,opt.imu.att_def,&fusing.ins.earth);
    }

    back?fusing.ep_sensors_meas.idx.imu++:fusing.ep_sensors_meas.idx.imu--;
    getStateCov(fusing.ep_kf.P,fusing.ins.state);

    return stat;
}

static void calibrateImu(const imuopt_t &opts,ins_t &ins,double dt,Vector3d &da,Vector3d &dv)
{
    Vector3d v_acc=dv,v_gyr=da;

    da=v_gyr-ins.state.bg*dt;
    dv=v_acc-ins.state.ba*dt;

    Vector3d gyr_scale,acc_scale;
    if(opts.est_Sg==+E_SwitchOpt::ON){
        gyr_scale=Eigen::Vector3d::Ones()+ins.state.sg;
        da=da.cwiseProduct(gyr_scale.cwiseInverse());
    }

    if(opts.est_Sa==+E_SwitchOpt::ON){
        acc_scale=Eigen::Vector3d::Ones()+ins.state.sa;
        dv=dv.cwiseProduct(acc_scale.cwiseInverse());
    }
}

static void traceIns(int epoch, bool pre,const imuopt_t &opt, const ins_t &ins,const Vector3d& da,const Vector3d& dv,double dt,bool trace) {
    Vector3d re,rn,ve,vn,an,att;

    if(!trace) return;

    if(opt.nav_coord==+E_InsNavCoord::ECEF){
        re=ins.state.pos;
        rn=Coordinate::ecef2llh(re);
        ve=ins.state.vel;
        Matrix3d Cne=Rotation::getCne(rn,opt.att_def);
        vn=Cne.transpose()*ve;
        an=ins.state.acc;
        att=ins.state.att;
    }
    else if(opt.nav_coord==+E_InsNavCoord::LLH){
        rn=ins.state.pos;
        re=Coordinate::llh2ecef(rn);
        vn=ins.state.vel;
        Matrix3d Cne=Rotation::getCne(rn,opt.att_def);
        ve=Cne*vn;
        an=ins.state.acc;
        att=ins.state.att;
    }

    double sow=ins.state.sow;
    if(pre){
        cout<<"INS Mech ["<<(opt.nav_coord==+E_InsNavCoord::ECEF?"ECEF":"LLH]")<<epoch<<"(-): "<<setprecision(12)<<sow<<" dt: "<<dt<<endl;
    }
    else{
        cout<<"INS Mech ["<<(opt.nav_coord==+E_InsNavCoord::ECEF?"ECEF":"LLH]")<<epoch<<"(+): "<<setprecision(12)<<sow<<" dt: "<<dt<<endl;
    }
    fprintf(stdout,"acc measure  : %12.9f %12.9f %12.9f m/s\n",   da[0],da[1],da[2]);
    fprintf(stdout,"gyr measure  : %12.9f %12.9f %12.9f rad/s\n", dv[0],dv[1],dv[2]);
    fprintf(stdout,"Position (e) : %12.5f %12.5f %12.5f m\n",     re[0],re[1],re[2]);
//    fprintf(stdout,"Position (n) : %12.3f %12.3f %12.3f deg m\n", rn[0]*R2D,rn[1]*R2D,rn[2]);
//    fprintf(stdout,"Velocity (e) : %12.3f %12.3f %12.3f m/s\n",   ve[0],ve[1],ve[2]);
    fprintf(stdout,"Velocity (n) : %12.5f %12.5f %12.5f m/s\n",   vn[0],vn[1],vn[2]);
//    fprintf(stdout,"Acceleration : %12.3f %12.3f %12.3f m/s^2\n", an[0],an[1],an[2]);
    fprintf(stdout,"Attitude(rpy): %12.3f %12.3f %12.3f deg\n",   att[0]*R2D,att[1]*R2D,att[2]*R2D);
//    fprintf(stdout,"Acc bias(xyz): %12.3f %12.3f %12.3f mg\n",ins.state.ba[0]*MPS22MG,ins.state.ba[1]*MPS22MG,ins.state.ba[2]*MPS22MG);
//    fprintf(stdout,"Gyr bias(xyz): %12.3f %12.3f %12.3f deg/h\n",ins.state.bg[0]*RPS2DPH,ins.state.bg[1]*RPS2DPH,ins.state.bg[2]*RPS2DPH);
    fflush( stdout);
}

static void insUpdateECEF(E_AttDefination att_def,double dt,const Vector3d& da,const Vector3d& dv,ins_t &ins)
{
    ins.fb = dv/dt; ins.wib = da/dt;
    /*1: update attitude*/
    Vector3d delta_ie(0.0,0.0,-WGS84_OMGE*dt);

    Quaterniond qe_ie = Rotation::rv2quat(delta_ie);
    Quaterniond qb_ib = Rotation::rv2quat(da);
    Quaterniond q_pre = ins.state.quat;
#if 1
    ins.state.quat = qe_ie*q_pre*qb_ib;
    ins.state.quat.normalize();
    ins.state.dcm =Rotation::quat2dcm(ins.state.quat);
    Vector3d llh=Coordinate::ecef2llh(ins.state.pos);
    ins.state.att=Rotation::qbe2att(ins.state.quat,llh,att_def);
#else
    ins.state.quat= quatUpdate(q_pre,da,delta_ie);
    ins.state.dcm=Rotation::quat2dcm(ins.state.quat);
    ins.state.att=Rotation::qbn2att(ins.state.quat,att_def);
#endif
    /*2: sf transform*/
    Vector3d delta_ie2{0.0,0.0,-WGS84_OMGE*dt*0.5};
    qe_ie = Rotation::rv2quat(delta_ie2);
    Vector3d pre_v=ins.state.vel;
    Vector3d dv_e= qe_ie*q_pre*dv;                 /*velocity increment in ecef-frame*/

    Vector3d ge=gravity_ecef(ins.state.pos);

    /*3: update velocity*/
    Matrix3d m=Rotation::skewSymmetric(Vector3d(0,0,WGS84_OMGE));
    Vector3d dv_gcor = (ge-2.0*m*pre_v)*dt;  /* */
    ins.state.vel = pre_v+dv_e+dv_gcor;
    ins.state.acc = Vector3d::Zero();

    /*4: update position*/
    ins.state.pos+=0.5*(ins.state.vel+pre_v)*dt;
}

static void insUpdateLLH(E_AttDefination att_def,double dt,const Vector3d& da,const Vector3d& dv,ins_t &ins)
{
    /*earth and angular rate update*/
    Vector3d pos01,vel01;
    vel01 = ins.state.vel+ins.state.acc*dt*0.5; pos01=ins.state.pos+ins.earth.Mpv*vel01*dt*0.5;
    updateEarthPar(pos01,vel01,att_def,&ins.earth);
    ins.fb = dv/dt; ins.wib = da/dt; ins.web=ins.wib-ins.state.dcm.transpose()*ins.earth.w_nie;

    /*velocity update*/
    ins.fn = ins.state.quat*ins.fb;
    Vector3d delta_wn_in = -0.5*ins.earth.w_nin*dt;
    Quaterniond qn_in = Rotation::rv2quat(delta_wn_in);
    ins.state.acc = qn_in*ins.fn+ins.earth.gcc;
    Vector3d pre_vel = ins.state.vel;
    ins.state.vel = pre_vel+ins.state.acc*dt;

    /*position update*/
    ins.state.pos += 0.5*ins.earth.Mpv*(ins.state.vel+pre_vel)*dt;

    /*attitude update*/
    ins.state.quat= quatUpdate(ins.state.quat,da,ins.earth.w_nin*dt);
    ins.state.dcm = Rotation::quat2dcm(ins.state.quat);
    ins.state.att = Rotation::qbn2att(ins.state.quat,att_def);
}

extern bool insUpdate(const imuopt_t &opt,epImuData_t &ep_imu,ins_t &ins, bool back)
{

    ins.epoch++;

    calibrateImu(opt,ins,ep_imu.dt,ins.da,ins.dv);

    traceIns(ins.epoch,true,opt,ins,ins.da,ins.dv,ep_imu.dt,false);
    if(opt.nav_coord==+E_InsNavCoord::ECEF){
        insUpdateECEF(opt.att_def,ep_imu.dt,ins.da,ins.dv,ins);
    }
    else if(opt.nav_coord==+E_InsNavCoord::LLH){
        insUpdateLLH(opt.att_def,ep_imu.dt,ins.da,ins.dv,ins);
    }

    ep_imu.da=ins.da,ep_imu.dv=ins.dv,ins.dt=ep_imu.dt;
    ins.state.sow=ep_imu.data.begin()->sow;
    ins.pre_imu=ep_imu;
    traceIns(ins.epoch,false,opt,ins,ins.da,ins.dv,ep_imu.dt, false);
    ins.gstat=SOLQ_NONE;

    return true;
}
