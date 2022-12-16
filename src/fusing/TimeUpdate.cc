//
// Created by hw on 10/30/22.
//

#include "TimeUpdate.h"
#include "OutSol.h"


extern void jacobiTransA2A_ECEF(MatrixXd &F,double dt)
{
    Matrix3d Fxx=M30;
    Vector3d omge_ie(0,0,WGS84_OMGE);
    Fxx=Matrix3d::Identity()-Rotation::skewSymmetric(omge_ie)*dt;

    F.block(xiA(),xiA(),3,3)=Fxx;
}

extern void jacobiTransA2Bg_ECEF(const Matrix3d &dcm,MatrixXd &F,double dt)
{
    F.block(xiA(),xiBg(),3,3)=-dcm*dt;
}

extern void jacobiTransA2Sg_ECEF(const imuopt_t &opt,const Matrix3d &dcm,const Vector3d &da,MatrixXd &F)
{
    F.block(xiA(), xiSg(),3,3)=-dcm*da.asDiagonal();
}

extern void jacobiTransV2A_ECEF(const Matrix3d &dcm,const Vector3d &dv,MatrixXd &F)
{
    /*[-Cbe*fib_b]x(*dt)*/
    F.block(xiV(),xiA(),3,3)=-Rotation::skewSymmetric(dcm*dv);
}

extern void jacobiTransV2V_ECEF(const Vector3d &dv,MatrixXd &F,double dt)
{
    /*[I3-2*OMGE(*dt)x]*/
    Vector3d omge_ie(0,0,WGS84_OMGE);

    F.block(xiV(),xiV(),3,3)=Matrix3d::Identity()-2.0*Rotation::skewSymmetric(omge_ie)*dt;
}

extern void jacobiTransV2P_ECEF(MatrixXd &F)
{
    F.block(xiV(),xiP(),3,3)=M30;
}

extern void jacobiTransV2Ba_ECEF(const Matrix3d &dcm,MatrixXd &F,double dt)
{
    F.block(xiV(),xiBa(),3,3)=-dcm*dt;
}

extern void jacobiTransV2Sa_ECEF(const imuopt_t &opt,const Matrix3d &dcm,const Vector3d dv,MatrixXd &F)
{
    F.block(xiV(), xiSa(opt),3,3)=-dcm*dv.asDiagonal();
}

extern void jacobiTransP2V_ECEF(MatrixXd &F,double dt)
{
    Matrix3d Fxx=M30;
    for(int i=0;i<3;i++) Fxx(i,i)=dt;
    F.block(xiP(),xiV(),3,3)=Fxx;
}

extern void jacobiTransP2P_ECEF(MatrixXd &F,double dt)
{
    F.block(xiP(),xiP(),3,3)=Matrix3d::Identity();
}

extern void jacobiTransMarkov(double dt,double tao,MatrixXd &F,int i,int j){
    Matrix3d Fxx=M30;
    Fxx(0,0)=tao==0.0?1.0:1.0-dt/tao;
    Fxx(1,1)=tao==0.0?1.0:1.0-dt/tao;
    Fxx(2,2)=tao==0.0?1.0:1.0-dt/tao;

    F.block(i,j,3,3)=Fxx;
}

static void getMp_LLH(E_AttDefination att_def,const ins_t &ins,Matrix3d *M1,Matrix3d *M2,Matrix3d *M){
    double tl=ins.earth.tl, secl=1.0/ins.earth.cl;
    double f_RMh=1.0/ins.earth.RMh,f_RNh=1.0/ins.earth.RNh;
    double f_clRNh=1.0/ins.earth.clRNh;
    double f_RMh2=f_RMh*f_RMh,f_RNh2=f_RNh*f_RNh;
    double vE_clRNh,vE_RNh2,vN_RMh2;
    Vector3d vn=ins.state.vel;

    if(att_def==+E_AttDefination::NED_FRD){
        vE_clRNh=vn[1]*f_clRNh;vE_RNh2=vn[1]*f_RNh2;vN_RMh2=vn[0]*f_RMh2;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        vE_clRNh=vn[0]*f_clRNh;vE_RNh2=vn[0]*f_RNh2;vN_RMh2=vn[1]*f_RMh2;
    }

    Matrix3d O33=Matrix3d::Zero();
    Matrix3d Mp1=O33;
    if(att_def==+E_AttDefination::NED_FRD){
        Mp1(1)=ins.earth.w_nie(2),Mp1(2)=-ins.earth.w_nie(1);
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mp1(1)=-ins.earth.w_nie(2),Mp1(2)=ins.earth.w_nie(1);
    }
    Matrix3d Mp2=O33;
    if(att_def==+E_AttDefination::NED_FRD){
        Mp2(2)=-vE_clRNh*secl;Mp2(6)=-vE_RNh2;
        Mp2(7)=vN_RMh2;Mp2(8)=vE_RNh2*tl;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mp2(2)=vE_clRNh*secl;Mp2(6)=vN_RMh2;
        Mp2(7)=-vE_RNh2;Mp2(8)=-vE_RNh2*tl;
    }

    if(M1) *M1=Mp1;
    if(M2) *M2=Mp2;
    if(M)  *M=Mp1+Mp2;
}


extern void jacobiTransA2A_LLH(const Vector3d &w_nin,MatrixXd &F,double dt){
    Matrix3d Fxx=-Rotation::skewSymmetric(w_nin*dt);
    Fxx+=Matrix3d::Identity();

    F.block(xiA(),xiA(),3,3)=Fxx;
}

extern void jacobiTransA2V_LLH(E_AttDefination att_def,const ins_t &ins,MatrixXd &F,double dt)
{
    double tl=ins.earth.tl;
    double f_RMh=1.0/ins.earth.RMh,f_RNh=1.0/ins.earth.RNh;
    Matrix3d Fxx=Matrix3d::Zero();

    if(att_def==+E_AttDefination::NED_FRD){
        Fxx(1)=-f_RMh;Fxx(3)=f_RNh;Fxx(5)=-f_RNh*tl;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Fxx(1)=f_RNh;Fxx(2)=f_RNh*tl;Fxx(3)=-f_RMh;
    }
    Fxx*=dt;
    F.block(xiA(),xiV(),3,3)=Fxx;
}

extern void jacobiTransA2P_LLH(E_AttDefination att_def,const ins_t &ins,MatrixXd &F,double dt)
{
    Matrix3d Mp1,Mp2;

    getMp_LLH(att_def,ins,&Mp1,&Mp2, nullptr);

    F.block(xiA(),xiP(),3,3)=(Mp1+Mp2)*dt;
}

extern void jacobiTransA2Bg_LLH(const Matrix3d &dcm,double dt,MatrixXd &F)
{
    F.block(xiA(),xiBg(),3,3)=-dcm*dt;
}

extern void jacobiTransA2Sg_LLH(const Matrix3d &dcm,const Vector3d &da,MatrixXd &F)
{
    F.block(xiA(),xiSg(),3,3)=-dcm*da.asDiagonal();
}

extern void jacobiTransV2A_LLH(const Vector3d &fn,double dt,MatrixXd &F)
{
    F.block(xiV(),xiA(),3,3)= Rotation::skewSymmetric(fn)*dt;
}

extern void jacobiTransV2V_LLH(const ins_t &ins,E_AttDefination att_def,double dt,MatrixXd &F)
{
    double tl=ins.earth.tl;
    double f_RMh=1.0/ins.earth.RMh,f_RNh=1.0/ins.earth.RNh;
    Matrix3d Avn=Rotation::skewSymmetric(ins.state.vel);
    Matrix3d Awn=Rotation::skewSymmetric(ins.earth.w_nien); /*to do check Awn,[2*w_n_ie+w_n_en]x*/
    Matrix3d Maa=-Rotation::skewSymmetric(ins.earth.w_nin);
    Matrix3d Mav=Matrix3d::Zero();
    if(att_def==+E_AttDefination::NED_FRD){
        Mav(1)=-f_RMh;Mav(3)=f_RNh;Mav(5)=-f_RNh*tl;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mav(1)=f_RNh;Mav(2)=f_RNh*tl;Mav(3)=-f_RMh;
    }
    F.block(xiV(),xiV(),3,3)=Matrix3d::Identity()+(Avn*Mav-Awn)*dt;
}

extern void jacobiTransV2P_LLH(const ins_t &ins,E_AttDefination att_def,double dt,MatrixXd &F)
{
    Matrix3d Avn=Rotation::skewSymmetric(ins.state.vel);

    Matrix3d Mp1,Map;
    getMp_LLH(att_def,ins, &Mp1, nullptr,&Map);

    Matrix3d Mvp=Avn*(Mp1+Map); /*to do check*/

    double scl=ins.earth.sl*ins.earth.cl,sl2=ins.earth.sl*ins.earth.sl;
    if(att_def==+E_AttDefination::NED_FRD){
        Mvp(2)=Mvp(2)+GRAVITY0*(5.27094e-3*2+2.32718e-5*4*sl2)*scl;
        Mvp(8)=Mvp(8)-3.086e-6;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mvp(2)=Mvp(2)-GRAVITY0*(5.27094e-3*2+2.32718e-5*4*sl2)*scl;
        Mvp(8)=Mvp(8)+3.086e-6;
    }

    F.block(xiV(),xiP(),3,3)=Avn*(Mp1+Map)*dt;
}

extern void jacobiTransV2Ba_LLH(const Matrix3d &dcm,double dt,MatrixXd &F)
{
    F.block(xiV(),xiBa(),3,3)=dcm*dt;
}

extern void jacobiTransV2Sa_LLH(const imuopt_t &opt,const Matrix3d &dcm,const Vector3d &dv,MatrixXd &F)
{
    F.block(xiV(), xiSa(opt),3,3)=dcm*dv.asDiagonal();
}

extern void jacobiTransP2P_LLH(const ins_t &ins,E_AttDefination att_def,double dt,MatrixXd &F,Matrix3d *M)
{
    double tl=ins.earth.tl, secl=1.0/ins.earth.cl;
    double f_RMh=1.0/ins.earth.RMh,f_RNh=1.0/ins.earth.RNh;
    double f_clRNh=1.0/ins.earth.clRNh;
    double f_RMh2=f_RMh*f_RMh,f_RNh2=f_RNh*f_RNh;
    double vE_clRNh,vE_RNh2,vN_RMh2;
    Vector3d vn=ins.state.vel;
    if(att_def==+E_AttDefination::NED_FRD){
        vE_clRNh=vn[1]*f_clRNh;vE_RNh2=vn[1]*f_RNh2;vN_RMh2=vn[0]*f_RMh2;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        vE_clRNh=vn[0]*f_clRNh;vE_RNh2=vn[0]*f_RNh2;vN_RMh2=vn[1]*f_RMh2;
    }

    Matrix3d Mpv=Matrix3d::Zero(),Mpp=Matrix3d::Zero();
    if(att_def==+E_AttDefination::NED_FRD){
        Mpv(0)=f_clRNh;Mpv(4)=f_RMh;Mpv(8)=-1.0;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mpv(1)=f_clRNh;Mpv(3)=f_RMh;Mpv(8)=1.0;
    }
    Mpp(1)=vE_clRNh*tl;Mpp(6)=-vN_RMh2;Mpp(7)=-vE_RNh2*secl;

    if(M) *M=Mpv;
    else F.block(xiP(),xiP(),3,3)=Matrix3d::Identity()+Mpp*dt;
}

extern void jacobiTransP2V_LLH(const ins_t &ins,E_AttDefination att_def,double dt,MatrixXd &F)
{
    Matrix3d Mpv;
    jacobiTransP2P_LLH(ins,att_def,dt,F,&Mpv);

    F.block(xiP(),xiV(),3,3)=Mpv*dt;
}

static void updateF_LLH1(const imuopt_t &opt,const ins_t &ins,const imuProperty_t& imup,E_AttDefination att_def,MatrixXd& phi)
{
    double tl=ins.earth.tl, secl=1.0/ins.earth.cl;
    double f_RMh=1.0/ins.earth.RMh,f_RNh=1.0/ins.earth.RNh;
    double f_clRNh=1.0/ins.earth.clRNh;
    double f_RMh2=f_RMh*f_RMh,f_RNh2=f_RNh*f_RNh;
    Vector3d vn=ins.state.vel;
    double vE_clRNh,vE_RNh2,vN_RMh2;
    if(att_def==+E_AttDefination::NED_FRD){
        vE_clRNh=vn[1]*f_clRNh;vE_RNh2=vn[1]*f_RNh2;vN_RMh2=vn[0]*f_RMh2;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        vE_clRNh=vn[0]*f_clRNh;vE_RNh2=vn[0]*f_RNh2;vN_RMh2=vn[1]*f_RMh2;
    }
    Matrix3d O33=Matrix3d::Zero();
    Matrix3d Mp1=O33;
    if(att_def==+E_AttDefination::NED_FRD){
        Mp1(1)=ins.earth.w_nie(2),Mp1(2)=-ins.earth.w_nie(1);
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mp1(1)=-ins.earth.w_nie(2),Mp1(2)=ins.earth.w_nie(1);
    }
    Matrix3d Mp2=O33;
    if(att_def==+E_AttDefination::NED_FRD){
        Mp2(2)=-vE_clRNh*secl;Mp2(6)=-vE_RNh2;
        Mp2(7)=vN_RMh2;Mp2(8)=vE_RNh2*tl;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mp2(2)=vE_clRNh*secl;Mp2(6)=vN_RMh2;
        Mp2(7)=-vE_RNh2;Mp2(8)=-vE_RNh2*tl;
    }

    Matrix3d Avn=Rotation::skewSymmetric(vn);
    Matrix3d Awn=Rotation::skewSymmetric(ins.earth.w_nien); /*to do check Awn,[2*w_n_ie+w_n_en]x*/
    Matrix3d Maa=-Rotation::skewSymmetric(ins.earth.w_nin);
    Matrix3d Mav=O33;
    if(att_def==+E_AttDefination::NED_FRD){
        Mav(1)=-f_RMh;Mav(3)=f_RNh;Mav(5)=-f_RNh*tl;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mav(1)=f_RNh;Mav(2)=f_RNh*tl;Mav(3)=-f_RMh;
    }
    Matrix3d Map=Mp1+Mp2;
    Matrix3d Mva= Rotation::skewSymmetric(ins.fn);
    Matrix3d Mvv=Avn*Mav-Awn;
    Matrix3d Mvp=Avn*(Mp1+Map); /*to do check*/

#if 1
    double scl=ins.earth.sl*ins.earth.cl,sl2=ins.earth.sl*ins.earth.sl;
    if(att_def==+E_AttDefination::NED_FRD){
        Mvp(2)=Mvp(2)+GRAVITY0*(5.27094e-3*2+2.32718e-5*4*sl2)*scl;
        Mvp(8)=Mvp(8)-3.086e-6;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mvp(2)=Mvp(2)-GRAVITY0*(5.27094e-3*2+2.32718e-5*4*sl2)*scl;
        Mvp(8)=Mvp(8)+3.086e-6;
    }
#endif

    Matrix3d Mpv=O33,Mpp=O33;
    if(att_def==+E_AttDefination::NED_FRD){
        Mpv(0)=f_clRNh;Mpv(4)=f_RMh;Mpv(8)=-1.0;
    }
    else if(att_def==+E_AttDefination::ENU_RFU){
        Mpv(1)=f_clRNh;Mpv(3)=f_RMh;Mpv(8)=1.0;
    }
    Mpp(1)=vE_clRNh*tl;Mpp(6)=-vN_RMh2;Mpp(7)=-vE_RNh2*secl;

    phi.block(xiA(),xiA(), 3,3)=Maa;
    phi.block(xiA(),xiV(), 3,3)=Mav;
    phi.block(xiA(),xiP(), 3,3)=Map;
    phi.block(xiA(),xiBg(),3,3)=-ins.state.dcm;
    phi.block(xiA(),xiSg(),3,3)=-ins.state.dcm*ins.da.asDiagonal();

    phi.block(xiV(),xiA(), 3,3)=Mva;
    phi.block(xiV(),xiV(), 3,3)=Mvv;
    phi.block(xiV(),xiP(), 3,3)=Mvp;
    phi.block(xiV(),xiBa(),3,3)=ins.state.dcm;
    phi.block(xiV(),xiSa(opt),3,3)=ins.state.dcm*ins.dv.asDiagonal();

    phi.block(xiP(),xiV(), 3,3)=Mpv;
    phi.block(xiP(),xiP(), 3,3)=Mpp;

    Matrix3d tauG=Matrix3d::Zero(),tauA=Matrix3d::Zero();
    tauG(0)=tauG(4)=tauG(8)=imup.imu_noise.tau_bg==0.0?0.0:-1.0/imup.imu_noise.tau_bg;
    tauA(0)=tauA(4)=tauA(8)=imup.imu_noise.tau_ba==0.0?0.0:-1.0/imup.imu_noise.tau_ba;
    phi.block(xiBg(),xiBg(),3,3)=tauG;
    phi.block(xiBa(),xiBa(),3,3)=tauA;

    if(opt.est_Sg==+E_SwitchOpt::ON){
        phi.block(xiSg(),xiSg(),3,3)=tauG;
    }
    if(opt.est_Sa==+E_SwitchOpt::ON){
        phi.block(xiSa(opt),xiSa(opt),3,3)=tauA;
    }
}

static void updateF_ECEF1(const imuopt_t &opt,const ins_t &ins,const imuProperty_t& imup,MatrixXd& phi)
{
    /*attitude related*/
    Vector3d omge_ie(0,0,WGS84_OMGE);
    Matrix3d Maa= -Rotation::skewSymmetric(omge_ie);
    Matrix3d MaBg=-ins.state.dcm;
    Matrix3d MaSg=Matrix3d::Zero();
    if(opt.est_Sg){
        MaSg=(ins.state.dcm*ins.da.asDiagonal());
    }

    /*velocity related*/
    Matrix3d Mva=-Rotation::skewSymmetric(ins.state.dcm*ins.fb);
    Matrix3d Mvv=-2*Rotation::skewSymmetric(omge_ie);
    Matrix3d Mvp=Matrix3d::Zero();
    Matrix3d MvBa=-ins.state.dcm;
    Matrix3d MvSa=Matrix3d::Zero();
    if(opt.est_Sa){
        MvSa=(ins.state.dcm*ins.dv.asDiagonal());
    }

    /*position related*/
    Matrix3d Mpv=Matrix3d::Identity();

    phi.block(xiA(),xiA(), 3,3)=Maa;
    phi.block(xiA(),xiBg(),3,3)=MaBg;
//    phi.block(xiSg(),xiSg(),3,3)=MaSg;

    phi.block(xiV(),xiA(), 3,3)=Mva;
    phi.block(xiV(),xiV(), 3,3)=Mvv;
    phi.block(xiV(),xiP(), 3,3)=Mvp;
    phi.block(xiV(),xiBa(),3,3)=MvBa;
//    phi.block(xiV(), xiSa(opt),3,3)=MvSa;

    phi.block(xiP(),xiV(),3,3)=Mpv;

    Matrix3d tauG=Matrix3d::Zero(),tauA=Matrix3d::Zero();
    tauG(0)=tauG(4)=tauG(8)=imup.imu_noise.tau_bg==0.0?0.0:-1.0/imup.imu_noise.tau_bg;
    tauA(0)=tauA(4)=tauA(8)=imup.imu_noise.tau_ba==0.0?0.0:-1.0/imup.imu_noise.tau_ba;
    phi.block(xiBg(),xiBg(),3,3)=tauG;
    phi.block(xiBa(),xiBa(),3,3)=tauA;

    if(opt.est_Sg==+E_SwitchOpt::ON){
        phi.block(xiSg(),xiSg(),3,3)=tauG;
    }
    if(opt.est_Sa==+E_SwitchOpt::ON){
        phi.block(xiSa(opt),xiSa(opt),3,3)=tauA;
    }
}

static void updateF_ECEF(const imuopt_t &opt,const ins_t &ins,const imuProperty_t& imup,MatrixXd& phi){

    /*attitude*/
    jacobiTransA2A_ECEF(phi,ins.dt);
    jacobiTransA2Bg_ECEF(ins.state.dcm,phi,ins.dt);
    if(opt.est_Sg){
        jacobiTransA2Sg_ECEF(opt,ins.state.dcm,ins.da,phi);
    }

    /*velocity*/
    jacobiTransV2A_ECEF(ins.state.dcm,ins.dv,phi);
    jacobiTransV2V_ECEF(ins.dv,phi,ins.dt);
    jacobiTransV2P_ECEF(phi);
    jacobiTransV2Ba_ECEF(ins.state.dcm,phi,ins.dt);
    if(opt.est_Sa){
        jacobiTransV2Sa_ECEF(opt,ins.state.dcm,ins.dv,phi);
    }

    /*position*/
    jacobiTransP2V_ECEF(phi,ins.dt);
    jacobiTransP2P_ECEF(phi,ins.dt);

    /*bias*/
    jacobiTransMarkov(ins.dt,imup.imu_noise.tau_bg,phi,xiBg(),xiBg());
    jacobiTransMarkov(ins.dt,imup.imu_noise.tau_ba,phi,xiBa(),xiBa());
    if(opt.est_Sg){
        jacobiTransMarkov(ins.dt,imup.imu_noise.tau_bg,phi,xiSg(),xiSg());
    }
    if(opt.est_Sa){
        jacobiTransMarkov(ins.dt,imup.imu_noise.tau_ba,phi,xiSa(opt),xiSa(opt));
    }
}

static void updateF_LLH(const imuopt_t &opt,const ins_t &ins,const imuProperty_t& imup,MatrixXd& phi){

    /*attitude*/
    jacobiTransA2A_LLH(ins.earth.w_nin,phi,ins.dt);
    jacobiTransA2V_LLH(opt.att_def,ins,phi,ins.dt);
    jacobiTransA2P_LLH(opt.att_def,ins,phi,ins.dt);
    jacobiTransA2Bg_LLH(ins.state.dcm,ins.dt,phi);
    if(opt.est_Sg){
        jacobiTransA2Sg_LLH(ins.state.dcm,ins.da,phi);
    }

    /*velocity*/
    jacobiTransV2A_LLH(ins.fn,ins.dt,phi);
    jacobiTransV2V_LLH(ins,opt.att_def,ins.dt,phi);
    jacobiTransV2P_LLH(ins,opt.att_def,ins.dt,phi);
    jacobiTransV2Ba_LLH(ins.state.dcm,ins.dt,phi);
    if(opt.est_Sa){
        jacobiTransV2Sa_LLH(opt,ins.state.dcm,ins.dv,phi);
    }

    /*position*/
    jacobiTransP2P_LLH(ins,opt.att_def,ins.dt,phi, nullptr);
    jacobiTransP2V_LLH(ins,opt.att_def,ins.dt,phi);

    /*bias*/
    jacobiTransMarkov(ins.dt,imup.imu_noise.tau_bg,phi,xiBg(),xiBg());
    jacobiTransMarkov(ins.dt,imup.imu_noise.tau_ba,phi,xiBa(),xiBa());
    if(opt.est_Sg){
        jacobiTransMarkov(ins.dt,imup.imu_noise.tau_bg,phi,xiSg(),xiSg());
    }
    if(opt.est_Sa){
        jacobiTransMarkov(ins.dt,imup.imu_noise.tau_ba,phi,xiSa(opt),xiSa(opt));
    }
}

static void transferMatrix1(const imuopt_t opt,const ins_t &ins,const imuProperty_t& imup,MatrixXd& phi)
{
    if(opt.nav_coord==+E_InsNavCoord::ECEF){
        updateF_ECEF1(opt,ins,imup,phi);
    }
    else if(opt.nav_coord==+E_InsNavCoord::LLH){
        updateF_LLH1(opt,ins,imup,opt.att_def,phi);
    }
}

static void transferMatrix(const imuopt_t opt,const ins_t &ins,const imuProperty_t& imup,MatrixXd& phi)
{
    if(opt.nav_coord==+E_InsNavCoord::ECEF){
        updateF_ECEF(opt,ins,imup,phi);
    }
    else if(opt.nav_coord==+E_InsNavCoord::LLH){
        updateF_LLH(opt,ins,imup,phi);
    }
}

static void updateG(const Matrix3d &dcm,E_InsNavCoord nav_coord,MatrixXd& G)
{
    if(nav_coord==+E_InsNavCoord::ECEF){
        G.block(xiV(),xiBg(),3,3)=dcm;
        G.block(xiA(),xiBa(),3,3)=-dcm;
    }
    else if(nav_coord==+E_InsNavCoord::LLH){
        G.block(xiA(),xiBg(),3,3)=-dcm;
        G.block(xiV(),xiBa(),3,3)=dcm;
    }
}

static void updateQ(const Matrix3d &dcm,E_InsNavCoord nav_coord, kf_t& kf,MatrixXd& Q,double dt)
{
    updateG(dcm,nav_coord,kf.G);
    Q=kf.G*kf.Q*kf.G.transpose()*dt;
    Q=(kf.F*Q*kf.F.transpose()+Q)/2.0;
}

static void updateP(const MatrixXd &Q,kf_t *kf,double dt)
{
    double *P_,*Pp_,*F_,*Q_;
    int i,j,k,*ix;

    ix=imat(kf->nx,1);
    for(i=k=0;i<kf->nx;i++) if(kf->x[i]!=0.0&&kf->P.data()[i+i*kf->nx]>0.0) ix[k++]=i;
    P_=mat(k,k);
    Pp_=mat(k,k);
    F_=mat(k,k);
    Q_=mat(k,k);

    for(i=0;i<k;i++){
        for(j=0;j<k;j++) P_[i+j*k]=kf->P.data()[ix[i]+ix[j]*kf->nx];
        for(j=0;j<k;j++) F_[i+j*k]=kf->F.data()[ix[i]+ix[j]*kf->nx];
        for(j=0;j<k;j++) Q_[i+j*k]=Q.data()[ix[i]+ix[j]*kf->nx]*dt;
    }

    matprint(-1,kf->x.data(),kf->nx,1,12,7,"x_:");
    matprint(-1,P_,k,k,20,10,"P_:");
    matprint(-1,F_,k,k,20,10,"F_:");
    matprint(-1,Q_,k,k,20,10,"Q_:");

    double *FP=mat(k,k);
    matmul("NN",k,k,k,1.0,F_,P_,0.0,FP);
    matmul("NT",k,k,k,1.0,FP,F_,0.0,Pp_);

    for(i=0;i<k;i++){
        for(j=0;j<k;j++){
            Pp_[i+j*k]+=Q_[i+j*k];
        }
    }

    for(i=0;i<k;i++){
        for(j=0;j<k;j++) kf->P.data()[ix[i]+ix[j]*kf->nx]=Pp_[i+j*k];
    }

    free(P_);free(Pp_);free(F_);free(Q_);
    free(FP);
}

static void updateP_TC(kf_t *kf,double dt)
{
    double *P_,*Pp_,*F_,*Q_;
    int i,j,k,*ix;

    ix=imat(kf->nx,1);
    for(i=k=0;i<kf->nx;i++) if(kf->x[i]!=0.0&&kf->P.data()[i+i*kf->nx]>0.0) ix[k++]=i;
    P_=mat(k,k);
    Pp_=mat(k,k);
    F_=mat(k,k);
    Q_=mat(k,k);

    for(i=0;i<k;i++){
        for(j=0;j<k;j++) P_[i+j*k]=kf->P.data()[ix[i]+ix[j]*kf->nx];
        for(j=0;j<k;j++) F_[i+j*k]=kf->F.data()[ix[i]+ix[j]*kf->nx];
        for(j=0;j<k;j++) Q_[i+j*k]=kf->Q.data()[ix[i]+ix[j]*kf->nx]*fabs(dt);
    }

    matprint(-1,kf->x.data(),kf->nx,1,12,7,"x_:");
    matprint(-1,P_,k,k,20,10,"P_:");
    matprint(-1,F_,k,k,20,10,"F_:");
    matprint(-1,Q_,k,k,20,10,"Q_:");

    double *FP=mat(k,k);
    matmul("NN",k,k,k,1.0,F_,P_,0.0,FP);
    matmul("NT",k,k,k,1.0,FP,F_,0.0,Pp_);

    for(i=0;i<k;i++){
        for(j=0;j<k;j++){
            Pp_[i+j*k]+=Q_[i+j*k];
        }
    }

    for(i=0;i<k;i++){
        for(j=0;j<k;j++) kf->P.data()[ix[i]+ix[j]*kf->nx]=Pp_[i+j*k];
    }

    free(P_);free(Pp_);free(F_);free(Q_);
    free(FP);
}

static void predicted(fusing_t &fusing)
{
    MatrixXd Q;
//    updateQ(fusing.ins.state.dcm,fusing.opts.imu.nav_coord,fusing.ep_kf,Q,fusing.ins.dt);
    updateP(fusing.ep_kf.Q,&fusing.ep_kf,fabs(fusing.ins.dt));
}

extern void timeUpdate(const imuProperty_t &imup,fusing_t &fusing,bool back)
{
    insUpdate(fusing.opts.imu,fusing.ep_sensors_meas.imu,fusing.ins,back);

    transferMatrix(fusing.opts.imu,fusing.ins,imup,fusing.ep_kf.F);

    predicted(fusing);

    getStateCov(fusing.ep_kf.P,fusing.ins.state);

    state2sol(fusing.opts.imu,fusing,fusing.ins,fusing.ep_sol,imup.week,E_PrcMode::INS);
}
