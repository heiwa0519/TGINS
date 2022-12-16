//
// Created by hw on 10/16/22.
//

#include "Rotation.h"
#include "Coordinate.h"
#include "OutSol.h"

#define SQR(x)     ((x)<0.0?-(x)*(x):(x)*(x))
#define SQRT(x)    ((x)<0.0||(x)!=(x)?0.0:sqrt(x))

extern void writeSol1(FILE *fp,const sol_t& sol)
{
    double sow;
    sow= time2gpst(sol.time, nullptr);

    fprintf(fp,"%4d %8.4f %14.10f %14.10f %14.10f %8.4f %8.4f %8.4f %15.9f %15.9f %15.9f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n",
            2140,
            sow,
            sol.rr[0]*R2D,sol.rr[1]*R2D,sol.rr[2],
            sol.rr[3],sol.rr[4],sol.rr[5],
            sol.rpy[0]*R2D,sol.rpy[1]*R2D,sol.rpy[2]*R2D,
            sol.bg[0]*RPS2DPH,sol.bg[1]*RPS2DPH,sol.bg[2]*RPS2DPH,
            sol.ba[0]*MPS22MG,sol.ba[1]*MPS22MG,sol.ba[2]*MPS22MG);
}

extern void writeSol(FILE *fp,const state_t& state,E_InsNavCoord mech_coord,E_AttDefination att_type,bool tc,const sol_t &sol)
{
    Vector3d rn,vn,rpy,sow;

    if(mech_coord==+E_InsNavCoord::ECEF){
        rn=Coordinate::ecef2llh(state.pos);
        Matrix3d Cne=Rotation::getCne(rn,att_type);
        vn=Cne.transpose()*state.vel;
        if(att_type==+E_AttDefination::ENU_RFU){
            rpy[0]=state.att[1],rpy[1]=state.att[0],rpy[2]=2*PI-state.att[2];
            double tmp=vn[0];
            vn[0]=vn[1];vn[1]=tmp;vn[2]=-vn[2];
        }
        else{
            rpy=state.att;
        }
    }
    else if(mech_coord==+E_InsNavCoord::LLH){
        rn=state.pos,vn=state.vel;
        Vector3d xyz;
        xyz=Coordinate::llh2ecef(state.pos);
        if(att_type==+E_AttDefination::ENU_RFU){
            rpy[0]=state.att[1],rpy[1]=state.att[0],rpy[2]=2*PI-state.att[2];
            vn[0]=state.vel[1],vn[1]=state.vel[0],vn[2]=-state.vel[2];
        }
        else{
            rpy=state.att;
        }
    }

    fprintf(fp,"%4d %8.4f %12.9f %12.9f %9.5f %8.4f %8.4f %8.4f %15.9f %15.9f %15.9f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n",
            2140,
            state.sow,
            rn[0]*R2D,rn[1]*R2D,rn[2],
            vn[0],vn[1],vn[2],
            rpy[0]*R2D,rpy[1]*R2D,rpy[2]*R2D,
            state.bg[0]*RPS2DPH,state.bg[1]*RPS2DPH,state.bg[2]*RPS2DPH,
            state.ba[0]*MPS22MG,state.ba[1]*MPS22MG,state.ba[2]*MPS22MG);
}

extern void state2sol(const imuopt_t opt,const fusing_t &fusing,const ins_t &ins,sol_t &sol,int week,E_PrcMode prc_mode)
{
    Vector3d re,ve,rpy,rn,vn;
    Matrix3d cov_re,cov_ve,cov_rpy;
    sol_t sol0={{0}};
    sol=sol0;

    if(prc_mode!=+E_PrcMode::INS) sol=fusing.rtk.sol;
    if(opt.nav_coord==+E_InsNavCoord::ECEF){
        re=ins.state.pos;
        rn=Coordinate::ecef2llh(ins.state.pos);
        Matrix3d Cne=Rotation::getCne(rn,opt.att_def);
        vn=Cne.transpose()*ins.state.vel;
        ve=ins.state.vel;
        if(opt.att_def==+E_AttDefination::ENU_RFU){
            rpy[0]=ins.state.att[1],rpy[1]=ins.state.att[0],rpy[2]=2*PI-ins.state.att[2];
            double tmp=vn[0];
            vn[0]=vn[1];vn[1]=tmp;vn[2]=-vn[2];
        }
        else{
            rpy=ins.state.att;
        }
        cov_re=ins.state.cov.pos;
        cov_ve=ins.state.cov.vel;
        cov_rpy=ins.state.cov.att;
    }
    else if(opt.nav_coord==+E_InsNavCoord::LLH){
        rn=ins.state.pos,vn=ins.state.vel;
        re=Coordinate::llh2ecef(rn);
        Matrix3d Cne=Rotation::getCne(rn,opt.att_def);
        ve=Cne*vn;
        if(opt.att_def==+E_AttDefination::ENU_RFU){
            rpy[0]=ins.state.att[1],rpy[1]=ins.state.att[0],rpy[2]=2*PI-ins.state.att[2];
            vn[0]=ins.state.vel[1],vn[1]=ins.state.vel[0],vn[2]=-ins.state.vel[2];
        }
        else{
            rpy=ins.state.att;
        }

        cov_re=Cne*(ins.earth.Mpv*ins.state.cov.pos*ins.earth.Mpv.transpose())*Cne.transpose();
        cov_ve=Cne*ins.state.cov.vel*Cne.transpose();
        cov_rpy=ins.state.cov.att;
    }

    sol.stat=prc_mode._value+6;

    sol.gstat=ins.gstat;
    sol.ns=ins.ns;
    sol.time=gpst2time(week,ins.state.sow);

    matcpy(sol.rr,re.data(),3,1);
    for(int i=0;i<3;i++) sol.qr[i]=(float)cov_re(i,i);
    sol.qr[3]=(float)cov_re(1,0);
    sol.qr[4]=(float)cov_re(2,1);
    sol.qr[5]=(float)cov_re(2,0);

    matcpy(sol.rr+3,ve.data(),3,1);
    for(int i=0;i<3;i++) sol.qv[i]=(float)cov_ve(i,i);
    sol.qv[3]=(float)cov_ve(1,0);
    sol.qv[4]=(float)cov_ve(2,1);
    sol.qv[5]=(float)cov_ve(2,0);

    matcpy(sol.rpy,rpy.data(),3,1);
    for(int i=0;i<3;i++) sol.qa[i]=(float)cov_rpy(i,i);
    sol.qa[3]=(float)cov_rpy(1,0);
    sol.qa[4]=(float)cov_rpy(2,1);
    sol.qa[5]=(float)cov_rpy(2,0);

    matcpy(sol.bg,ins.state.bg.data(),3,1);
    matcpy(sol.ba,ins.state.ba.data(),3,1);
    matcpy(sol.sg,ins.state.sg.data(),3,1);
    matcpy(sol.sa,ins.state.sa.data(),3,1);
}

/* sqrt of covariance --------------------------------------------------------*/
static double sqvar(double covar)
{
    return covar<0.0?-sqrt(-covar):sqrt(covar);
}

/* solution option to field separator ----------------------------------------*/
static const char *opt2sep(const solopt_t *opt)
{
    if (!*opt->sep) return " ";
    else if (!strcmp(opt->sep,"\\t")) return "\t";
    return opt->sep;
}

extern int outInsState(uint8_t *buff,const char *s,const sol_t *sol, const solopt_t *opt)
{
    const char *sep=opt2sep(opt);
    char *p=(char *)buff;

    trace(5,"outecef:\n");

    p+=sprintf(p,"%s%3d%s%10.5f%s%10.5f%s%10.5f%s%9.5f%s%8.5f%s%8.5f%s%8.5f%s"
                 "%8.5f%s%8.5f",
               sep,sol->gstat,sep,sol->rpy[0]*R2D,sep,sol->rpy[1]*R2D,sep,sol->rpy[2]*R2D,sep,
               SQRT(sol->qa[0])*R2D,sep,SQRT(sol->qa[1])*R2D,sep,SQRT(sol->qa[2])*R2D,
               sep,sqvar(sol->qa[3])*R2D,sep,sqvar(sol->qa[4])*R2D,sep,
               sqvar(sol->qa[5])*R2D);

    p+= sprintf(p,"%s%8.3f%s%8.3f%s%8.3f",sep,sol->ba[0]*MPS22MG,sep,sol->ba[1]*MPS22MG,sep,sol->ba[2]*MPS22MG);

    p+= sprintf(p,"%s%10.3f%s%10.3f%s%10.3f",sep,sol->bg[0]*RPS2DPH,sep,sol->bg[1]*RPS2DPH,sep,sol->bg[2]*RPS2DPH);

    p+= sprintf(p,"%s%8.3f%s%8.3f%s%8.3f",sep,sol->sg[0]*SCALE2PPM,sep,sol->sg[1]*SCALE2PPM,sep,sol->sg[2]*SCALE2PPM);

    p+= sprintf(p,"%s%8.3f%s%8.3f%s%8.3f",sep,sol->sa[0]*SCALE2PPM,sep,sol->sa[1]*SCALE2PPM,sep,sol->sa[2]*SCALE2PPM);

#if 0
    p+= sprintf(p,"%s%8.3f%s%8.3f%s%8.3f",sep,sol->sa[0]*SCALE2PPM,sep,sol->sa[1]*SCALE2PPM,sep,sol->sa[2]*SCALE2PPM);

    p+= sprintf(p,"%s%8.3f%s%8.3f%s%8.3f",sep,sol->sg[0]*SCALE2PPM,sep,sol->sg[1]*SCALE2PPM,sep,sol->sg[2]*SCALE2PPM);
#endif
    return (int)(p-(char *)buff);
}
