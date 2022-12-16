//
// Created by hw on 10/16/22.
//

#include "OutSol.h"
#include "Rotation.h"
#include "Coordinate.h"
#include "Fusing.h"
#include "Fbs.h"
#include "jprogress_bar.hpp"

static int cmpState(const void *p1,const void *p2){
    ins_t *q1=(ins_t *)p1,*q2=(ins_t *)p2;
    double tt=q1->state.sow-q2->state.sow;
    return tt<0?-1:1;
}

extern int sortTrack(track_t *track)
{
    if(track->n<=0) return 0;
    qsort(track->data,track->n,sizeof(ins_t), cmpState);
    return 1;
}

static bool matchBwdSol(double sow_solf,int *idx,const track_t *solb)
{
    bool stat=false;
    int i;
    double dt,sow_solb;

    dt=fabs(solb->data[2].state.sow-solb->data[1].state.sow);

    for(i=*idx>5?*idx-3:*idx;i<solb->n-1;i++){
        sow_solb=solb->data[i].state.sow;
        if(fabs(sow_solf-sow_solb)<dt*0.5){
            *idx=i;
            LOG(TRACE)<<setprecision(9)<<"match backward sol, sow_solf: "<<sow_solf<<" sow_solb: "<<sow_solb;
            stat=true;
        }
        else if(sow_solb-sow_solf>1.0){
            break;
        }
    }

    return stat;
}

static Vector3d attSmoother(const Vector3d &pos,const Quaterniond &qf, const Matrix3d &Qf, const Quaterniond &qb,
                    const Matrix3d &Qb, Vector3d &xs, Matrix3d& Qs,E_InsNavCoord nav_coord,E_AttDefination att_def)
{
    Quaterniond qs;
    Vector3d phi;
    phi=Rotation::quat2rv(qf*qb.conjugate());
    Matrix3d Q;
    Qs=Qf+Qb;
    Q=Qf*Qs.inverse();
    qs=Rotation::rv2quat(-Q*phi)*qf;

    if(nav_coord==+E_InsNavCoord::LLH){
        xs=Rotation::qbn2att(qs,att_def);
    }
    else if(nav_coord==+E_InsNavCoord::ECEF){
        Vector3d llh;
        llh=Coordinate::ecef2llh(pos);
        xs=Rotation::qbe2att(qs,llh,att_def);
    }
    Qs=Q*Qb;

    return phi;
}

extern bool fbSmoother(FILE *fp,fusing_t& fusing)
{
    int i,idx=0;

    if(fusing.solb.n==0||fusing.solf.n==0) return false;
    sortTrack(&fusing.solb);

    LOG(DEBUG)<<"start forward-backward solution fusion: solf.n = "<<fusing.solf.n<<" solb.n = "<<fusing.solb.n;
    state_t fbs_data={0},solf_data,solb_data;
    Joger::ProgressBar::JProgressBar bar(fusing.solf.n/10000,"FBS");

    for(i=0;i<fusing.solf.n-1;i++){
        if(matchBwdSol(fusing.solf.data[i].state.sow,&idx,&fusing.solb)){
            fbs_data=solf_data=fusing.solf.data[i].state;
            solb_data=fusing.solb.data[idx].state;

            /*smooth position*/
            if(smoother(solf_data.pos.data(),solf_data.cov.pos.data(),solb_data.pos.data(),solb_data.cov.pos.data(),3,fbs_data.pos.data(),fbs_data.cov.pos.data())){
                continue;
            }

            /*smotth velocity*/
            if(smoother(solf_data.vel.data(),solf_data.cov.vel.data(),solb_data.vel.data(),solb_data.cov.vel.data(),3,fbs_data.vel.data(),fbs_data.cov.vel.data())){
                continue;
            }

            /*smooth attitude*/
            attSmoother(solf_data.pos,solf_data.quat,solf_data.cov.att,solb_data.quat,solb_data.cov.att,fbs_data.att,fbs_data.cov.att,fusing.opts.imu.nav_coord,fusing.opts.imu.att_def);

            /*smooth bg*/
            if(smoother(solf_data.bg.data(),solf_data.cov.bg.data(),solb_data.bg.data(),solb_data.cov.bg.data(),3,fbs_data.bg.data(),fbs_data.cov.bg.data())){
                continue;
            }

            /*smooth ba*/
            if(smoother(solf_data.ba.data(),solf_data.cov.ba.data(),solb_data.ba.data(),solb_data.cov.ba.data(),3,fbs_data.ba.data(),fbs_data.cov.ba.data())){
                continue;
            }

            fusing.ig_ins.state=fbs_data;
            if(fusing.solf.data[i].gstat==1){
                state2sol(fusing.opts.imu,fusing,fusing.ig_ins,fusing.ep_sol,fusing.week,fusing.opts.common.prc_mode);
                outsol(fp,&fusing.ep_sol,fusing.opts.rtklib.prc_opt.rb,&fusing.opts.rtklib.sol_opt,0);
            }
        }
    }
    return true;
}