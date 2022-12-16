//
// Created by hw on 10/30/22.
//

#ifndef FUSING_SYNCSENSORS_H
#define FUSING_SYNCSENSORS_H

#include "Logger.h"
#include "EigenInc.h"
#include "Coordinate.h"
#include "Rotation.h"
#include "Enums.h"
#include "Ins.h"
#include "Gnss.h"
#include "Fusing.h"

extern bool igSyncPv(double pre_imu,double cur_imu,const pv_t &pvs,int *pv_idx,bool back);
extern bool igSyncObs(double pre_imu,double cur_imu,const obs_t &obs,measIdx_t *idx,bool back);
extern int inputEpGnssObs(const fusing_t &fusing,const obs_t& obs,measIdx_t &idx,vector<obsd_t>& gnss,bool back);

#endif //FUSING_SYNCSENSORS_H
