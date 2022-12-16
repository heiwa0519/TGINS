//
// Created by hw on 10/2/22.
//

#ifndef FUSING_GNSS_H
#define FUSING_GNSS_H

#include "Logger.h"
#include "EigenInc.h"
#include "Enums.h"
#include "rtklib.h"

struct pvData_t{
    double sow;
    int state;
    Vector3d pos,vel;
    Vector3d cov_pos,cov_vel;
};

typedef struct{
    int n,nmax;
    pvData_t *data;
}pv_t;

extern void sol2pvData(sol_t &sol,pvData_t &pv,E_InsNavCoord nav_coord,E_AttDefination att_def);
extern bool loadPVs(string pv_file,E_GnssPvFmt fmt,pv_t *pvs,E_InsNavCoord nav_coord,E_AttDefination att_def);

#endif //FUSING_GNSS_H
