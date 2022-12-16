//
// Created by hw on 10/11/22.
//

#ifndef FUSING_REFSOL_H
#define FUSING_REFSOL_H

#include "EigenInc.h"
#include "Logger.h"
#include "Enums.h"
#include "Imu.h"
#include "Gnss.h"
#include "Fusing.h"


extern bool loadRef(string ref_file,E_RefSolFmt fmt,solbuf_t *ref_sol);
extern bool matchRefSol(double sow,int *idx,const track_t *ref_sols,state_t *ref_data,bool back,int hz);
extern int makeSolDiff(const solbuf_t *sol,const solbuf_t *refSol,solbuf_t *errSol);
#endif //FUSING_REFSOL_H
