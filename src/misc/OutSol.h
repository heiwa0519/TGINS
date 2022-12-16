//
// Created by hw on 10/16/22.
//

#ifndef FUSING_OUTSOL_H
#define FUSING_OUTSOL_H

#include "Fusing.h"

extern int outInsState(uint8_t *buff,const char *s,const sol_t *sol, const solopt_t *opt);
extern void state2sol(const imuopt_t opt,const fusing_t &fusing,const ins_t &ins,sol_t &sol,int week,E_PrcMode prc_mode);
extern void writeSol(FILE *fp,const state_t &state,E_InsNavCoord mech_coord,E_AttDefination att_type,bool tc,const sol_t &sol);
extern void writeSol1(FILE *fp,const sol_t& sol);

#endif //FUSING_OUTSOL_H
