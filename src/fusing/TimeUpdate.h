//
// Created by hw on 10/30/22.
//

#ifndef FUSING_TIMEUPDATE_H
#define FUSING_TIMEUPDATE_H

#include "Logger.h"
#include "EigenInc.h"
#include "Coordinate.h"
#include "Rotation.h"
#include "Enums.h"
#include "Ins.h"
#include "Gnss.h"
#include "Fusing.h"

extern void timeUpdate(const imuProperty_t &imup,fusing_t &fusing,bool back);

#endif //FUSING_TIMEUPDATE_H
