//
// Created by hw on 10/30/22.
//

#ifndef FUSING_TIGHTCOUPLE_H
#define FUSING_TIGHTCOUPLE_H

#include "Logger.h"
#include "EigenInc.h"
#include "Coordinate.h"
#include "Rotation.h"
#include "Enums.h"
#include "Ins.h"
#include "Gnss.h"
#include "Fusing.h"

extern bool igSensorTc1(fusing_t &fusing);
extern bool igSensorTC(fusing_t &fusing);
#endif //FUSING_TIGHTCOUPLE_H
