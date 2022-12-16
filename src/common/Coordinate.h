//
// Created by hw on 8/27/22.
//

#ifndef FUSING_COORDINATE_H
#define FUSING_COORDINATE_H

#include "EigenInc.h"
#include "Enums.h"


class Coordinate {
public:
    Coordinate() = default;
    ~Coordinate() = default;

public:
    static Vector3d ecef2llh(Vector3d re);
    static Vector3d llh2ecef(Vector3d llh);
    static Vector3d ecef2local(Vector3d llh,Vector3d re,E_AttDefination llh_type);
    static Vector3d local2ecef(Vector3d llh,Vector3d rn,E_AttDefination llh_type);
};


#endif //FUSING_COORDINATE_H
