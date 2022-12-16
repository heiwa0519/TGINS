//
// Created by hw on 8/27/22.
//

#ifndef FUSING_ROTATION_H
#define FUSING_ROTATION_H

#include "Enums.h"
#include "EigenInc.h"

class Rotation {
public:
    Rotation() = default;
    ~Rotation() = default;

public:
    static Quaterniond rv2quat(Vector3d rv);
    static Vector3d quat2rv(Quaterniond quat);
    static Matrix3d quat2dcm(Quaterniond quat);
    static Quaterniond dcm2quat(Matrix3d dcm);
    static Matrix3d att2Cbn(Vector3d att, E_AttDefination att_type);
    static Vector3d Cbn2att(Matrix3d Cbn, E_AttDefination att_type);
    static Matrix3d att2Cbe(Vector3d att,Vector3d llh, E_AttDefination att_type);
    static Vector3d Cbe2att(Matrix3d Cbe, Vector3d llh, E_AttDefination att_type);
    static Quaterniond att2qbn(Vector3d att,E_AttDefination att_type);
    static Vector3d qbn2att(Quaterniond quat,E_AttDefination att_type);
    static Quaterniond att2qbe(Vector3d att,Vector3d llh,E_AttDefination att_type);
    static Vector3d qbe2att(Quaterniond quat,Vector3d llh, E_AttDefination att_type);

    static Matrix3d getCne(Vector3d llh,E_AttDefination att_type);
    static Matrix3d skewSymmetric(Vector3d vec);
};


#endif //FUSING_ROTATION_H
