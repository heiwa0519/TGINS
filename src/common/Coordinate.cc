//
// Created by hw on 8/27/22.
//

#include "Coordinate.h"
#include "Constants.h"
#include "Rotation.h"


Vector3d Coordinate::ecef2llh(Eigen::Vector3d re) {
    double e2=WGS84_FE*(2.0-WGS84_FE),r2=re[0]*re[0]+re[1]*re[1],z,zk,v=WGS84_RE,sinp;

    for (z=re[2],zk=0.0;fabs(z-zk)>=1E-4;) {
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=WGS84_RE/sqrt(1.0-e2*sinp*sinp);
        z=re[2]+v*e2*sinp;
    }
    Vector3d llh;
    llh[0]=r2>1E-12?atan(z/sqrt(r2)):(re[2]>0.0?M_PI/2.0:-M_PI/2.0);
    llh[1]=r2>1E-12?atan2(re[1],re[0]):0.0;
    llh[2]=sqrt(r2+z*z)-v;

    return llh;
}

Vector3d Coordinate::llh2ecef(Eigen::Vector3d llh) {
    double sinp=sin(llh[0]),cosp=cos(llh[0]),sinl=sin(llh[1]),cosl=cos(llh[1]);
    double e2=WGS84_FE*(2.0-WGS84_FE),v=WGS84_RE/sqrt(1.0-e2*sinp*sinp);

    Vector3d re;
    re[0]=(v+llh[2])*cosp*cosl;
    re[1]=(v+llh[2])*cosp*sinl;
    re[2]=(v*(1.0-e2)+llh[2])*sinp;

    return re;
}

Vector3d Coordinate::ecef2local(Eigen::Vector3d llh, Eigen::Vector3d re, E_AttDefination llh_type) {
    Matrix3d Cne=Rotation::getCne(llh,llh_type);
    return Cne.transpose()*re;
}

Vector3d Coordinate::local2ecef(Eigen::Vector3d llh, Eigen::Vector3d rn, E_AttDefination llh_type) {
    Matrix3d Cne=Rotation::getCne(llh,llh_type);
    return Cne*rn;
}