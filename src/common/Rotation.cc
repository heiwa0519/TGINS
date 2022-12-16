//
// Created by hw on 8/27/22.
//

#include "Rotation.h"

Quaterniond Rotation::rv2quat(Vector3d rv){
    double angle = rv.norm();
    Vector3d vec = rv.normalized();
    return Quaterniond(Eigen::AngleAxisd(angle, vec));
}

Vector3d Rotation::quat2rv(Eigen::Quaterniond quat) {
    Eigen::AngleAxisd axisd(quat);
    return axisd.angle() * axisd.axis();
}

Matrix3d Rotation::quat2dcm(Eigen::Quaterniond quat) {
    return quat.toRotationMatrix();
}

Quaterniond Rotation::dcm2quat(Eigen::Matrix3d dcm) {
    return Quaterniond(dcm);
}

Matrix3d Rotation::att2Cbn(Eigen::Vector3d att, E_AttDefination att_type) {
    Matrix3d Cbn;
    switch (att_type) {
        case E_AttDefination::NED_FRD: /*321: roll pitch yaw*/
            Cbn=Matrix3d(Eigen::AngleAxisd(att[2], Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(att[1], Vector3d::UnitY()) *
                         Eigen::AngleAxisd(att[0], Vector3d::UnitX()));
            break;
        case E_AttDefination::ENU_RFU: /*312: pitch roll yaw*/
            Cbn=Matrix3d(Eigen::AngleAxisd(att[2], Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(att[0], Vector3d::UnitX()) *
                         Eigen::AngleAxisd(att[1], Vector3d::UnitY()));
    }
    return Cbn;
}

Vector3d Rotation::Cbn2att(Eigen::Matrix3d Cbn, E_AttDefination att_type) {
    Vector3d att;

    switch (att_type) {
        case E_AttDefination::NED_FRD:
            att[1]=atan(-Cbn(2,0)/sqrt(Cbn(2,1)*Cbn(2,1)+Cbn(2,2)*Cbn(2,2))); /*roll*/

            if (Cbn(2,0)<=-0.999){
                att[0]=atan2(Cbn(2,1),Cbn(2,2)); /*pitch*/
                att[2]=atan2((Cbn(1,2)-Cbn(0,1)),(Cbn(0,2)+Cbn(1,1))); /*yaw*/
            } else if (Cbn(2,0)>=0.999){
                att[0]=atan2(Cbn(2,1),Cbn(2,2));
                att[2]=M_PI+atan2((Cbn(1,2)+Cbn(0, 1)),(Cbn(0,2)-Cbn(1,1)));
            } else {
                att[0]=atan2(Cbn(2,1),Cbn(2,2));
                att[2]=atan2(Cbn(1,0),Cbn(0,0));
            }
            break;

        case E_AttDefination::ENU_RFU:
            att[0] = asin(Cbn(2,1)); /*pitch*/
            if(fabs(Cbn(2,1))<=0.999){
                att[1]=-atan2(Cbn(2,0),Cbn(2,2)); /*roll*/
                att[2]=-atan2(Cbn(0,1),Cbn(1,1)); /*yaw*/
            }
            else{
                att[1]= atan2(Cbn(0,2),Cbn(0,0)); /*pitch*/
                att[2]=0.0;                                               /*yaw*/
            }
            break;
    }

    // heading 0~2PI
    if (att[2] < 0) {
        att[2] = M_PI * 2 + att[2];
    }

    return att;
}

Matrix3d Rotation::att2Cbe(Eigen::Vector3d att, Eigen::Vector3d llh, E_AttDefination att_type) {
    Matrix3d Cne,Cbn;
    Cne = getCne(llh,att_type);
    Cbn = att2Cbn(att,att_type);
    return Cne*Cbn;
}

Vector3d Rotation::Cbe2att(Eigen::Matrix3d Cbe, Eigen::Vector3d llh, E_AttDefination att_type) {
    Matrix3d Cne,Cbn;
    Cne = getCne(llh,att_type);
    Cbn = Cne.transpose()*Cbe;
    return Cbn2att(Cbn,att_type);
}

Quaterniond Rotation::att2qbn(Eigen::Vector3d att, E_AttDefination att_type) {
    Quaterniond qbn;
    switch (att_type) {
        case E_AttDefination::NED_FRD:
            qbn=Quaterniond(Eigen::AngleAxisd(att[2], Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(att[1], Vector3d::UnitY()) *
                            Eigen::AngleAxisd(att[0], Vector3d::UnitX()));
            break;
        case E_AttDefination::ENU_RFU:
            qbn=Quaterniond(Eigen::AngleAxisd(att[2], Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(att[0], Vector3d::UnitX()) *
                            Eigen::AngleAxisd(att[1], Vector3d::UnitY()));
            break;

    }
    return qbn;
}

Vector3d Rotation::qbn2att(Eigen::Quaterniond quat, E_AttDefination att_type) {
    return Cbn2att(quat.toRotationMatrix(),att_type);
}

Quaterniond Rotation::att2qbe(Eigen::Vector3d att, Eigen::Vector3d llh, E_AttDefination att_type) {
    Matrix3d Cbe= att2Cbe(llh,att,att_type);
    return dcm2quat(Cbe);
}

Vector3d Rotation::qbe2att(Eigen::Quaterniond quat, Eigen::Vector3d llh, E_AttDefination att_type) {
    Matrix3d Cbe= quat2dcm(quat);
    return Cbe2att(Cbe,llh,att_type);
}

Matrix3d Rotation::getCne(Eigen::Vector3d llh, E_AttDefination att_type) {
    Matrix3d Cne=Matrix3d::Zero();
    double sinp=sin(llh[0]),cosp=cos(llh[0]);
    double sinl=sin(llh[1]),cosl=cos(llh[1]);
    switch (att_type) {
        case E_AttDefination::NED_FRD:
            Cne(0,0)=-sinp*cosl; Cne(0,1)=-sinl; Cne(0,2)=-cosp*cosl;
            Cne(1,0)=-sinp*sinl; Cne(1,1)=cosl;  Cne(1,2)=-cosp*sinl;
            Cne(2,0)=cosp;       Cne(2,1)=0.0;   Cne(2,2)=-sinp;
            break;
        case E_AttDefination::ENU_RFU:
            Cne(0,0)=-sinl; Cne(0,1)=-sinp*cosl; Cne(0,2)=cosp*cosl;
            Cne(1,0)=cosl;  Cne(1,1)=-sinp*sinl; Cne(1,2)=cosp*sinl;
            Cne(2,0)=0.0;   Cne(2,1)=cosp;       Cne(2,2)=sinp;
            break;
        default:
            return Cne;
    }
    return Cne;
}

Matrix3d Rotation::skewSymmetric(Eigen::Vector3d vec) {
    Matrix3d mat;
    mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return mat;
}