//
// Created by hw on 8/29/22.
//

#ifndef FUSING_EIGENINC_H
#define FUSING_EIGENINC_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

using Eigen::SimplicialLLT;
using Eigen::SimplicialLDLT;
using Eigen::COLAMDOrdering;
using Eigen::LDLT;
using Eigen::LLT;
using Eigen::PartialPivLU;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::MatrixXi;
using Eigen::SparseMatrix;
using Eigen::SparseVector;
using Eigen::SparseQR;
using Eigen::Map;
using Eigen::Quaterniond;
using Eigen::Triplet;
using Eigen::ArrayXd;
typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;
#define  V30 Eigen::Vector3d::Zero()
#define  M30 Eigen::Matrix3d::Zero()
#endif //FUSING_EIGENINC_H
