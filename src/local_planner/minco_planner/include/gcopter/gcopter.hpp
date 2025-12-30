/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include <Eigen/Eigen>
#include <cfloat>
#include <cmath>
#include <gcopter/geo_utils.hpp>
#include <iostream>
#include <vector>

#include "gcopter/flatness.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"



namespace gcopter {

class GCOPTER_PolytopeSFC {
public:
  typedef Eigen::Matrix3Xd PolyhedronV;
  typedef Eigen::MatrixX4d PolyhedronH;
  typedef std::vector<PolyhedronV> PolyhedraV;
  typedef std::vector<PolyhedronH> PolyhedraH;

private:
  // minco::MINCO_S3NU minco;
  // minco::MINCO_S3NU minco;
  minco::MINCO_S4NU minco;
  minco::MINCO_S3NU yaw_minco;
  flatness::FlatnessMap flatmap;
  flatness::FlatnessMap_Ground flatmap_ground;

  double rho;
  Eigen::Matrix3d headPVA;
  Eigen::Matrix3d tailPVA;

  // Eigen::Matrix4d headPVAJ;
  // Eigen::Matrix4d tailPVAJ;
  Eigen::Matrix<double, 3, 4> headPVAJ;
  Eigen::Matrix<double, 3, 4> tailPVAJ;

  PolyhedraV vPolytopes;
  PolyhedraH hPolytopes;
  Eigen::Matrix3Xd shortPath;

  Eigen::VectorXi pieceIdx;
  Eigen::VectorXi vPolyIdx;
  Eigen::VectorXi hPolyIdx;

  int polyN;
  int pieceN;

  int spatialDim;
  int temporalDim;
  int spatialDim_yaw;
  int temporalDim_yaw;
  double dilate_radius_ = 0.2;
  double smoothEps;
  int integralRes;
  Eigen::VectorXd magnitudeBd;
  Eigen::VectorXd penaltyWt;
  Eigen::VectorXd physicalPm;
  double allocSpeed;
  double minTimeBound;

  // yaw minco
  double yaw_rho_vis;
  double yaw_max_vel;
  double yaw_time_fwd;
  int integralResolution_yaw;
  bool use_shorten_path = false;
  Eigen::Matrix3Xd guide_path;

  lbfgs::lbfgs_parameter_t lbfgs_params;

  Eigen::Matrix3Xd points;
  Eigen::VectorXd times;
  Eigen::Matrix3Xd gradByPoints;
  Eigen::VectorXd gradByTimes;
  Eigen::MatrixX3d partialGradByCoeffs;
  Eigen::VectorXd partialGradByTimes;


  double wheelbase = 0.4;        // 轴距 (m)
  double mass = 10.0;          // 质量 (kg)
  double g = 9.81;               // 重力加速度
  double drag_coeff = 0.3;       // 纵向阻力系数
  double max_steering = 0.5;     // 最大转向角 (rad)
  double max_speed = 2.0;       // 最大速度 (m/s)
  double smooth_factor = 1e-3;   // 数值稳定性参数
  //车辆参数 yjz修改 2025.12.16

private:

  static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T) {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++) {
      T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                          : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau) {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++) {
      tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT,
                                   EIGENVEC &gradTau) {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++) {
      if (tau(i) > 0) {
        gradTau(i) = gradT(i) * (tau(i) + 1.0);
      } else {
        denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
        gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
      }
    }

    return;
  }

  static inline void forwardP(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                              const PolyhedraV &vPolys, Eigen::Matrix3Xd &P) {
    const int sizeP = vIdx.size();
    P.resize(3, sizeP);
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      q = xi.segment(j, k).normalized().head(k - 1);
      P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) + vPolys[l].col(0);
    }
    return;
  }

  static inline double costTinyNLS(void *ptr, const Eigen::VectorXd &xi, Eigen::VectorXd &gradXi) {
    const int n = xi.size();
    const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

    const double sqrNormXi = xi.squaredNorm();
    const double invNormXi = 1.0 / sqrt(sqrNormXi);
    const Eigen::VectorXd unitXi = xi * invNormXi;
    const Eigen::VectorXd r = unitXi.head(n - 1);
    const Eigen::Vector3d delta =
    ovPoly.rightCols(n - 1) * r.cwiseProduct(r) + ovPoly.col(1) - ovPoly.col(0);

    double cost = delta.squaredNorm();
    gradXi.head(n - 1) =
    (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() * r.array() * 2.0;
    gradXi(n - 1) = 0.0;
    gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

    const double sqrNormViolation = sqrNormXi - 1.0;
    if (sqrNormViolation > 0.0) {
      double c = sqrNormViolation * sqrNormViolation;
      const double dc = 3.0 * c;
      c *= sqrNormViolation;
      cost += c;
      gradXi += dc * 2.0 * xi;
    }

    return cost;
  }

  template <typename EIGENVEC>
  static inline void backwardP(const Eigen::Matrix3Xd &P, const Eigen::VectorXi &vIdx,
                               const PolyhedraV &vPolys, EIGENVEC &xi) {
    const int sizeP = P.cols();

    double minSqrD;
    lbfgs::lbfgs_parameter_t tiny_nls_params;
    tiny_nls_params.past = 0;
    tiny_nls_params.delta = 1.0e-5;
    tiny_nls_params.g_epsilon = FLT_EPSILON;
    tiny_nls_params.max_iterations = 128;

    Eigen::Matrix3Xd ovPoly;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();

      ovPoly.resize(3, k + 1);
      ovPoly.col(0) = P.col(i);
      ovPoly.rightCols(k) = vPolys[l];
      Eigen::VectorXd x(k);
      x.setConstant(sqrt(1.0 / k));
      lbfgs::lbfgs_optimize(x, minSqrD, &GCOPTER_PolytopeSFC::costTinyNLS, nullptr, nullptr,
                            &ovPoly, tiny_nls_params);

      xi.segment(j, k) = x;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradP(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                                   const PolyhedraV &vPolys, const Eigen::Matrix3Xd &gradP,
                                   EIGENVEC &gradXi) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double normInv;
    Eigen::VectorXd q, gradQ, unitQ;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      q = xi.segment(j, k);
      normInv = 1.0 / q.norm();
      unitQ = q * normInv;
      gradQ.resize(k);
      gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void normRetrictionLayer(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys, double &cost, EIGENVEC &gradXi) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double sqrNormQ, sqrNormViolation, c, dc;
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k; i < sizeP; i++, j += k) {
      k = vPolys[vIdx(i)].cols();

      q = xi.segment(j, k);
      sqrNormQ = q.squaredNorm();
      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradXi.segment(j, k) += dc * 2.0 * q;
      }
    }

    return;
  }


  static inline bool smoothedL1(const double &x, const double &mu, double &f, double &df) {
    if (x < 0.0) {
      return false;
    } else if (x > mu) {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    } else {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  // 梯度裁剪函数：限制梯度的最大范数
  static inline void clipGradient(Eigen::Vector3d &grad, const double max_norm) {
    double norm = grad.norm();
    if (norm > max_norm && norm > 1e-12) {
      grad *= (max_norm / norm);
    }
  }

  // 标量梯度裁剪
  static inline void clipGradient(double &grad, const double max_norm) {
    if (std::fabs(grad) > max_norm) {
      grad = (grad > 0) ? max_norm : -max_norm;
    }
  }

  // yjz修改 添加梯度裁剪函数 2025.12.25

  inline bool grad_cost_a(const Eigen::Vector3d &a, Eigen::Vector3d &grada, double &costa) {

    double apen = a.squaredNorm() - 2.0 * 2.0;
    if (apen > 0) {
      // grada = yaw_rho_acc * 6 * apen * apen * a;
      // costa = yaw_rho_acc * apen * apen * apen;
      // return true;
      double mu_ = 1.5e-1;
      double smooth_cost, smooth_grad;

      if (smoothedL1(apen, mu_, smooth_cost, smooth_grad)) {
        costa = 100 * smooth_cost;
        grada = 100 * smooth_grad * 2.0 * a; // 链式法则
        return true;
      }
    }
    return false;
  }

  inline bool grad_cost_visibility(const Eigen::Vector3d &p, const Eigen::Vector3d &guide_p,
                                   Eigen::Vector3d &gradp, double &costp) {

    // Eigen::Vector3d diff = p - guide_p;
    // double norm_diff = (p - guide_p).norm();
    // if (norm_diff <= 0) {
    //   return false;
    // }
    // double smooth_cost, smooth_grad;
    // if (smoothedL1(norm_diff, mu_, smooth_cost, smooth_grad)) {
    //   costp = yaw_rho_vis * smooth_cost;
    //   gradp = yaw_rho_vis * smooth_grad * (diff / norm_diff); // 链式法则
    //   // gradp = yaw_rho_vis * smooth_grad * 2 * diff; // 链式法则
    //   cout << "guide_p = " << guide_p.transpose() << endl;
    //   cout << "p = " << p.transpose() << endl;
    //   cout << "cost = " << costp << endl;
    //   cout << endl;
    //   return true;
    // }
    // return false;
    double diff = p.x() - guide_p.x();
    double abs_diff = abs(diff);

    double mu_ = 1.5e-1;
    double smooth_cost, smooth_grad;

    if (smoothedL1(abs_diff, mu_, smooth_cost, smooth_grad)) {
      costp = yaw_rho_vis * smooth_cost;
      gradp.setZero();
      gradp.x() = yaw_rho_vis * smooth_grad * (diff / abs_diff); // 链式法则
      return true;
    }

    return false;
  }

  static inline void attachPenaltyFunctional(
  const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs, const Eigen::VectorXi &hIdx,
  const PolyhedraH &hPolys, const double &smoothFactor, const int &integralResolution,
  const Eigen::VectorXd &magnitudeBounds, const Eigen::VectorXd &penaltyWeights,
  flatness::FlatnessMap &flatMap, double &cost, Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC,
  double &dilate_radius_) {
    const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
    const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
    const double thetaMax = magnitudeBounds(2);
    const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
    const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
    const double thrustSqrRadi = thrustRadi * thrustRadi;

    const double weightPos = penaltyWeights(0);
    const double weightVel = penaltyWeights(1);
    const double weightOmg = penaltyWeights(2);
    const double weightTheta = penaltyWeights(3);
    const double weightThrust = penaltyWeights(4);

    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
    double totalGradPsi, totalGradPsiD;
    double thr, cos_theta;
    Eigen::Vector4d quat;
    Eigen::Vector3d omg;
    double gradThr;
    Eigen::Vector4d gradQuat;
    Eigen::Vector3d gradPos, gradVel, gradOmg;

    double step, alpha;
    // double s1, s2, s3, s4, s5;
    double s1, s2, s3, s4, s5, s6, s7;
    // Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Matrix<double, 8, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy, beta4_xy;

    Eigen::Vector3d outerNormal;
    int K, L;
    double violaPos, violaVel, violaAcc, violaOmg, violaTheta, violaThrust;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaOmgPena, violaThetaPena, violaThrustPena;
    double node, pena;

    const int pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    for (int i = 0; i < pieceNum; i++) {
      // const Eigen::Matrix<double, 8, 3> &c = coeffs.block<8, 3>(i * 8, 0);
      const Eigen::Matrix<double, 8, 3> &c_xy = coeffs.block<8, 3>(i * 8, 0);

      step = T(i) * integralFrac;
      for (int j = 1; j < integralResolution; j++) {
        s1 = j * step;
        // s2 = s1 * s1;
        // s3 = s2 * s1;
        // s4 = s2 * s2;
        // s5 = s4 * s1;
        // beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) =
        // s5; beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4)
        // = 4.0 * s3, beta1(5) = 5.0 * s4; beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3)
        // = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3; beta3(0) = 0.0, beta3(1) = 0.0,
        // beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2; beta4(0) =
        // 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 *
        // s1; pos = c.transpose() * beta0; vel = c.transpose() * beta1; acc = c.transpose() *
        // beta2; jer = c.transpose() * beta3; sna = c.transpose() * beta4; analyse xy
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s4 * s2;
        s7 = s4 * s3;
        beta0_xy << 1.0, s1, s2, s3, s4, s5, s6, s7;
        beta1_xy << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
        beta2_xy << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
        beta3_xy << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
        beta4_xy << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1, 360.0 * s2, 840.0 * s3;
        pos = c_xy.transpose() * beta0_xy;
        vel = c_xy.transpose() * beta1_xy;
        acc = c_xy.transpose() * beta2_xy;
        jer = c_xy.transpose() * beta3_xy;
        sna = c_xy.transpose() * beta4_xy;

        flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

        violaVel = vel.squaredNorm() - velSqrMax;
        violaOmg = omg.squaredNorm() - omgSqrMax;
        violaAcc = acc.squaredNorm() - 3.0;
        cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
        violaTheta = acos(cos_theta) - thetaMax;
        violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

        gradThr = 0.0;
        gradQuat.setZero();
        gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
        pena = 0.0;

        L = hIdx(i);
        K = hPolys[L].rows();
        for (int k = 0; k < K; k++) {
          outerNormal = hPolys[L].block<1, 3>(k, 0);
          violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
          if (smoothedL1(violaPos, smoothFactor, violaPosPena,
                         violaPosPenaD)) // violaPos > 0表示超过pos限制
          {
            gradPos += weightPos * violaPosPenaD * outerNormal;
            pena += weightPos * violaPosPena;
          }
        }

        if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
          gradVel += weightVel * violaVelPenaD * 2.0 * vel;
          pena += weightVel * violaVelPena;
        }



        if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
          gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
          pena += weightOmg * violaOmgPena;
        }

        if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD)) {
          gradQuat += weightTheta * violaThetaPenaD / sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                      Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
          pena += weightTheta * violaThetaPena;
        }

        if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD)) {
          gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
          pena += weightThrust * violaThrustPena;
        }

        flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg, totalGradPos, totalGradVel,
                         totalGradAcc, totalGradJer, totalGradPsi, totalGradPsiD);

        node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
        alpha = j * integralFrac;

        gradC.block<8, 3>(i * 8, 0) +=
        (beta0_xy * totalGradPos.transpose() + beta1_xy * totalGradVel.transpose() +
         beta2_xy * totalGradAcc.transpose() + beta3_xy * totalGradJer.transpose()) *
        node * step;

        gradT(i) += (totalGradPos.dot(vel) + totalGradVel.dot(acc) + totalGradAcc.dot(jer) +
                     totalGradJer.dot(sna)) *
                    alpha * node * step +
                    node * integralFrac * pena;
        cost += node * step * pena;
      }
    }

    return;
  }







  static inline void attachPenaltyFunctional_Ground(
  const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs, const Eigen::VectorXi &hIdx,
  const PolyhedraH &hPolys, const double &smoothFactor, const int &integralResolution,
  const Eigen::VectorXd &magnitudeBounds, const Eigen::VectorXd &penaltyWeights,
  flatness::FlatnessMap_Ground &flatMap, double &cost, Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC,
  double &dilate_radius_) {
    // ========== 数值稳定性参数 ==========
    const double MIN_SAFE_SPEED = 1e-8;         // 最小安全速度，避免除以零
    const double MAX_GRAD_NORM = 1e6;           // 最大梯度范数，超过则裁剪
    const double SPEED_MARGIN = 0.01;           // 速度约束边际
    
    // 地面车辆约束边界
    const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);    // 最大速度平方
    const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);    // 最大角速度平方
    const double deltaMax = magnitudeBounds(2);                         // 最大转向角
    const double curvatureMax = magnitudeBounds(3);                     // 最大曲率
    const double accSqrMax = magnitudeBounds(4) * magnitudeBounds(4);   // 最大加速度平方
    // 地面车辆约束权重
    const double weightPos = penaltyWeights(0);
    const double weightVel = penaltyWeights(1);
    const double weightOmg = penaltyWeights(2);
    const double weightDelta = penaltyWeights(3);     // 转向角约束权重
    const double weightCurvature = penaltyWeights(4); // 曲率约束权重
    const double weightAcc = penaltyWeights(5);       // 加速度约束权重


    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
    double totalGradPsi, totalGradPsiD;
    
    // 地面车辆状态
    double thr;          // 纵向力（推力）
    Eigen::Vector4d quat; // 姿态四元数
    Eigen::Vector3d omg;  // 角速度
    
    // 地面车辆控制梯度
    double gradThr;
    Eigen::Vector4d gradQuat;
    Eigen::Vector3d gradPos, gradVel, gradOmg;

    double step, alpha;
    double s1, s2, s3, s4, s5, s6, s7;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;

    Eigen::Vector3d outerNormal;
    int K, L;
    
    // 地面车辆约束违反
    double violaPos, violaVel, violaOmg, violaDelta, violaCurvature, violaAcc;
    double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaDeltaPenaD, violaCurvaturePenaD, violaAccPenaD;
    double violaPosPena, violaVelPena, violaOmgPena, violaDeltaPena, violaCurvaturePena, violaAccPena;
    double node, pena;

    const int pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    
    // 重置梯度
    gradT.setZero();
    gradC.setZero();
    cost = 0.0;

    for (int i = 0; i < pieceNum; i++) {
      const Eigen::Matrix<double, 8, 3> &c = coeffs.block<8, 3>(i * 8, 0);

      step = T(i) * integralFrac;
      for (int j = 0; j <= integralResolution; j++) {
        s1 = j * step;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s4 * s2;
        s7 = s4 * s3;
        
        // 7次多项式基函数
        beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1, 360.0 * s2, 840.0 * s3;
        
        // 计算状态
        pos = c.transpose() * beta0;   // (x, y, z) - z通常为0或固定高度
        vel = c.transpose() * beta1;   // (vx, vy, vz) - vz通常为0
        acc = c.transpose() * beta2;   // (ax, ay, az) - az通常为0
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;
        
        // 使用地面车辆平坦映射
        // 注意：对于地面车辆，我们通常假设偏航角和偏航角速度为0
        // 因为轨迹规划时主要关注位置，偏航角可以通过速度方向决定
        flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
        
        // ========== 计算约束违反 ==========
        
        // 1. 速度约束：前向速度大小
        double forward_speed = flatMap.getForwardSpeed();
        violaVel = forward_speed * forward_speed - velSqrMax;
        
        // // 2. 角速度约束：只有z轴角速度
        double omega_z = omg(2);
        violaOmg = omega_z * omega_z - omgSqrMax;
        
        // // 3. 加速度约束：切向加速度
        double a_t = thr / flatMap.getVehicleMass() - flatMap.getDragCoeff() * forward_speed / flatMap.getVehicleMass();
        violaAcc = a_t * a_t - accSqrMax;
        
        // // 4. 转向角约束
        double delta = flatMap.getSteeringAngle();
        violaDelta = fabs(delta) - deltaMax;
        
        // // 5. 曲率约束
        double curvature = flatMap.computeCurvature();
        violaCurvature = fabs(curvature) - curvatureMax;
        
        // 6. 位置约束（与多旋翼相同，检查是否在多面体内）
        // 注意：地面车辆的位置约束通常只检查x,y，z固定
        violaPos = 0.0;
        double max_pos_violation = -std::numeric_limits<double>::infinity();
        L = hIdx(i);
        K = hPolys[L].rows();
        
        // 保存最违反的面的法向量用于梯度计算
        Eigen::Vector3d worst_normal = Eigen::Vector3d::Zero();
        
        for (int k = 0; k < K; k++) {
          outerNormal = hPolys[L].block<1, 3>(k, 0);
          double constraint_val = outerNormal.dot(pos) + hPolys[L](k, 3);
          if (constraint_val > max_pos_violation) {
            max_pos_violation = constraint_val;
            worst_normal = outerNormal;
          }
        }
        violaPos = max_pos_violation;
        
        // ========== 计算惩罚项和梯度 ==========
        
        gradThr = 0.0;
        gradQuat.setZero();
        gradPos.setZero();
        gradVel.setZero();
        gradOmg.setZero();
        pena = 0.0;
        
        // 1. 位置约束惩罚
        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
          gradPos += weightPos * violaPosPenaD * worst_normal;
          pena += weightPos * violaPosPena;
        }
        
        // 2. 速度约束惩罚
        if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
          // 前向速度对速度分量的梯度
          // v_forward = sqrt(vx^2 + vy^2)
          // ∂(v²)/∂vx = 2*vx, ∂(v²)/∂vy = 2*vy
          double v_forward = forward_speed;
          double safe_v = std::max(v_forward, 1e-8);  // 数值稳定性保护
          
          // gradVel = ∂penalty/∂vel = ∂penalty/∂violation * ∂violation/∂vel
          //         = violaVelPenaD * 2*vel
          gradVel(0) += weightVel * violaVelPenaD * 2.0 * vel(0);
          gradVel(1) += weightVel * violaVelPenaD * 2.0 * vel(1);
          // z方向速度为0，不考虑梯度
          
          pena += weightVel * violaVelPena;
        }
        
        // 3. 角速度约束惩罚
        if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
          // 只有z轴角速度
          gradOmg(2) += weightOmg * violaOmgPenaD * 2.0 * omega_z;
          pena += weightOmg * violaOmgPena;
        }
        
        // // 4. 加速度约束惩罚
        if (smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {
          // 加速度约束惩罚，主要影响推力
          // a_t = F/m - d*v/m
          // 这里简单地将惩罚加到推力梯度上
          gradThr += weightAcc * violaAccPenaD * 2.0 * a_t / flatMap.getVehicleMass();
          pena += weightAcc * violaAccPena;
        }
        
        // 5. 转向角约束惩罚
        if (smoothedL1(violaDelta, smoothFactor, violaDeltaPena, violaDeltaPenaD)) {
          // 转向角梯度需要传播到速度和角速度
          // δ = atan(ω*L/v)
          // 对δ的梯度传播到ω和v
          double sign_delta = (delta > 0) ? 1.0 : -1.0;
          
          // δ对ω的梯度
          double delta_omega_grad = violaDeltaPenaD * sign_delta;
          // 注意：这里简化处理，实际需要计算∂δ/∂ω
          // ∂δ/∂ω = L/(v*(1 + (ωL/v)^2))
          double v = forward_speed + 1e-6;
          double omega_L_over_v = omega_z * flatMap.getWheelbase() / v;
          double delta_omega_partial = flatMap.getWheelbase() / (v * (1.0 + omega_L_over_v * omega_L_over_v));
          
          gradOmg(2) += weightDelta * delta_omega_grad * delta_omega_partial;
          
          // δ对v的梯度
          double delta_v_grad = violaDeltaPenaD * sign_delta;
          // ∂δ/∂v = -ωL/(v^2*(1 + (ωL/v)^2))
          double delta_v_partial = -omega_z * flatMap.getWheelbase() / (v * v * (1.0 + omega_L_over_v * omega_L_over_v));
          
          // v对速度分量的梯度
          if (v > 1e-6) {
            gradVel(0) += weightDelta * delta_v_grad * delta_v_partial * (vel(0) / v);
            gradVel(1) += weightDelta * delta_v_grad * delta_v_partial * (vel(1) / v);
          }
          
          pena += weightDelta * violaDeltaPena;
        }
        
        // // 6. 曲率约束惩罚
        if (smoothedL1(violaCurvature, smoothFactor, violaCurvaturePena, violaCurvaturePenaD)) {
          // 曲率 κ = ω/v
          // 对κ的梯度传播到ω和v
          double sign_curvature = (curvature > 0) ? 1.0 : -1.0;
          double v = forward_speed + 1e-6;
          
          // κ对ω的梯度：∂κ/∂ω = 1/v
          gradOmg(2) += weightCurvature * violaCurvaturePenaD * sign_curvature * (1.0 / v);
          
          // κ对v的梯度：∂κ/∂v = -ω/v^2
          double kappa_v_partial = -omega_z / (v * v);
          
          // v对速度分量的梯度
          if (v > 1e-6) {
            gradVel(0) += weightCurvature * violaCurvaturePenaD * sign_curvature * kappa_v_partial * (vel(0) / v);
            gradVel(1) += weightCurvature * violaCurvaturePenaD * sign_curvature * kappa_v_partial * (vel(1) / v);
          }
          
          pena += weightCurvature * violaCurvaturePena;
        }
        
        // ========== 梯度反向传播 ==========
        
        // 梯度裁剪以防止梯度爆炸
        // clipGradient(gradPos, MAX_GRAD_NORM);
        // clipGradient(gradVel, MAX_GRAD_NORM);
        // clipGradient(gradOmg, MAX_GRAD_NORM);
        // clipGradient(gradThr, MAX_GRAD_NORM);
        
        // 初始化总梯度
        totalGradPos.setZero();
        totalGradVel.setZero();
        totalGradAcc.setZero();
        totalGradJer.setZero();
        totalGradPsi = 0.0;
        totalGradPsiD = 0.0;
        
        // 使用地面车辆的backward函数传播梯度
        flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg, 
                         totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                         totalGradPsi, totalGradPsiD);
        
        // 在传播后再次裁剪梯度
        // clipGradient(totalGradPos, MAX_GRAD_NORM);
        // clipGradient(totalGradVel, MAX_GRAD_NORM);
        // clipGradient(totalGradAcc, MAX_GRAD_NORM);
        // clipGradient(totalGradJer, MAX_GRAD_NORM);
        
        // ========== 累加到轨迹梯度 ==========
        
        // 梯形积分权重
        node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
        alpha = j * integralFrac;
        
        // 对系数矩阵的梯度
        gradC.block<8, 3>(i * 8, 0) +=
          (beta0 * totalGradPos.transpose() + 
           beta1 * totalGradVel.transpose() + 
           beta2 * totalGradAcc.transpose() + 
           beta3 * totalGradJer.transpose()) *
          node * step;
        
        // 对时间向量的梯度
        gradT(i) += (totalGradPos.dot(vel) + 
                     totalGradVel.dot(acc) + 
                     totalGradAcc.dot(jer) + 
                     totalGradJer.dot(sna)) *
                    alpha * node * step +
                    node * integralFrac * pena;
        
        cost += node * step * pena;
      }
    }
    
    return;
  } //yjz修改 添加地面车辆约束惩罚函数 2025.12.16

  // yjz修改 修复 attachPenaltyFunctional_Ground 中的数值稳定性与梯度裁剪 2025.12.25


  //2025.11.16   今晚发现新问题  A星搜的路径基本在平面但是优化后不在











  inline void PenaltyFunctional_yaw(const Eigen::VectorXd &T, double &cost, Eigen::MatrixXd &gdC_,
                                    Eigen::VectorXd &gdT_) {
    cost = 0.0;
    gdC_.resize(6 * temporalDim_yaw, 3);
    gdC_.setZero();
    gdT_.resize(temporalDim_yaw);
    gdT_.setZero();

    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d grad_tmp, grad_tmp_p, grad_tmp_v;
    double cost_tmp, cost_tmp_p, cost_tmp_v;

    double step, alpha;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg = 1.0;

    const double integralFrac = 1.0 / integralResolution_yaw;
    for (int i = 0; i < temporalDim_yaw; i++) {
      const Eigen::Matrix<double, 6, 3> &c = yaw_minco.getCoeffs().block<6, 3>(i * 6, 0);
      step = T(i) * integralFrac;
      for (int j = 1; j < integralResolution_yaw; j++) {
        s1 = j * step;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2,
        beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2,
        beta2(5) = 20.0 * s3;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1,
        beta3(5) = 60.0 * s2;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0,
        beta4(5) = 120.0 * s1;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        alpha = j * integralFrac;
        Eigen::Vector3d start_pt, end_pt;
        if (i == 0) {
          start_pt = headPVA.col(0);
        } else {
          start_pt = guide_path.col(i - 1);
        }
        if (i == temporalDim_yaw - 1) {
          end_pt = tailPVA.col(0);
        } else {
          end_pt = guide_path.col(i);
        }
        double start_ratio = 1 - j / integralResolution_yaw;
        Eigen::Vector3d guide_pos = start_ratio * start_pt + (1 - start_ratio) * end_pt;
        if (grad_cost_visibility(pos, guide_pos, grad_tmp, cost_tmp)) {
          double node = (j == 0 || j == integralResolution_yaw) ? 0.5 : 1.0;

          gradViolaPc = beta0 * grad_tmp.transpose();
          gradViolaPt = alpha * grad_tmp.dot(vel);
          gdC_.block<6, 3>(i * 6, 0) += node * step * gradViolaPc;
          gdT_(i) += node * (cost_tmp * integralFrac + step * gradViolaPt);
          cost += node * step * cost_tmp;
        }
        // if (grad_cost_a(acc, grad_tmp, cost_tmp)) {
        //   gradViolaAc = beta2 * grad_tmp.transpose();
        //   gradViolaAt = alpha * grad_tmp.dot(jer);
        //   gdC_.block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
        //   gdT_(i) += omg * (cost_tmp * integralFrac + step * gradViolaAt);
        //   cost += omg * step * cost_tmp;
        // }
      }
    }

    return;
  }

  static inline double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
    const int dimTau = obj.temporalDim;
    const int dimXi = obj.spatialDim;
    const double weightT = obj.rho;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

    forwardT(tau, obj.times);
    forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

    double cost;
    obj.minco.setParameters(obj.points, obj.times);
    obj.minco.getEnergy(cost);
    obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

    attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(), obj.hPolyIdx, obj.hPolytopes,
                            obj.smoothEps, obj.integralRes, obj.magnitudeBd, obj.penaltyWt,
                            obj.flatmap, cost, obj.partialGradByTimes, obj.partialGradByCoeffs,
                            obj.dilate_radius_);

    obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes, obj.gradByPoints,
                            obj.gradByTimes);
    double cost_time, grad_time;
    if (smoothedL1(obj.times.sum() - obj.minTimeBound, 1.5e-1, cost_time, grad_time)) {
      cost += weightT * cost_time;
      obj.gradByTimes.array() += weightT * grad_time;
    } else if (smoothedL1(obj.minTimeBound - obj.times.sum(), 1.5e-1, cost_time, grad_time)) {
      cost += weightT * cost_time;
      obj.gradByTimes.array() -= weightT * grad_time;
    }

    backwardGradT(tau, obj.gradByTimes, gradTau);
    backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
    normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

    return cost;
  }





  static inline double costFunctional_ground(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
    const int dimTau = obj.temporalDim;
    const int dimXi = obj.spatialDim;
    const double weightT = obj.rho;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

    forwardT(tau, obj.times);
    forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

    double cost;
    obj.minco.setParameters(obj.points, obj.times);
    obj.minco.getEnergy(cost);
    obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

    attachPenaltyFunctional_Ground(obj.times, obj.minco.getCoeffs(), obj.hPolyIdx, obj.hPolytopes,
                            obj.smoothEps, obj.integralRes, obj.magnitudeBd, obj.penaltyWt,
                            obj.flatmap_ground, cost, obj.partialGradByTimes, obj.partialGradByCoeffs,
                            obj.dilate_radius_);//改用新写的attachPenaltyFunctional_Ground



    obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes, obj.gradByPoints,
                            obj.gradByTimes);
    double cost_time, grad_time;
    if (smoothedL1(obj.times.sum() - obj.minTimeBound, 1.5e-1, cost_time, grad_time)) {
      cost += weightT * cost_time;
      obj.gradByTimes.array() += weightT * grad_time;
    } else if (smoothedL1(obj.minTimeBound - obj.times.sum(), 1.5e-1, cost_time, grad_time)) {
      cost += weightT * cost_time;
      obj.gradByTimes.array() -= weightT * grad_time;
    }

    backwardGradT(tau, obj.gradByTimes, gradTau);
    backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
    normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

    return cost;
  }// yjz修改 添加地面车辆costfunctional 2025.12.16 




  static inline double costFunctional_yaw(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
    const int dimTau = obj.temporalDim_yaw;
    const int dimXi = obj.spatialDim_yaw / 3;
    Eigen::Map<const Eigen::MatrixXd> tau(x.data(), dimTau, 1);
    Eigen::Map<const Eigen::Matrix3Xd> xi(x.data() + dimTau, 3, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::Matrix3Xd> gradXi(g.data() + dimTau, 3, dimXi);
    gradTau.setZero();
    gradXi.setZero();

    // if(obj.use_shorten_path){
    //   cout<<"xi: "<<xi<<endl;
    //   cout<<"guide_path: "<<obj.guide_path<<endl;
    //   }

    Eigen::VectorXd times_yaw;
    forwardT(tau, times_yaw);
    // cout<<"times_yaw: "<<times_yaw<<endl;

    double cost = 0.0;
    Eigen::MatrixX3d gdC_;
    Eigen::VectorXd gdT_;
    obj.yaw_minco.setParameters(xi, times_yaw);
    obj.yaw_minco.getEnergy(cost);
    obj.yaw_minco.getEnergyPartialGradByCoeffs(gdC_);
    obj.yaw_minco.getEnergyPartialGradByTimes(gdT_);

    double cost_constarin = 0.0;
    Eigen::MatrixXd gdC_constrain;
    Eigen::VectorXd gdT_constrain;
    obj.PenaltyFunctional_yaw(times_yaw, cost_constarin, gdC_constrain, gdT_constrain);
    cost += cost_constarin;
    Eigen::MatrixXd gdC_now = gdC_ + gdC_constrain;
    Eigen::VectorXd gdT_now = gdT_ + gdT_constrain;
    Eigen::Matrix3Xd gradP_temp;
    Eigen::VectorXd gradT_temp;
    obj.yaw_minco.propogateGrad(gdC_now, gdT_now, gradP_temp, gradT_temp);

    // cost += obj.yaw_rho_t * times_yaw.sum();
    gradT_temp.setZero();

    obj.backwardGradT(tau, gradT_temp, gradTau);
    gradXi = gradP_temp;
    // cout<<"gradXi: "<<gradXi<<endl;
    return cost;
  }

  static inline double costDistance(void *ptr, const Eigen::VectorXd &xi, Eigen::VectorXd &gradXi) {
    void **dataPtrs = (void **)ptr;
    const double &dEps = *((const double *)(dataPtrs[0]));
    const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
    const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
    const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

    double cost = 0.0;
    const int overlaps = vPolys.size() / 2;

    Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
    Eigen::Vector3d a, b, d;
    Eigen::VectorXd r;
    double smoothedDistance;
    for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k) {
      a = i == 0 ? ini : b;
      if (i < overlaps) {
        k = vPolys[2 * i + 1].cols();
        Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
        r = q.normalized().head(k - 1);
        b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) + vPolys[2 * i + 1].col(0);
      } else {
        b = fin;
      }

      d = b - a;
      smoothedDistance = sqrt(d.squaredNorm() + dEps);
      cost += smoothedDistance;

      if (i < overlaps) {
        gradP.col(i) += d / smoothedDistance;
      }
      if (i > 0) {
        gradP.col(i - 1) -= d / smoothedDistance;
      }
    }

    Eigen::VectorXd unitQ;
    double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
      sqrNormQ = q.squaredNorm();
      invNormQ = 1.0 / sqrt(sqrNormQ);
      unitQ = q * invNormQ;
      gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradQ += dc * 2.0 * q;
      }
    }

    return cost;
  }

  static inline void getShortestPath(const Eigen::Vector3d &ini, const Eigen::Vector3d &fin,
                                     const PolyhedraV &vPolys, const double &smoothD,
                                     Eigen::Matrix3Xd &path) {
    const int overlaps = vPolys.size() / 2;
    Eigen::VectorXi vSizes(overlaps);
    for (int i = 0; i < overlaps; i++) {
      vSizes(i) = vPolys[2 * i + 1].cols();
    }
    Eigen::VectorXd xi(vSizes.sum());
    for (int i = 0, j = 0; i < overlaps; i++) {
      xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
      j += vSizes(i);
    }

    double minDistance;
    void *dataPtrs[4];
    dataPtrs[0] = (void *)(&smoothD);
    dataPtrs[1] = (void *)(&ini);
    dataPtrs[2] = (void *)(&fin);
    dataPtrs[3] = (void *)(&vPolys);
    lbfgs::lbfgs_parameter_t shortest_path_params;
    shortest_path_params.past = 3;
    shortest_path_params.delta = 1.0e-3;
    shortest_path_params.g_epsilon = 1.0e-5;

    lbfgs::lbfgs_optimize(xi, minDistance, &GCOPTER_PolytopeSFC::costDistance, nullptr, nullptr,
                          dataPtrs, shortest_path_params);

    path.resize(3, overlaps + 2);
    path.leftCols<1>() = ini;
    path.rightCols<1>() = fin;
    Eigen::VectorXd r;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      r = q.normalized().head(k - 1);
      path.col(i + 1) =
      vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) + vPolys[2 * i + 1].col(0);
    }

    return;
  }

  static inline bool processCorridor(const PolyhedraH &hPs, PolyhedraV &vPs) {
    const int sizeCorridor = hPs.size() - 1;

    vPs.clear();
    vPs.reserve(2 * sizeCorridor + 1);

    int nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    for (int i = 0; i < sizeCorridor; i++) {
      if (!geo_utils::enumerateVs(hPs[i], curIV)) {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);

      curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
      curIH.topRows(hPs[i].rows()) = hPs[i];
      curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
      if (!geo_utils::enumerateVs(curIH, curIV)) {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);
    }

    if (!geo_utils::enumerateVs(hPs.back(), curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    return true;
  }

  static inline void setInitial(const Eigen::Matrix3Xd &path, const double &speed,
                                const Eigen::VectorXi &intervalNs, Eigen::Matrix3Xd &innerPoints,
                                Eigen::VectorXd &timeAlloc) {
    const int sizeM = intervalNs.size();
    const int sizeN = intervalNs.sum();
    innerPoints.resize(3, sizeN - 1);
    timeAlloc.resize(sizeN);

    Eigen::Vector3d a, b, c;
    for (int i = 0, j = 0, k = 0, l; i < sizeM; i++) {
      l = intervalNs(i);
      a = path.col(i);
      b = path.col(i + 1);
      c = (b - a) / l;
      timeAlloc.segment(j, l).setConstant(c.norm() / speed);
      j += l;
      for (int m = 0; m < l; m++) {
        if (i > 0 || m > 0) {
          innerPoints.col(k++) = a + c * m;
        }
      }
    }
  }

public:
  // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
  // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
  // physicalParams = [vehicle_mass,gravitational_acceleration, horitonral_drag_coeff,
  // vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
  inline bool setup(const double &timeWeight, const double &dilate_radius,
                    const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                    const Eigen::Matrix<double, 3, 4> &terminalPVAJ, const PolyhedraH &safeCorridor,
                    const double &lengthPerPiece, const double &smoothingFactor,
                    const int &integralResolution, const Eigen::VectorXd &magnitudeBounds,
                    const Eigen::VectorXd &penaltyWeights, const Eigen::VectorXd &physicalParams) {
    dilate_radius_ = dilate_radius; // 膨胀半径
    rho = timeWeight;
    headPVAJ = initialPVAJ;
    tailPVAJ = terminalPVAJ;

    hPolytopes = safeCorridor;
    for (size_t i = 0; i < hPolytopes.size(); i++) {
      const Eigen::ArrayXd norms = hPolytopes[i].leftCols<3>().rowwise().norm();
      hPolytopes[i].array().colwise() /= norms;
    } // hPolytope 的每个平面 法向量 归一化
    if (!processCorridor(hPolytopes, vPolytopes)) {
      return false;
    }

    polyN = hPolytopes.size();
    smoothEps = smoothingFactor;
    integralRes = integralResolution;
    magnitudeBd = magnitudeBounds;
    penaltyWt = penaltyWeights;
    physicalPm = physicalParams;
    allocSpeed = magnitudeBd(0) * 3.0;

    getShortestPath(headPVAJ.col(0), tailPVAJ.col(0), vPolytopes, smoothEps, shortPath);
    const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
    pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
    pieceIdx.array() += 1;
    pieceN = pieceIdx.sum();

    temporalDim = pieceN;
    spatialDim = 0;
    vPolyIdx.resize(pieceN - 1);
    hPolyIdx.resize(pieceN);
    for (int i = 0, j = 0, k; i < polyN; i++) {
      k = pieceIdx(i);
      for (int l = 0; l < k; l++, j++) {
        if (l < k - 1) {
          vPolyIdx(j) = 2 * i;
          spatialDim += vPolytopes[2 * i].cols();
        } else if (i < polyN - 1) {
          vPolyIdx(j) = 2 * i + 1;
          spatialDim += vPolytopes[2 * i + 1].cols();
        }
        hPolyIdx(j) = i;
      }
    }

    // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
    minco.setConditions(headPVAJ, tailPVAJ, pieceN);
    flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2), physicalPm(3), physicalPm(4),
                  physicalPm(5));
    flatmap_ground.reset(wheelbase, mass, g, drag_coeff, max_steering, max_speed, smooth_factor);//yjz修改 添加地面车辆flatmap 2025.12.16
    

    // Allocate temp variables
    points.resize(3, pieceN - 1);
    times.resize(pieceN);
    gradByPoints.resize(3, pieceN - 1);
    gradByTimes.resize(pieceN);
    partialGradByCoeffs.resize(8 * pieceN, 3);
    partialGradByTimes.resize(pieceN);

    return true;
  }

  inline bool setup_yaw(const double &rho_vis, const int &integralResolution) {
    integralResolution_yaw = integralResolution;
    yaw_rho_vis = rho_vis;
    return true;
  }

  inline double optimize(Trajectory<7> &traj, const double &relCostTol, const double &time_lb, const std::string& vehicle_type) {
    Eigen::VectorXd x(temporalDim + spatialDim);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

    setInitial(shortPath, allocSpeed, pieceIdx, points, times);
    backwardT(times, tau);
    backwardP(points, vPolyIdx, vPolytopes, xi);

    double minCostFunctional;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-20;
    lbfgs_params.g_epsilon = FLT_EPSILON;
    lbfgs_params.delta = relCostTol;

    minTimeBound = time_lb;
    int ret;

    if(vehicle_type == "drone")
    ret = lbfgs::lbfgs_optimize(x, minCostFunctional, &GCOPTER_PolytopeSFC::costFunctional,
                                    nullptr, nullptr, this, lbfgs_params);
    else if(vehicle_type == "car")
    ret = lbfgs::lbfgs_optimize(x, minCostFunctional, &GCOPTER_PolytopeSFC::costFunctional_ground,
                                    nullptr, nullptr, this, lbfgs_params);//yjz修改 添加地面车辆优化器 2025.12.16


    // if (ret == lbfgs::LBFGSERR_MINIMUMSTEP) {
    //   ROS_ERROR("min-step!!!!");
    //   Eigen::Vector3d start = headPVAJ.col(0);
    //   Eigen::Vector3d end = tailPVAJ.col(0);
    //   if ((start - end).norm() > 0.5) {
    //     minCostFunctional = INFINITY;
    //     std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    //   } else {
    //     Eigen::Vector3d center = (start + end) / 2.0;
    //     points.resize(3, 1);
    //     points.col(0) = center;
    //     times.resize(2);
    //     times[0] = time_lb / 2.0;
    //     times[1] = time_lb / 2.0;
    //     minco.setParameters(points, times);
    //     minco.getTrajectory(traj);
    //   }

    // } else 
    if (ret >= 0) {
      forwardT(tau, times);
      forwardP(xi, vPolyIdx, vPolytopes, points);
      minco.setParameters(points, times);
      minco.getTrajectory(traj);
    } else {
      // traj.clear();
      minCostFunctional = INFINITY;
      std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    }

    return minCostFunctional;
  }

  inline double optimize_yaw(const Eigen::Matrix3d &inityaw, const Eigen::Matrix3d &endyaw,
                             const int &pieceNum, const Eigen::Matrix3Xd &yaw_points,
                             const Eigen::VectorXd &ts, Trajectory<5> &traj) {
    use_shorten_path = false;
    // if(use_shorten_path){
    guide_path.setZero();
    guide_path = yaw_points;
    // cout<<"guide_path: "<<guide_path.cols()<<endl;
    // cout<<"pieceNum: "<<pieceNum<<endl;
    // }

    yaw_minco.setConditions(inityaw, endyaw, pieceNum);
    yaw_minco.setParameters(guide_path, ts);

    Eigen::VectorXd x_yaw, init_tau, yaw_vec;
    yaw_vec = Eigen::Map<const Eigen::VectorXd>(yaw_points.data(), yaw_points.size());
    int yaw_pt_count = yaw_vec.size();
    temporalDim_yaw = pieceNum;
    spatialDim_yaw = yaw_pt_count;
    int opt_dim = pieceNum + yaw_pt_count;
    x_yaw.resize(opt_dim);
    backwardT(ts, init_tau);
    x_yaw.segment(0, pieceNum) = init_tau;
    x_yaw.segment(pieceNum, yaw_pt_count) = yaw_vec;

    double minCostFunctional;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-20;
    lbfgs_params.g_epsilon = 1e-5;
    lbfgs_params.delta = 1e-5;

    // int ret =0;
    int ret =
    lbfgs::lbfgs_optimize(x_yaw, minCostFunctional, &GCOPTER_PolytopeSFC::costFunctional_yaw,
                          nullptr, nullptr, this, lbfgs_params);
    Eigen::VectorXd t = Eigen::Map<Eigen::VectorXd>(x_yaw.data(), pieceNum);
    forwardT(t, t);

    if (ret >= 0) {
      yaw_minco.getTrajectory(traj);
    } else {
      // traj.clear();
      minCostFunctional = INFINITY;
      std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    }

    return minCostFunctional;
  }
};

} // namespace gcopter

#endif
