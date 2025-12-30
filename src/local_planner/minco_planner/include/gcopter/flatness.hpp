/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>

namespace flatness
{
    class FlatnessMap  // See https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf
    {
    public:
        inline void reset(const double &vehicle_mass,
                          const double &gravitational_acceleration,
                          const double &horitonral_drag_coeff,
                          const double &vertical_drag_coeff,
                          const double &parasitic_drag_coeff,
                          const double &speed_smooth_factor)
        {
            mass = vehicle_mass;
            grav = gravitational_acceleration;
            dh = horitonral_drag_coeff;
            dv = vertical_drag_coeff;
            cp = parasitic_drag_coeff;
            veps = speed_smooth_factor;

            return;
        }

        inline void forward(const Eigen::Vector3d &vel,
                            const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jer,
                            const double &psi,
                            const double &dpsi,
                            double &thr,
                            Eigen::Vector4d &quat,
                            Eigen::Vector3d &omg)
        {
            double w0, w1, w2, dw0, dw1, dw2;

            v0 = vel(0);
            v1 = vel(1);
            v2 = vel(2);
            a0 = acc(0);
            a1 = acc(1);
            a2 = acc(2);
            cp_term = sqrt(v0 * v0 + v1 * v1 + v2 * v2 + veps);
            w_term = 1.0 + cp * cp_term;
            w0 = w_term * v0;
            w1 = w_term * v1;
            w2 = w_term * v2;
            dh_over_m = dh / mass;
            zu0 = a0 + dh_over_m * w0;
            zu1 = a1 + dh_over_m * w1;
            zu2 = a2 + dh_over_m * w2 + grav;
            zu_sqr0 = zu0 * zu0;
            zu_sqr1 = zu1 * zu1;
            zu_sqr2 = zu2 * zu2;
            zu01 = zu0 * zu1;
            zu12 = zu1 * zu2;
            zu02 = zu0 * zu2;
            zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
            zu_norm = sqrt(zu_sqr_norm);
            z0 = zu0 / zu_norm;
            z1 = zu1 / zu_norm;
            z2 = zu2 / zu_norm;
            ng_den = zu_sqr_norm * zu_norm;
            ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
            ng01 = -zu01 / ng_den;
            ng02 = -zu02 / ng_den;
            ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
            ng12 = -zu12 / ng_den;
            ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
            v_dot_a = v0 * a0 + v1 * a1 + v2 * a2;
            dw_term = cp * v_dot_a / cp_term;
            dw0 = w_term * a0 + dw_term * v0;
            dw1 = w_term * a1 + dw_term * v1;
            dw2 = w_term * a2 + dw_term * v2;
            dz_term0 = jer(0) + dh_over_m * dw0;
            dz_term1 = jer(1) + dh_over_m * dw1;
            dz_term2 = jer(2) + dh_over_m * dw2;
            dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
            dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
            dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
            f_term0 = mass * a0 + dv * w0;
            f_term1 = mass * a1 + dv * w1;
            f_term2 = mass * (a2 + grav) + dv * w2;
            thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;
            tilt_den = sqrt(2.0 * (1.0 + z2));
            tilt0 = 0.5 * tilt_den;
            tilt1 = -z1 / tilt_den;
            tilt2 = z0 / tilt_den;
            c_half_psi = cos(0.5 * psi);
            s_half_psi = sin(0.5 * psi);
            quat(0) = tilt0 * c_half_psi;
            quat(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
            quat(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
            quat(3) = tilt0 * s_half_psi;
            c_psi = cos(psi);
            s_psi = sin(psi);
            omg_den = z2 + 1.0;
            omg_term = dz2 / omg_den;
            omg(0) = dz0 * s_psi - dz1 * c_psi -
                     (z0 * s_psi - z1 * c_psi) * omg_term;
            omg(1) = dz0 * c_psi + dz1 * s_psi -
                     (z0 * c_psi + z1 * s_psi) * omg_term;
            omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

            return;
        }

        inline void backward(const Eigen::Vector3d &pos_grad,
                             const Eigen::Vector3d &vel_grad,
                             const double &thr_grad,
                             const Eigen::Vector4d &quat_grad,
                             const Eigen::Vector3d &omg_grad,
                             Eigen::Vector3d &pos_total_grad,
                             Eigen::Vector3d &vel_total_grad,
                             Eigen::Vector3d &acc_total_grad,
                             Eigen::Vector3d &jer_total_grad,
                             double &psi_total_grad,
                             double &dpsi_total_grad) const
        {
            double w0b, w1b, w2b, dw0b, dw1b, dw2b;
            double z0b, z1b, z2b, dz0b, dz1b, dz2b;
            double v_sqr_normb, cp_termb, w_termb;
            double zu_sqr_normb, zu_normb, zu0b, zu1b, zu2b;
            double zu_sqr0b, zu_sqr1b, zu_sqr2b, zu01b, zu12b, zu02b;
            double ng00b, ng01b, ng02b, ng11b, ng12b, ng22b, ng_denb;
            double dz_term0b, dz_term1b, dz_term2b, f_term0b, f_term1b, f_term2b;
            double tilt_denb, tilt0b, tilt1b, tilt2b, head0b, head3b;
            double cpsib, spsib, omg_denb, omg_termb;
            double tempb, tilt_den_sqr;

            tilt0b = s_half_psi * (quat_grad(3)) + c_half_psi * (quat_grad(0));
            head3b = tilt0 * (quat_grad(3)) + tilt2 * (quat_grad(1)) - tilt1 * (quat_grad(2));
            tilt2b = c_half_psi * (quat_grad(2)) + s_half_psi * (quat_grad(1));
            head0b = tilt2 * (quat_grad(2)) + tilt1 * (quat_grad(1)) + tilt0 * (quat_grad(0));
            tilt1b = c_half_psi * (quat_grad(1)) - s_half_psi * (quat_grad(2));
            tilt_den_sqr = tilt_den * tilt_den;
            tilt_denb = (z1 * tilt1b - z0 * tilt2b) / tilt_den_sqr + 0.5 * tilt0b;
            omg_termb = -((z0 * c_psi + z1 * s_psi) * (omg_grad(1))) -
                        (z0 * s_psi - z1 * c_psi) * (omg_grad(0));
            tempb = omg_grad(2) / omg_den;
            dpsi_total_grad = omg_grad(2);
            z1b = dz0 * tempb;
            dz0b = z1 * tempb + c_psi * (omg_grad(1)) + s_psi * (omg_grad(0));
            z0b = -(dz1 * tempb);
            dz1b = s_psi * (omg_grad(1)) - z0 * tempb - c_psi * (omg_grad(0));
            omg_denb = -((z1 * dz0 - z0 * dz1) * tempb / omg_den) -
                       dz2 * omg_termb / (omg_den * omg_den);
            tempb = -(omg_term * (omg_grad(1)));
            cpsib = dz0 * (omg_grad(1)) + z0 * tempb;
            spsib = dz1 * (omg_grad(1)) + z1 * tempb;
            z0b += c_psi * tempb;
            z1b += s_psi * tempb;
            tempb = -(omg_term * (omg_grad(0)));
            spsib += dz0 * (omg_grad(0)) + z0 * tempb;
            cpsib += -dz1 * (omg_grad(0)) - z1 * tempb;
            z0b += s_psi * tempb + tilt2b / tilt_den + f_term0 * (thr_grad);
            z1b += -c_psi * tempb - tilt1b / tilt_den + f_term1 * (thr_grad);
            dz2b = omg_termb / omg_den;
            z2b = omg_denb + tilt_denb / tilt_den + f_term2 * (thr_grad);
            psi_total_grad = c_psi * spsib + 0.5 * c_half_psi * head3b -
                             s_psi * cpsib - 0.5 * s_half_psi * head0b;
            f_term0b = z0 * (thr_grad);
            f_term1b = z1 * (thr_grad);
            f_term2b = z2 * (thr_grad);
            ng02b = dz_term0 * dz2b + dz_term2 * dz0b;
            dz_term0b = ng02 * dz2b + ng01 * dz1b + ng00 * dz0b;
            ng12b = dz_term1 * dz2b + dz_term2 * dz1b;
            dz_term1b = ng12 * dz2b + ng11 * dz1b + ng01 * dz0b;
            ng22b = dz_term2 * dz2b;
            dz_term2b = ng22 * dz2b + ng12 * dz1b + ng02 * dz0b;
            ng01b = dz_term0 * dz1b + dz_term1 * dz0b;
            ng11b = dz_term1 * dz1b;
            ng00b = dz_term0 * dz0b;
            jer_total_grad(2) = dz_term2b;
            dw2b = dh_over_m * dz_term2b;
            jer_total_grad(1) = dz_term1b;
            dw1b = dh_over_m * dz_term1b;
            jer_total_grad(0) = dz_term0b;
            dw0b = dh_over_m * dz_term0b;
            tempb = cp * (v2 * dw2b + v1 * dw1b + v0 * dw0b) / cp_term;
            acc_total_grad(2) = mass * f_term2b + w_term * dw2b + v2 * tempb;
            acc_total_grad(1) = mass * f_term1b + w_term * dw1b + v1 * tempb;
            acc_total_grad(0) = mass * f_term0b + w_term * dw0b + v0 * tempb;
            vel_total_grad(2) = dw_term * dw2b + a2 * tempb;
            vel_total_grad(1) = dw_term * dw1b + a1 * tempb;
            vel_total_grad(0) = dw_term * dw0b + a0 * tempb;
            cp_termb = -(v_dot_a * tempb / cp_term);
            tempb = ng22b / ng_den;
            zu_sqr0b = tempb;
            zu_sqr1b = tempb;
            ng_denb = -((zu_sqr0 + zu_sqr1) * tempb / ng_den);
            zu12b = -(ng12b / ng_den);
            tempb = ng11b / ng_den;
            ng_denb += zu12 * ng12b / (ng_den * ng_den) -
                       (zu_sqr0 + zu_sqr2) * tempb / ng_den;
            zu_sqr0b += tempb;
            zu_sqr2b = tempb;
            zu02b = -(ng02b / ng_den);
            zu01b = -(ng01b / ng_den);
            tempb = ng00b / ng_den;
            ng_denb += zu02 * ng02b / (ng_den * ng_den) +
                       zu01 * ng01b / (ng_den * ng_den) -
                       (zu_sqr1 + zu_sqr2) * tempb / ng_den;
            zu_normb = zu_sqr_norm * ng_denb -
                       (zu2 * z2b + zu1 * z1b + zu0 * z0b) / zu_sqr_norm;
            zu_sqr_normb = zu_norm * ng_denb + zu_normb / (2.0 * zu_norm);
            tempb += zu_sqr_normb;
            zu_sqr1b += tempb;
            zu_sqr2b += tempb;
            zu2b = z2b / zu_norm + zu0 * zu02b + zu1 * zu12b + 2 * zu2 * zu_sqr2b;
            w2b = dv * f_term2b + dh_over_m * zu2b;
            zu1b = z1b / zu_norm + zu2 * zu12b + zu0 * zu01b + 2 * zu1 * zu_sqr1b;
            w1b = dv * f_term1b + dh_over_m * zu1b;
            zu_sqr0b += zu_sqr_normb;
            zu0b = z0b / zu_norm + zu2 * zu02b + zu1 * zu01b + 2 * zu0 * zu_sqr0b;
            w0b = dv * f_term0b + dh_over_m * zu0b;
            w_termb = a2 * dw2b + a1 * dw1b + a0 * dw0b +
                      v2 * w2b + v1 * w1b + v0 * w0b;
            acc_total_grad(2) += zu2b;
            acc_total_grad(1) += zu1b;
            acc_total_grad(0) += zu0b;
            cp_termb += cp * w_termb;
            v_sqr_normb = cp_termb / (2.0 * cp_term);
            vel_total_grad(2) += w_term * w2b + 2 * v2 * v_sqr_normb + vel_grad(2);
            vel_total_grad(1) += w_term * w1b + 2 * v1 * v_sqr_normb + vel_grad(1);
            vel_total_grad(0) += w_term * w0b + 2 * v0 * v_sqr_normb + vel_grad(0);
            pos_total_grad(2) = pos_grad(2);
            pos_total_grad(1) = pos_grad(1);
            pos_total_grad(0) = pos_grad(0);

            return;
        }

    private:
        double mass, grav, dh, dv, cp, veps;

        double v0, v1, v2, a0, a1, a2, v_dot_a;
        double z0, z1, z2, dz0, dz1, dz2;
        double cp_term, w_term, dh_over_m;
        double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
        double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
        double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
        double dw_term, dz_term0, dz_term1, dz_term2, f_term0, f_term1, f_term2;
        double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
        double c_psi, s_psi, omg_den, omg_term;
    };


    
class FlatnessMap_Ground
{
public:
    // 重置车辆参数
    inline void reset(const double &wheelbase,
                      const double &mass,
                      const double &gravitational_acceleration,
                      const double &longitudinal_drag_coeff,
                      const double &max_steering_angle,
                      const double &max_speed,
                      const double &speed_smooth_factor)
    {
        L = wheelbase;
        m = mass;
        g = gravitational_acceleration;
        d = longitudinal_drag_coeff;  // 纵向阻力系数
        delta_max = max_steering_angle;
        v_max = max_speed;
        veps = speed_smooth_factor;

        return;
    }

    // Forward函数：从平坦输出计算状态和控制输入
    // 输入：vel(x,y,z), acc(x,y,z), jer(x,y,z), psi(偏航角), dpsi(偏航角速度)
    // 输出：thrust(纵向力), quat(姿态四元数), omg(角速度)
    // 注意：z轴位置有值，但z轴速度和加速度为0
    inline void forward(const Eigen::Vector3d &vel,
                        const Eigen::Vector3d &acc,
                        const Eigen::Vector3d &jer,
                        const double &psi,
                        const double &dpsi,
                        double &thrust,
                        Eigen::Vector4d &quat,
                        Eigen::Vector3d &omg)
    {
        // 保存中间变量用于backward
        vx = vel(0);
        vy = vel(1);
        vz = vel(2);  // z方向速度为0
        ax = acc(0);
        ay = acc(1);
        az = acc(2);  // z方向加速度为0
        jx = jer(0);
        jy = jer(1);
        jz = jer(2);  // z方向jerk为0
        
        psi_val = psi;
        dpsi_val = dpsi;
        
        // 1. 计算前向速度大小
        v_sq = vx * vx + vy * vy;
        v_norm = sqrt(v_sq + veps);
        
        // 2. 计算航向角（从速度方向）
        theta = atan2(vy, vx);
        
        // 3. 计算角速度（从偏航角速度）
        // 对于地面车辆，我们假设绕z轴的旋转就是偏航角速度
        omega_z = dpsi;
        
        // 4. 计算转向角（根据阿克曼几何）
        // δ = atan(ω * L / v)
        if (v_norm > 1e-6) {
            delta = atan2(omega_z * L, v_norm);
        } else {
            delta = 0.0;
        }
        
        // 限制转向角在物理范围内
        if (delta > delta_max) delta = delta_max;
        if (delta < -delta_max) delta = -delta_max;
        
        // 5. 计算前向加速度（切向加速度）
        // a_t = (vx*ax + vy*ay) / v
        a_t = (vx * ax + vy * ay) / (v_norm + veps);
        
        // 6. 计算纵向力（考虑阻力和重力分量）
        // 假设地面是平的，所以没有重力在前进方向的分量
        // F = m*a_t + d*v
        thrust = m * a_t + d * v_norm;
        
        // 7. 构造姿态四元数（只有偏航角）
        // 对于地面车辆，俯仰和横滚为0，只有偏航角
        double half_psi = 0.5 * psi;
        quat(0) = cos(half_psi);  // w
        quat(1) = 0.0;            // x
        quat(2) = 0.0;            // y
        quat(3) = sin(half_psi);  // z
        
        // 8. 角速度向量（只有z轴分量）
        omg(0) = 0.0;
        omg(1) = 0.0;
        omg(2) = omega_z;
        
        // 保存一些中间变量用于backward
        cos_half_psi = cos(half_psi);
        sin_half_psi = sin(half_psi);
        v_norm_eps = v_norm + veps;
        v_sq_eps = v_sq + veps;
        
        return;
    }

    // Backward函数：从状态/控制的梯度反向传播到平坦输出的梯度
    inline void backward(const Eigen::Vector3d &pos_grad,
                         const Eigen::Vector3d &vel_grad,
                         const double &thrust_grad,
                         const Eigen::Vector4d &quat_grad,
                         const Eigen::Vector3d &omg_grad,
                         Eigen::Vector3d &pos_total_grad,
                         Eigen::Vector3d &vel_total_grad,
                         Eigen::Vector3d &acc_total_grad,
                         Eigen::Vector3d &jer_total_grad,
                         double &psi_total_grad,
                         double &dpsi_total_grad) const
    {
        // 初始化所有梯度为0
        double dvx = 0.0, dvy = 0.0, dvz = 0.0;
        double dax = 0.0, day = 0.0, daz = 0.0;
        double djx = 0.0, djy = 0.0, djz = 0.0;
        double dpsi = 0.0, ddpsi = 0.0;
        
        // 1. 位置梯度的传播（直接传递）
        pos_total_grad = pos_grad;
        
        // 2. 从角速度梯度传播到偏航角速度梯度
        // omg(2) = omega_z = dpsi
        ddpsi = omg_grad(2);
        
        // 3. 从四元数梯度传播到偏航角梯度
        // quat = [cos(psi/2), 0, 0, sin(psi/2)]
        // dL/dpsi = dL/dquat_w * (-sin(psi/2)/2) + dL/dquat_z * (cos(psi/2)/2)
        dpsi = quat_grad(0) * (-sin_half_psi * 0.5) + 
               quat_grad(3) * (cos_half_psi * 0.5);
        
        // 4. 从推力梯度传播到速度和加速度梯度
        // thrust = m*a_t + d*v，其中 a_t = (vx*ax + vy*ay)/v
        double dthrust = thrust_grad;
        
        if (dthrust != 0.0) {
            // 计算前向加速度对速度分量的梯度
            // a_t = (vx*ax + vy*ay) / v_norm
            // ∂a_t/∂vx = ax/v - vx*(vx*ax + vy*ay)/v^3
            double v_dot_a = vx * ax + vy * ay;
            double v_norm_sq = v_norm_eps * v_norm_eps;
            double v_norm_cb = v_norm_sq * v_norm_eps;
            
            double da_t_dvx = ax / v_norm_eps - (vx * v_dot_a) / v_norm_cb;
            double da_t_dvy = ay / v_norm_eps - (vy * v_dot_a) / v_norm_cb;
            
            // 计算前向加速度对加速度分量的梯度
            // ∂a_t/∂ax = vx / v_norm
            double da_t_dax = vx / v_norm_eps;
            double da_t_day = vy / v_norm_eps;
            
            // 推力对速度的梯度（来自阻力项）
            // ∂thrust/∂v = d + m * ∂a_t/∂v
            double dthrust_dvx = d * (vx / v_norm_eps) + m * da_t_dvx;
            double dthrust_dvy = d * (vy / v_norm_eps) + m * da_t_dvy;
            
            // 推力对加速度的梯度
            // ∂thrust/∂ax = m * ∂a_t/∂ax
            double dthrust_dax = m * da_t_dax;
            double dthrust_day = m * da_t_day;
            
            // 速度梯度传播（来自推力）
            dvx += dthrust * dthrust_dvx;
            dvy += dthrust * dthrust_dvy;
            
            // 加速度梯度传播（来自推力）
            dax += dthrust * dthrust_dax;
            day += dthrust * dthrust_day;
        }
        
        // 5. 从速度梯度累加（这些是约束直接对速度的梯度）
        dvx += vel_grad(0);
        dvy += vel_grad(1);
        dvz += vel_grad(2);
        
        // 6. jerk梯度直接使用（不通过车辆动力学）
        jer_total_grad = jer_total_grad;  // 保持原值
        
        // 7. 设置输出梯度
        vel_total_grad(0) = dvx;
        vel_total_grad(1) = dvy;
        vel_total_grad(2) = dvz;
        
        acc_total_grad(0) = dax;
        acc_total_grad(1) = day;
        acc_total_grad(2) = daz;
        
        psi_total_grad = dpsi;
        dpsi_total_grad = ddpsi;
        
        return;
    }

    // yjz修改 修复 FlatnessMap_Ground::backward() 中梯度初始化与数值稳定性 2025.12.25
    // 添加一些辅助函数
    
    // 计算曲率
    inline double computeCurvature() const {
        if (v_norm < 1e-6) return 0.0;
        return fabs(omega_z) / v_norm;
    }
    
    // 获取转向角
    inline double getSteeringAngle() const {
        return delta;
    }
    
    // 获取前向速度
    inline double getForwardSpeed() const {
        return v_norm;
    }
    
    // 获取角速度
    inline double getAngularVelocity() const {
        return omega_z;
    }

    inline double getVehicleMass() const {
        return m;
    }

    inline double getDragCoeff() const {
        return d;
    }

    inline double getWheelbase() const {
        return L;
    }

private:
    // 车辆参数
    double L;           // 轴距
    double m;           // 质量
    double g;           // 重力加速度
    double d;           // 纵向阻力系数
    double delta_max;   // 最大转向角
    double v_max;       // 最大速度
    double veps;        // 数值稳定性参数
    
    // 前向计算中的中间变量（用于反向传播）
    double vx, vy, vz;      // 速度分量
    double ax, ay, az;      // 加速度分量
    double jx, jy, jz;      // jerk分量
    double psi_val, dpsi_val; // 偏航角和偏航角速度
    
    double theta;       // 速度方向的航向角
    double delta;       // 转向角
    double v_sq;        // 速度平方
    double v_norm;      // 速度大小
    double a_t;         // 切向加速度
    double omega_z;     // z轴角速度
    
    double cos_half_psi, sin_half_psi;
    double v_norm_eps;  // v_norm + eps
    double v_sq_eps;    // v_sq + eps
};//yjz修改 添加地面车辆平坦化类  2025.12.16
}

#endif
