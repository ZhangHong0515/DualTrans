#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>
namespace Eigen
{
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
    typedef Eigen::Matrix<double, 5, 5> Matrix5d;
}
namespace flatness
{
    class PayloadFlatnessMap
    {
    public:
        inline void reset(const double &quad_mass,
                          const double &load_mass,
                          const double &gravitational_acceleration,
                          const double &l_length_,
                          const double &r_length_)
        {
            massQ = quad_mass;
            massL = load_mass;
            grav = gravitational_acceleration;
            l_length = l_length_;
            r_length = r_length_;
        }

        void  ca_qnorm_1(const Eigen::Vector3d &q, const Eigen::Vector3d &qd,
                   const Eigen::Vector3d &qdd, const Eigen::Vector3d &q3d,
                   Eigen::Vector3d &norm, Eigen::Vector3d &qnormd, Eigen::Vector3d &qnormdd, Eigen::Vector3d &qnorm3d)
        {
            q_dot_q_1 = q.squaredNorm();
            q_norm_1 = sqrt(q_dot_q_1);
            // If here need to check if q_norm is zero?
            // if (q_norm < 1e-8)
            // {
            //     norm=Eigen::Vector3d::Zero();
            //     norm.z() = -1.0;
            //     qnormd=Eigen::Vector3d::Zero();
            //     qnormdd = Eigen::Vector3d::Zero();
            //     qnorm3d = Eigen::Vector3d::Zero();
            // }
            assert(q_norm_1 > 1e-8);

            q_norm_3_1 = q_dot_q_1 * q_norm_1;
            q_norm_5_1 = q_dot_q_1 * q_norm_3_1;
        
            q_dot_qd_1 = q.dot(qd);
            q_dot_qdd_1 = q.dot(qdd);
            qd_dot_qd_1 = qd.dot(qd);
            
        
            sqr_q_dot_qd_1 = q_dot_qd_1 * q_dot_qd_1;
            
            norm = q / q_norm_1;

            qnormd = (-q_dot_qd_1 * q + q_dot_q_1 * qd) / q_norm_3_1;

            qnormdd = (3 * (sqr_q_dot_qd_1)*q 
            - q_dot_q_1 * ((q_dot_qdd_1 + qd_dot_qd_1) * q 
            + 2 * q_dot_qd_1 * qd)) / q_norm_5_1 + qdd / q_norm_1;

            qnormdd0_1 = qnormdd[0];
            qnormdd1_1 = qnormdd[1];
            qnormdd2_1 = qnormdd[2];
            // qnorm3d = (-15 * q_dot_qd_3 * q 
            // + 9 * q_dot_q * q_dot_qd * ((qd_dot_qd + q_dot_qdd) * q 
            // + q_dot_qd * qd)) / q_norm_7 
            // - ((q_dot_q3d + 3 * qd_dot_qdd) * q 
            // + 3 * (qd_dot_qd + q_dot_qdd) * qd + 3 * q_dot_qd * qdd) / q_norm_3 
            // + q3d / q_norm;
        }

        void  ca_qnorm_2(const Eigen::Vector3d &q, const Eigen::Vector3d &qd,
                   const Eigen::Vector3d &qdd, const Eigen::Vector3d &q3d,
                   Eigen::Vector3d &norm, Eigen::Vector3d &qnormd, Eigen::Vector3d &qnormdd, Eigen::Vector3d &qnorm3d)
        {
            q_dot_q_2 = q.squaredNorm();
            q_norm_2 = sqrt(q_dot_q_2);
            // If here need to check if q_norm is zero?
            // if (q_norm < 1e-8)
            // {
            //     norm=Eigen::Vector3d::Zero();
            //     norm.z() = -1.0;
            //     qnormd=Eigen::Vector3d::Zero();
            //     qnormdd = Eigen::Vector3d::Zero();
            //     qnorm3d = Eigen::Vector3d::Zero();
            // }
            assert(q_norm_2 > 1e-8);

            q_norm_3_2 = q_dot_q_2 * q_norm_2;
            q_norm_5_2 = q_dot_q_2 * q_norm_3_2;
        
            q_dot_qd_2 = q.dot(qd);
            q_dot_qdd_2 = q.dot(qdd);
            qd_dot_qd_2 = qd.dot(qd);
            
        
            sqr_q_dot_qd_2 = q_dot_qd_2 * q_dot_qd_2;
            
            norm = q / q_norm_2;

            qnormd = (-q_dot_qd_2 * q + q_dot_q_2 * qd) / q_norm_3_2;

            qnormdd = (3 * (sqr_q_dot_qd_2)*q 
            - q_dot_q_2 * ((q_dot_qdd_2 + qd_dot_qd_2) * q 
            + 2 * q_dot_qd_2 * qd)) / q_norm_5_2 + qdd / q_norm_2;

            qnormdd0_2 = qnormdd[0];
            qnormdd1_2 = qnormdd[1];
            qnormdd2_2 = qnormdd[2];
            // qnorm3d = (-15 * q_dot_qd_3 * q 
            // + 9 * q_dot_q * q_dot_qd * ((qd_dot_qd + q_dot_qdd) * q 
            // + q_dot_qd * qd)) / q_norm_7 
            // - ((q_dot_q3d + 3 * qd_dot_qdd) * q 
            // + 3 * (qd_dot_qd + q_dot_qdd) * qd + 3 * q_dot_qd * qdd) / q_norm_3 
            // + q3d / q_norm;
        }
        
        void forward(const Eigen::Vector5d &pos,
                     const Eigen::Vector5d &vel,
                     const Eigen::Vector5d &acc,
                     const Eigen::Vector5d &jerk,
                     const Eigen::Vector5d &snap,
                     const Eigen::Vector5d &crackle,
                     double &thr_1, Eigen::Vector3d &z_1,
                     double &thr_2, Eigen::Vector3d &z_2)
        {
            double theta = pos[3];
            double   phi = pos[4];
            Eigen::Vector3d qr   = Eigen::Vector3d(std::cos(theta)*std::cos(phi), std::cos(theta)*std::sin(phi), std::sin(theta));
            Eigen::Vector3d dqr  = Eigen::Vector3d(-std::sin(theta)*std::cos(phi)*vel[3] - std::cos(theta)*std::sin(phi)*vel[4], 
                                                  -std::sin(theta)*std::sin(phi)*vel[3] + std::cos(theta)*std::cos(phi)*vel[4],
                                                  std::cos(theta)*vel[3]);
            Eigen::Vector3d ddqr = Eigen::Vector3d(-std::cos(theta)*std::cos(phi)*vel[3]*vel[3] + std::sin(theta)*std::sin(phi)*vel[3]*vel[4] - std::sin(theta)*std::cos(phi)*acc[3] + std::sin(theta)*std::sin(phi)*vel[3]*vel[4] -std::cos(theta)*std::cos(phi)*vel[4]*vel[4]-std::cos(theta)*std::sin(phi)*acc[4],
                                                  -std::cos(theta)*std::sin(phi)*vel[3]*vel[3] - std::sin(theta)*std::cos(phi)*vel[3]*vel[4] - std::sin(theta)*std::sin(phi)*acc[3] - std::sin(theta)*std::cos(phi)*vel[3]*vel[4] -std::cos(theta)*std::sin(phi)*vel[4]*vel[4]+std::cos(theta)*std::cos(phi)*acc[4],
                                                  -std::sin(theta)*vel[3]*vel[3] + std::cos(theta)*acc[3]);

            Eigen::Vector3d d3qr = Eigen::Vector3d(std::cos(theta)*std::sin(phi)*vel[4]*vel[4]*vel[4] + std::cos(phi)*std::sin(theta)*vel[3]*vel[3]*vel[3] - std::cos(theta)*std::sin(phi)*jerk[4] - std::cos(phi)*std::sin(theta)*jerk[3] + 3*sin(phi)*sin(theta)*acc[3]*vel[4] + 3*sin(phi)*sin(theta)*vel[3]*acc[4] + 3*cos(phi)*sin(theta)*vel[4]*vel[4]*vel[3] + 3*cos(theta)*sin(phi)*vel[4]*vel[3]*vel[3]-3*cos(phi)*cos(theta)*vel[4]*acc[4] -3*cos(phi)*cos(theta)*vel[3]*acc[3],
                                                   sin(phi)*sin(theta)*pow(vel[3],3) - cos(phi)*cos(theta)*pow(vel[4],3) + cos(phi)*cos(theta)*jerk[4] - sin(phi)*sin(theta)*jerk[3] - 3*cos(phi)*cos(theta)*vel[4]*pow(vel[3],2) + 3*sin(phi)*sin(theta)*pow(vel[4],2)*vel[3] - 3*cos(theta)*sin(phi)*vel[4]*acc[4] - 3*cos(phi)*sin(theta)*vel[4]*acc[3] - 3*cos(phi)*sin(theta)*vel[3]*acc[4] - 3*cos(theta)*sin(phi)*vel[3]*acc[3],
                                                   cos(theta)*jerk[3] - cos(theta)*pow(vel[3],3) - 3*sin(theta)*vel[3]*acc[3]);

            Eigen::Vector3d d4qr = Eigen::Vector3d(cos(phi)*cos(theta)*pow(vel[4],4)+cos(theta)*cos(phi)*pow(vel[3],4)-cos(theta)*sin(phi)*snap[4]-cos(phi)*sin(theta)*snap[3]-3*cos(phi)*cos(theta)*pow(acc[4],2)-3*cos(phi)*cos(theta)*pow(acc[3],2)+6*cos(phi)*cos(theta)*pow(vel[4],2)*pow(vel[3],2)+6*cos(theta)*sin(phi)*pow(vel[4],2)*acc[4]+6*cos(phi)*sin(theta)*pow(vel[4],2)*acc[3]+6*cos(theta)*sin(phi)*pow(vel[3],2)*acc[4]-4*cos(phi)*cos(theta)*vel[4]*jerk[4]+6*cos(phi)*sin(theta)*pow(vel[3],2)*acc[3]-4*cos(phi)*cos(theta)*vel[3]*jerk[3]+4*sin(phi)*sin(theta)*vel[4]*jerk[3]+4*sin(phi)*sin(theta)*vel[3]*jerk[4]+6*sin(phi)*sin(theta)*acc[4]*acc[3]-4*sin(phi)*sin(theta)*pow(vel[4],3)*vel[3]-4*sin(phi)*sin(theta)*vel[4]*pow(vel[3],3)+12*cos(phi)*sin(theta)*vel[4]*vel[3]*acc[4]+12*cos(theta)*sin(phi)*vel[4]*vel[3]*acc[3],
                                                   cos(phi)*cos(theta)*snap[4] + cos(theta)*sin(phi)*pow(vel[4],4) + cos(theta)*sin(phi)*pow(vel[3],4) - sin(phi)*sin(theta)*snap[3] - 3*cos(theta)*sin(phi)*pow(acc[4],2) - 3*cos(theta)*sin(phi)*pow(acc[3],2) + 6*cos(theta)*sin(phi)*pow(vel[4],2)*pow(vel[3],2) - 6*cos(phi)*cos(theta)*pow(vel[4],2)*acc[4] - 6*cos(phi)*cos(theta)*pow(vel[3],2)*acc[4] + 6*sin(phi)*sin(theta)*pow(vel[4],2)*acc[3] - 4*cos(theta)*sin(phi)*vel[4]*jerk[4] + 6*sin(phi)*sin(theta)*pow(vel[3],2)*acc[3] - 4*cos(phi)*sin(theta)*vel[4]*jerk[3] - 4*cos(phi)*sin(theta)*vel[3]*jerk[4] - 6*cos(phi)*sin(theta)*acc[4]*acc[3] - 4*cos(theta)*sin(phi)*vel[3]*jerk[3] + 4*cos(phi)*sin(theta)*pow(vel[4],3)*vel[3] + 4*cos(phi)*sin(theta)*vel[4]*pow(vel[3],3) - 12*cos(phi)*cos(theta)*vel[4]*vel[3]*acc[3] + 12*sin(phi)*sin(theta)*vel[4]*vel[3]*acc[4],
                                                   cos(theta)*snap[3] - 3*sin(theta)*pow(acc[3],2) + sin(theta)*pow(vel[3],4) - 6*cos(theta)*pow(vel[3],2)*acc[3] - 4*sin(theta)*vel[3]*jerk[3]);

            Eigen::Vector3d d5qr = Eigen::Vector3d(10*cos(phi)*cos(theta)*pow(vel[4],3)*acc[4] - cos(phi)*sin(theta)*pow(vel[3],5) - cos(theta)*sin(phi)*crackle[4] - cos(phi)*sin(theta)*crackle[3] - 10*cos(phi)*sin(theta)*pow(vel[4],2)*pow(vel[3],3) - 10*cos(theta)*sin(phi)*pow(vel[4],3)*pow(vel[3],2) - cos(theta)*sin(phi)*pow(vel[4],5) + 10*cos(phi)*cos(theta)*pow(vel[3],3)*acc[3] + 15*cos(theta)*sin(phi)*vel[4]*pow(acc[4],2) + 15*cos(phi)*sin(theta)*vel[3]*pow(acc[4],2) + 15*cos(theta)*sin(phi)*vel[4]*pow(acc[3],2) + 15*cos(phi)*sin(theta)*vel[3]*pow(acc[3],2) - 10*sin(phi)*sin(theta)*pow(vel[4],3)*acc[3] - 10*sin(phi)*sin(theta)*pow(vel[3],3)*acc[4] + 10*cos(theta)*sin(phi)*pow(vel[4],2)*jerk[4] + 10*cos(phi)*sin(theta)*pow(vel[4],2)*jerk[3] + 10*cos(theta)*sin(phi)*pow(vel[3],2)*jerk[4] - 5*cos(phi)*cos(theta)*vel[4]*snap[4] - 10*cos(phi)*cos(theta)*acc[4]*jerk[4] + 10*cos(phi)*sin(theta)*pow(vel[3],2)*jerk[3] - 5*cos(phi)*cos(theta)*vel[3]*snap[3] - 10*cos(phi)*cos(theta)*acc[3]*jerk[3] - 5*cos(phi)*sin(theta)*pow(vel[4],4)*vel[3] - 5*cos(theta)*sin(phi)*vel[4]*pow(vel[3],4) + 5*sin(phi)*sin(theta)*vel[4]*snap[3] + 5*sin(phi)*sin(theta)*vel[3]*snap[4] + 10*sin(phi)*sin(theta)*acc[4]*jerk[3] + 10*sin(phi)*sin(theta)*acc[3]*jerk[4] + 30*cos(phi)*cos(theta)*vel[4]*pow(vel[3],2)*acc[4] + 30*cos(phi)*cos(theta)*pow(vel[4],2)*vel[3]*acc[3] - 30*sin(phi)*sin(theta)*pow(vel[4],2)*vel[3]*acc[4] - 30*sin(phi)*sin(theta)*vel[4]*pow(vel[3],2)*acc[3] + 20*cos(phi)*sin(theta)*vel[4]*vel[3]*jerk[4] + 30*cos(phi)*sin(theta)*vel[4]*acc[4]*acc[3] + 20*cos(theta)*sin(phi)*vel[4]*vel[3]*jerk[3] + 30*cos(theta)*sin(phi)*vel[3]*acc[4]*acc[3],
                                                   cos(phi)*cos(theta)*pow(vel[4],5) - sin(phi)*sin(theta)*pow(vel[3],5) + cos(phi)*cos(theta)*crackle[4] - sin(phi)*sin(theta)*crackle[3] + 10*cos(phi)*cos(theta)*pow(vel[3],3)*pow(vel[3],2) - 15*cos(phi)*cos(theta)*vel[4]*pow(acc[4],2) - 15*cos(phi)*cos(theta)*vel[4]*pow(acc[3],2) - 10*sin(phi)*sin(theta)*pow(vel[4],2)*pow(vel[3],3) + 10*cos(theta)*sin(phi)*pow(vel[3],3)*acc[4] + 10*cos(phi)*sin(theta)*pow(vel[3],3)*acc[3] + 10*cos(phi)*sin(theta)*pow(vel[3],3)*acc[4] + 10*cos(theta)*sin(phi)*pow(vel[3],3)*acc[3] + 15*sin(phi)*sin(theta)*vel[3]*pow(acc[4],2) + 15*sin(phi)*sin(theta)*vel[3]*pow(acc[3],2) - 10*cos(phi)*cos(theta)*pow(vel[4],2)*jerk[4] - 10*cos(phi)*cos(theta)*pow(vel[3],2)*jerk[4] + 5*cos(phi)*cos(theta)*vel[4]*pow(vel[4],4) + 10*sin(phi)*sin(theta)*pow(vel[4],2)*jerk[3] - 5*cos(theta)*sin(phi)*vel[4]*snap[4] - 10*cos(theta)*sin(phi)*acc[4]*jerk[4] + 10*sin(phi)*sin(theta)*pow(vel[3],2)*jerk[3] - 5*cos(phi)*sin(theta)*vel[4]*snap[3] - 5*cos(phi)*sin(theta)*vel[3]*snap[4] - 10*cos(phi)*sin(theta)*acc[4]*jerk[3] - 10*cos(phi)*sin(theta)*acc[3]*jerk[4] - 5*cos(theta)*sin(phi)*vel[3]*snap[3] - 10*cos(theta)*sin(phi)*acc[3]*jerk[3] - 5*sin(phi)*sin(theta)*pow(vel[4],4)*vel[3] + 30*cos(phi)*sin(theta)*pow(vel[4],2)*vel[3]*acc[4] + 30*cos(theta)*sin(phi)*vel[4]*pow(vel[3],2)*acc[4] + 30*cos(phi)*sin(theta)*vel[4]*pow(vel[3],2)*acc[3] + 30*cos(theta)*sin(phi)*pow(vel[4],2)*vel[3]*acc[3] - 20*cos(phi)*cos(theta)*vel[4]*vel[3]*jerk[3] - 30*cos(phi)*cos(theta)*vel[3]*acc[4]*acc[3] + 20*sin(phi)*sin(theta)*vel[4]*vel[3]*jerk[4] + 30*sin(phi)*sin(theta)*vel[4]*acc[4]*acc[3],
                                                   cos(theta)*crackle[3] + cos(theta)*pow(vel[3],5) - 15*cos(theta)*vel[3]*pow(acc[3],2) + 10*sin(theta)*pow(vel[3],3)*acc[3] - 10*cos(theta)*pow(vel[3],2)*jerk[3] - 5*sin(theta)*vel[3]*snap[3] - 10*sin(theta)*acc[3]*jerk[3]);

            Eigen::Vector3d attach_pos_1 = pos.head<3>() - r_length * qr;
            Eigen::Vector3d attach_vel_1 = vel.head<3>() - r_length * dqr;
            Eigen::Vector3d attach_acc_1 = acc.head<3>() - r_length * ddqr;
            Eigen::Vector3d attach_jerk_1 = jerk.head<3>() - r_length * d3qr;
            Eigen::Vector3d attach_snap_1 = snap.head<3>() - r_length * d4qr;
            Eigen::Vector3d attach_crackle_1 = crackle.head<3>() - r_length * d5qr;
            Eigen::Vector3d attach_pos_2 = pos.head<3>() + r_length * qr;
            Eigen::Vector3d attach_vel_2 = vel.head<3>() + r_length * dqr;
            Eigen::Vector3d attach_acc_2 = acc.head<3>() + r_length * ddqr;
            Eigen::Vector3d attach_jerk_2 = jerk.head<3>() + r_length * d3qr;
            Eigen::Vector3d attach_snap_2 = snap.head<3>() + r_length * d4qr;
            Eigen::Vector3d attach_crackle_2 = crackle.head<3>() + r_length * d5qr;
            
            q0_1 = attach_acc_1[0];
            q1_1 = attach_acc_1[1];
            q2_1 = attach_acc_1[2] + grav;
            qd0_1 = attach_jerk_1[0];
            qd1_1 = attach_jerk_1[1];
            qd2_1 = attach_jerk_1[2];
            qdd0_1 = attach_snap_1[0];
            qdd1_1 = attach_snap_1[1];
            qdd2_1 = attach_snap_1[2];

            Eigen::Vector3d Tp_1 = -attach_acc_1; ////here is -Tp
            Tp_1(2) -= grav;
            Eigen::Vector3d qnorm_1, qnormd_1, qnormdd_1, qnorm3d_1; 
            Eigen::Vector3d qnorm_2, qnormd_2, qnormdd_2, qnorm3d_2;
            ca_qnorm_1(Tp_1, -attach_jerk_1, -attach_snap_1, -attach_crackle_1, qnorm_1, qnormd_1, qnormdd_1, qnorm3d_1);
           
            Eigen::Vector3d Tp_2 = -attach_acc_2;
            Tp_2(2) -= grav;
            ca_qnorm_2(Tp_2, -attach_jerk_2, -attach_snap_2, -attach_crackle_2, qnorm_2, qnormd_2, qnormdd_2, qnorm3d_2);

            
            zu0_1 = (massQ + 0.5 * massL) * attach_acc_1[0] + massQ * l_length * qnormdd_1[0];
            zu1_1 = (massQ + 0.5 * massL) * attach_acc_1[1] + massQ * l_length * qnormdd_1[1];
            zu2_1 = (massQ + 0.5 * massL) * (attach_acc_1[2] + grav) + massQ * l_length * qnormdd_1[2];

            zu_sqr_norm_1 = zu0_1 * zu0_1 + zu1_1 * zu1_1 + zu2_1 * zu2_1;
            zu_norm_1 = sqrt(zu_sqr_norm_1);

            z0_1 = zu0_1 / zu_norm_1;
            z1_1 = zu1_1 / zu_norm_1;
            z2_1 = zu2_1 / zu_norm_1;

            z_1(0) = z0_1;
            z_1(1) = z1_1;
            z_1(2) = z2_1;
            thr_1 = zu_norm_1;

            zu0_2 = (massQ + 0.5 * massL) * attach_acc_2[0] + massQ * l_length * qnormdd_2[0];
            zu1_2 = (massQ + 0.5 * massL) * attach_acc_2[1] + massQ * l_length * qnormdd_2[1];
            zu2_2 = (massQ + 0.5 * massL) * (attach_acc_2[2] + grav) + massQ * l_length * qnormdd_2[2];

            zu_sqr_norm_2 = zu0_2 * zu0_2 + zu1_2 * zu1_2 + zu2_2 * zu2_2;
            zu_norm_2 = sqrt(zu_sqr_norm_2);

            z0_2 = zu0_2 / zu_norm_2;
            z1_2 = zu1_2 / zu_norm_2;
            z2_2 = zu2_2 / zu_norm_2;

            z_2(0) = z0_2;
            z_2(1) = z1_2;
            z_2(2) = z2_2;
            thr_2 = zu_norm_2;
            return;
        }

        void backward(const double &thr_grad_1,
                             const Eigen::Vector3d &z_grad_1,
                             Eigen::Vector5d &load_acc_grad_1,
                             Eigen::Vector5d &load_jer_grad_1,
                             Eigen::Vector5d &load_snap_grad_1,
                     const double &thr_grad_2,
                             const Eigen::Vector3d &z_grad_2,
                             Eigen::Vector5d &load_acc_grad_2,
                             Eigen::Vector5d &load_jer_grad_2,
                             Eigen::Vector5d &load_snap_grad_2) const
        {
            double z0b = z_grad_1(0);
            double z1b = z_grad_1(1);
            double z2b = z_grad_1(2);

            double q0b = 0.0;
            double q1b = 0.0;
            double q2b = 0.0;
            double qd0b = 0.0;
            double qd1b = 0.0;
            double qd2b = 0.0;
            double qdd0b = 0.0;
            double qdd1b = 0.0;
            double qdd2b = 0.0;
            double q_dot_qb = 0.0;
            double q_norm_3b = 0.0;
            double q_norm_5b = 0.0;
            double q_dot_qdb = 0.0;
            double q_dot_qddb = 0.0;
            double qd_dot_qdb = 0.0;
            double sqr_q_dot_qdb = 0.0;
            double qnormdd0b = 0.0;
            double qnormdd1b = 0.0;
            double qnormdd2b = 0.0;
            double zu0b = 0.0;
            double zu1b = 0.0;
            double zu2b = 0.0;
            double zu_sqr_normb = 0.0;
            double zu_normb = thr_grad_1;
            double temp;
            double tempb;
            double temp0;
            double tempb0;
            double q_normb;
            zu_sqr_normb = (zu_sqr_norm_1 == 0.0 ? 0.0 : zu_normb / (2.0 * sqrt(zu_sqr_norm_1)));
            zu2b = z2b / zu_norm_1 + 2 * zu2_1 * zu_sqr_normb;
            zu_normb = -(zu2_1 * (z2b) / (zu_norm_1 * zu_norm_1));
            zu1b = z1b / zu_norm_1 + 2 * zu1_1 * zu_sqr_normb;
            zu_normb = -(zu1_1 * (z1b) / (zu_norm_1 * zu_norm_1));
            zu0b = z0b / zu_norm_1 + 2 * zu0_1 * zu_sqr_normb;
            zu_normb = -(zu0_1 * (z0b) / (zu_norm_1 * zu_norm_1));
            qnormdd2b = massQ * l_length * zu2b;
            qnormdd1b = massQ * l_length * zu1b;
            qnormdd0b = massQ * l_length * zu0b;
            temp = (q_dot_qdd_1 + qd_dot_qd_1) * q2_1 + 2 * q_dot_qd_1 * qd2_1;
            tempb0 = qnormdd2b / q_norm_5_1;
            sqr_q_dot_qdb = 3 * q2_1 * tempb0;
            q_dot_qb = -(temp * tempb0);
            tempb = -(q_dot_q_1 * tempb0);
            q2b = 3 * sqr_q_dot_qd_1 * tempb0 + (q_dot_qdd_1 + qd_dot_qd_1) * tempb;
            q_norm_5b = -((3 * sqr_q_dot_qd_1 * q2_1 - q_dot_q_1 * temp) * tempb0 / q_norm_5_1);
            q_dot_qddb = q2_1 * tempb;
            qd_dot_qdb = q2_1 * tempb;
            q_dot_qdb = 2 * qd2_1 * tempb;
            qd2b = 2 * q_dot_qd_1 * tempb;
            temp = (q_dot_qdd_1 + qd_dot_qd_1) * q1_1 + 2 * q_dot_qd_1 * qd1_1;
            tempb0 = qnormdd1b / q_norm_5_1;
            tempb = -(q_dot_q_1 * tempb0);
            q1b = 3 * sqr_q_dot_qd_1 * tempb0 + (q_dot_qdd_1 + qd_dot_qd_1) * tempb;
            q_dot_qddb = q_dot_qddb + q1_1 * tempb;
            qd_dot_qdb = qd_dot_qdb + q1_1 * tempb;
            q_dot_qdb = q_dot_qdb + 2 * qd1_1 * tempb;
            qd1b = 2 * q_dot_qd_1 * tempb;
            temp0 = (q_dot_qdd_1 + qd_dot_qd_1) * q0_1 + 2 * q_dot_qd_1 * qd0_1;
            tempb = qnormdd0b / q_norm_5_1;
            sqr_q_dot_qdb = sqr_q_dot_qdb + 3 * q1_1 * tempb0 + 3 * q0_1 * tempb;
            q_norm_5b = q_norm_5b - (3 * sqr_q_dot_qd_1 * q1_1 - q_dot_q_1 * temp) * tempb0 / q_norm_5_1 -
                        (3 * sqr_q_dot_qd_1 * q0_1 - q_dot_q_1 * temp0) * tempb / q_norm_5_1;
            q_norm_3b = q_dot_q_1 * q_norm_5b;
            q_normb = q_dot_q_1 * q_norm_3b - qdd2_1 * qnormdd2b / (q_norm_1 * q_norm_1) - qdd1_1 * qnormdd1b / (q_norm_1 * q_norm_1) - qdd0_1 * qnormdd0b / (q_norm_1 * q_norm_1);
            if (q_dot_q_1 == 0.0)
                q_dot_qb = q_dot_qb + q_norm_3_1 * q_norm_5b - temp * tempb0 - temp0 * tempb +
                           q_norm_1 * q_norm_3b;
            else
                q_dot_qb = q_dot_qb + q_norm_3_1 * q_norm_5b - temp * tempb0 - temp0 * tempb +
                           q_norm_1 * q_norm_3b + q_normb / (2.0 * sqrt(q_dot_q_1));
            tempb0 = -(q_dot_q_1 * tempb);
            q_dot_qddb = q_dot_qddb + q0_1 * tempb0;
            qdd2b = qnormdd2b / q_norm_1 + q2_1 * q_dot_qddb;
            qdd1b = qnormdd1b / q_norm_1 + q1_1 * q_dot_qddb;
            qdd0b = qnormdd0b / q_norm_1 + q0_1 * q_dot_qddb;
            qd_dot_qdb = qd_dot_qdb + q0_1 * tempb0;
            q_dot_qdb = q_dot_qdb + 2 * qd0_1 * tempb0 + 2 * q_dot_qd_1 * sqr_q_dot_qdb;
            q0b = 3 * sqr_q_dot_qd_1 * tempb + (q_dot_qdd_1 + qd_dot_qd_1) * tempb0 + qdd0_1 * q_dot_qddb + qd0_1 * q_dot_qdb + 2 * q0_1 * q_dot_qb;
            double load_acc0b = (massQ + 0.5* massL) * zu0b + q0b;
            qd0b = 2 * q_dot_qd_1 * tempb0 + 2 * qd0_1 * qd_dot_qdb + q0_1 * q_dot_qdb;
            qd1b = qd1b + 2 * qd1_1 * qd_dot_qdb + q1_1 * q_dot_qdb;
            qd2b = qd2b + 2 * qd2_1 * qd_dot_qdb + q2_1 * q_dot_qdb;
            q1b = q1b + qdd1_1 * q_dot_qddb + qd1_1 * q_dot_qdb + 2 * q1_1 * q_dot_qb;
            double load_acc1b = (massQ + 0.5 * massL) * zu1b + q1b;
            q2b = q2b + qdd2_1 * q_dot_qddb + qd2_1 * q_dot_qdb + 2 * q2_1 * q_dot_qb;
            double load_acc2b = (massQ + 0.5 * massL) * zu2b + q2b;
            double load_snap2b = qdd2b;
            double load_snap1b = qdd1b;
            double load_snap0b = qdd0b;
            double load_jer2b = qd2b;
            double load_jer1b = qd1b;
            double load_jer0b = qd0b;

            load_acc_grad_1 << load_acc0b, load_acc1b, load_acc2b, 0, 0;
            load_jer_grad_1 << load_jer0b, load_jer1b, load_jer2b, 0, 0;
            load_snap_grad_1 << load_snap0b, load_snap1b, load_snap2b, 0, 0;


            z0b = z_grad_2(0);
            z1b = z_grad_2(1);
            z2b = z_grad_2(2);

            q0b = 0.0;
            q1b = 0.0;
            q2b = 0.0;
            qd0b = 0.0;
            qd1b = 0.0;
            qd2b = 0.0;
            qdd0b = 0.0;
            qdd1b = 0.0;
            qdd2b = 0.0;
            q_dot_qb = 0.0;
            q_norm_3b = 0.0;
            q_norm_5b = 0.0;
            q_dot_qdb = 0.0;
            q_dot_qddb = 0.0;
            qd_dot_qdb = 0.0;
            sqr_q_dot_qdb = 0.0;
            qnormdd0b = 0.0;
            qnormdd1b = 0.0;
            qnormdd2b = 0.0;
            zu0b = 0.0;
            zu1b = 0.0;
            zu2b = 0.0;
            zu_sqr_normb = 0.0;
            zu_normb = thr_grad_2;
            temp = 0;
            tempb = 0;
            temp0 = 0;
            tempb0 = 0;
            q_normb = 0;

            zu_sqr_normb = (zu_sqr_norm_2 == 0.0 ? 0.0 : zu_normb / (2.0 * sqrt(zu_sqr_norm_2)));
            zu2b = z2b / zu_norm_2 + 2 * zu2_2 * zu_sqr_normb;
            zu_normb = -(zu2_2 * (z2b) / (zu_norm_2 * zu_norm_2));
            zu1b = z1b / zu_norm_2 + 2 * zu1_2 * zu_sqr_normb;
            zu_normb = -(zu1_2 * (z1b) / (zu_norm_2 * zu_norm_2));
            zu0b = z0b / zu_norm_2 + 2 * zu0_2 * zu_sqr_normb;
            zu_normb = -(zu0_2 * (z0b) / (zu_norm_2 * zu_norm_2));
            qnormdd2b = massQ * l_length * zu2b;
            qnormdd1b = massQ * l_length * zu1b;
            qnormdd0b = massQ * l_length * zu0b;
            temp = (q_dot_qdd_2 + qd_dot_qd_2) * q2_2 + 2 * q_dot_qd_2 * qd2_2;
            tempb0 = qnormdd2b / q_norm_5_2;
            sqr_q_dot_qdb = 3 * q2_2 * tempb0;
            q_dot_qb = -(temp * tempb0);
            tempb = -(q_dot_q_2 * tempb0);
            q2b = 3 * sqr_q_dot_qd_2 * tempb0 + (q_dot_qdd_2 + qd_dot_qd_2) * tempb;
            q_norm_5b = -((3 * sqr_q_dot_qd_2 * q2_2 - q_dot_q_2 * temp) * tempb0 / q_norm_5_2);
            q_dot_qddb = q2_2 * tempb;
            qd_dot_qdb = q2_2 * tempb;
            q_dot_qdb = 2 * qd2_2 * tempb;
            qd2b = 2 * q_dot_qd_2 * tempb;
            temp = (q_dot_qdd_2 + qd_dot_qd_2) * q1_2 + 2 * q_dot_qd_2 * qd1_2;
            tempb0 = qnormdd1b / q_norm_5_2;
            tempb = -(q_dot_q_2 * tempb0);
            q1b = 3 * sqr_q_dot_qd_2 * tempb0 + (q_dot_qdd_2 + qd_dot_qd_2) * tempb;
            q_dot_qddb = q_dot_qddb + q1_2 * tempb;
            qd_dot_qdb = qd_dot_qdb + q1_2 * tempb;
            q_dot_qdb = q_dot_qdb + 2 * qd1_2 * tempb;
            qd1b = 2 * q_dot_qd_2 * tempb;
            temp0 = (q_dot_qdd_2 + qd_dot_qd_2) * q0_2 + 2 * q_dot_qd_2 * qd0_2;
            tempb = qnormdd0b / q_norm_5_2;
            sqr_q_dot_qdb = sqr_q_dot_qdb + 3 * q1_2 * tempb0 + 3 * q0_2 * tempb;
            q_norm_5b = q_norm_5b - (3 * sqr_q_dot_qd_2 * q1_2 - q_dot_q_2 * temp) * tempb0 / q_norm_5_2 -
                        (3 * sqr_q_dot_qd_2 * q0_2 - q_dot_q_2 * temp0) * tempb / q_norm_5_2;
            q_norm_3b = q_dot_q_2 * q_norm_5b;
            q_normb = q_dot_q_2 * q_norm_3b - qdd2_2 * qnormdd2b / (q_norm_2 * q_norm_2) - qdd1_2 * qnormdd1b / (q_norm_2 * q_norm_2) - qdd0_2 * qnormdd0b / (q_norm_2 * q_norm_2);
            if (q_dot_q_2 == 0.0)
                q_dot_qb = q_dot_qb + q_norm_3_2 * q_norm_5b - temp * tempb0 - temp0 * tempb +
                           q_norm_2 * q_norm_3b;
            else
                q_dot_qb = q_dot_qb + q_norm_3_2 * q_norm_5b - temp * tempb0 - temp0 * tempb +
                           q_norm_2 * q_norm_3b + q_normb / (2.0 * sqrt(q_dot_q_2));
            tempb0 = -(q_dot_q_2 * tempb);
            q_dot_qddb = q_dot_qddb + q0_2 * tempb0;
            qdd2b = qnormdd2b / q_norm_2 + q2_2 * q_dot_qddb;
            qdd1b = qnormdd1b / q_norm_2 + q1_2 * q_dot_qddb;
            qdd0b = qnormdd0b / q_norm_2 + q0_2 * q_dot_qddb;
            qd_dot_qdb = qd_dot_qdb + q0_2 * tempb0;
            q_dot_qdb = q_dot_qdb + 2 * qd0_2 * tempb0 + 2 * q_dot_qd_2 * sqr_q_dot_qdb;
            q0b = 3 * sqr_q_dot_qd_2 * tempb + (q_dot_qdd_2 + qd_dot_qd_2) * tempb0 + qdd0_2 * q_dot_qddb + qd0_2 * q_dot_qdb + 2 * q0_2 * q_dot_qb;
            load_acc0b = (massQ + 0.5* massL) * zu0b + q0b;
            qd0b = 2 * q_dot_qd_2 * tempb0 + 2 * qd0_2 * qd_dot_qdb + q0_2 * q_dot_qdb;
            qd1b = qd1b + 2 * qd1_2 * qd_dot_qdb + q1_2 * q_dot_qdb;
            qd2b = qd2b + 2 * qd2_2 * qd_dot_qdb + q2_2 * q_dot_qdb;
            q1b = q1b + qdd1_2 * q_dot_qddb + qd1_2 * q_dot_qdb + 2 * q1_2 * q_dot_qb;
            load_acc1b = (massQ + 0.5 * massL) * zu1b + q1b;
            q2b = q2b + qdd2_2 * q_dot_qddb + qd2_2 * q_dot_qdb + 2 * q2_2 * q_dot_qb;
            load_acc2b = (massQ + 0.5 * massL) * zu2b + q2b;
            load_snap2b = qdd2b;
            load_snap1b = qdd1b;
            load_snap0b = qdd0b;
            load_jer2b = qd2b;
            load_jer1b = qd1b;
            load_jer0b = qd0b;

            load_acc_grad_2 << load_acc0b, load_acc1b, load_acc2b, 0, 0;
            load_jer_grad_2 << load_jer0b, load_jer1b, load_jer2b, 0, 0;
            load_snap_grad_2 << load_snap0b, load_snap1b, load_snap2b, 0, 0;

            return;
        }

    private:
        // Parameters
        double massQ, massL, grav, eps, l_length, r_length;
        // forward params
        double q0_1 = 0, qd0_1 = 0, qdd0_1 = 0;
        double q1_1 = 0, qd1_1 = 0, qdd1_1 = 0;
        double q2_1 = 0, qd2_1 = 0, qdd2_1 = 0;
        double q_dot_q_1;
        double q_norm_1, q_norm_3_1, q_norm_5_1;
        double q_dot_qd_1;
        double q_dot_qdd_1;
        double qd_dot_qd_1;
        double sqr_q_dot_qd_1;
        double qnormdd0_1, qnormdd1_1, qnormdd2_1;

        double q0_2 = 0, qd0_2 = 0, qdd0_2 = 0;
        double q1_2 = 0, qd1_2 = 0, qdd1_2 = 0;
        double q2_2 = 0, qd2_2 = 0, qdd2_2 = 0;
        double q_dot_q_2;
        double q_norm_2, q_norm_3_2, q_norm_5_2;
        double q_dot_qd_2;
        double q_dot_qdd_2;
        double qd_dot_qd_2;
        double sqr_q_dot_qd_2;
        double qnormdd0_2, qnormdd1_2, qnormdd2_2;

        double zu0_1;
        double zu1_1;
        double zu2_1;
        double zu0_2;
        double zu1_2;
        double zu2_2;
        double zu_sqr_norm_1;
        double zu_sqr_norm_2;
        double zu_norm_1;
        double zu_norm_2;
        double z0_1;
        double z1_1;
        double z2_1;
        double z0_2;
        double z1_2;
        double z2_2;

    };
}

#endif
