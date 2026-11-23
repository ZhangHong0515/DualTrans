#pragma once

#include <Eigen/Eigen>

#include <cmath>

namespace Eigen
{
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
    typedef Eigen::Matrix<double, 5, 5> Matrix5d;
}
namespace PayloadMPC
{
    class Flatness
    {
    public:
        void reset(const double &quadrotor_mass,
                   const double &payload_mass,
                   const double &l_length,
                   const double &r_length,
                   const double gravity = 9.81)
        {
            mass_q_ = quadrotor_mass;
            mass_l_ = payload_mass;
            l_length_ = l_length;
            r_length_ = r_length;
            grav_ = gravity;
        }

        void  qnorm(const Eigen::Vector3d &q, const Eigen::Vector3d &qd,
                   const Eigen::Vector3d &qdd, const Eigen::Vector3d &q3d,
                   Eigen::Vector3d &norm, Eigen::Vector3d &qnormd, Eigen::Vector3d &qnormdd, Eigen::Vector3d &qnorm3d)
        {
            double q_dot_q = q.squaredNorm();
            double q_norm = sqrt(q_dot_q);
            // If here need to check if q_norm is zero?
            // if (q_norm < 1e-8)
            // {
            //     norm=Eigen::Vector3d::Zero();
            //     norm.z() = -1.0;
            //     qnormd=Eigen::Vector3d::Zero();
            //     qnormdd = Eigen::Vector3d::Zero();
            //     qnorm3d = Eigen::Vector3d::Zero();
            // }
            assert(q_norm > 1e-8);

            double q_norm_3 = q_dot_q * q_norm;
            double q_norm_5 = q_dot_q * q_norm_3;
            double q_norm_7 = q_dot_q * q_norm_5;
            double q_dot_qd = q.dot(qd);
            double q_dot_qdd = q.dot(qdd);
            double qd_dot_qd = qd.dot(qd);
            double qd_dot_qdd = qd.dot(qdd);
            double q_dot_q3d = q.dot(q3d);

            double sqr_q_dot_qd = q_dot_qd * q_dot_qd;
            double q_dot_qd_3 = sqr_q_dot_qd * q_dot_qd;

            norm = q / q_norm;

            qnormd = (-q_dot_qd * q + q_dot_q * qd) / q_norm_3;

            qnormdd = (3 * (sqr_q_dot_qd)*q 
            - q_dot_q * ((q_dot_qdd + qd_dot_qd) * q 
            + 2 * q_dot_qd * qd)) / q_norm_5 + qdd / q_norm;

            qnorm3d = (-15 * q_dot_qd_3 * q 
            + 9 * q_dot_q * q_dot_qd * ((qd_dot_qd + q_dot_qdd) * q 
            + q_dot_qd * qd)) / q_norm_7 
            - ((q_dot_q3d + 3 * qd_dot_qdd) * q 
            + 3 * (qd_dot_qd + q_dot_qdd) * qd + 3 * q_dot_qd * qdd) / q_norm_3 
            + q3d / q_norm;
        }

        void normalizeFDF(const Eigen::Vector3d &x,
                             Eigen::Vector3d &xNor,
                             Eigen::Matrix3d &G)
        {
            const double a = x(0), b = x(1), c = x(2);
            const double aSqr = a * a, bSqr = b * b, cSqr = c * c;
            const double ab = a * b, bc = b * c, ca = c * a;
            const double xSqrNorm = aSqr + bSqr + cSqr;
            const double xNorm = sqrt(xSqrNorm);
            const double den = xSqrNorm * xNorm;
            xNor = x / xNorm;
            G(0, 0) = bSqr + cSqr;
            G(0, 1) = -ab;
            G(0, 2) = -ca;
            G(1, 0) = -ab;
            G(1, 1) = aSqr + cSqr;
            G(1, 2) = -bc;
            G(2, 0) = -ca;
            G(2, 1) = -bc;
            G(2, 2) = aSqr + bSqr;
            G /= den;
            // return;
        }

        // void qdotdot(const Eigen::Vector3d &q, const Eigen::Vector3d &jerk,
        //                                 Eigen::Matrix3d &G)
        // {
        //     double t0 = q.norm();
        //     double t1 = 1.0 / pow(t0, 3);
        //     double t2 = q.dot(jerk);

        //     G = -(t1 * jerk * q.transpose() + t1 * t2 * Eigen::Matrix3d::Identity() + t1 * q * jerk.transpose() - (3 * t2) * q * q.transpose() / pow(t0, 5));

        //     return;
        // }
        inline void forward_p( Eigen::Vector5d pos,
                             Eigen::Vector5d vel,
                             Eigen::Vector5d acc,
                             Eigen::Vector5d jerk,
                             Eigen::Vector5d snap,
                             Eigen::Vector5d crackle,

                            Eigen::Vector3d &pos_quad_1,
                            Eigen::Vector3d &vel_quad_1,
                            Eigen::Vector3d &acc_quad_1,
                            Eigen::Vector3d &jerk_quad_1,
                            Eigen::Vector3d &acc_attach_1,
                            Eigen::Vector3d &jerk_attach_1,
                            Eigen::Vector3d &p_1,
                            Eigen::Vector3d &pd_1,
                            
                            Eigen::Vector3d &pos_quad_2,
                            Eigen::Vector3d &vel_quad_2,
                            Eigen::Vector3d &acc_quad_2,
                            Eigen::Vector3d &jerk_quad_2,
                            Eigen::Vector3d &acc_attach_2,
                            Eigen::Vector3d &jerk_attach_2,
                            Eigen::Vector3d &p_2,
                            Eigen::Vector3d &pd_2)
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
 
            Eigen::Vector3d attach_pos_1 = pos.head(3) - r_length_ * qr;
            Eigen::Vector3d attach_vel_1 = vel.head(3) - r_length_ * dqr;
            Eigen::Vector3d attach_acc_1 = acc.head(3) - r_length_ * ddqr;
            Eigen::Vector3d attach_jerk_1 = jerk.head(3) - r_length_ * d3qr;
            Eigen::Vector3d attach_snap_1 = snap.head(3) - r_length_ * d4qr;
            Eigen::Vector3d attach_crackle_1 = crackle.head(3) - r_length_ * d5qr;
            Eigen::Vector3d attach_pos_2 = pos.head(3) + r_length_ * qr;
            Eigen::Vector3d attach_vel_2 = vel.head(3) + r_length_ * dqr;
            Eigen::Vector3d attach_acc_2 = acc.head(3) + r_length_ * ddqr;
            Eigen::Vector3d attach_jerk_2 = jerk.head(3) + r_length_ * d3qr;
            Eigen::Vector3d attach_snap_2 = snap.head(3) + r_length_ * d4qr;
            Eigen::Vector3d attach_crackle_2 = crackle.head(3) + r_length_ * d5qr;

            acc_attach_1 = attach_acc_1;
            acc_attach_2 = attach_acc_2;
            jerk_attach_1 = attach_jerk_1;
            jerk_attach_2 = attach_jerk_2;
            Eigen::Vector3d Tp_1 = -attach_acc_1; ////here is -Tp
            Tp_1(2) -= grav_;
            Eigen::Vector3d pdd, p3d;

            qnorm(Tp_1, -attach_jerk_1, -attach_snap_1, -attach_crackle_1, p_1, pd_1, pdd, p3d);
            pos_quad_1 = pos.head(3) - r_length_ * qr  - l_length_ * p_1;
            vel_quad_1 = vel.head(3) - r_length_ * dqr - l_length_ * pd_1;
            acc_quad_1 = acc.head(3) - r_length_ * ddqr- l_length_ * pdd;
            jerk_quad_1= jerk.head(3)- r_length_ * d3qr- l_length_ * p3d;

            Eigen::Vector3d Tp_2 = -attach_acc_2;
            Tp_2(2) -= grav_;
            qnorm(Tp_2, -attach_jerk_2, -attach_snap_2, -attach_crackle_2, p_2, pd_2, pdd, p3d);
            pos_quad_2 = pos.head(3) + r_length_ * qr  - l_length_ * p_2;
            vel_quad_2 = vel.head(3) + r_length_ * dqr - l_length_ * pd_2;
            acc_quad_2 = acc.head(3) + r_length_ * ddqr- l_length_ * pdd;
            jerk_quad_2= jerk.head(3)+ r_length_ * d3qr- l_length_ * p3d;
    
        }

    void forward_rot(const Eigen::Vector3d &acc_attach,
                  const Eigen::Vector3d &jerk_attach,
                  const Eigen::Vector3d &acc_quad,
                  const Eigen::Vector3d &jerk_quad,
                  const Eigen::Vector3d &fq,
                  const Eigen::Vector3d &fl,
                  const double &heading,
                  const double &heading_rate,

                  Eigen::Quaterniond &quat,
                  double &thr,
                  Eigen::Vector3d &omg)
    {

        Eigen::Vector3d des_f_in_world = mass_q_ * acc_quad + 0.5 * mass_l_ * acc_attach + Eigen::Vector3d(0, 0, (mass_q_ + 0.5 * mass_l_) * grav_) - fq - fl;

        thr = des_f_in_world.norm();
        Eigen::Vector3d z_B;
        z_B = des_f_in_world.normalized();

        const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
            Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));

        // Compute desired orientation
        const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

        const Eigen::Vector3d x_B = y_C.cross(z_B).normalized();

        const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

        // From the computed desired body axes we can now compose a desired attitude
        const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

        quat = Eigen::Quaterniond(R_W_B);

        // Reference body rates
        if (fabs(thr) < kAlmostZeroThrustThreshold_)
        {
            omg.x() = 0.0;
            omg.y() = 0.0;
        }
        else
        {
            omg.x() = -1.0 / thr * y_B.dot(mass_q_ * jerk_quad + 0.5 * mass_l_ * jerk_attach);
            omg.y() = 1.0 / thr * x_B.dot(mass_q_ * jerk_quad + 0.5 * mass_l_ * jerk_attach);
        }

        double normalize_y = (y_C.cross(z_B)).norm();

        if (fabs(normalize_y) < kAlmostZeroValueThreshold_)
        {
            omg.z() = 0.0;
        }
        else
        {
            omg.z() =
                1.0 / normalize_y *
                (heading_rate * x_C.dot(x_B) + omg.y() * y_C.dot(z_B));
        }
    }  

    private:
        double mass_q_, mass_l_, grav_, l_length_, r_length_;
        const double kAlmostZeroThrustThreshold_{0.01};
        const double kAlmostZeroValueThreshold_{0.001};
    };
}