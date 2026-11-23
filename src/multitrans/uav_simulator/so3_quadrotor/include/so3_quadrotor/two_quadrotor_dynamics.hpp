#pragma once 
#include "so3_quadrotor/geometry_utils.hpp"
#include "random"
#include "iostream"

const double kLengthError = 0.025;
namespace so3_quadrotor
{
    #define RED "\033[31m"    /* Red */
    #define YELLOW "\033[33m" /* Yellow */
    #define RESET "\033[0m"
    
    // data types 
    struct Config
    {
        double              g; // gravity
        double              mass_q; // uav mass
        Eigen::Matrix3d     J_q; // uav inertia
        double              kf;
        double              km;
        double              arm_length;
        double              motor_time_constant; // unit: sec
        double              max_rpm;
        double              min_rpm;
        double              mass_r; // load mass 
        double              cable_length; // load length
        double              rode_length; // 1/2 * rod length
        double              kOmega[3];
        double              kdOmega[3];
    };

    struct Control
    {
        double rpm[4];
    };

    struct Cmd
    {
        double thrust;
        Eigen::Vector3d body_rates;
    };

    class TwoQuadrotor
    {
        private:
            // parameters
            Config config_;

            // state
            struct State
            {
                Eigen::Vector3d x_1 = Eigen::Vector3d::Zero(); // uav_1 position
                Eigen::Vector3d v_1 = Eigen::Vector3d::Zero(); // uav_1 velocity
                Eigen::Matrix3d R_1 = Eigen::Matrix3d::Identity(); // uav_1 attitude
                Eigen::Vector3d omega_1 = Eigen::Vector3d::Zero(); // uav_1 angle velocity
                Eigen::Vector4d motor_rpm_1 = Eigen::Vector4d::Zero(); // uav_1 motor rpm

                Eigen::Vector3d x_2 = Eigen::Vector3d::Zero(); // uav_2 position
                Eigen::Vector3d v_2 = Eigen::Vector3d::Zero(); // uav_2 velocity
                Eigen::Matrix3d R_2 = Eigen::Matrix3d::Identity(); // uav_2 attitude
                Eigen::Vector3d omega_2 = Eigen::Vector3d::Zero(); // uav_2 angle velocity
                Eigen::Vector4d motor_rpm_2 = Eigen::Vector4d::Zero(); // uav_2 motor rpm
                
                Eigen::Vector3d xr  = Eigen::Vector3d(0, 0, 0); // position of rod center
                Eigen::Vector3d vr  = Eigen::Vector3d::Zero();  // velocity of rod center
                Eigen::Vector3d qr  = Eigen::Vector3d(0, 1, 0); // Attitude of rod in inertial frame
                Eigen::Vector3d dqr = Eigen::Vector3d::Zero();  // Attitude velocity of rode in inertial frame
                
                Eigen::Vector3d ql_1  = Eigen::Vector3d(0, 0, -1); // cable_1 vector
                Eigen::Vector3d dql_1 = Eigen::Vector3d::Zero();   // cable_1 vector rate
                Eigen::Vector3d ddql_1= Eigen::Vector3d::Zero();   
                Eigen::Vector3d ql_2  = Eigen::Vector3d(0, 0, -1); // cable_2 vector
                Eigen::Vector3d dql_2 = Eigen::Vector3d::Zero();   // cable_2 vector rate
                Eigen::Vector3d ddql_2= Eigen::Vector3d::Zero(); 
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

                inline State operator+(const State &t) const 
                {
                    State sum;
                    sum.x_1 = x_1 + t.x_1;
                    sum.v_1 = v_1 + t.v_1;
                    sum.R_1 = R_1 + t.R_1;
                    sum.omega_1 = omega_1 + t.omega_1;
                    sum.motor_rpm_1 = motor_rpm_1 + t.motor_rpm_1;
                    sum.x_2 = x_2 + t.x_2;
                    sum.v_2 = v_2 + t.v_2;
                    sum.R_2 = R_2 + t.R_2;
                    sum.omega_2 = omega_2 + t.omega_2;
                    sum.motor_rpm_2 = motor_rpm_2 + t.motor_rpm_2;

                    sum.xr = xr + t.xr;
                    sum.vr = vr + t.vr;
                    sum.qr = qr + t.qr;
                    sum.dqr = dqr + t.dqr;

                    sum.ql_1 = ql_1 + t.ql_1;
                    sum.dql_1 = dql_1 + t.dql_1;
                    sum.ddql_1 = ddql_1 + t.ddql_1;
                    sum.ql_2 = ql_2 + t.ql_2;
                    sum.dql_2 = dql_2 + t.dql_2;
                    sum.ddql_2 = ddql_2 + t.ddql_2;
                    // sum.wl = wl + t.wl;     
                    return sum;
                }

                inline State operator*(const double &t) const 
                {
                    State mul;
                    mul.x_1 = x_1 * t;
                    mul.v_1 = v_1 * t;
                    mul.R_1 = R_1 * t;
                    mul.omega_1 = omega_1 * t;
                    mul.motor_rpm_1 = motor_rpm_1 * t;
                    mul.x_2 = x_2 * t;
                    mul.v_2 = v_2 * t;
                    mul.R_2 = R_2 * t;
                    mul.omega_2 = omega_2 * t;
                    mul.motor_rpm_2 = motor_rpm_2 * t;

                    mul.xr = xr * t;
                    mul.vr = vr * t;
                    mul.qr = qr * t;
                    mul.dqr = dqr * t;

                    mul.ql_1 = ql_1 * t;
                    mul.dql_1 = dql_1 * t;
                    mul.ddql_1 = ddql_1 * t;
                    mul.ql_2 = ql_2 * t;
                    mul.dql_2 = dql_2 * t;
                    mul.ddql_2 = ddql_2 * t;
                    // mul.wl = wl * t;
                    return mul;
                }

                inline State operator/(const double &t) const 
                {
                    State mul;
                    mul.x_1 = x_1 / t;
                    mul.v_1 = v_1 / t;
                    mul.R_1 = R_1 / t;
                    mul.omega_1 = omega_1 / t;
                    mul.motor_rpm_1 = motor_rpm_1 / t;
                    mul.x_2 = x_2 / t;
                    mul.v_2 = v_2 / t;
                    mul.R_2 = R_2 / t;
                    mul.omega_2 = omega_2 / t;
                    mul.motor_rpm_2 = motor_rpm_2 / t;

                    mul.xr = xr / t;
                    mul.vr = vr / t;
                    mul.qr = qr / t;
                    mul.dqr = dqr / t;
                    // mul.wl = wl / t;
                    mul.ql_1 = ql_1 / t;
                    mul.dql_1 = dql_1 / t;
                    mul.ddql_1 = ddql_1 / t;
                    mul.ql_2 = ql_2 / t;
                    mul.dql_2 = dql_2 / t;
                    mul.ddql_1 = ddql_1 / t;
                    return mul;
                }
            } state_;

            Eigen::Vector4d input_1 = Eigen::Vector4d::Zero(); 
            Eigen::Vector4d input_2 = Eigen::Vector4d::Zero();

        
            
        public:

            std::default_random_engine generator;
            std::normal_distribution<double> distribution{0.0,1.0};
            const Config &config;
            const State &state;
            State last_state;
            double last_dt;
            TwoQuadrotor(const Config &conf) : config_(conf), config(config_), state(state_) {};
            ~TwoQuadrotor() {};

            // Inputs are desired RPM for the motors
            // Rotor numbering is:
            //   *1*    Front
            // 3     4
            //    2
            // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
            inline void setInput_1(double u1, double u2, double u3, double u4) 
            {
                input_1(0) = u1;
                input_1(1) = u2;
                input_1(2) = u3;
                input_1(3) = u4;
                for (size_t i=0; i<4; ++i) {
                if (std::isnan(input_1(i))) {
                    printf("so3_quadrotor: NAN input!\n");
                }
                input_1(i) = input_1(i) < config_.max_rpm ? input_1(i) : config_.max_rpm;
                input_1(i) = input_1(i) > config_.min_rpm ? input_1(i) : config_.min_rpm;
                }
            }

            // Inputs are desired RPM for the motors
            // Rotor numbering is:
            //   *1*    Front
            // 3     4
            //    2
            // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
            inline void setInput_2(double u1, double u2, double u3, double u4) 
            {
                input_2(0) = u1;
                input_2(1) = u2;
                input_2(2) = u3;
                input_2(3) = u4;
                for (size_t i=0; i<4; ++i) {
                if (std::isnan(input_2(i))) {
                    printf("so3_quadrotor: NAN input!\n");
                }
                input_2(i) = input_2(i) < config_.max_rpm ? input_2(i) : config_.max_rpm;
                input_2(i) = input_2(i) > config_.min_rpm ? input_2(i) : config_.min_rpm;
                }
            }

            Eigen::Vector3d random_force(double std)
            {
                Eigen::Vector3d noise(distribution(generator),distribution(generator),distribution(generator));
                return noise*std;
            }


            // calculate dot of state 
            inline State diff(const State &state)
            {
                State state_dot;

                // Re-orthonormalize R (polar decomposition)
                Eigen::LLT<Eigen::Matrix3d> llt_1(state.R_1.transpose() * state.R_1);
                Eigen::Matrix3d             P_1 = llt_1.matrixL();
                Eigen::Matrix3d             R_1 = state.R_1 * P_1.inverse();
                // Re-orthonormalize R (polar decomposition)
                Eigen::LLT<Eigen::Matrix3d> llt_2(state.R_2.transpose() * state.R_2);
                Eigen::Matrix3d             P_2 = llt_2.matrixL();
                Eigen::Matrix3d             R_2 = state.R_2 * P_2.inverse();

                // 反对称矩阵
                Eigen::Matrix3d omega_vee_1(Eigen::Matrix3d::Zero());
                Eigen::Matrix3d omega_vee_2(Eigen::Matrix3d::Zero());

                omega_vee_1(2, 1) = state.omega_1(0);
                omega_vee_1(1, 2) = -state.omega_1(0);
                omega_vee_1(0, 2) = state.omega_1(1);
                omega_vee_1(2, 0) = -state.omega_1(1);
                omega_vee_1(1, 0) = state.omega_1(2);
                omega_vee_1(0, 1) = -state.omega_1(2);
                
                omega_vee_2(2, 1) = state.omega_2(0);
                omega_vee_2(1, 2) = -state.omega_2(0);
                omega_vee_2(0, 2) = state.omega_2(1);
                omega_vee_2(2, 0) = -state.omega_2(1);
                omega_vee_2(1, 0) = state.omega_2(2);
                omega_vee_2(0, 1) = -state.omega_2(2);
                
                Eigen::Vector3d norm_qr = state.qr.normalized();
                Eigen::Vector3d norm_ql_1 = state.ql_1.normalized();
                Eigen::Vector3d norm_ql_2 = state.ql_2.normalized();
                
                Eigen::Matrix3d payload_qr_vee(Eigen::Matrix3d::Zero());
                Eigen::Matrix3d cable_ql_1_vee(Eigen::Matrix3d::Zero());
                Eigen::Matrix3d cable_ql_2_vee(Eigen::Matrix3d::Zero());
                payload_qr_vee(2, 1) = norm_qr(0);
                payload_qr_vee(1, 2) = -norm_qr(0);
                payload_qr_vee(0, 2) = norm_qr(1);
                payload_qr_vee(2, 0) = -norm_qr(1);
                payload_qr_vee(1, 0) = norm_qr(2);
                payload_qr_vee(0, 1) = -norm_qr(2);
                
                cable_ql_1_vee(2, 1) = norm_ql_1(0);
                cable_ql_1_vee(1, 2) = -norm_ql_1(0);
                cable_ql_1_vee(0, 2) = norm_ql_1(1);
                cable_ql_1_vee(2, 0) = -norm_ql_1(1);
                cable_ql_1_vee(1, 0) = norm_ql_1(2);
                cable_ql_1_vee(0, 1) = -norm_ql_1(2);

                cable_ql_2_vee(2, 1) = norm_ql_2(0);
                cable_ql_2_vee(1, 2) = -norm_ql_2(0);
                cable_ql_2_vee(0, 2) = norm_ql_2(1);
                cable_ql_2_vee(2, 0) = -norm_ql_2(1);
                cable_ql_2_vee(1, 0) = norm_ql_2(2);
                cable_ql_2_vee(0, 1) = -norm_ql_2(2);

                // 计算油门和力矩
                Eigen::Vector4d motor_rpm_sq_1 = state.motor_rpm_1.array().square();
                Eigen::Vector4d motor_rpm_sq_2 = state.motor_rpm_2.array().square();

                double thrust_1 = config_.kf * motor_rpm_sq_1.sum();
                double thrust_2 = config_.kf * motor_rpm_sq_2.sum();

                Eigen::Vector3d moments_1;
                Eigen::Vector3d moments_2;
                moments_1(0) = config_.kf * (motor_rpm_sq_1(2) - motor_rpm_sq_1(3)) * config_.arm_length;
                moments_1(1) = config_.kf * (motor_rpm_sq_1(1) - motor_rpm_sq_1(0)) * config_.arm_length;
                moments_1(2) = config_.km * (motor_rpm_sq_1(0) + motor_rpm_sq_1(1) - motor_rpm_sq_1(2) - motor_rpm_sq_1(3));

                moments_2(0) = config_.kf * (motor_rpm_sq_2(2) - motor_rpm_sq_2(3)) * config_.arm_length;
                moments_2(1) = config_.kf * (motor_rpm_sq_2(1) - motor_rpm_sq_2(0)) * config_.arm_length;
                moments_2(2) = config_.km * (motor_rpm_sq_2(0) + motor_rpm_sq_2(1) - motor_rpm_sq_2(2) - motor_rpm_sq_2(3));

                double resistancequad_1 = 0.05 *                                        // C
                                    3.14159265 * (config_.arm_length) * (config_.arm_length) * // S
                                    state.v_1.norm() * state.v_1.norm();
                double resistancequad_2 = 0.05 *                                        // C
                                    3.14159265 * (config_.arm_length) * (config_.arm_length) * // S
                                    state.v_2.norm() * state.v_2.norm();
                double resistanceload_1= 0.05 *                                        // C
                                    3.14159265 * (config_.arm_length) * (config_.arm_length) * // S
                                    state.vr.norm() * state.vr.norm();
    
                Eigen::Vector3d vquadnorm_1 = state.v_1.normalized();
                Eigen::Vector3d vquadnorm_2 = state.v_2.normalized();
                Eigen::Vector3d vloadnorm   = state.vr.normalized();

                Eigen::Vector3d fl = Eigen::Vector3d(0,0.0,0) ;
                Eigen::Vector3d fq = Eigen::Vector3d(0.0,0,0) ;

               
                // payload acc 
                Eigen::Vector3d payload_acc  = (1 / (2 * config_.mass_q + config_.mass_r)) * (thrust_2  * R_2.col(2) + thrust_1 * R_1.col(2) + config_.mass_q * config_.cable_length * (state_.ddql_1 + state_.ddql_2)) - Eigen::Vector3d(0, 0, config_.g);
                Eigen::Vector3d payload_ddqr = (- 1 / ((2 * config_.mass_q +  2 / 3 * config_.mass_r) * config_.rode_length)) * (payload_qr_vee * payload_qr_vee * (thrust_2  * R_2.col(2) - thrust_1 * R_1.col(2) + config_.mass_q * config_.rode_length * (state_.ddql_2 - state_.ddql_1))) - state.dqr.dot(state.dqr) * norm_qr;

                Eigen::Vector3d attach_accel_1 = payload_acc - config_.rode_length * payload_ddqr + Eigen::Vector3d(0, 0, config.g);
                Eigen::Vector3d attach_accel_2 = payload_acc + config_.rode_length * payload_ddqr + Eigen::Vector3d(0, 0, config.g);

                double  cbl_omg_sqr_1 = (cable_ql_1_vee * state.dql_1).squaredNorm();
                double  cbl_omg_sqr_2 = (cable_ql_2_vee * state.dql_2).squaredNorm();
                
                Eigen::Vector3d tensor_1 = norm_ql_1 * ((config_.mass_q * config_.cable_length * cbl_omg_sqr_1) - norm_ql_1.transpose() * (thrust_1 * R_1.col(2) - config_.mass_q * attach_accel_1));
                Eigen::Vector3d tensor_2 = norm_ql_2 * ((config_.mass_q * config_.cable_length * cbl_omg_sqr_2) - norm_ql_2.transpose() * (thrust_2 * R_2.col(2) - config_.mass_q * attach_accel_2));
                

                Eigen::Vector3d dd_cable_1 = cable_ql_1_vee * cable_ql_1_vee * (thrust_1 * R_1.col(2) - config_.mass_q * attach_accel_1) / (config_.mass_q * config_.cable_length) - norm_ql_1 * state.dql_1.dot(state.dql_1);
                Eigen::Vector3d dd_cable_2 = cable_ql_2_vee * cable_ql_2_vee * (thrust_2 * R_2.col(2) - config_.mass_q * attach_accel_2) / (config_.mass_q * config_.cable_length) - norm_ql_2 * state.dql_2.dot(state.dql_2);
                

                // 先不考虑绳短的问题

                state_dot.x_1 = state.v_1;
                state_dot.v_1 = (thrust_1 * R_1.col(2) + tensor_1) / config_.mass_q - Eigen::Vector3d(0, 0, config.g);
                state_dot.R_1 = R_1 * omega_vee_1;
                state_dot.omega_1 = config_.J_q.inverse() * (moments_1 - state.omega_1.cross(config_.J_q * state.omega_1)); //the same
                state_dot.motor_rpm_1 = (input_1 - state.motor_rpm_1) / config_.motor_time_constant;

                state_dot.x_2 = state.v_2;
                state_dot.v_2 = (thrust_2 * R_2.col(2) + tensor_2) / config_.mass_q - Eigen::Vector3d(0, 0, config.g);
                state_dot.R_2 = R_2 * omega_vee_2;
                state_dot.omega_2 = config_.J_q.inverse() * (moments_2 - state.omega_2.cross(config_.J_q * state.omega_2)); //the same
                state_dot.motor_rpm_2 = (input_2 - state.motor_rpm_2) / config_.motor_time_constant;

                state_dot.xr = state.vr;
                state_dot.vr = payload_acc;
                state_dot.qr = state.dqr;
                state_dot.dqr = payload_ddqr;

                state_dot.ql_1 = state.dql_1;
                state_dot.dql_1 = dd_cable_1;
                state_dot.ddql_1 = Eigen::Vector3d(0,0,0);
                state_dot.ql_2 = state.dql_2;
                state_dot.dql_2 = dd_cable_2;
                state_dot.ddql_2 = Eigen::Vector3d(0,0,0);
                return state_dot;
            };


            // Runs the actual dynamics simulation with a time step of dt
            inline void step(const double &dt)
            {
                // Runge-kutta
                last_state = state_;
                last_dt = dt; // for numerical differentiation
                State k1 = diff(state_);
                state_.ddql_1 = k1.dql_1;
                state_.ddql_2 = k1.dql_2;
                State k2 = diff(state_+k1*dt/2);
                State k3 = diff(state_+k2*dt/2);
                State k4 = diff(state_+k3*dt);
                state_ = state_ + (k1+k2*2+k3*2+k4) * dt/6;
            }

            // get control from cmd
            inline Control getControl_1(const Cmd& cmd)
            {
                static double LasteOm1 = 0;
                static double LasteOm2 = 0;
                static double LasteOm3 = 0;

                static float LastOm1 = 0;
                static float LastOm2 = 0;
                static float LastOm3 = 0;

                static double LastRpm1 = 0;
                static double LastRpm2 = 0;
                static double LastRpm3 = 0;
                static double LastRpm4 = 0;   

                double         kf = config_.kf;
                double         km = config_.km / kf * kf;
                double          d = config_.arm_length;
                Eigen::Matrix3f J = config_.J_q.cast<float>();
                float     I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                                    { J(1, 0), J(1, 1), J(1, 2) },
                                    { J(2, 0), J(2, 1), J(2, 2) } };

                // rotation, may use external yaw
                Eigen::Vector3d ypr = uav_utils::R_to_ypr(state_.R_1);
                // if (cmd.use_external_yaw) {
                //   ypr[0] = cmd.current_yaw;
                // }
                Eigen::Matrix3d R; 
                R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());

                float Om1 = state_.omega_1(0);
                float Om2 = state_.omega_1(1);
                float Om3 = state_.omega_1(2);

                double rpm1 = state_.motor_rpm_1(0);
                double rpm2 = state_.motor_rpm_1(1);
                double rpm3 = state_.motor_rpm_1(2);
                double rpm4 = state_.motor_rpm_1(3);

                float force = cmd.thrust;

                float eOm1 = cmd.body_rates[0] - Om1;
                float eOm2 = cmd.body_rates[1] - Om2;
                float eOm3 = cmd.body_rates[2] - Om3;

                float cOm1 = cmd.body_rates[0];
                float cOm2 = cmd.body_rates[1];
                float cOm3 = cmd.body_rates[2];

                float in1 = cOm2 * (I[2][0] * cOm1 + I[2][1] * cOm2 + I[2][2] * cOm3) -
                            cOm3 * (I[1][0] * cOm1 + I[1][1] * cOm2 + I[1][2] * cOm3);
                float in2 = cOm3 * (I[0][0] * cOm1 + I[0][1] * cOm2 + I[0][2] * cOm3) -
                            cOm1 * (I[2][0] * cOm1 + I[2][1] * cOm2 + I[2][2] * cOm3);
                float in3 = cOm1 * (I[1][0] * cOm1 + I[1][1] * cOm2 + I[1][2] * cOm3) -
                            cOm2 * (I[0][0] * cOm1 + I[0][1] * cOm2 + I[0][2] * cOm3);

                float dOm1 = (Om1-LastOm1);
                float dOm2 = (Om2-LastOm2);
                float dOm3 = (Om3-LastOm3);

                float drpm1 = (rpm1-LastRpm1);
                float drpm2 = (rpm2-LastRpm2);
                float drpm3 = (rpm3-LastRpm3);
                float drpm4 = (rpm4-LastRpm4);

                float M2_from_rpm = (rpm2*rpm2 - rpm1*rpm1)*d*kf;
                float M1_from_rpm = (rpm3*rpm3 - rpm4*rpm4)*d*kf;
                float M3_from_rpm = (rpm1*rpm1 + rpm2*rpm2 - rpm3*rpm3 - rpm4*rpm4)*km + 0*(drpm1 + drpm2  - drpm3  -drpm4); 

                //INDI enable
                float t_com1 = M1_from_rpm - (dOm1 * I[0][0] + dOm2 * I[0][1] + dOm3 * I[0][2]);
                float t_com2 = M2_from_rpm - (dOm1 * I[1][0] + dOm2 * I[1][1] + dOm3 * I[1][2]);
                float t_com3 = M3_from_rpm - (dOm1 * I[2][0] + dOm2 * I[2][1] + dOm3 * I[2][2]);
                // t_com1 = 0;
                // t_com2 = 0;
                // t_com3 = 0;
                float M1 = config_.kOmega[0] * eOm1 + config_.kdOmega[0] * (eOm1-LasteOm1) + in1 + t_com1;
                float M2 = config_.kOmega[1] * eOm2 + config_.kdOmega[1] * (eOm2-LasteOm2) + in2 + t_com2;
                float M3 = config_.kOmega[2] * eOm3 + config_.kdOmega[2] * (eOm3-LasteOm3) + in3 + t_com3;

                LasteOm1 = eOm1;
                LasteOm2 = eOm2;
                LasteOm3 = eOm3;
                LastOm1 = Om1;
                LastOm2 = Om2;
                LastOm3 = Om3;
                LastRpm1 = rpm1;
                LastRpm2 = rpm2;
                LastRpm3 = rpm3;
                LastRpm4 = rpm4;

                float w_sq[4];
                w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
                w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
                w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
                w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

                Control control;
                for (int i = 0; i < 4; i++) {
                if (w_sq[i] < 0) w_sq[i] = 0;
                control.rpm[i] = sqrtf(w_sq[i]);
                }
                // std::cout << "t_com1: " << t_com1 << " t_com2: " << t_com2 << " t_com3: " << t_com3 << std::endl;
                return control;
            }

            // get control from cmd
            inline Control getControl_2(const Cmd& cmd)
            {
                static double LasteOm1 = 0;
                static double LasteOm2 = 0;
                static double LasteOm3 = 0;

                static float LastOm1 = 0;
                static float LastOm2 = 0;
                static float LastOm3 = 0;

                static double LastRpm1 = 0;
                static double LastRpm2 = 0;
                static double LastRpm3 = 0;
                static double LastRpm4 = 0;   

                double         kf = config_.kf;
                double         km = config_.km / kf * kf;
                double          d = config_.arm_length;
                Eigen::Matrix3f J = config_.J_q.cast<float>();
                float     I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                                    { J(1, 0), J(1, 1), J(1, 2) },
                                    { J(2, 0), J(2, 1), J(2, 2) } };

                // rotation, may use external yaw
                Eigen::Vector3d ypr = uav_utils::R_to_ypr(state_.R_2);
                // if (cmd.use_external_yaw) {
                //   ypr[0] = cmd.current_yaw;
                // }
                Eigen::Matrix3d R; 
                R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());

                float Om1 = state_.omega_2(0);
                float Om2 = state_.omega_2(1);
                float Om3 = state_.omega_2(2);

                double rpm1 = state_.motor_rpm_2(0);
                double rpm2 = state_.motor_rpm_2(1);
                double rpm3 = state_.motor_rpm_2(2);
                double rpm4 = state_.motor_rpm_2(3);

                float force = cmd.thrust;

                float eOm1 = cmd.body_rates[0] - Om1;
                float eOm2 = cmd.body_rates[1] - Om2;
                float eOm3 = cmd.body_rates[2] - Om3;

                float cOm1 = cmd.body_rates[0];
                float cOm2 = cmd.body_rates[1];
                float cOm3 = cmd.body_rates[2];

                float in1 = cOm2 * (I[2][0] * cOm1 + I[2][1] * cOm2 + I[2][2] * cOm3) -
                            cOm3 * (I[1][0] * cOm1 + I[1][1] * cOm2 + I[1][2] * cOm3);
                float in2 = cOm3 * (I[0][0] * cOm1 + I[0][1] * cOm2 + I[0][2] * cOm3) -
                            cOm1 * (I[2][0] * cOm1 + I[2][1] * cOm2 + I[2][2] * cOm3);
                float in3 = cOm1 * (I[1][0] * cOm1 + I[1][1] * cOm2 + I[1][2] * cOm3) -
                            cOm2 * (I[0][0] * cOm1 + I[0][1] * cOm2 + I[0][2] * cOm3);

                float dOm1 = (Om1-LastOm1);
                float dOm2 = (Om2-LastOm2);
                float dOm3 = (Om3-LastOm3);

                float drpm1 = (rpm1-LastRpm1);
                float drpm2 = (rpm2-LastRpm2);
                float drpm3 = (rpm3-LastRpm3);
                float drpm4 = (rpm4-LastRpm4);

                float M2_from_rpm = (rpm2*rpm2 - rpm1*rpm1)*d*kf;
                float M1_from_rpm = (rpm3*rpm3 - rpm4*rpm4)*d*kf;
                float M3_from_rpm = (rpm1*rpm1 + rpm2*rpm2 - rpm3*rpm3 - rpm4*rpm4)*km + 0*(drpm1 + drpm2  - drpm3  -drpm4); 

                //INDI enable
                float t_com1 = M1_from_rpm - (dOm1 * I[0][0] + dOm2 * I[0][1] + dOm3 * I[0][2]);
                float t_com2 = M2_from_rpm - (dOm1 * I[1][0] + dOm2 * I[1][1] + dOm3 * I[1][2]);
                float t_com3 = M3_from_rpm - (dOm1 * I[2][0] + dOm2 * I[2][1] + dOm3 * I[2][2]);
                // t_com1 = 0;
                // t_com2 = 0;
                // t_com3 = 0;
                float M1 = config_.kOmega[0] * eOm1 + config_.kdOmega[0] * (eOm1-LasteOm1) + in1 + t_com1;
                float M2 = config_.kOmega[1] * eOm2 + config_.kdOmega[1] * (eOm2-LasteOm2) + in2 + t_com2;
                float M3 = config_.kOmega[2] * eOm3 + config_.kdOmega[2] * (eOm3-LasteOm3) + in3 + t_com3;

                LasteOm1 = eOm1;
                LasteOm2 = eOm2;
                LasteOm3 = eOm3;
                LastOm1 = Om1;
                LastOm2 = Om2;
                LastOm3 = Om3;
                LastRpm1 = rpm1;
                LastRpm2 = rpm2;
                LastRpm3 = rpm3;
                LastRpm4 = rpm4;

                float w_sq[4];
                w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
                w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
                w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
                w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

                Control control;
                for (int i = 0; i < 4; i++) {
                if (w_sq[i] < 0) w_sq[i] = 0;
                control.rpm[i] = sqrtf(w_sq[i]);
                }
                // std::cout << "t_com1: " << t_com1 << " t_com2: " << t_com2 << " t_com3: " << t_com3 << std::endl;
                return control;
            }

            // set initial state 
            inline void setUAV1Pos(const Eigen::Vector3d &pos)
            {
                state_.x_1 = pos;
            }

            inline void setUAV2Pos(const Eigen::Vector3d &pos)
            {
                state_.x_2 = pos;
            }

            inline void setLoadPos(const Eigen::Vector3d &pos)
            {
                state_.xr = pos;
            }

            inline void setUAV1Ypr(const Eigen::Vector3d &ypr)
            {
                state_.R_1 = uav_utils::ypr_to_R(ypr);
            }

            inline void setUAV2Ypr(const Eigen::Vector3d &ypr)
            {
                state_.R_2 = uav_utils::ypr_to_R(ypr);
            }

            inline void setUAV1Rpm(const Eigen::Vector4d &rpm) 
            {
                state_.motor_rpm_1 = rpm;
            }

            inline void setUAV2Rpm(const Eigen::Vector4d &rpm) 
            {
                state_.motor_rpm_2 = rpm;
            }

            inline void setLoadqr(const Eigen::Vector3d &qr)
            {
                state_.qr = qr;
            }

            inline void setUAV1ql(const Eigen::Vector3d &ql)
            {
                state_.ql_1 = ql;
            }

            inline void setUAV2ql(const Eigen::Vector3d &ql)
            {
                state_.ql_2 = ql;
            }

            // get values of state
            inline double getGrav() const 
            {
                return config_.g * config_.mass_q;
            }

            inline Eigen::Vector3d getUAV1Pos() const 
            {
                return state_.x_1;
            }

            inline Eigen::Vector3d getUAV1Vel() const 
            {
                return state_.v_1;
            }

            inline Eigen::Vector3d getUAV1Acc() const 
            {
                return (state_.v_1 - last_state.v_1)/last_dt;
            }

            inline Eigen::Vector3d getUAV2Pos() const 
            {
                return state_.x_2;
            }

            inline Eigen::Vector3d getUAV2Vel() const 
            {
                return state_.v_2;
            }

            inline Eigen::Vector3d getUAV2Acc() const 
            {
                return (state_.v_2 - last_state.v_2)/last_dt;
            }

            inline Eigen::Vector3d getLoadPos() const 
            {
                return state_.xr;
            }

            inline Eigen::Vector3d getLoadVel() const 
            {
                return state_.vr;
            }

            inline Eigen::Vector3d getLoadAcc() const 
            {
                return (state_.vr - last_state.vr)/last_dt;
            }

            inline Eigen::Quaterniond getUAV1Quat() const 
            {
                return Eigen::Quaterniond(state_.R_1);
            }

            inline Eigen::Vector3d getUAV1Omega() const 
            {
                return state_.omega_1;
            }

            inline Eigen::Quaterniond getUAV2Quat() const 
            {
                return Eigen::Quaterniond(state_.R_2);
            }

            inline Eigen::Vector3d getUAV2Omega() const 
            {
                return state_.omega_2;
            }

            inline Eigen::Vector4d getUAV1Rpm() const 
            {
                return state_.motor_rpm_1 ;
            }

            inline Eigen::Vector4d getUAV2Rpm() const 
            {
                return state_.motor_rpm_2 ;
            }

            inline Eigen::Vector3d getUAV1q() const 
            {
                return state_.ql_1;
            }

            inline Eigen::Vector3d getUAV2q() const 
            {
                return state_.ql_2;
            }

            inline Eigen::Vector3d getUAV1dq() const 
            {
                return state_.dql_1;
            }

            inline Eigen::Vector3d getUAV2dq() const 
            {
                return state_.dql_2;
            }

            inline Eigen::Vector3d getUAV1ddq() const 
            {
                return (state_.dql_1 - last_state.dql_1) / last_dt;
            }

            inline Eigen::Vector3d getUAV2ddq() const 
            {
                 return (state_.dql_2 - last_state.dql_2) / last_dt;
            }

            inline Eigen::Vector3d getLoadq() const 
            {
                return state_.qr;
            }

            inline Eigen::Vector3d getLoaddq() const 
            {
                return state_.dqr;
            }

            inline Eigen::Vector3d getUAV1attachPos() const 
            {
                return state_.xr - config.rode_length * state_.qr;
            }

            inline Eigen::Vector3d getUAV2attachPos() const 
            {
                return state_.xr + config.rode_length * state_.qr;
            }
    };
};