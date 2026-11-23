#pragma once
#include "so3_quadrotor/geometry_utils.hpp"
#include <iostream>
#include "random"

const double kLengthError = 0.025;
namespace so3_quadrotor {
#define RED "\033[31m"    /* Red */
#define YELLOW "\033[33m" /* Yellow */
#define RESET "\033[0m"
// data types
struct Config {
  double          g;      // gravity
  double          mass;
  Eigen::Matrix3d J;      // Inertia
  double          kf;
  double          km;
  double          arm_length;
  double          motor_time_constant; // unit: sec
  double          max_rpm;
  double          min_rpm;
  double          mass_rod; //  load mass
  double          cable_length; //load length 
  double          rod_length; // 1/2 * rod length
  Eigen::Matrix3d J_rod;
  double          kOmega[3]; 
  double          kdOmega[3];
};
struct Control {
  double rpm[4];
};
struct Cmd {
  double thrust;
  Eigen::Vector3d body_rates; 
};

class Quadrotor {
 private:
  // parameters
  Config config_;
  // state
  struct State {
    Eigen::Vector3d x_1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_1 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_1 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega_1 = Eigen::Vector3d::Zero();
    Eigen::Vector4d motor_rpm_1 = Eigen::Vector4d::Zero();
    Eigen::Vector3d x_2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_2 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_2 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega_2 = Eigen::Vector3d::Zero();
    Eigen::Vector4d motor_rpm_2 = Eigen::Vector4d::Zero();

    Eigen::Vector3d xr = Eigen::Vector3d(0, 0, 0); // position of rod center
    Eigen::Vector3d vr = Eigen::Vector3d::Zero();     // velocity of rod center
    Eigen::Vector3d qr = Eigen::Vector3d::(0, 1, 0);  // Attitude of rod in inertial frame
    Eigen::Vector3d dqr = Eigen::Vector3d::Zero();    // Attitude rate of rod in inertial frame
   

    Eigen::Vector3d ql_1 = Eigen::Vector3d(0,0,-1);//line vector
    Eigen::Vector3d dql_1 = Eigen::Vector3d::Zero();//line vector rate
    Eigen::Vector3d ql_2 = Eigen::Vector3d(0,0,-1);//line vector
    Eigen::Vector3d dql_2 = Eigen::Vector3d::Zero();//line vector rate
  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    inline State operator+(const State &t) const {
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
      sum.dql_1 = ql_1 + t.dql_1;
      sum.ql_2 = ql_2 + t.ql_2;
      sum.dql_2 = ql_2 + t.dql_2;
      // sum.wl = wl + t.wl;     
      return sum;
    }
    inline State operator*(const double &t) const {
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
      mul.dql_1 = ql_1 * t;
      mul.ql_2 = ql_2 * t;
      mul.dql_2 = ql_2 * t;
      // mul.wl = wl * t;
      return mul;
    }
    inline State operator/(const double &t) const {
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
      mul.dql_1 = ql_1 / t;
      mul.ql_2 = ql_2 / t;
      mul.dql_2 = ql_2 / t;
      return mul;
    }
  } state_;
  Eigen::Vector4d  input_1 = Eigen::Vector4d::Zero();
  Eigen::Vector4d  input_2 = Eigen::Vector4d::Zero();

  Eigen::Vector3d  pl_accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d  pl_ang_accel = Eigen::Vector3d::Zero();
 public:
  std::default_random_engine generator;
  std::normal_distribution<double> distribution{0.0,1.0};
  const Config &config;
  const State &state;
  State last_state;
  double last_dt;
  Quadrotor(const Config &conf) : config_(conf), 
    config(config_), state(state_) {};
  ~Quadrotor() {};

  // Inputs are desired RPM for the motors
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
  inline void setInput_1(double u1, double u2, double u3, double u4) {
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
  inline void setInput_2(double u1, double u2, double u3, double u4) {
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
  inline State diff(const State &state) {
    State state_dot;
    // Re-orthonormalize R (polar decomposition)
    Eigen::LLT<Eigen::Matrix3d> llt_1(state.R_1.transpose() * state.R_1);
    Eigen::Matrix3d             P_1 = llt_1.matrixL();
    Eigen::Matrix3d             R_1 = state.R_1 * P_1.inverse();
    // Re-orthonormalize R (polar decomposition)
    Eigen::LLT<Eigen::Matrix3d> llt_2(state.R_2.transpose() * state.R_2);
    Eigen::Matrix3d             P_2 = llt_2.matrixL();
    Eigen::Matrix3d             R_2 = state.R_2 * P_2.inverse();

    
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

    Eigen::Matrix3d 

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
    Eigen::Vector3d vquadnorm_1 = state.v_1.normalized();
    Eigen::Vector3d vquadnorm_2 = state.v_2.normalized();
    // double resistanceload_1= 0.05 *                                        // C
    //                     3.14159265 * (config_.arm_length) * (config_.arm_length) * // S
    //                     state.vl.norm() * state.vl.norm();
    // Eigen::Vector3d vloadnorm = state.vl.normalized();

    // Eigen::Vector3d fl = Eigen::Vector3d(0,0.0,0) + random_force(0.0)-resistanceload *vloadnorm ;
    // Eigen::Vector3d fq = Eigen::Vector3d(0.0,0,0) + random_force(0.0)-resistancequad *vquadnorm;
    Eigen::Vector3d fl = Eigen::Vector3d(0,0.0,0) ;
    Eigen::Vector3d fq = Eigen::Vector3d(0.0,0,0) ;

    Eigen::Vector3d robot_attach_position_1 = state.xr - state.qr * config.rod_length;
    Eigen::Vector3d robot_attach_position_2 = state.xr + state.qr * config.rod_length;

    Eigen::Vector3d attach_centrifugal_accel_1 = omega_vee_1 * omega_vee_1 * state.qr * config.rod_length;
    Eigen::Vector3d attach_centrifugal_accel_2 = omega_vee_2 * omega_vee_2 * state.qr * config.rod_length;

    Eigen::Vector3d attach_accel_1 = state.

    double delta_1 = (state.x_1 - state.xr - state.qr * config.rod_length).norm() - config_.cable_length;
    double delta_2 = (state.x_2 - state.xr + state.qr * config.rod_length).norm() - config_.cable_length;
    
    if ( delta_1 > -kLengthError && delta_1 < kLengthError)
    {
      state_dot.x_1 = state.v_1;
      state_dot.v_1 = ( ) 
    }

    if (delta_1 <= -kLengthError ) //Nontaut dynamics
    {
      
      state_dot.x_1 = state.v_1;
      state_dot.v_1 = -Eigen::Vector3d(0, 0, config_.g) + thrust_1 * R_1.col(2) / config_.mass + fq/ config_.mass;
      state_dot.R_1 = R_1 * omega_vee_1;

      state_dot.omega_1 = config_.J.inverse() * (moments - state.omega_1.cross(config_.J * state.omega_1)); //the same
      state_dot.motor_rpm_1 = (input_1 - state.motor_rpm_1) / config_.motor_time_constant;

      // state_dot.wl = -state.ql.cross(thrust * R.col(2))/(config_.mass * config_.l_length);
    }
    else if (delta > -kLengthError && delta < kLengthError) //taut dynamics
    {
      
      Eigen::Vector3d norm_ql = state.ql.normalized();
      state_dot.x = state.v;
      state_dot.v = -Eigen::Vector3d(0, 0, config_.g) + (thrust * R.col(2)+fq) / config_.mass + (config_.mass_l*config_.l_length* state.dql.dot(state.dql) + norm_ql.dot(fl-config_.mass_l/config_.mass*(fq+thrust * R.col(2))))*norm_ql/(config_.mass+config_.mass_l);
      state_dot.R = R * omega_vee;

      state_dot.omega = config_.J.inverse() * (moments - state.omega.cross(config_.J * state.omega)); //the same
      state_dot.motor_rpm = (input_ - state.motor_rpm) / config_.motor_time_constant;
    }
    
    return state_dot;
  }
  // Runs the actual dynamics simulation with a time step of dt
  inline void step(const double &dt) {
    // Runge–Kutta
    last_state = state_;
    last_dt = dt; //for numerical differentiation
    State k1 = diff(state_);
    State k2 = diff(state_+k1*dt/2);
    State k3 = diff(state_+k2*dt/2);
    State k4 = diff(state_+k3*dt);
    state_ = state_ + (k1+k2*2+k3*2+k4) * dt/6;
    
    state_.ql.normalize();
    double delta = (state_.x - state_.xl).norm() - config_.l_length;
    if (delta > -kLengthError && delta < kLengthError) //taut dynamics
    {
      state_.x = state_.xl - config_.l_length* state_.ql;
      state_.v = state_.vl - config_.l_length* state_.dql;
    }
    else if (delta >= kLengthError)
    {
      std::cout<<RED<<"DYNAMIC ERROR: Length is too long"<<RESET<<std::endl;
      state_.dql = Eigen::Vector3d::Zero();
      state_.x = state_.xl - config_.l_length* state_.ql;
      state_.v = state_.vl - config_.l_length* state_.dql;
    }
  }
  // get control from cmd
  inline Control getControl(const Cmd& cmd) {
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
    Eigen::Matrix3f J = config_.J.cast<float>();
    float     I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                          { J(1, 0), J(1, 1), J(1, 2) },
                          { J(2, 0), J(2, 1), J(2, 2) } };
    // rotation, may use external yaw
    Eigen::Vector3d ypr = uav_utils::R_to_ypr(state_.R);
    // if (cmd.use_external_yaw) {
    //   ypr[0] = cmd.current_yaw;
    // }
    Eigen::Matrix3d R; 
    R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
    // float R11 = R(0, 0);
    // float R12 = R(0, 1);
    // float R13 = R(0, 2);
    // float R21 = R(1, 0);
    // float R22 = R(1, 1);
    // float R23 = R(1, 2);
    // float R31 = R(2, 0);
    // float R32 = R(2, 1);
    // float R33 = R(2, 2);
    float Om1 = state_.omega(0);
    float Om2 = state_.omega(1);
    float Om3 = state_.omega(2);

    double rpm1 = state_.motor_rpm(0);
    double rpm2 = state_.motor_rpm(1);
    double rpm3 = state_.motor_rpm(2);
    double rpm4 = state_.motor_rpm(3);
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
  inline void setPos(const Eigen::Vector3d &pos) {
    state_.xl = pos;
  }
  inline void setQuadPos(const Eigen::Vector3d &pos) {
    state_.x = pos;
  }
  inline void setYpr(const Eigen::Vector3d &ypr) {
    state_.R = uav_utils::ypr_to_R(ypr);
  }
  inline void setRpm(const Eigen::Vector4d &rpm) {
    state_.motor_rpm = rpm;
  }
  inline void setq(const Eigen::Vector3d &q) {
    state_.ql = q;
  }
  // get values of state
  inline double getGrav() const {
    return config_.g * config_.mass;
  }
  inline Eigen::Vector3d getq() const {
    return state_.ql;
  }
  inline Eigen::Vector3d getPos() const {
    return state_.x;
  }
  inline Eigen::Vector3d getVel() const {
    return state_.v;
  }
  inline Eigen::Vector3d getLoadPos() const {
    return state_.xl ;
  }
    inline Eigen::Vector3d getLoadVel() const {
    return state_.vl ;
  }
  inline Eigen::Vector3d getLoadAcc() const {
    return (state_.vl - last_state.vl)/last_dt;
    // return (state_.xl - last_state.xl)/last_dt;
  }
  inline Eigen::Vector3d getLoadq() const {
    return state_.ql ;
  }
    inline Eigen::Vector3d getLoaddq() const {
    return state_.dql ;
  }
  inline Eigen::Vector4d getRpm() const {
    return state_.motor_rpm ;
  }
  inline Eigen::Vector3d getLoadddq() const {
    return (state_.dql - last_state.dql)/last_dt;
  }
  inline Eigen::Vector3d getQuadAcc() const {
    // Eigen::Vector3d acc = getLoadAcc();
    // Eigen::Vector3d ddq = getLoadddq();
    // return acc - config.l_length*ddq;
    return (state_.v - last_state.v)/last_dt;
  }
  inline Eigen::Quaterniond getQuat() const {
    return Eigen::Quaterniond(state_.R);
  }
  inline Eigen::Vector3d getOmega() const {
    return state_.omega;
  }

};

} // namespace so3_quadrotor
