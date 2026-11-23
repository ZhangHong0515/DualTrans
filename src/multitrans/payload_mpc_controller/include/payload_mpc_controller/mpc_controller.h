#pragma once 

#include "thread"

#include "Eigen/Eigen"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <flatness.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>

#include <queue>
#include "mpc_wrapper.h"
#include "mpc_params.h"
#include "polynomial_trajectory.h"


namespace PayloadMPC
{
  enum STATE
  {
    kPosX_1 = 0,
    kPosY_1 = 1,
    kPosZ_1 = 2,
    kOriW_1 = 3,
    kOriX_1 = 4,
    kOriY_1 = 5,
    kOriZ_1 = 6,
    kVelX_1 = 7,
    kVelY_1 = 8,
    kVelZ_1 = 9,
    kPosX_2 = 10,
    kPosY_2 = 11,
    kPosZ_2 = 12,
    kOriW_2 = 13,
    kOriX_2 = 14,
    kOriY_2 = 15,
    kOriZ_2 = 16,
    kVelX_2 = 17,
    kVelY_2 = 18,
    kVelZ_2 = 19,
    kPayloadX = 20,
    kPayloadY = 21,
    kPayloadZ = 22,
    kPayloadVx = 23,
    kPayloadVy = 24,
    kPayloadVz = 25,
    kPayloadRx = 26,
    kPayloadRy = 27,
    kPayloadRz = 28,
    kPayloadDrx = 29,
    kPayloadDry = 30,
    kPayloadDrz = 31,
    kCableX_1 = 32,
    kCableY_1 = 33,
    kCableZ_1 = 34,
    kDcableX_1 = 35,
    kDcableY_1 = 36,
    kDcableZ_1 = 37,
    kCableX_2 = 38,
    kCableY_2 = 39,
    kCableZ_2 = 40,
    kDcableX_2 = 41,
    kDcableY_2 = 42,
    kDcableZ_2 = 43,
  };

  enum INPUT_BODYRATE
  {
    kThrust_1 = 0,
    kRateX_1 = 1,
    kRateY_1 = 2,
    kRateZ_1 = 3,
    kThrust_2 = 4,
    kRateX_2 = 5,
    kRateY_2 = 6,
    kRateZ_2 = 7,
    
  };

  struct TorquesAndThrust
  {
    Eigen::Vector3d body_torques;
    double collective_thrust;
  }; 

  class MpcController 
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      static_assert(kStateSize == 44,
                  "MpcController: Wrong model size. Number of states does not match.");
      static_assert(kInputSize == 8,
                    "MpcController: Wrong model size. Number of inputs does not match.");

      MpcController(MpcParams &params);

      void execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_state,
                 const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_input,
                 const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
      void execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                  Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                  Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
      void setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &payload_position_1, const double yaw_1);
      void setTrajectoyReference(Trajectory &traj, double tstart, double start_yaw_1, double start_yaw_2);
      double getTimeStep(){return mpc_time_step_;}
      void setDynamicParams(const real_t mass_q, const real_t mass_l)
        {mpc_wrapper_.setDynamicParams(mass_q, mass_l);}
      void setExternalForce(const Eigen::Ref<const Eigen::Vector3d>& fl_1, const Eigen::Ref<const Eigen::Vector3d>& fq_1,
                            const Eigen::Ref<const Eigen::Vector3d>& fl_2, const Eigen::Ref<const Eigen::Vector3d>& fq_2)
        {mpc_wrapper_.setExternalForce(fl_1, fq_1, fl_2, fq_2);fq_1_=fq_1; fl_1_=fl_1; fq_2_=fq_2; fl_2_=fl_2;}
      // Thrust to control
      std::queue<std::pair<ros::Time, double>> timed_thrust_1;
      std::queue<std::pair<ros::Time, double>> timed_thrust_2;
      
      double thr_scale_compensate;
      const double rho2 = 0.998; // do not change
      double thrustscale_;
      double P;
      quadrotor_msgs::Px4ctrlDebug debug;

      void resetThrustMapping(void);
      double convertThrust(const double& thrust, const double voltage);
      bool estimateThrustModel(
        const Eigen::Vector3d &est_a,
        const Eigen::Quaterniond &quad_q,
        const Eigen::Vector3d &est_loadp,
        const Eigen::Vector3d &est_loaddp,
        const Eigen::Vector4d &rpm,
        const double voltage,
        const MpcParams &param);
      double AccurateThrustAccMapping(
      const double des_acc_z,
      double voltage,
      const MpcParams &param) const;

    private:
      double last_yaw_1;
      double last_yaw_dot_1;
      double last_yaw_2;
      double last_yaw_dot_2;
      // Internal helper functions.

      // void offCallback(const std_msgs::Empty::ConstPtr& msg);
      void calculate_yaw_1(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot);
      void calculate_yaw_2(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot);
      double inline rotor2thrust(const double& sqrt_kf, const Eigen::Vector4d& rpm);
      double inline acc2thrust(const double& Ml, const double& Mq,const double& l_length, const double& g, const double& acc_z,
                                const Eigen::Quaterniond& quad, const Eigen::Vector3d& cable, const Eigen::Vector3d& dcable);
      void preparationThread();
      double angle_limit(double ang);
      double angle_diff(double a, double b);

      // Parameters
      MpcParams& params_;

      Flatness payload_flat_;
      Eigen::Vector3d fq_1_;
      Eigen::Vector3d fl_1_;
      Eigen::Vector3d fq_2_;
      Eigen::Vector3d fl_2_;
      // MPC
      MpcWrapper mpc_wrapper_;
      const double mpc_time_step_;

      // Preparation Thread
      std::thread preparation_thread_;

      // Variables
      real_t timing_feedback_, timing_preparation_;
      bool solve_from_scratch_;
      
    public:
      Eigen::Matrix<real_t, kStateSize, kSamples + 1> reference_states_;
      Eigen::Matrix<real_t, kInputSize, kSamples + 1> reference_inputs_;
      Eigen::Matrix<real_t, kInputSize, kSamples + 1> hover_input_;
  };
}