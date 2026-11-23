#include "mpc_controller.h"
#include "nav_msgs/Odometry.h"
#include "ctime"

namespace PayloadMPC
{
  MpcController::MpcController(MpcParams &params) : params_(params), mpc_time_step_(params_.step_T_)
  {
    auto & q_gain_ = params_.q_gain_;
    auto & r_gain_ = params_.r_gain_;
    mpc_wrapper_.setDynamicParams(params_.dyn_params_.mass_q, params_.dyn_params_.mass_l);
    Eigen::Matrix<real_t, kCostSize, kCostSize> Q = (Eigen::Matrix<real_t, kCostSize, 1>() << q_gain_.Q_pos_xy, q_gain_.Q_pos_xy, q_gain_.Q_pos_z,
                                                      q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_yaw,
                                                      q_gain_.Q_velocity, q_gain_.Q_velocity, q_gain_.Q_velocity,
                                                      q_gain_.Q_pos_xy, q_gain_.Q_pos_xy, q_gain_.Q_pos_z,
                                                      q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_yaw,
                                                      q_gain_.Q_velocity, q_gain_.Q_velocity, q_gain_.Q_velocity,

                                                      q_gain_.Q_payload_xy, q_gain_.Q_payload_xy, q_gain_.Q_payload_z,
                                                      q_gain_.Q_payload_velocity, q_gain_.Q_payload_velocity, q_gain_.Q_payload_velocity,
                                                      q_gain_.Q_payload_r, q_gain_.Q_payload_r, q_gain_.Q_payload_r,
                                                      q_gain_.Q_payload_dr, q_gain_.Q_payload_dr, q_gain_.Q_payload_dr,

                                                      q_gain_.Q_cable, q_gain_.Q_cable, q_gain_.Q_cable,
                                                      q_gain_.Q_dcable, q_gain_.Q_dcable, q_gain_.Q_dcable,
                                                      q_gain_.Q_cable, q_gain_.Q_cable, q_gain_.Q_cable,
                                                      q_gain_.Q_dcable, q_gain_.Q_dcable, q_gain_.Q_dcable)
                                                        .finished()
                                                        .asDiagonal();
    Eigen::Matrix<real_t, kInputSize, kInputSize> R = (Eigen::Matrix<real_t, kInputSize, 1>() << r_gain_.R_thrust, r_gain_.R_pitchroll, r_gain_.R_pitchroll, r_gain_.R_yaw, r_gain_.R_thrust, r_gain_.R_pitchroll, r_gain_.R_pitchroll, r_gain_.R_yaw).finished().asDiagonal();

    Eigen::Matrix<real_t, kStateSize, 1> initial_state = (Eigen::Matrix<real_t, kStateSize, 1>() << 0.0, -0.4, 0.6,
                                                                                                    1.0, 0.0, 0.0, 0.0,
                                                                                                    0.0, 0.0, 0.0,
                                                                                                    0.0,  0.4, 0.6,
                                                                                                    1.0, 0.0, 0.0, 0.0,
                                                                                                    0.0, 0.0, 0.0,
                                                                                                    0.0, 0.0, 0.0,
                                                                                                    0.0, 0.0, 0.0,
                                                                                                    0.0, 1.0, 0.0,
                                                                                                    0.0, 0.0, 0.0,
                                                                                                    0.0, 0.0, -1.0,
                                                                                                    0.0, 0.0, 0.0,
                                                                                                    0.0, 0.0, -1.0,
                                                                                                    0.0, 0.0, 0.0)
                                                                                                      .finished();
    //std::cout << initial_state.size() << std::endl;
    reference_states_ = initial_state.replicate(1,kSamples+1);
    Eigen::Matrix<real_t, kInputSize, 1> initial_input = (Eigen::Matrix<real_t, kInputSize, 1>() << ( 0.5 * params_.dyn_params_.mass_l + params_.dyn_params_.mass_q) * params_.gravity_, 0, 0, 0, ( 0.5 * params_.dyn_params_.mass_l + params_.dyn_params_.mass_q) * params_.gravity_, 0, 0, 0 ).finished();

    hover_input_ = initial_input.replicate(1, kSamples + 1);
    reference_inputs_ = hover_input_;

    mpc_wrapper_.initialize(Q, R, initial_state, initial_input, params_.state_cost_exponential_, params_.input_cost_exponential_);
    mpc_wrapper_.setExternalForce(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    mpc_wrapper_.setLimits(
        params_.min_thrust_, params_.max_thrust_,
        params_.max_bodyrate_xy_, params_.max_bodyrate_z_);

    payload_flat_.reset(params_.dyn_params_.mass_q, params_.dyn_params_.mass_l, params_.dyn_params_.l_length_, params_.dyn_params_.r_length_, params_.gravity_);
    // first_traj_received_ = false;
    solve_from_scratch_ = false;
    timing_feedback_ = 0;
    timing_preparation_ = 0;
    preparation_thread_ = std::thread(&MpcWrapper::prepare, mpc_wrapper_);
    
  } 

  void MpcController::execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_states,
                              const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_inputs,
                              const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                              Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                              Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs)
  {
    const clock_t start = clock();

    preparation_thread_.join(); // waiting the preparation_thread_ finished

    // Get the feedback from MPC.
   
    mpc_wrapper_.setTrajectory(reference_states, reference_inputs);
    
    //std::cout << estimated_state << std::endl;
    if (solve_from_scratch_)
    {
      ROS_INFO("Solving MPC with hover as initial guess.");
      mpc_wrapper_.solve(estimated_state);
      solve_from_scratch_ = false;
    }
    else
    {
      constexpr bool do_preparation_step(false); // the preparation step has been done by another thread
      mpc_wrapper_.update(estimated_state, do_preparation_step);
    }

    mpc_wrapper_.getStates(predicted_states);
    mpc_wrapper_.getInputs(control_inputs);
    //std::cout << control_inputs.size() << std::endl;
    // Start a thread to prepare for the next execution.
    preparation_thread_ = std::thread(&MpcController::preparationThread, this);

    // Timing
    const clock_t end = clock();
    timing_feedback_ = 0.9*timing_feedback_ + 0.1* double(end - start) / CLOCKS_PER_SEC;
    if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.2f ms  |  Total: %1.2f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);
  }

  void MpcController::execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                              Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                              Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs)
  {
    //std::cout << estimated_state.col(0) << std::endl;
    execMPC(reference_states_, reference_inputs_, estimated_state, predicted_states, control_inputs);
  }
  // drone pos
  void MpcController::setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &payload_position, const double yaw)
  {
    real_t psi_1 = yaw;
    real_t psi_2 = yaw;

    Eigen::Vector3d payload_r(cos(yaw + M_PI / 2 ), sin(yaw + M_PI / 2), 0);
    Eigen::Vector3d quad_pos_1 = (payload_position - params_.dyn_params_.r_length_ * payload_r + Eigen::Vector3d(0,0,params_.dyn_params_.l_length_));
    Eigen::Vector3d quad_pos_2 = (payload_position + params_.dyn_params_.r_length_ * payload_r + Eigen::Vector3d(0,0,params_.dyn_params_.l_length_));

    
    Eigen::Vector3d local_fq = fq_1_;
    Eigen::Vector3d local_fl = fl_1_;
    if (params_.force_estimator_param_.USE_CONSTANT_MOMENT == 2)
    {
      local_fq = fq_1_.cross(Eigen::Vector3d(0,0,-1));
    }
    else if (params_.force_estimator_param_.USE_CONSTANT_MOMENT == 1)
    {
      local_fl = fl_1_.cross(Eigen::Vector3d(0,0,-1));
    }
    Eigen::Quaterniond quad_q_1;
    Eigen::Quaterniond quad_q_2;
    double thr_1, thr_2;
    Eigen::Vector3d omg_1, omg_2;
    payload_flat_.forward_rot(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), local_fq, local_fl, psi_1, 0, quad_q_1, thr_1, omg_1);
    payload_flat_.forward_rot(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), local_fq, local_fl, psi_2, 0, quad_q_2, thr_2, omg_2);
  
    Eigen::Quaternion<real_t> q_1 = quad_q_1.cast<real_t>();
    Eigen::Quaternion<real_t> q_2 = quad_q_2.cast<real_t>();

    // std::cout << payload_position.x() << " " <<  payload_position.y() << " " << payload_position.z() << std::endl;
    Eigen::Matrix<real_t, kStateSize, 1> reference_state;
    q_1.normalize();
    q_2.normalize();
    reference_state << quad_pos_1.x(), quad_pos_1.y(), quad_pos_1.z(),
                       q_1.w(), q_1.x(), q_1.y(), q_1.z(),
                       0.0, 0.0, 0.0,
                       quad_pos_2.x(), quad_pos_2.y(), quad_pos_2.z(),
                       q_2.w(), q_2.x(), q_2.y(), q_2.z(),
                       0.0, 0.0, 0.0,
                       payload_position.x(), payload_position.y(), payload_position.z(),
                       0.0, 0.0, 0.0,
                       payload_r.x(), payload_r.y(), payload_r.z(),
                       0.0, 0.0, 0.0,
                       0.0, 0.0, -1.0,
                       0.0, 0.0, 0.0,
                       0.0, 0.0, -1.0,
                       0.0, 0.0, 0.0;
    
    Eigen::Matrix<real_t, kInputSize, 1> hover_in = hover_input_.col(0);
    hover_in(kThrust_1,0)=thr_1;
    hover_in(kThrust_2,0)=thr_2;
    
    
    reference_states_ = reference_state.replicate(1, kSamples + 1);
    reference_inputs_ = hover_in.replicate(1, kSamples + 1);
  }

  // void MpcController::setTrajectoyReference_1(Eigen::Vector3d load_pos, double tstart, double start_yaw)
  // {

  // }

  void MpcController::setTrajectoyReference(Trajectory &traj, double tstart, double start_yaw_1, double start_yaw_2)
  {
    const double t_step = mpc_time_step_;
    double t_all = traj.getTotalDuration() - 1.0e-3;
    double t = tstart;
    double yaw_1, yaw_dot_1;
    double yaw_2, yaw_dot_2;
    Eigen::Vector5d pos, vel, acc, jerk, snap, crackle;
    Eigen::Vector3d pos_quad_1, vel_quad_1, acc_quad_1, jerk_quad_1, acc_attach_1, jerk_attach_1, p_1, pd_1;
    Eigen::Vector3d pos_quad_2, vel_quad_2, acc_quad_2, jerk_quad_2, acc_attach_2, jerk_attach_2, p_2, pd_2;
    
    Eigen::Quaterniond quat_1;
    Eigen::Quaterniond quat_2;
    double thr_1;
    double thr_2;
    Eigen::Vector3d omg_1;
    Eigen::Vector3d omg_2;
    //last_yaw_ = start_yaw;  // must reset the last_yaw_ 

    for (int i = 0; i < (kSamples + 1); i++)
    {
      Eigen::MatrixXd pvajs;
      if (t > t_all)
      {
        // if t is larger than the total time, use the last point
        // TODO : Consider 2 trajectories
        t = t_all;
        pvajs = traj.getPVAJSC(t);
        //std::cout << pvajs.size() << std::endl;
        pos = pvajs.col(0);
        vel = Eigen::Vector5d::Zero();
        acc =  Eigen::Vector5d::Zero();
        jerk = Eigen::Vector5d::Zero();
        snap = Eigen::Vector5d::Zero();
        crackle =  Eigen::Vector5d::Zero();
      }
      else 
      {
        pvajs = traj.getPVAJSC(t);
        pos = pvajs.col(0);
        vel = pvajs.col(1);
        acc = pvajs.col(2);
        jerk = pvajs.col(3);
        snap = pvajs.col(4);
        crackle = pvajs.col(5);
      }

      payload_flat_.forward_p(pos, vel, acc, jerk, snap, crackle, pos_quad_1, vel_quad_1, acc_quad_1, jerk_quad_1,acc_attach_1, jerk_attach_1,p_1, pd_1,
                                                                  pos_quad_2, vel_quad_2, acc_quad_2, jerk_quad_2,acc_attach_2, jerk_attach_2,p_2, pd_2 );
      if (params_.use_fix_yaw_)
      {
        yaw_1 = start_yaw_1;
        yaw_dot_1 = 0.0;
        yaw_2 = start_yaw_2;
        yaw_dot_2 = 0.0;
      }
      else 
      {
        if (i == 0) // reset last_yaw_ from the first point 
        {
          if (t - t_step <= 0.0)
          {
            if (fabs(vel_quad_1(0))+fabs(vel_quad_1(1))>0.1)
            {
              last_yaw_1 = atan2(vel_quad_1(1), vel_quad_1(0));
              last_yaw_dot_1 = 0.0;
            }
            else
            {
              last_yaw_1 = start_yaw_1;
              last_yaw_dot_1 = 0.0;
            }
            if (fabs(vel_quad_2(0))+fabs(vel_quad_2(1))>0.1)
            {
              last_yaw_2 = atan2(vel_quad_2(1), vel_quad_2(0));
              last_yaw_dot_2 = 0.0;
            }
            else
            {
              last_yaw_2 = start_yaw_2;
              last_yaw_dot_2 = 0.0;
            }
          }
          else // Get yaw in last step
          {
            Eigen::MatrixXd last_pvajs = traj.getPVAJSC(t-t_step);
            Eigen::Vector5d last_pos = last_pvajs.col(0);
            Eigen::Vector5d last_vel = last_pvajs.col(1);
            Eigen::Vector5d last_acc = last_pvajs.col(2);
            Eigen::Vector5d last_jerk = last_pvajs.col(3);
            Eigen::Vector5d last_snap = last_pvajs.col(4);
            Eigen::Vector5d last_crackle = last_pvajs.col(5); 
            Eigen::Vector3d last_pos_quad_1, last_vel_quad_1, last_acc_quad_1, last_jerk_quad_1,last_acc_attach_1, last_jerk_attach_1, last_p_1, last_pd_1;
            Eigen::Vector3d last_pos_quad_2, last_vel_quad_2, last_acc_quad_2, last_jerk_quad_2,last_acc_attach_2, last_jerk_attach_2, last_p_2, last_pd_2;
      
            payload_flat_.forward_p(last_pos, last_vel, last_acc, last_jerk, last_snap, last_crackle, last_pos_quad_1, last_vel_quad_1, last_acc_quad_1, last_jerk_quad_1, last_acc_attach_1, last_jerk_attach_1, last_p_1, last_pd_1,
                                                                                                      last_pos_quad_2, last_vel_quad_2, last_acc_quad_2, last_jerk_quad_2, last_acc_attach_2, last_jerk_attach_2, last_p_2, last_pd_2);
     
          
            if (fabs(last_vel_quad_1(0))+fabs(last_vel_quad_1(1))>0.1)
            {
              last_yaw_1 = atan2(last_vel_quad_1(1), last_vel_quad_1(0));
              last_yaw_dot_1 = 0.0;
            }
            else
            {
              last_yaw_1 = start_yaw_1;
              last_yaw_dot_1 = 0.0;
            }
            if (fabs(last_vel_quad_2(0))+fabs(last_vel_quad_2(1))>0.1)
            {
              last_yaw_2 = atan2(last_vel_quad_2(1), last_vel_quad_2(0));
              last_yaw_dot_2 = 0.0;
            }
            else
            {
              last_yaw_2 = start_yaw_2;
              last_yaw_dot_2 = 0.0;
            }
          }
        }
        calculate_yaw_1(vel_quad_1, t_step, yaw_1, yaw_dot_1);
        calculate_yaw_2(vel_quad_2, t_step, yaw_2, yaw_dot_2);
      }
      Eigen::Vector3d local_fq = fq_1_;
      Eigen::Vector3d local_fl = fl_1_;
      if (params_.force_estimator_param_.USE_CONSTANT_MOMENT == 2)
      {
        local_fq = fq_1_;
      }
      else if (params_.force_estimator_param_.USE_CONSTANT_MOMENT == 1)
      {
        local_fl = fl_1_;
      }
      
      payload_flat_.forward_rot(acc_attach_1,jerk_attach_1, acc_quad_1, jerk_quad_1,local_fq,local_fl,yaw_1,yaw_dot_1,quat_1,thr_1,omg_1);
      payload_flat_.forward_rot(acc_attach_2,jerk_attach_2, acc_quad_2, jerk_quad_2,local_fq,local_fl,yaw_2,yaw_dot_2,quat_2,thr_2,omg_2);
      //std::cout << quat.w() << "," <<  quat.x()  << "," << quat.y()  << "," << quat.z() << std::endl;
      Eigen::Matrix<real_t, kStateSize, 1> reference_state;
      reference_state << pos_quad_1.x(), pos_quad_1.y(), pos_quad_1.z(),
          quat_1.w(), quat_1.x(), quat_1.y(), quat_1.z(),
          vel_quad_1.x(), vel_quad_1.y(), vel_quad_1.z(),
          pos_quad_2.x(), pos_quad_2.y(), pos_quad_2.z(),
          quat_2.w(), quat_2.x(), quat_2.y(), quat_2.z(),
          vel_quad_2.x(), vel_quad_2.y(), vel_quad_2.z(),
          pos[0], pos[1], pos[2],
          vel[0], vel[1], vel[2],
          cos(pos[3])*cos(pos[4]), cos(pos[3])*sin(pos[4]), sin(pos[3]),
          -std::sin(pos[3])*std::cos(pos[4])*vel[3] - std::cos(pos[3])*std::sin(pos[4])*vel[4], -std::sin(pos[3])*std::sin(pos[4])*vel[3] + std::cos(pos[3])*std::cos(pos[4])*vel[4], std::cos(pos[3])*vel[3],
          p_1.x(), p_1.y(), p_1.z(),
          pd_1.x(), pd_1.y(), pd_1.z(),
          p_2.x(), p_2.y(), p_2.z(),
          pd_2.x(), pd_2.y(), pd_2.z();
      reference_states_.col(i) = reference_state;
      
      Eigen::Matrix<real_t, kInputSize, 1> reference_input;
      // reference_input << thr, omg.x(), omg.y(), omg.z();
      reference_input << thr_1, omg_1.x(), omg_1.y(), 0.0, thr_2, omg_2.x(), omg_2.y(), 0.0;
      reference_inputs_.col(i) = reference_input;
      t += t_step;
    }
  }


  void MpcController::preparationThread()
  {
    const clock_t start = clock();

    mpc_wrapper_.prepare();

    // Timing
    const clock_t end = clock();
    timing_preparation_ = timing_preparation_*0.9 + 0.1* double(end - start) / CLOCKS_PER_SEC;
  } 

  double MpcController::angle_limit(double ang)
  {
    while (ang > M_PI)
    {
      ang -= 2.0 * M_PI;
    }
    while (ang <= -M_PI)
    {
      ang += 2.0 * M_PI;
    }
    return ang;
  }
  double MpcController::angle_diff(double a, double b)
  {
    double d1, d2;
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
      d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
      return(d1);
    else
      return(d2);
  }
  void MpcController::calculate_yaw_2(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot)
  {
    const double YAW_DOT_MAX_PER_SEC = params_.max_bodyrate_z_;

    double yaw_temp;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;

    // tangent line
    if ((fabs(vel(1)) + fabs(vel(0))) < 0.1)
    {
      yaw_temp = last_yaw_2;
    }
    else
    {
      yaw_temp = atan2(vel(1), vel(0));     
    }
    double yaw_diff = angle_diff(yaw_temp, last_yaw_2);
    
    if (yaw_diff > max_yaw_change )
    {
      yaw_diff = max_yaw_change;
      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else if(yaw_diff < -max_yaw_change)
    {
      yaw_diff = -max_yaw_change;
      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yawdot = yaw_diff / dt;
    }
    
    yaw = last_yaw_2 + yaw_diff;

    // std::cout<<"last_yaw: "<< last_yaw_ << "yaw: " << yaw << " yawdot: " << yawdot << std::endl;
    
    // std::cout<< "dt: " << dt << " max_yaw_change: " << max_yaw_change << " vel: " << vel << " last_yaw_: " << last_yaw_ << " yaw: " << yaw << " yawdot: " << yawdot << std::endl;

    last_yaw_2 = yaw;
    last_yaw_dot_2 = yawdot;
  }


  void MpcController::calculate_yaw_1(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot)
  {
    const double YAW_DOT_MAX_PER_SEC = params_.max_bodyrate_z_;

    double yaw_temp;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;

    // tangent line
    if ((fabs(vel(1)) + fabs(vel(0))) < 0.1)
    {
      yaw_temp = last_yaw_1;
    }
    else
    {
      yaw_temp = atan2(vel(1), vel(0));     
    }
    double yaw_diff = angle_diff(yaw_temp, last_yaw_1);
    
    if (yaw_diff > max_yaw_change )
    {
      yaw_diff = max_yaw_change;
      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else if(yaw_diff < -max_yaw_change)
    {
      yaw_diff = -max_yaw_change;
      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yawdot = yaw_diff / dt;
    }
    
    yaw = last_yaw_1 + yaw_diff;

    // std::cout<<"last_yaw: "<< last_yaw_ << "yaw: " << yaw << " yawdot: " << yawdot << std::endl;
    
    // std::cout<< "dt: " << dt << " max_yaw_change: " << max_yaw_change << " vel: " << vel << " last_yaw_: " << last_yaw_ << " yaw: " << yaw << " yawdot: " << yawdot << std::endl;

    last_yaw_1 = yaw;
    last_yaw_dot_1 = yawdot;
  }

  double MpcController::convertThrust(const double& thrust, const double voltage)
  {
    double normalized_thrust;
    // double des_acc_norm = thrust;
    // This compensates for an acceleration component in thrust direction due
    // to the square of the body-horizontal velocity.
    // des_acc_norm -= param.rt_drag.k_thrust_horz * (pow(est_v.x(), 2) + pow(est_v.y(), 2));

    normalized_thrust = thrust / thrustscale_;
    timed_thrust_1.emplace(std::make_pair(ros::Time::now(), normalized_thrust));

    return normalized_thrust;
  }

  bool MpcController::estimateThrustModel(
      const Eigen::Vector3d &est_a,
      const Eigen::Quaterniond &quad_q,
      const Eigen::Vector3d &est_loadp,
      const Eigen::Vector3d &est_loaddp,
      const Eigen::Vector4d &rpm,
      const double voltage,
      const MpcParams &param)
  {
    ros::Time t_now = ros::Time::now();

    while (timed_thrust_1.size() >= 1)
    {
      // Choose data before 35-45ms ago 
      std::pair<ros::Time, double> t_t = timed_thrust_1.front();
      double time_passed = (t_now - t_t.first).toSec();
      if (time_passed > 0.045) // 45ms
      {
        // printf("continue, time_passed=%f\n", time_passed);
        timed_thrust_1.pop();
        continue;
      }
      if (time_passed < 0.035) // 35ms
      {
        // printf("skip, time_passed=%f\n", time_passed);
        return false;
      }
      /***********************************************************/
      /* Recursive least squares algorithm with vanishing memory */
      /***********************************************************/
      double normlized_thr = t_t.second;
      timed_thrust_1.pop();
      if (param.thr_map_.accurate_thrust_model == 0)
      {
        resetThrustMapping(); // Or skip
      }
      else if(param.thr_map_.accurate_thrust_model == 1 )
      {
        double thr_fb = rotor2thrust(param.force_estimator_param_.sqrt_kf,rpm);

        /***********************************/
        /* Model: thr = thrustscale_ * normlized_thr */
        /***********************************/
          double gamma = 1 / (rho2 + normlized_thr * P * normlized_thr);
          double K = gamma * P * normlized_thr;
          double curr_thrustscale = thrustscale_ + K * (thr_fb - normlized_thr * thrustscale_);
          thrustscale_ = param.thr_map_.filter_factor*curr_thrustscale +(1- param.thr_map_.filter_factor)*thrustscale_;
          P = (1 - K * normlized_thr) * P / rho2;
          // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc, gamma, K, P);
          // fflush(stdout);
        const double hover_percentage = (param.gravity_ * (param.dyn_params_.mass_q))/ thrustscale_;
        if (hover_percentage > 0.8 || hover_percentage < 0.1)
        {
          ROS_ERROR("Estimated hover_percentage >0.8 or <0.1! Perhaps the accel vibration is too high!");
          thrustscale_ = hover_percentage > 0.8 ? param.gravity_ / 0.8 : thrustscale_;
          thrustscale_ = hover_percentage < 0.1 ? param.gravity_ / 0.1 : thrustscale_;
        }
        debug.hover_percentage = hover_percentage; // debug
        if (param.thr_map_.print_val)
        {
          ROS_WARN("hover_percentage = %f  thrustscale = %f", debug.hover_percentage,thrustscale_);
        }
      }
      else if (param.thr_map_.accurate_thrust_model == 2)
      {
        double thr_fb = acc2thrust(param.dyn_params_.mass_l,param.dyn_params_.mass_q,param.dyn_params_.l_length_,param.gravity_,
        est_a(2),quad_q,est_loadp,est_loaddp);

        /***********************************/
        /* Model: thr = thrustscale_ * normlized_thr */
        /***********************************/
          double gamma = 1 / (rho2 + normlized_thr * P * normlized_thr);
          double K = gamma * P * normlized_thr;
          double curr_thrustscale = thrustscale_ + K * (thr_fb - normlized_thr * thrustscale_);
          thrustscale_ = param.thr_map_.filter_factor*curr_thrustscale +(1- param.thr_map_.filter_factor)*thrustscale_;
          P = (1 - K * normlized_thr) * P / rho2;
          // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc, gamma, K, P);
          // fflush(stdout);
        const double hover_percentage = (param.gravity_ * (param.dyn_params_.mass_q))/ thrustscale_;
        if (hover_percentage > 0.8 || hover_percentage < 0.1)
        {
          ROS_ERROR("Estimated hover_percentage >0.8 or <0.1! Perhaps the accel vibration is too high!");
          thrustscale_ = hover_percentage > 0.8 ? param.gravity_ / 0.8 : thrustscale_;
          thrustscale_ = hover_percentage < 0.1 ? param.gravity_ / 0.1 : thrustscale_;
        }
        debug.hover_percentage = hover_percentage; // debug
        if (param.thr_map_.print_val)
        {
          ROS_WARN("hover_percentage = %f  thrustscale = %f", debug.hover_percentage,thrustscale_);
        }
      }

      return true; 
    }
    return false;
  }

  double inline MpcController::rotor2thrust(const double& sqrt_kf, const Eigen::Vector4d& rpm)
  {
    return (sqrt_kf * rpm).squaredNorm();
  }

  double inline MpcController::acc2thrust(const double& Ml, const double& Mq,const double& l_length, const double& g, const double& acc_z,
                              const Eigen::Quaterniond& quad, const Eigen::Vector3d& cable, const Eigen::Vector3d& dcable)

  {
    const double q_w = quad.w();
    const double q_x = quad.x();
    const double q_y = quad.y();
    const double q_z = quad.z();
    const double cable_x = cable.x();
    const double cable_y = cable.y();
    const double cable_z = cable.z();
    
    const double q_sqr = q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z;
    const double dcable_sqr = dcable.squaredNorm();//dcable_x^2 + dcable_y^2 + dcable_z^2;
    const double qwqxminusqyqz = 2*q_w*q_x - 2*q_y*q_z;
    const double qwqyminusqxqz = 2*q_w*q_y + 2*q_x*q_z;

    double des_T = -(Mq*(acc_z - g*(q_sqr) + (g*(Ml + Mq)*(q_sqr) - Ml*(g + (Mq*cable_z*l_length*(dcable_sqr))/(Ml + Mq))*(q_sqr) - (Ml*Mq*cable_x*l_length*(qwqyminusqxqz)*(dcable_sqr))/(Ml + Mq) + (Ml*Mq*cable_y*l_length*(qwqxminusqyqz)*(dcable_sqr))/(Ml + Mq))/Mq))/((Ml*cable_x*(qwqyminusqxqz)*(cable_z*(q_sqr) - cable_y*(qwqxminusqyqz) + cable_x*(qwqyminusqxqz)))/(Ml + Mq) - (Ml*cable_y*(qwqxminusqyqz)*(cable_z*(q_sqr) - cable_y*(qwqxminusqyqz) + cable_x*(qwqyminusqxqz)))/(Ml + Mq) + (Ml*cable_z*(cable_z*(q_sqr) - cable_y*(qwqxminusqyqz) + cable_x*(qwqyminusqxqz))*(q_sqr))/(Ml + Mq) - 1);

    return des_T; //accelration in body z axis(including g)
  }

  void MpcController::resetThrustMapping(void)
  {
    thrustscale_ = (params_.gravity_ * (params_.dyn_params_.mass_q)) / params_.thr_map_.hover_percentage;
    thr_scale_compensate = 1.0;
    P = 1e6;
  }
}