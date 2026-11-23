#include "optimizer/poly_traj_optimizer.h"
// using namespace std;

namespace payload_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);

    double final_cost;
    variable_num_ = 6 * (piece_num_ - 1) + 1;

    ros::Time t1, t2;

    Eigen::VectorXd q(variable_num_);

    Eigen::Map<Eigen::MatrixXd> pt(q.data(), 5, piece_num_ - 1);
    pt = initInnerPts;
    Eigen::Map<Eigen::VectorXd> Vt(q.data() + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-6;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.max_linesearch = 256;
    lbfgs_params.past = 5;

    /* trick : for real-time optimization */
    lbfgs_params.max_iterations = 1000; // 200 for replan

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;
    /* ---------- optimize ---------- */

    t1 = ros::Time::now();

    int result = lbfgs::lbfgs_optimize(
        q,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        lbfgs_params);

    // test collision
    bool occ = false;
    // occ = checkCollision();

    t2 = ros::Time::now();
    double time_ms = (t2 - t1).toSec() * 1000;

    printf("\033[32miter=%d,time(ms)=%5.3f, \n\033[0m", iter_num_, time_ms);
    // if ((result != lbfgs::LBFGS_CONVERGENCE) && (result != lbfgs::LBFGS_STOP))
    // {
    //   ROS_ERROR("The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    // }
    // else
    // {
    //   ROS_INFO("The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    // }
    optimal_points = cps_.points;

    if (occ)
      return false;
    else
      return true;
  }

  bool PolyTrajOptimizer::smoothedL1(const double &x,
                                     const double &mu,
                                     double &f,
                                     double &df)
  {
    if (x < 0.0)
    {
      f = 0;
      df = 0;
      return false;
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);

      // const double xdmu = x / mu;
      // f = 0.5*xdmu * x;
      // df = xdmu;
      return true;
    }
  }

  bool PolyTrajOptimizer::checkCollision(void)
  {
    // only check the 2/3 of trajectory
    bool occ = false;
    double dt = 0.01;
    poly_traj::Trajectory<7> traj = jerkOpt_.getTraj();
    double T_all = traj.getTotalDuration();
    int i_end = round(T_all / dt) * 3 / 5;
    double t = 0.05; // skip the start point

    for (int i = 5; i < i_end; i++)
    {
      occ = checkCollision(traj, t);
      t += dt;
      if (occ)
      {
        ROS_WARN("111drone is too close to the obstacle at relative time %f!", t);
        break;
      }
    }
    return occ;
  }

  bool PolyTrajOptimizer::checkCollision(poly_traj::Trajectory<7> &traj, double t)
  {
    Eigen::Vector5d pos = traj.getPos(t);
    Eigen::Vector5d vel = traj.getVel(t);
    Eigen::Vector5d acc = traj.getAcc(t);
    Eigen::Vector3d load_position = pos.head<3>();
    Eigen::Vector3d load_rq       = Eigen::Vector3d(cos(pos[3])*cos(pos[4]), cos(pos[3])*sin(pos[4]), sin(pos[3]));
    Eigen::Vector3d attach_pos_1 = load_position - r_length_ * load_rq;
    Eigen::Vector3d attach_pos_2 = load_position + r_length_ * load_rq;
    double theta = pos[3];
    double   phi = pos[4];
    // std::cout << "theta :" << theta << std::endl;
    // std::cout << "phi :" << phi << std::endl;
    
    // std::cout << load_rq << std::endl; 
    Eigen::Vector3d ddqr = Eigen::Vector3d(-std::cos(theta)*std::cos(phi)*vel[3]*vel[3] + std::sin(theta)*std::sin(phi)*vel[3]*vel[4] - std::sin(theta)*std::cos(phi)*acc[3] + std::sin(theta)*std::sin(phi)*vel[3]*vel[4] -std::cos(theta)*std::cos(phi)*vel[4]*vel[4]-std::cos(theta)*std::sin(phi)*acc[4],
                                                  -std::cos(theta)*std::sin(phi)*vel[3]*vel[3] - std::sin(theta)*std::cos(phi)*vel[3]*vel[4] - std::sin(theta)*std::sin(phi)*acc[3] - std::sin(theta)*std::cos(phi)*vel[3]*vel[4] -std::cos(theta)*std::sin(phi)*vel[4]*vel[4]+std::cos(theta)*std::cos(phi)*acc[4],
                                                  -std::sin(theta)*vel[3]*vel[3] + std::cos(theta)*acc[3]);

    Eigen::Vector3d attach_acc_1 = acc.head<3>() - r_length_ * ddqr;
    Eigen::Vector3d attach_acc_2 = acc.head<3>() + r_length_ * ddqr;
    Eigen::Vector3d temp_p = (attach_pos_2 - attach_pos_1).normalized();
    for (double l_dis = 0; l_dis <= 2 * r_length_; l_dis += sample_ball_distance_)
    {
      Eigen::Vector3d pos_on_line = attach_pos_1 + l_dis * temp_p; // the sample points on line
      // if(grid_map_->getInflateOccupancy(pos_on_line))
      if (grid_map_->getDistance(pos_on_line) < payload_size_)
      {
        ROS_WARN_STREAM("line distance  " << grid_map_->getDistance(pos_on_line) << " Line pos" << l_dis << "!");
        return true;
      }
    }
    Eigen::Vector3d norm_negTp_1 = attach_acc_1;
    norm_negTp_1(2) += grav_;
    norm_negTp_1.normalize();

    // check the points on line
    double check_length = L_length_ - drone_size_;
    for (double l_dis = payload_size_; l_dis <= check_length; l_dis += sample_ball_distance_)
    {
      Eigen::Vector3d pos_on_line = attach_pos_1 + l_dis * norm_negTp_1; // the sample points on line

      // if(grid_map_->getInflateOccupancy(pos_on_line))
      if (grid_map_->getDistance(pos_on_line) < payload_size_)
      {
        ROS_WARN_STREAM("line distance  " << grid_map_->getDistance(pos_on_line) << " Line pos" << l_dis << "!");
        return true;
      }
    }

    Eigen::Vector3d norm_negTp_2 = attach_acc_2;
    norm_negTp_2(2) += grav_;
    norm_negTp_2.normalize();

    // check the points on line
    check_length = L_length_ - drone_size_;
    for (double l_dis = payload_size_; l_dis <= check_length; l_dis += sample_ball_distance_)
    {
      Eigen::Vector3d pos_on_line = attach_pos_2 + l_dis * norm_negTp_2; // the sample points on line

      // if(grid_map_->getInflateOccupancy(pos_on_line))
      if (grid_map_->getDistance(pos_on_line) < payload_size_)
      {
        ROS_WARN_STREAM("line distance  " << grid_map_->getDistance(pos_on_line) << " Line pos" << l_dis << "!");
        return true;
      }
    }
    // check the drone
    Eigen::Vector3d posQ = attach_pos_1 + L_length_ * norm_negTp_1;
    if (grid_map_->getDistance(posQ) < drone_size_)
    {
      ROS_WARN_STREAM("drone distance  " << grid_map_->getDistance(posQ) << "!");
      return true;
    }
    posQ = attach_pos_2 + L_length_ * norm_negTp_2;
    if (grid_map_->getDistance(posQ) < drone_size_)
    {
      ROS_WARN_STREAM("drone distance  " << grid_map_->getDistance(posQ) << "!");
      return true;
    }
    return false;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *instance,
                                                 const Eigen::VectorXd &x,
                                                 Eigen::VectorXd &g)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);

    Eigen::Map<const Eigen::MatrixXd> P(x.data(), 5, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x.data() + (5 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data(), 5, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(g.data() + (5 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd cost_terms(6);

    opt->jerkOpt_.generate(P, T);

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAGradCost2CT(gradT, cost_terms, opt->cps_num_prePiece_); // Time int cost

    opt->jerkOpt_.getGrad2TP(gradT, gradP);

    // opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);
    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    opt->iter_num_ += 1;
    double costall = smoo_cost + cost_terms.sum() + time_cost;
    return costall;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const Eigen::VectorXd &x,
                                           const Eigen::VectorXd &g,
                                           const double fx,
                                           const double step,
                                           const int k,
                                           const int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {

    int N = gdT.size();
    Eigen::Vector5d pos, vel, acc, jer, snap, crakle;
    Eigen::Vector5d gradp, gradv, grada, gradj, grads;
    double costp, costv, costa, costd;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4, beta5;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::Matrix<double, 8, 5> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc, gradViolaSc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt, gradViolaSt;
    double omg;
    int i_dp = 0;
    costs.setZero();

    // int innerLoop;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 8, 5> &c = jerkOpt_.get_b().block<8, 5>(i * 8, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s3 * s3;
        s7 = s6 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;
        beta5(0) = 0.0, beta5(1) = 0.0, beta5(2) = 0.0, beta5(3) = 0.0, beta5(4) = 0.0, beta4(5) = 120.0, beta5(6) = 720.0 * s1, beta5(7) = 2520.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snap = c.transpose() * beta4;
        crakle = c.transpose() * beta5;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;
        // collision
        gradp.setZero();
        gradv.setZero();
        grada.setZero();
        costp = 0;
        if (obstacleGradCostfordrone(i_dp, pos, vel, acc, gradp, gradv, grada, costp))
        {
          // std::cout << "gradp[3]:" << gradp[3] << std::endl;
          // std::cout << "gradp[4]:" << gradp[4] << std::endl;
          
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);

          costs(0) += omg * step * costp;
        }

        gradp.setZero();
        gradv.setZero();
        grada.setZero();
        gradj.setZero();
        grads.setZero();
      
        costd = 0;

        if (feasibilityGradCostDroneDynamic(pos, vel, acc, jer, snap, crakle, gradp, gradv, grada, gradj, grads, costd))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costd / K + step * gradViolaAt);

          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snap;
          jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * step * gradViolaJc;
          gdT(i) += omg * (costd / K + step * gradViolaJt);

          gradViolaSc = beta4 * grads.transpose();
          gradViolaSt = alpha * grads.transpose() * crakle;
          jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * step * gradViolaSc;
          gdT(i) += omg * (costd / K + step * gradViolaSt);

          costs(1) += omg * step * costd;
        }

        // feasibility
        gradv.setZero();
        costv = 0;
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(2) += omg * step * costv;
        }
        grada.setZero();
        costa = 0;
        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(3) += omg * step * costa;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
      // t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s3 * s3;
        s7 = s6 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<8, 5>(i * 8, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<8, 5>(i * 8, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }
    costs(4) += var;
  }

  inline void PolyTrajOptimizer::normalizeFDF(const Eigen::Vector3d &x,
                                              Eigen::Vector3d &xNor,
                                              Eigen::Matrix3d &G)
  {
    const double eps = 1.0e-6;
    const double t_0 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + eps; // Add eps to smooth the gradient
    const double t_1 = sqrt(t_0);
    const double inv_t_1 = 1.0 / t_1;
    xNor = x * inv_t_1;
    G = inv_t_1 * Eigen::Matrix3d::Identity() - 1.0 / (t_0)*inv_t_1 * x * x.transpose();
    return;
  }

  bool PolyTrajOptimizer::obstacleGradCostfordrone(const int i_dp,
                                                   const Eigen::Vector5d &pos,
                                                   const Eigen::Vector5d &vel,
                                                   const Eigen::Vector5d &acc,
                                                   Eigen::Vector5d &gradp,
                                                   Eigen::Vector5d &gradvel,
                                                   Eigen::Vector5d &gradacc,
                                                   double &costp)
  {
    // if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
    //   return false;

    bool ret = false;

    costp = 0;
  
    Eigen::Vector3d load_position = pos.head<3>();
    Eigen::Vector3d load_rq       = Eigen::Vector3d(cos(pos[3])*cos(pos[4]), cos(pos[3])*sin(pos[4]), sin(pos[3]));
    Eigen::Vector3d attach_pos_1 = load_position - r_length_ * load_rq;
    Eigen::Vector3d attach_pos_2 = load_position + r_length_ * load_rq;
    double theta = pos[3];
    double   phi = pos[4];

    Eigen::Vector3d ddqr = Eigen::Vector3d(-std::cos(theta)*std::cos(phi)*vel[3]*vel[3] + std::sin(theta)*std::sin(phi)*vel[3]*vel[4] - std::sin(theta)*std::cos(phi)*acc[3] + std::sin(theta)*std::sin(phi)*vel[3]*vel[4] -std::cos(theta)*std::cos(phi)*vel[4]*vel[4]-std::cos(theta)*std::sin(phi)*acc[4],
                                                  -std::cos(theta)*std::sin(phi)*vel[3]*vel[3] - std::sin(theta)*std::cos(phi)*vel[3]*vel[4] - std::sin(theta)*std::sin(phi)*acc[3] - std::sin(theta)*std::cos(phi)*vel[3]*vel[4] -std::cos(theta)*std::sin(phi)*vel[4]*vel[4]+std::cos(theta)*std::cos(phi)*acc[4],
                                                  -std::sin(theta)*vel[3]*vel[3] + std::cos(theta)*acc[3]);

    Eigen::Vector3d dpdtheta = Eigen::Vector3d(-sin(theta)*cos(phi), -sin(theta)*sin(phi), cos(theta));
    Eigen::Vector3d dpdphi   = Eigen::Vector3d(-cos(theta)*sin(phi), cos(theta)*cos(phi), 0);

    // dpdtheta = Eigen::Vector3d(0, 0, 0);
    // dpdphi   = Eigen::Vector3d(0, 0, 0);

    Eigen::Vector3d attach_acc_1 = acc.head<3>() - r_length_ * ddqr;
    Eigen::Vector3d attach_acc_2 = acc.head<3>() + r_length_ * ddqr;
    // compute the drone pos
    Eigen::Vector3d negTp_1 = attach_acc_1.head<3>();
    Eigen::Vector3d negTp_2 = attach_acc_2.head<3>();
    negTp_1(2) += grav_;
    negTp_2(2) += grav_;
    Eigen::Vector3d norm_negTp_1;
    Eigen::Vector3d norm_negTp_2;
    Eigen::Matrix3d grad_norm_negTp_1;
    Eigen::Matrix3d grad_norm_negTp_2;
    normalizeFDF(negTp_1, norm_negTp_1, grad_norm_negTp_1);
    normalizeFDF(negTp_2, norm_negTp_2, grad_norm_negTp_2);
    Eigen::Matrix5d grad_norm_negTp_5d_1 = Eigen::Matrix5d::Zero() ;
    grad_norm_negTp_5d_1.block<3,3>(0,0) = grad_norm_negTp_1; 
    Eigen::Matrix5d grad_norm_negTp_5d_2 = Eigen::Matrix5d::Zero() ;
    grad_norm_negTp_5d_2.block<3,3>(0,0) = grad_norm_negTp_2; 

    double dist;
    double dist_err;
    double map_resolution = grid_map_->getResolution();
    double min_size = payload_size_ < drone_size_ ? payload_size_ : drone_size_;
    // use esdf check the payload pos
    double check_length = r_length_ - min_size;
    int avg = (int)(check_length / sample_ball_distance_) + 1;
    double factor = 1.0 / (double)avg;
  
    for (double l_dis = -r_length_; l_dis < r_length_; l_dis += sample_ball_distance_)
    {
      Eigen::Vector3d temp_p = (attach_pos_2 - attach_pos_1).normalized();
      Eigen::Vector3d pos_on_line = load_position + l_dis * temp_p; // the sample points on line
      dist = grid_map_->getDistance(pos_on_line);                 // coarse distance
      dist_err = min_size + safe_margin_ + map_resolution - dist; // conservative checking
      if (dist_err > 0)
      {

        Eigen::Vector3d dist_grad_3d;
        grid_map_->evaluateEDTWithGrad(pos_on_line, dist, dist_grad_3d);
        Eigen::Vector5d dist_grad;
        dist_grad.head<3>() = dist_grad_3d; 
        if (l_dis < 0)
        {
          dist_grad[3] = l_dis * dist_grad_3d.transpose() * dpdtheta;
          dist_grad[4] = l_dis * dist_grad_3d.transpose() * dpdphi;
        }
        //dist_grad[3] = 0.0;
        //std::cout << dist_grad << std::endl;
        dist_err = min_size + safe_margin_ - dist; // refine distance
        if (dist_err > 0)
        {
          ret = true;

          double grad_l1, cost_p_dist;
          smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
          costp += factor * wei_obs_ * cost_p_dist;
          Eigen::Vector5d curr_gradp = -wei_obs_ * factor * grad_l1 * dist_grad;
          gradp += curr_gradp;
        }
    }

    // check the points on line
     check_length = L_length_ - drone_size_;
    avg = (int)(check_length / sample_ball_distance_) + 1;
    factor = 1.0 / (double)avg;
    int num_size = 0;
    for (double l_dis = payload_size_; l_dis <= check_length; l_dis += sample_ball_distance_)
    {
      pos_on_line = attach_pos_1 + l_dis * norm_negTp_1; // the sample points on line

      dist = grid_map_->getDistance(pos_on_line);                 // coarse distance
      dist_err = min_size + safe_margin_ + map_resolution - dist; // conservative checking
      if (dist_err > 0)
      {

        Eigen::Vector3d dist_grad_3d;
        grid_map_->evaluateEDTWithGrad(pos_on_line, dist, dist_grad_3d);
        Eigen::Vector5d dist_grad;
        dist_grad.head<3>() = dist_grad_3d; 
        dist_grad[3] = -dist_grad_3d.transpose() * dpdtheta;
        dist_grad[4] = -dist_grad_3d.transpose() * dpdphi;
        dist_err = min_size + safe_margin_ - dist; // refine distance
        if (dist_err > 0)
        {
          ret = true;

          double grad_l1, cost_p_dist;
          smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
          costp += factor * wei_obs_ * cost_p_dist;
          Eigen::Vector5d curr_gradp = -wei_obs_ * factor * grad_l1 * dist_grad;
          gradp += curr_gradp;

          gradacc += (l_dis * curr_gradp.transpose() * grad_norm_negTp_5d_1).transpose();
        }
      }

      pos_on_line = attach_pos_2 + l_dis * norm_negTp_2; // the sample points on line

      dist = grid_map_->getDistance(pos_on_line);                 // coarse distance
      dist_err = min_size + safe_margin_ + map_resolution - dist; // conservative checking
      if (dist_err > 0)
      {

        Eigen::Vector3d dist_grad_3d;
        grid_map_->evaluateEDTWithGrad(pos_on_line, dist, dist_grad_3d);
        Eigen::Vector5d dist_grad;
        dist_grad.head<3>() = dist_grad_3d; 
        // dist_grad[3] = -dist_grad_3d.transpose() * dpdtheta;
        // dist_grad[4] = -dist_grad_3d.transpose() * dpdphi;
        dist_err = min_size + safe_margin_ - dist; // refine distance
        if (dist_err > 0)
        {
          ret = true;

          double grad_l1, cost_p_dist;
          smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
          costp += factor * wei_obs_ * cost_p_dist;
          Eigen::Vector5d curr_gradp = -wei_obs_ * factor * grad_l1 * dist_grad;
          gradp += curr_gradp;

          gradacc += (l_dis * curr_gradp.transpose() * grad_norm_negTp_5d_2).transpose();
        }
      }
      num_size++;
    }
    // check the drone
    Eigen::Vector3d posQ = attach_pos_1 + L_length_ * norm_negTp_1;
    dist = grid_map_->getDistance(posQ);                           // coarse distance
    dist_err = drone_size_ + safe_margin_ + map_resolution - dist; // conservative checking
    if (dist_err > 0)
    {

      Eigen::Vector3d dist_grad_3d;
      grid_map_->evaluateEDTWithGrad(posQ, dist, dist_grad_3d);
      dist_err = drone_size_ + safe_margin_ - dist; // refine distance
      Eigen::Vector5d dist_grad;
      dist_grad.head<3>() = dist_grad_3d; 
      dist_grad[3] = -dist_grad_3d.transpose() * dpdtheta;
      dist_grad[4] = -dist_grad_3d.transpose() * dpdphi;
      if (dist_err > 0)
      {
        ret = true;
        double grad_l1, cost_p_dist;
        smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
        costp += wei_obs_ * cost_p_dist;
        Eigen::Vector5d curr_gradp = -wei_obs_ * grad_l1 * dist_grad;
        gradp += curr_gradp;

        gradacc += (L_length_ * curr_gradp.transpose() * grad_norm_negTp_5d_1).transpose();
      }
    }

    posQ = attach_pos_2 + L_length_ * norm_negTp_2;
    dist = grid_map_->getDistance(posQ);                           // coarse distance
    dist_err = drone_size_ + safe_margin_ + map_resolution - dist; // conservative checking
    if (dist_err > 0)
    {

      Eigen::Vector3d dist_grad_3d;
      grid_map_->evaluateEDTWithGrad(posQ, dist, dist_grad_3d);
      dist_err = drone_size_ + safe_margin_ - dist; // refine distance
      Eigen::Vector5d dist_grad;
      dist_grad.head<3>() = dist_grad_3d; 
      // dist_grad[3] = -dist_grad_3d.transpose() * dpdtheta;
      // dist_grad[4] = -dist_grad_3d.transpose() * dpdphi;
      if (dist_err > 0)
      {
        ret = true;
        double grad_l1, cost_p_dist;
        smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
        costp += wei_obs_ * cost_p_dist;
        Eigen::Vector5d curr_gradp = -wei_obs_ * grad_l1 * dist_grad;
        gradp += curr_gradp;

        gradacc += (L_length_ * curr_gradp.transpose() * grad_norm_negTp_5d_2).transpose();
      }
    }

    return ret;
  }
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector5d &v,
                                               Eigen::Vector5d &gradv,
                                               double &costv)
  {
    bool ret = false;
    double vpen = v.head<3>().squaredNorm() - max_vel_ * max_vel_;

    // 后续修改
    double v_thetaopen = v[3] * v[3] - max_theta_vel_ * max_theta_vel_;
    double v_phiopen = v[4] * v[4] - max_phi_vel_ * max_phi_vel_;

    double f = 0, df = 0;
    if (smoothedL1(vpen, 0.01, f, df))
    {
      gradv.head<3>() = wei_feas_ * 2.0 * df * v.head<3>();
      costv = wei_feas_ * f;
      ret =  true;
    }

    if (smoothedL1(v_thetaopen, 0.01, f, df))
    {
      gradv[3] = wei_feas_ * 2.0 * df * v[3];
      costv += wei_feas_ * f;
      ret = true;
    }

    if (smoothedL1(v_phiopen, 0.01, f, df))
    {
      gradv[4] = wei_feas_ * 2.0 * df * v[4];
      costv += wei_feas_ * f;
      ret = true;
    }
    
    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector5d &a,
                                               Eigen::Vector5d &grada,
                                               double &costa)
  {
    bool ret;
    double apen = a.head<3>().squaredNorm() - max_acc_ * max_acc_;

    // 后续修改
    double v_thetaopen = a[3] * a[3] - max_theta_acc_ * max_theta_acc_;
    double v_phiopen = a[4] * a[4] - max_phi_acc_ * max_phi_acc_;

    double f = 0, df = 0;
    if (smoothedL1(apen, 0.01, f, df))
    {
      grada.head<3>() = wei_feas_ * 2.0 * df * a.head<3>();
      costa = wei_feas_ * f;
      ret =  true;
    }

    if (smoothedL1(v_thetaopen, 0.01, f, df))
    {
      grada[3] = wei_feas_ * 2.0 * df * a[3];
      costa += wei_feas_ * f;
      ret = true;
    }
    
    if (smoothedL1(v_phiopen, 0.01, f, df))
    {
      grada[4] = wei_feas_ * 2.0 * df * a[4];
      costa += wei_feas_ * f;
      ret = true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostDroneDynamic(const Eigen::Vector5d &pos,
                                                          const Eigen::Vector5d &vel,
                                                          const Eigen::Vector5d &acc,
                                                          const Eigen::Vector5d &jerk,
                                                          const Eigen::Vector5d &snap,
                                                          const Eigen::Vector5d &crakle,
                                                          Eigen::Vector5d &gradp,
                                                          Eigen::Vector5d &gradv,
                                                          Eigen::Vector5d &grada,
                                                          Eigen::Vector5d &gradj,
                                                          Eigen::Vector5d &grads,
                                                          double &costd)
  {
    bool ret = false;
    double thr_1, thr_2;
    Eigen::Vector3d zdir_1, zdir_2, zdir_grad_1, zdir_grad_2 = Eigen::Vector3d::Zero();
    flat.forward(pos, vel, acc, jerk, snap, crakle, thr_1, zdir_1, thr_2, zdir_2);
    double violaThrust_1 = (thr_1 - thrust_mean_) * (thr_1 - thrust_mean_) - thrust_sqr_radi_;
    double violaThrustPena_1, violaThrustPenaD_1;
    double gradThr_1 = 0.0;
    double violaThrust_2 = (thr_2 - thrust_mean_) * (thr_2 - thrust_mean_) - thrust_sqr_radi_;
    double violaThrustPena_2, violaThrustPenaD_2;
    double gradThr_2 = 0.0;
    
    if (smoothedL1(violaThrust_1, 0.01, violaThrustPena_1, violaThrustPenaD_1))
    {
      gradThr_1 += wei_quad_feas_ * violaThrustPenaD_1 * 2.0 * (thr_1 - thrust_mean_);
      costd += wei_quad_feas_ * violaThrustPena_1;
      ret = true;
    }
    if (smoothedL1(violaThrust_2, 0.01, violaThrustPena_2, violaThrustPenaD_2))
    {
      gradThr_2 += wei_quad_feas_ * violaThrustPenaD_2 * 2.0 * (thr_2 - thrust_mean_);
      costd += wei_quad_feas_ * violaThrustPena_2;
      ret = true;
    }
    // cos_theta_max_ - e^w_z dot zdir
    double violaTilt_1 = pow((cos_theta_max_ - zdir_1(2)), 3);
    double violaTiltPena_1, violaTiltPenaD_1;
    double violaTilt_2 = pow((cos_theta_max_ - zdir_2(2)), 3);
    double violaTiltPena_2, violaTiltPenaD_2;

    if (smoothedL1(violaTilt_1, 0.01, violaTiltPena_1, violaTiltPenaD_1))
    {
      zdir_grad_1 += Eigen::Vector3d(0, 0, -wei_quad_feas_) * violaTiltPenaD_1 * 3 * (cos_theta_max_ - zdir_1(2)) * (cos_theta_max_ - zdir_1(2));
      costd += wei_quad_feas_ * violaTiltPena_1;
      ret = true;
    }

    if (smoothedL1(violaTilt_2, 0.01, violaTiltPena_2, violaTiltPenaD_2))
    {
      zdir_grad_2 += Eigen::Vector3d(0, 0, -wei_quad_feas_) * violaTiltPenaD_2 * 3 * (cos_theta_max_ - zdir_2(2)) * (cos_theta_max_ - zdir_2(2));
      costd += wei_quad_feas_ * violaTiltPena_2;
      ret = true;
    }
    Eigen::Vector5d temp_grada_1, temp_gradj_1, temp_grads_1 = Eigen::Vector5d::Zero();
    Eigen::Vector5d temp_grada_2, temp_gradj_2, temp_grads_2 = Eigen::Vector5d::Zero();
    flat.backward(gradThr_1, zdir_grad_1, temp_grada_1, temp_gradj_1, temp_grads_1,
                  gradThr_2, zdir_grad_2, temp_grada_2, temp_gradj_2, temp_grads_2);
    grada = 0.5 * temp_grada_1+ 0.5 * temp_grada_2; 
    gradj = 0.5 * temp_gradj_1+ 0.5 * temp_gradj_2; 
    grads = 0.5 * temp_grads_1+ 0.5 * temp_grads_2;

    return ret;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps =
        ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(5, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
    return;
  }

  int PolyTrajOptimizer::astarWithMinTraj(const Eigen::Matrix<double, 5, 4> &iniState,
                                          const Eigen::Matrix<double, 5, 4> &finState,
                                          vector<Eigen::Vector5d> &simple_path,
                                          Eigen::MatrixXd &ctl_points,
                                          poly_traj::MinSnapOpt &frontendMJ)
  {
    Eigen::Vector5d start_pos = iniState.col(0);
    Eigen::Vector5d end_pos = finState.col(0);
    Eigen::Vector5d start_vel = iniState.col(1);
    Eigen::Vector5d end_vel = finState.col(1);
    Eigen::Vector5d start_acc = iniState.col(2);
    // Eigen::Vector3d end_acc = finState.col(2);
    vector<Eigen::Vector3d> simple_path_3d;
    std::vector<double> t_list;
    /* astar search and get the simple path*/
    simple_path.clear();
    simple_path_3d.clear();
    int status = kino_a_star_->KinoAstarSearchAndGetSimplePath(start_pos.head<3>(), start_vel.head<3>(), start_acc.head<3>(), end_pos.head<3>(), end_vel.head<3>(), simple_path_3d, t_list);
    for (int i = 0; i < simple_path_3d.size(); i++)
    {
      Eigen::Vector5d position = Eigen::Vector5d::Zero();
      position.head<3>() = simple_path_3d[i];
      position[4] = M_PI / 2;
      simple_path.push_back(position);
    }
    if (status == KinodynamicAstar::NO_PATH)
    {
      return status;
    }

    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1;
    Eigen::MatrixXd innerPts;

    if (piece_num > 1)
    {
      innerPts.resize(5, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) = simple_path[i + 1];
    }
    else
    {
      if ((simple_path[0] - simple_path[1]).norm() <= 0.1)  // Too short to ignore 
      {
        return KinodynamicAstar::TOO_SHORT;
      }
      ROS_WARN("simple path is too short");
      piece_num = 2;
      innerPts.resize(5, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2.0;
      t_list.resize(2);
      t_list[0] = (simple_path[0] - innerPts.col(0)).norm() / max_vel_*2.5;
      t_list[1] = (simple_path[1] - innerPts.col(0)).norm() / max_vel_*2.5;
    }
    // ctl_points = innerPts;

    frontendMJ.reset(iniState, finState, piece_num);

    Eigen::Map<Eigen::VectorXd> time_vec(t_list.data(), t_list.size());
    time_vec = time_vec * 1.2;
    frontendMJ.generate(innerPts, time_vec);
    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);
    return status;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constrain_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_quad_feasibility", wei_quad_feas_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);

    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_theta_vel", max_theta_vel_, -1.0);
    nh.param("optimization/max_theta_acc", max_theta_acc_, -1.0);
    nh.param("optimization/max_phi_vel", max_phi_vel_, -1.0);
    nh.param("optimization/max_phi_acc", max_phi_acc_, -1.0);

    nh.param("optimization/payload_size", payload_size_, 0.1);
    nh.param("optimization/drone_size", drone_size_, 0.25);
    nh.param("optimization/L_length", L_length_, 0.6);
    nh.param("optimization/r_length", r_length_, 0.4);
    nh.param("optimization/safe_margin", safe_margin_, 0.1);
    nh.param("optimization/sample_ball_distance", sample_ball_distance_, 0.1);
    nh.param("optimization/grav", grav_, 9.81);

    nh.param("optimization/mass_payload", mass_payload_, 0.1);
    nh.param("optimization/mass_quad", mass_quad_, 1.0);
    double max_theta;
    nh.param("optimization/max_theta", max_theta, 60.0);
    cos_theta_max_ = cos(max_theta * M_PI / 180.0);
    double max_thr, min_thr;
    nh.param("optimization/max_thrust", max_thr, 40.0);
    nh.param("optimization/min_thrust", min_thr, 0.5);

    thrust_mean_ = (max_thr + min_thr) / 2.0;
    thrust_sqr_radi_ = (max_thr - min_thr) * (max_thr - min_thr) / 4.0;
    flat.reset(mass_quad_, mass_payload_, grav_, L_length_, r_length_);

    kino_a_star_.reset(new KinodynamicAstar);
    kino_a_star_->setParam(nh);
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    // should be called after setParam
    grid_map_ = map;
    kino_a_star_->setEnvironment(grid_map_);
    kino_a_star_->init();
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

}