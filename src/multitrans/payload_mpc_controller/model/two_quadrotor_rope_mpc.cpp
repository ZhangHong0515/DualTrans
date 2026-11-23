#include <memory>
#include <math.h>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <Yaml.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input.
#define RED "\033[31m"    /* Red */
#define YELLOW "\033[33m" /* Yellow */
#define RESET "\033[0m"

/*if you want to close the CONSTANT_MOMENT, please set this to 0
 *0 mean close
 *1 mean use fl as constant moment
 *2 mean use fq as constant moment
 **/
#define USE_CONSTANT_MOMENT 0

/*
Switch between code generation and analysis.

If CODE_GEN is true the system is compiled into an optimizaiton problem
for real-time iteration and all code to run it online is generated.
Constraints and reference structure is used but the values will be set on
runtinme.

If CODE_GEN is false, the system is compiled into a standalone optimization
and solved on execution. The reference and constraints must be set in here.
*/
#define CODE_GEN false

// if you want use your own parameters, run the program with your own parameter's file path. Such as,
//  ./quadrotor_payload_mpc_codegen ../config/mpc.yaml
int main(int argc, char **argv)
{
  // Use Acado
  USING_NAMESPACE_ACADO

  std::string cfg_file_path = "../config/twomodel.yaml";
  if (argc > 1)
    cfg_file_path = argv[1];
  Yaml::Node cfg_root;

  try
  {
    Yaml::Parse(cfg_root, cfg_file_path.data());
  }
  catch (const std::exception &e)
  {
    std::cerr << RED << e.what() << std::endl;
  }
  std::cout << "Parameters file path: " << cfg_file_path << RESET << std::endl
            << std::endl;

  // System variables
  DifferentialState p_x_1, p_y_1, p_z_1;
  DifferentialState q_w_1, q_x_1, q_y_1, q_z_1;
  DifferentialState v_x_1, v_y_1, v_z_1;
  DifferentialState p_x_2, p_y_2, p_z_2;
  DifferentialState q_w_2, q_x_2, q_y_2, q_z_2;
  DifferentialState v_x_2, v_y_2, v_z_2;

  DifferentialState payload_x, payload_y, payload_z;
  DifferentialState payload_vx, payload_vy, payload_vz;
  
  DifferentialState payload_rx, payload_ry, payload_rz;
  DifferentialState payload_drx, payload_dry, payload_drz;

  DifferentialState cable_x_1, cable_y_1, cable_z_1;
  DifferentialState dcable_x_1, dcable_y_1, dcable_z_1;
  DifferentialState cable_x_2, cable_y_2, cable_z_2;
  DifferentialState dcable_x_2, dcable_y_2, dcable_z_2;
  
  Control T_1, w_x_1, w_y_1, w_z_1, T_2, w_x_2, w_y_2, w_z_2;
  DifferentialEquation f;
  Function h, hN;
  

  #if CODE_GEN
  OnlineData Mq;        //Quadrotor mass[kg]
  OnlineData Ml;        //payload mass [kg]
  // double Mq = cfg_root["mass_q"].As<double>(0.98);        // Quadrotor mass[kg] default 0.98
  // double Ml = cfg_root["mass_l"].As<double>(0.2);         // payload mass [kg] default 0.2
  // OnlineData fl_x, fl_y, fl_z; //external force [N]

    #if (USE_CONSTANT_MOMENT==1)
    OnlineData M_x, M_y, M_z; //constant moment [N*m]
    IntermediateState fl_x, fl_y, fl_z;
    OnlineData fq_x, fq_y, fq_z;  //external force on quadrotor [N]
    #elif (USE_CONSTANT_MOMENT==2)
    OnlineData fl_x, fl_y, fl_z;  //external force on quadrotor [N]
    OnlineData M_x, M_y, M_z; //constant moment [N*m]
    IntermediateState fq_x, fq_y, fq_z;
    #else
    OnlineData fl_x_1, fl_y_1, fl_z_1;  //external force on load [N] uav1
    OnlineData fl_x_2, fl_y_2, fl_z_2;  //external force on load [N] uav2
    OnlineData fq_x_1, fq_y_1, fq_z_1;
    OnlineData fq_x_2, fq_y_2, fq_z_2; //external force [N]
    #endif

  double cable_length = cfg_root["l_length"].As<double>(0.6); // cable length [m]  default 0.6
  double rote_length  = cfg_root["len_r"].As<double>(0.19);
  
  #else
  double Mq = cfg_root["mass_q"].As<double>(0.98);        // Quadrotor mass[kg] default 0.98
  double Ml = cfg_root["mass_l"].As<double>(0.2);         // payload mass [kg] default 0.2
  double cable_length = cfg_root["l_length"].As<double>(0.6); // cable length [m]  default 0.6
  double rote_length  = cfg_root["len_r"].As<double>(0.19);
  double fq_x_1 = 0.0, fq_y_1 = 0.0, fq_z_1 = 0.0;              // external force on quadrotor [N]
  double fq_x_2 = 0.0, fq_y_2 = 0.0, fq_z_2 = 0.0;              // external force on quadrotor [N]
  double fl_x_1 = 0.0, fl_y_1 = 0.0, fl_z_1 = 0.0; //external force [N]
  double fl_x_2 = 0.0, fl_y_2 = 0.0, fl_z_2 = 0.0; //external force [N]
  #endif

  // dynamics
  // double Mq = cfg_root["mass_q"].As<double>(0.98);        // Quadrotor mass[kg] default 0.98
  // double Ml = cfg_root["mass_l"].As<double>(0.2);         // payload mass [kg] default 0.2
 
  double g_z = cfg_root["gravity"].As<double>(9.81);                          // Gravity is everywhere [m/s^2]
  double dt = cfg_root["step_T"].As<double>(0.05);                           // Discretization time [s]
  int N = cfg_root["step_N"].As<int>(20);                  // Number of nodes
  std::cout << YELLOW;
  std::cout << "Mq:       " << Mq << std::endl;
  std::cout << "Ml:       " << Ml << std::endl;
  std::cout << "cable_length: " << cable_length << std::endl;
  std::cout << "g_z:      " << g_z << std::endl;
  std::cout << "N:        " << N << std::endl;
  std::cout << "dt:       " << dt << std::endl;
  std::cout << "rot_length" << rote_length << std::endl;
  std::cout << "fl_x:       " << fl_x_1 << std::endl;
  std::cout << "fl_y:       " << fl_y_1 << std::endl;
  std::cout << "fl_z:       " << fl_z_1 << std::endl;
  std::cout << "fq_x:       " << fq_x_1 << std::endl;
  std::cout << "fq_y:       " << fq_y_1 << std::endl;
  std::cout << "fq_z:       " << fq_z_1 << std::endl;
  std::cout << RESET << std::endl;

  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double w_max_yaw = 1;                       // Maximal yaw rate [rad/s]
  const double w_max_xy = 5;                        // Maximal pitch and roll rate [rad/s]
  const double T_min = 4.0;                         // Minimal total thrust [N]
  const double T_max = 40.0;                        // Maximal total thrust [N]
  const double K_min_cos_cable_angle = cos(M_PI_4); // Minimal cos cable angle (cable.dot(gravity) => K_max_cable_angle)
  const double K_max_cos_cable_angle = 1.0;

  #if (USE_CONSTANT_MOMENT==1)&&CODE_GEN
    fl_x = M_y*cable_z - M_z*cable_y;
    fl_y = M_z*cable_x - M_x*cable_z;
    fl_z = M_x*cable_y - M_y*cable_x;
  #elif (USE_CONSTANT_MOMENT==2)&&CODE_GEN
    fq_x = M_y*cable_z - M_z*cable_y;
    fq_y = M_z*cable_x - M_x*cable_z;
    fq_z = M_x*cable_y - M_y*cable_x;
  #endif
  IntermediateState dcable_sqr_1, dcable_sqr_2, q_sqr_1, q_sqr_2;
  IntermediateState payload_dr_sqr;
  IntermediateState qwqy_plus_qxqz_1, qwqx_minus_qyqz_1;
  IntermediateState qwqy_plus_qxqz_2, qwqx_minus_qyqz_2;

  IntermediateState attach_accel_x_1, attach_accel_y_1, attach_accel_z_1;
  IntermediateState attach_accel_x_2, attach_accel_y_2, attach_accel_z_2;
  IntermediateState tensor_x_1, tensor_y_1, tensor_z_1;
  IntermediateState tensor_x_2, tensor_y_2, tensor_z_2;
  
  IntermediateState dd_cable_x_1, dd_cable_y_1, dd_cable_z_1;
  IntermediateState dd_cable_x_2, dd_cable_y_2, dd_cable_z_2;
  IntermediateState cbl_omg_sqr_1, cbl_omg_sqr_2;
  
  IntermediateState payload_ax, payload_ay, payload_az;
  IntermediateState payload_ddrx, payload_ddry, payload_ddrz;
  // IntermediateState pl_omg_x, pl_omg_y, pl_omg_z;
  // IntermediateState pl_domg_x, pl_domg_y, pl_domg_z;

  dcable_sqr_1 = dcable_x_1*dcable_x_1+ dcable_y_1*dcable_y_1+ dcable_z_1*dcable_z_1;
  dcable_sqr_2 = dcable_x_2*dcable_x_2+ dcable_y_2*dcable_y_2+ dcable_z_2*dcable_z_2;

  payload_dr_sqr = payload_drx * payload_drx + payload_dry * payload_dry + payload_drz * payload_drz;
  q_sqr_1 = q_w_1*q_w_1 - q_x_1*q_x_1- q_y_1*q_y_1 + q_z_1*q_z_1;
  q_sqr_2 = q_w_2*q_w_2 - q_x_2*q_x_2- q_y_2*q_y_2 + q_z_2*q_z_2;
  
  qwqy_plus_qxqz_1 = 2*(q_w_1*q_y_1 + q_x_1*q_z_1);
  qwqx_minus_qyqz_1 = 2*(q_w_1*q_x_1 - q_y_1*q_z_1);
  qwqy_plus_qxqz_2 = 2*(q_w_2*q_y_2 + q_x_2*q_z_2);
  qwqx_minus_qyqz_2 = 2*(q_w_2*q_x_2 - q_y_2*q_z_2);


  payload_ax = (1 / (2 * Mq + Ml)) * (T_2*(qwqy_plus_qxqz_2) + T_1*(qwqy_plus_qxqz_1) + Mq * cable_length * (dd_cable_x_2 + dd_cable_x_1));
  payload_ay = (1 / (2 * Mq + Ml)) * (-T_2*(qwqx_minus_qyqz_2) - T_1*(qwqx_minus_qyqz_1) + Mq * cable_length * (dd_cable_y_2 + dd_cable_y_1));
  payload_az = (1 / (2 * Mq + Ml)) * (T_2*(q_sqr_2) + T_1*(q_sqr_1) + Mq * cable_length * (dd_cable_z_2 + dd_cable_z_1)) - g_z;
 
  payload_ddrx = -(1 / ((2 * Mq + 2 / 3 * Ml) * rote_length)) * ((-payload_rz * payload_rz - payload_ry * payload_ry) * (T_2*(qwqy_plus_qxqz_2) - T_1*(qwqy_plus_qxqz_1) + Mq * rote_length * (dd_cable_x_2 - dd_cable_x_1))  + payload_ry * payload_rx * (-T_2*(qwqx_minus_qyqz_2) + T_1*(qwqx_minus_qyqz_1) + Mq * rote_length * (dd_cable_y_2 - dd_cable_y_1)) - payload_rx * payload_rz * (T_2*(q_sqr_2) - T_1*(q_sqr_1) + Mq * rote_length * (dd_cable_z_2 - dd_cable_z_1))) - payload_rx * (payload_dr_sqr);
  payload_ddry = -(1 / ((2 * Mq + 2 / 3 * Ml) * rote_length)) * ((payload_rx * payload_ry) * (T_2*(qwqy_plus_qxqz_2) - T_1*(qwqy_plus_qxqz_1) + Mq * rote_length * (dd_cable_x_2 - dd_cable_x_1)) + (-payload_rz * payload_rz - payload_rx * payload_rx) * (-T_2*(qwqx_minus_qyqz_2) + T_1*(qwqx_minus_qyqz_1) + Mq * rote_length * (dd_cable_y_2 - dd_cable_y_1)) + payload_ry * payload_rz * (T_2*(q_sqr_2) - T_1*(q_sqr_1) + Mq * rote_length * (dd_cable_z_2 - dd_cable_z_1))) - payload_ry * (payload_dr_sqr);
  payload_ddrz = -(1 / ((2 * Mq + 2 / 3 * Ml) * rote_length)) * ((payload_rx * payload_rz) * (T_2*(qwqy_plus_qxqz_2) - T_1*(qwqy_plus_qxqz_1) + Mq * rote_length * (dd_cable_x_2 - dd_cable_x_1)) + payload_ry * payload_rz * (-T_2*(qwqx_minus_qyqz_2) + T_1*(qwqx_minus_qyqz_1) + Mq * rote_length * (dd_cable_y_2 - dd_cable_y_1)) +(-payload_ry * payload_ry - payload_rx * payload_rx) * (T_2*(q_sqr_2) - T_1*(q_sqr_1) + Mq * rote_length * (dd_cable_z_2 - dd_cable_z_1)))- payload_rz * (payload_dr_sqr);
  

  attach_accel_x_1 = payload_ax - rote_length * payload_ddrx; 
  attach_accel_y_1 = payload_ay - rote_length * payload_ddry;
  attach_accel_z_1 = payload_az - rote_length * payload_ddrz + g_z;
  attach_accel_x_2 = payload_ax + rote_length * payload_ddrx; 
  attach_accel_y_2 = payload_ay + rote_length * payload_ddry;
  attach_accel_z_2 = payload_az + rote_length * payload_ddrz + g_z;
 
  cbl_omg_sqr_1 = (-cable_z_1 * dcable_y_1 + cable_y_1 * dcable_z_1) * (-cable_z_1 * dcable_y_1 + cable_y_1 * dcable_z_1) + (cable_z_1 * dcable_x_1 - cable_x_1 * dcable_z_1) * (cable_z_1 * dcable_x_1 - cable_x_1 * dcable_z_1) + (-cable_y_1 * dcable_x_1 + cable_x_1 * dcable_y_1) * (-cable_y_1 * dcable_x_1 + cable_x_1 * dcable_y_1);
  cbl_omg_sqr_2 = (-cable_z_2 * dcable_y_2 + cable_y_2 * dcable_z_2) * (-cable_z_2 * dcable_y_2 + cable_y_2 * dcable_z_2) + (cable_z_2 * dcable_x_2 - cable_x_2 * dcable_z_2) * (cable_z_2 * dcable_x_2 - cable_x_2 * dcable_z_2) + (-cable_y_2 * dcable_x_2 + cable_x_2 * dcable_y_2) * (-cable_y_2 * dcable_x_2 + cable_x_2 * dcable_y_2);
  
  tensor_x_1 = cable_x_1 * ((Mq * cable_length * cbl_omg_sqr_1) - cable_x_1 * (T_1*(qwqy_plus_qxqz_1) + fq_x_1 - Mq * attach_accel_x_1) - cable_y_1 * (- T_1*(qwqx_minus_qyqz_1) + fq_y_1 - Mq * attach_accel_y_1) - cable_z_1 * (T_1 * (q_sqr_1) - Mq * attach_accel_z_1));
  tensor_y_1 = cable_y_1 * ((Mq * cable_length * cbl_omg_sqr_1) - cable_x_1 * (T_1*(qwqy_plus_qxqz_1) + fq_x_1 - Mq * attach_accel_x_1) - cable_y_1 * (- T_1*(qwqx_minus_qyqz_1) + fq_y_1 - Mq * attach_accel_y_1) - cable_z_1 * (T_1 * (q_sqr_1) - Mq * attach_accel_z_1));
  tensor_z_1 = cable_z_1 * ((Mq * cable_length * cbl_omg_sqr_1) - cable_x_1 * (T_1*(qwqy_plus_qxqz_1) + fq_x_1 - Mq * attach_accel_x_1) - cable_y_1 * (- T_1*(qwqx_minus_qyqz_1) + fq_y_1 - Mq * attach_accel_y_1) - cable_z_1 * (T_1 * (q_sqr_1) - Mq * attach_accel_z_1));
  
  tensor_x_2 = cable_x_2 * ((Mq * cable_length * cbl_omg_sqr_2) - cable_x_2 * (T_2*(qwqy_plus_qxqz_2) + fq_x_2 - Mq * attach_accel_x_2) - cable_y_2 * (- T_2*(qwqx_minus_qyqz_2) + fq_y_2 - Mq * attach_accel_y_2) - cable_z_2 * (T_2 * (q_sqr_2) - Mq * attach_accel_z_2));
  tensor_y_2 = cable_y_2 * ((Mq * cable_length * cbl_omg_sqr_2) - cable_x_2 * (T_2*(qwqy_plus_qxqz_2) + fq_x_2 - Mq * attach_accel_x_2) - cable_y_2 * (- T_2*(qwqx_minus_qyqz_2) + fq_y_2 - Mq * attach_accel_y_2) - cable_z_2 * (T_2 * (q_sqr_2) - Mq * attach_accel_z_2));
  tensor_z_2 = cable_z_2 * ((Mq * cable_length * cbl_omg_sqr_2) - cable_x_2 * (T_2*(qwqy_plus_qxqz_2) + fq_x_2 - Mq * attach_accel_x_2) - cable_y_2 * (- T_2*(qwqx_minus_qyqz_2) + fq_y_2 - Mq * attach_accel_y_2) - cable_z_2 * (T_2 * (q_sqr_2) - Mq * attach_accel_z_2));

  
  dd_cable_x_1 = ((-cable_z_1 * cable_z_1 - cable_y_1 * cable_y_1 ) * (T_1*(qwqy_plus_qxqz_1) - Mq * attach_accel_x_1) + cable_y_1 * cable_x_1 * (-T_1*(qwqx_minus_qyqz_1)- Mq * attach_accel_y_1) - cable_x_1 * cable_z_1 * (T_1 * (q_sqr_1) -  Mq * attach_accel_z_1)) / (Mq * cable_length) - cable_x_1 * dcable_sqr_1; 
  dd_cable_y_1 = ((cable_x_1 * cable_y_1 ) * (T_1*(qwqy_plus_qxqz_1) - Mq * attach_accel_x_1) + (-cable_z_1 * cable_z_1 - cable_x_1 * cable_x_1) * (-T_1*(qwqx_minus_qyqz_1)- Mq * attach_accel_y_1) + cable_y_1 * cable_z_1 * (T_1 * (q_sqr_1) -  Mq * attach_accel_z_1)) / (Mq * cable_length) - cable_y_1 * dcable_sqr_1; 
  dd_cable_z_1 = ((cable_x_1 * cable_z_1 ) * (T_1*(qwqy_plus_qxqz_1) - Mq * attach_accel_x_1) + cable_y_1 * cable_z_1 * (-T_1*(qwqx_minus_qyqz_1)- Mq * attach_accel_y_1) + (-cable_y_1 * cable_y_1 - cable_x_1 * cable_x_1) * (T_1 * (q_sqr_1) -  Mq * attach_accel_z_1)) / (Mq * cable_length) - cable_z_1 * dcable_sqr_1; 
  
  dd_cable_x_2 = ((-cable_z_2 * cable_z_2 - cable_y_2 * cable_y_2 ) * (T_2*(qwqy_plus_qxqz_2) - Mq * attach_accel_x_2) + cable_y_2 * cable_x_2 * (-T_2*(qwqx_minus_qyqz_2)- Mq * attach_accel_y_2) - cable_x_2 * cable_z_2 * (T_2 * (q_sqr_2) -  Mq * attach_accel_z_2)) / (Mq * cable_length) - cable_x_2 * dcable_sqr_2; 
  dd_cable_y_2 = ((cable_x_2 * cable_y_2 ) * (T_2*(qwqy_plus_qxqz_2) - Mq * attach_accel_x_2) + (-cable_z_2 * cable_z_2 - cable_x_2 * cable_x_2) * (-T_2*(qwqx_minus_qyqz_2)- Mq * attach_accel_y_2) + cable_y_2 * cable_z_2 * (T_2 * (q_sqr_2) -  Mq * attach_accel_z_2)) / (Mq * cable_length) - cable_y_2 * dcable_sqr_2; 
  dd_cable_z_2 = ((cable_x_2 * cable_z_2 ) * (T_2*(qwqy_plus_qxqz_2) - Mq * attach_accel_x_2) + cable_y_2 * cable_z_2 * (-T_2*(qwqx_minus_qyqz_2)- Mq * attach_accel_y_2) + (-cable_y_2 * cable_y_2 - cable_x_2 * cable_x_2) * (T_2 * (q_sqr_2) -  Mq * attach_accel_z_2)) / (Mq * cable_length) - cable_z_2 * dcable_sqr_2; 
  
  // uav_1
  f << dot(p_x_1) == v_x_1;
  f << dot(p_y_1) == v_y_1;
  f << dot(p_z_1) == v_z_1;

  f << dot(q_w_1) == 0.5 * (-w_x_1 * q_x_1 - w_y_1 * q_y_1 - w_z_1 * q_z_1);
  f << dot(q_x_1) == 0.5 * (w_x_1 * q_w_1 + w_z_1 * q_y_1 - w_y_1 * q_z_1);
  f << dot(q_y_1) == 0.5 * (w_y_1 * q_w_1 - w_z_1 * q_x_1 + w_x_1 * q_z_1);
  f << dot(q_z_1) == 0.5 * (w_z_1 * q_w_1 + w_y_1 * q_x_1 - w_x_1 * q_y_1);

  f << dot(v_x_1) == (fq_x_1 + T_1*(qwqy_plus_qxqz_1) + tensor_x_1)/Mq;
  f << dot(v_y_1) == (fq_y_1 - T_1*(qwqx_minus_qyqz_1) + tensor_y_1)/Mq;
  f << dot(v_z_1) == - g_z + (fq_z_1 + T_1*(q_sqr_1) + tensor_z_1)/Mq;

  // uav_2
  f << dot(p_x_2) == v_x_2;
  f << dot(p_y_2) == v_y_2;
  f << dot(p_z_2) == v_z_2;

  f << dot(q_w_2) == 0.5 * (-w_x_2 * q_x_2 - w_y_2 * q_y_2 - w_z_2 * q_z_2);
  f << dot(q_x_2) == 0.5 * (w_x_2 * q_w_2 + w_z_2 * q_y_2 - w_y_2 * q_z_2);
  f << dot(q_y_2) == 0.5 * (w_y_2 * q_w_2 - w_z_2 * q_x_2 + w_x_2 * q_z_2);
  f << dot(q_z_2) == 0.5 * (w_z_2 * q_w_2 + w_y_2 * q_x_2 - w_x_2 * q_y_2);

  f << dot(v_x_2) == (fq_x_2 + T_2*(qwqy_plus_qxqz_2) + tensor_x_2)/Mq;
  f << dot(v_y_2) == (fq_y_2 - T_2*(qwqx_minus_qyqz_2) + tensor_y_2)/Mq;
  f << dot(v_z_2) == - g_z + (fq_z_2 + T_2*(q_sqr_2) + tensor_z_2)/Mq;


  f << dot(payload_x) == payload_vx;
  f << dot(payload_y) == payload_vy;
  f << dot(payload_z) == payload_vz;

  f << dot(payload_vx) == payload_ax;
  f << dot(payload_vy) == payload_ay;
  f << dot(payload_vz) == payload_az;

  f << dot(payload_rx) == payload_drx;
  f << dot(payload_ry) == payload_dry;
  f << dot(payload_rz) == payload_drz;

  f << dot(payload_drx) == payload_ddrx;
  f << dot(payload_dry) == payload_ddry;
  f << dot(payload_drz) == payload_ddrz;
  
  f << dot(cable_x_1) == dcable_x_1;
  f << dot(cable_y_1) == dcable_y_1;
  f << dot(cable_z_1) == dcable_z_1;

  f << dot(dcable_x_1) ==  dd_cable_x_1;
  f << dot(dcable_y_1) == dd_cable_y_1;
  f << dot(dcable_z_1) == dd_cable_z_1;

  f << dot(cable_x_2) ==  dcable_x_2;
  f << dot(cable_y_2) == dcable_y_2;
  f << dot(cable_z_2) == dcable_z_2;

  f << dot(dcable_x_2) ==  dd_cable_x_2;
  f << dot(dcable_y_2) == dd_cable_y_2;
  f << dot(dcable_z_2) == dd_cable_z_2;

  

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x_1 << p_y_1 << p_z_1  //3
    << q_w_1 << q_x_1 << q_y_1 << q_z_1  //7 
    << v_x_1 << v_y_1 << v_z_1 //10
    << p_x_2 << p_y_2 << p_z_2  //13
    << q_w_2 << q_x_2 << q_y_2 << q_z_2  //17 
    << v_x_2 << v_y_2 << v_z_2 //20
    << payload_x << payload_y << payload_z //23
    << payload_vx << payload_vy << payload_vz //26
    << payload_rx << payload_ry << payload_rz //29
    << payload_drx << payload_dry << payload_drz //32
    << cable_x_1 << cable_y_1 << cable_z_1   //35
    << dcable_x_1 << dcable_y_1 << dcable_z_1 //38
    << cable_x_2 << cable_y_2 << cable_z_2   //41
    << dcable_x_2 << dcable_y_2 << dcable_z_2 //44
    << T_1 << w_x_1 << w_y_1 << w_z_1  // 48
    << T_2 << w_x_2 << w_y_2 << w_z_2; // 52
       

  // End cost vector consists of all states (no inputs at last state).
  hN  << p_x_1 << p_y_1 << p_z_1  //3
    << q_w_1 << q_x_1 << q_y_1 << q_z_1  //7 
    << v_x_1 << v_y_1 << v_z_1 //10
    << p_x_2 << p_y_2 << p_z_2  //13
    << q_w_2 << q_x_2 << q_y_2 << q_z_2  //17 
    << v_x_2 << v_y_2 << v_z_2 //20
    << payload_x << payload_y << payload_z //23
    << payload_vx << payload_vy << payload_vz //26
    << payload_rx << payload_ry << payload_rz //29
    << payload_drx << payload_dry << payload_drz //32
    << cable_x_1 << cable_y_1 << cable_z_1   //35
    << dcable_x_1 << dcable_y_1 << dcable_z_1 //38
    << cable_x_2 << cable_y_2 << cable_z_2   //41
    << dcable_x_2 << dcable_y_2 << dcable_z_2;//44

  // for non codegen cases
  //  Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();

  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(3) = 1.0;
  r(18) = -1.0;

  DVector rN(hN.getDim()); // End cost reference
  rN.setZero();
  rN(0) = r(0);
  rN(1) = r(1);
  rN(2) = r(2);
  rN(10) = r(10);
  rN(11) = r(11);
  rN(12) = r(12);
  rN(3) = r(3);
  rN(18) = r(18);

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp(0.0, dt * N, N);
  #if (!CODE_GEN)
  {
    Q(0, 0) = 200; // x_1
    Q(1, 1) = 200; // y_1
    Q(2, 2) = 200; // z_1
    Q(3, 3) = 100; // qw_1
    Q(4, 4) = 100; // qx_1
    Q(5, 5) = 100; // qy_1
    Q(6, 6) = 100; // qz_1
    Q(7, 7) = 1;   // vx_1
    Q(8, 8) = 1;   // vy_1
    Q(9, 9) = 1;   // vz_1
    Q(10, 10) = 200; // x_2
    Q(11, 11) = 200; // y_2
    Q(12, 12) = 200; // z_2
    Q(13, 13) = 100; // qw_2
    Q(14, 14) = 100; // qx_2
    Q(15, 15) = 100; // qy_2
    Q(16, 16) = 100; // qz_2
    Q(17, 17) = 1;   // vx_2
    Q(18, 18) = 1;   // vy_2
    Q(19, 19) = 1;   // vz_2
    Q(20, 20) = 200; // payload_x
    Q(21, 21) = 200; // payload_y
    Q(22, 22) = 200; // payload_z
    Q(23, 23) = 1;   // payload_vx
    Q(24, 24) = 1;   // payload_vy
    Q(25, 25) = 1;   // payload_vz
    Q(26, 26) = 100; // payload_rx
    Q(27, 27) = 100; // payload_ry
    Q(28, 28) = 100; // payload_rz
    Q(29, 29) = 10; // payload_drx
    Q(30, 30) = 10; // payload_dry
    Q(31, 31) = 10; // payload_drz
    Q(32, 32) = 100;  // cable_x_1
    Q(33, 33) = 100;  // cable_y_1
    Q(34, 34) = 100;  // cable_z_1
    Q(35, 35) = 1;   // dcable_x_1
    Q(36, 36) = 1;   // dcable_y_1
    Q(37, 37) = 1;   // dcable_z_1
    Q(38, 38) = 100;   // cable_x_2
    Q(39, 39) = 100;   // cable_y_2
    Q(40, 40) = 100;   // cable_z_2
    Q(41, 41) = 1;  // dcable_x_2
    Q(42, 42) = 1;  // dcable_y_2
    Q(43, 43) = 1;  // dcable_z_2
    // End cost weight matrix
    DMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();
    QN(0, 0) = Q(0, 0); // x
    QN(1, 1) = Q(1, 1); // y
    QN(2, 2) = Q(2, 2); // z
    QN(3, 3) = Q(3, 3); // qw
    QN(4, 4) = Q(4, 4); // qx
    QN(5, 5) = Q(5, 5); // qy
    QN(6, 6) = Q(6, 6); // qz
    // QN(7,7) = Q(7,7);   // vx
    // QN(8,8) = Q(8,8);   // vy
    // QN(9,9) = Q(9,9);   // vz
    QN(7, 7) = 100; // vx
    QN(8, 8) = 100; // vy
    QN(9, 9) = 100; // vz
    QN(10, 10) = Q(10, 10); // payload_x
    QN(11, 11) = Q(11, 11); // payload_y
    QN(12, 12) = Q(12, 12); // payload_z
    QN(13,13) = Q(13,13);   // payload_vx
    QN(14,14) = Q(14,14);   // payload_vy
    QN(15,15) = Q(15,15);   // payload_vz
    QN(16,16) = Q(16,16);   // cable_x
    QN(17,17) = Q(17,17);   // cable_y
    QN(18,18) = Q(18,18);   // cable_z

    QN(22, 22) = Q(22,22); // L
    QN(23, 23) = Q(23,23); // d_L
    // QN(13, 13) = 100;       // payload_vx
    // QN(14, 14) = 100;       // payload_vy
    // QN(15, 15) = 100;       // payload_vz
    // QN(10, 10) = 10; // cable_x
    // QN(11, 11) = 10; // cable_y
    // QN(12, 12) = 10; // cable_z
    // QN(13, 13) = 10; // dcable_x
    // QN(14, 14) = 10; // dcable_y
    // QN(15, 15) = 10; // dcable_z
    QN = Q;
    const double x_ref = 1.0; // x position of reference
    const double y_ref = 0.5; // y position of reference
    const double z_ref = 0.6; // z position of reference
    r.setZero();
    r(0) = x_ref;
    r(1) = y_ref - 0.4;
    r(2) = z_ref;
    r(10) = x_ref;
    r(11) = y_ref + 0.4;
    r(12) = z_ref;
    r(3) = 1.0;
    r(13) = 1.0;
    r(20) = x_ref;
    r(21) = y_ref;
    r(22) = z_ref - 0.6;
    r(26) = 0;
    r(27) = 1;
    r(28) = 0;
    r(34) = -1;
    r(40) = -1;

    r(44) = g_z * (Mq + 0.5 * Ml);
    r(48) = g_z * (Mq + 0.5 * Ml);

    rN.setZero();
    rN(0) = r(0);
    rN(1) = r(1);
    rN(2) = r(2);
    rN(10) = r(10);
    rN(11) = r(11);
    rN(12) = r(12);
    rN(3) = r(3);
    rN(13) = r(13);

    rN(20) = r(20);
    rN(21) = r(21);
    rN(22) = r(22);
    rN(26) = r(26);
    rN(27) = r(27);
    rN(28) = r(28);
    rN(34) = r(34);
    rN(40) = r(40);
    // For analysis, set references.
    ocp.minimizeLSQ(Q, h, r);
    ocp.minimizeLSQEndTerm(QN, hN, rN);
  }
  #else
  {
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN); // Least Square term
  }
  #endif

  // Add system dynamics
  ocp.subjectTo(f);
  // Add constraints
  ocp.subjectTo(-w_max_xy <= w_x_1 <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y_1 <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z_1 <= w_max_yaw);
  ocp.subjectTo(T_min <= T_1 <= T_max);
  ocp.subjectTo(-w_max_xy <= w_x_2 <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y_2 <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z_2 <= w_max_yaw);
  ocp.subjectTo(T_min <= T_2 <= T_max);
  
  // ocp.subjectTo( -1.01 <= cable_z <= -K_min_cos_cable_angle);
  // ocp.subjectTo( 0.2 <= len);
  // Set online data size 
  // https://github.com/acado/acado/issues/196
  ocp.setNOD(14); // num of OnlineData   //10?13?

  #if (!CODE_GEN)
  {
    // Set initial state

    ocp.subjectTo(AT_START, p_x_1 == 0.0);
    ocp.subjectTo(AT_START, p_y_1 == -0.19);
    ocp.subjectTo(AT_START, p_z_1 == 0.6);
    ocp.subjectTo(AT_START, q_w_1 == 1.0);
    ocp.subjectTo(AT_START, q_x_1 == 0.0);
    ocp.subjectTo(AT_START, q_y_1 == 0.0);
    ocp.subjectTo(AT_START, q_z_1 == 0.0);
    ocp.subjectTo(AT_START, v_x_1 == 0.0);
    ocp.subjectTo(AT_START, v_y_1 == 0.0);
    ocp.subjectTo(AT_START, v_z_1 == 0.0);
    ocp.subjectTo(AT_START, p_x_2 == 0.0);
    ocp.subjectTo(AT_START, p_y_2 == 0.19);
    ocp.subjectTo(AT_START, p_z_2 == 0.6);
    ocp.subjectTo(AT_START, q_w_2 == 1.0);
    ocp.subjectTo(AT_START, q_x_2 == 0.0);
    ocp.subjectTo(AT_START, q_y_2 == 0.0);
    ocp.subjectTo(AT_START, q_z_2 == 0.0);
    ocp.subjectTo(AT_START, v_x_2 == 0.0);
    ocp.subjectTo(AT_START, v_y_2 == 0.0);
    ocp.subjectTo(AT_START, v_z_2 == 0.0);

    // ocp.subjectTo(AT_START, T_1 == g_z * (Mq + 0.5 * Ml));
    // ocp.subjectTo(AT_START, T_2 == g_z * (Mq + 0.5 * Ml));
    ocp.subjectTo(AT_START, w_x_1 == 0.0);
    ocp.subjectTo(AT_START, w_y_1 == 0.0);
    ocp.subjectTo(AT_START, w_z_1 == 0.0);
    ocp.subjectTo(AT_START, w_x_2 == 0.0);
    ocp.subjectTo(AT_START, w_y_2 == 0.0);
    ocp.subjectTo(AT_START, w_z_2 == 0.0);

    ocp.subjectTo(AT_START, payload_x == 0.0);
    ocp.subjectTo(AT_START, payload_y == 0.0);
    ocp.subjectTo(AT_START, payload_z == 0.0);
    ocp.subjectTo(AT_START, payload_vx == 0);
    ocp.subjectTo(AT_START, payload_vy == 0);
    ocp.subjectTo(AT_START, payload_vz == 0);
  
    ocp.subjectTo(AT_START, payload_rx == 0);
    ocp.subjectTo(AT_START, payload_ry == 1);
    ocp.subjectTo(AT_START, payload_rz == 0);
    ocp.subjectTo(AT_START, payload_drx == 0);
    ocp.subjectTo(AT_START, payload_dry == 0);
    ocp.subjectTo(AT_START, payload_drz == 0);
    
    ocp.subjectTo(AT_START, cable_x_1 == 0.0);
    ocp.subjectTo(AT_START, cable_y_1 == 0.0);
    ocp.subjectTo(AT_START, cable_z_1 == -1.0);
    ocp.subjectTo(AT_START, dcable_x_1 == 0);
    ocp.subjectTo(AT_START, dcable_y_1 == 0.5);
    ocp.subjectTo(AT_START, dcable_z_1 == 0);
    ocp.subjectTo(AT_START, cable_x_2 == 0.0);
    ocp.subjectTo(AT_START, cable_y_2 == 0.0);
    ocp.subjectTo(AT_START, cable_z_2 == -1.0);
    ocp.subjectTo(AT_START, dcable_x_2 == 0);
    ocp.subjectTo(AT_START, dcable_y_2 == 0.5);
    ocp.subjectTo(AT_START, dcable_z_2 == 0);
  

    

    // Setup some visualization
    GnuplotWindow window1(PLOT_AT_EACH_ITERATION);
  
    window1.addSubplot(cable_x_1, "position x1");
    window1.addSubplot(cable_y_1, "position y1");
    window1.addSubplot(cable_z_1, "position z1");
    window1.addSubplot(cable_x_2, "position x2");
    window1.addSubplot(cable_y_2, "position y2");
    window1.addSubplot(cable_z_2, "position z2");

    // GnuplotWindow window2(PLOT_AT_EACH_ITERATION);
    // window2.addSubplot(payload_x, "payload position x2");
    // window2.addSubplot(payload_y, "payload position y2");
    // window2.addSubplot(payload_z, "payload position z2");
    // window1.addSubplot( v_x,"velocity x" );
    // window1.addSubplot( v_y,"velocity y" );
    // window1.addSubplot( v_z,"velocity z" );
    // window1.addSubplot(payload_x, "payload position x");
    // window1.addSubplot(payload_y, "payload position y");
    // window1.addSubplot(payload_z, "payload position z");
    //window1.addSubplot(cable_x*cable_x+cable_y*cable_y+cable_z*cable_z - 1.0,"cable length");
    // window1.addSubplot( v_x,"velocity x" );
    // window1.addSubplot( v_y,"velocity y" );
    // window1.addSubplot( v_z,"velocity z" );
    window1.addSubplot(dcable_x_1, "payload position x");
    window1.addSubplot(dcable_y_1, "payload position y");
    window1.addSubplot(dcable_z_1, "payload position z");
    //window1.addSubplot(cable_x*cable_x+cable_y*cable_y+cable_z*cable_z - 1.0,"cable length");
    // window1.addSubplot( ((payload_x-p_x)*(payload_x-p_x)+(payload_y-p_y)*(payload_y-p_y)+(payload_z-p_z)*(payload_z-p_z)-cable_length*cable_length) ,"distance" );

    GnuplotWindow window3(PLOT_AT_EACH_ITERATION);
    window3.addSubplot(T_1, "Thrust1");
    window3.addSubplot(w_x_1, "rotation-acc x1");
    window3.addSubplot(w_y_1, "rotation-acc y1");
    window3.addSubplot(w_z_1, "rotation-acc z1");
    window3.addSubplot(T_2, "Thrust");
    window3.addSubplot(w_x_2, "rotation-acc x2");
    window3.addSubplot(w_y_2, "rotation-acc y2");
    window3.addSubplot(w_z_2, "rotation-acc z2");
    
    // Define an algorithm to solve it.
    //OptimizationAlgorithm algorithm(ocp);
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set(INTEGRATOR_TOLERANCE, 1e-6);
    algorithm.set(KKT_TOLERANCE, 1e-3);
    algorithm << window1;
    //algorithm << window2;
    //algorithm << window3;
    algorithm.solve();
  }
  #else
  {
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);    // is robust, stable
    // mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // good convergence
    // mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); // due to qpOASES
    // mpc.set(INTEGRATOR_TYPE,        INT_RK4);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4); // accurate  INT_IRK_GL4
  
    mpc.set(CG_USE_OPENMP, YES);                    // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);     // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES); // time-varying costs
    mpc.set(USE_SINGLE_PRECISION, YES);             // Single precision
    mpc.set(NUM_INTEGRATOR_STEPS, N);
    mpc.set(QP_SOLVER, QP_QPOASES); // free, source code
    mpc.set(HOTSTART_QP, YES);
    mpc.set( LEVENBERG_MARQUARDT, 1e-10);

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if (mpc.exportCode("quadrotor_payload_mpc") != SUCCESSFUL_RETURN)
      exit(EXIT_FAILURE);
    mpc.printDimensionsQP();
  }
  #endif

  return EXIT_SUCCESS;
}