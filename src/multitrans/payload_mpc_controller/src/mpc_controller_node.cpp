#include "mpc_wrapper.h"
#include "mpc_fsm.h"
#include "mpc_controller.h"
#include <ros/ros.h>

std::unique_ptr<PayloadMPC::MPCFSM> fsm_ptr;
void MPC_controller_main(const ros::TimerEvent &)
{
    fsm_ptr->process();
}

void system_state_update_main(const ros::TimerEvent &)
{
    fsm_ptr->addNewForceObseverState();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPCctrl");
    ros::NodeHandle nh("~");

    PayloadMPC::MpcParams param;
    param.config_from_ros_handle(nh);

    PayloadMPC::MpcController controller(param);
    fsm_ptr.reset(new PayloadMPC::MPCFSM(nh, param, controller));
    PayloadMPC::MPCFSM &fsm = *fsm_ptr;
    ros::Subscriber state1_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state",
                                                                 10,
                                                                 boost::bind(&State_Data_t::feed, &fsm.state_data_1, _1));
    
    ros::Subscriber state2_sub = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state",
                                                                 10,
                                                                 boost::bind(&State_Data_t::feed, &fsm.state_data_2, _1));


    ros::Subscriber extended_state1_sub = nh.subscribe<mavros_msgs::ExtendedState>("/uav1/mavros/extended_state",
                                                                                  10,
                                                                                  boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data_1, _1));

    ros::Subscriber extended_state2_sub = nh.subscribe<mavros_msgs::ExtendedState>("/uav2/mavros/extended_state",
                                                                                  10,
                                                                                  boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data_2, _1));

    ros::Subscriber odom_1_sub =
        nh.subscribe<nav_msgs::Odometry>("/uav1/odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data_1, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber odom_2_sub =
        nh.subscribe<nav_msgs::Odometry>("/uav2/odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data_2, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber start_trig_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("start_trigger",
                                                 10,
                                                 boost::bind(&Start_Trigger_Data_t::feed, &fsm.start_trigger_data, _1));

    ros::Subscriber cmd_trig_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("cmd_trigger",
                                                 10,
                                                 boost::bind(&Cmd_Trigger_Data_t::feed, &fsm.cmd_trigger_data, _1));

    ros::Subscriber mpc_traj_sub =
        nh.subscribe<quadrotor_msgs::PolynomialTraj>("traj",
                                                     100,
                                                     boost::bind(&Trajectory_Data_t::feed, &fsm.trajectory_data, _1),
                                                     ros::VoidConstPtr(),
                                                     ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_1_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("/uav1/cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data_1, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_2_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("/uav2/cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data_2, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_1_sub =
        nh.subscribe<sensor_msgs::Imu>("/uav1/imu_data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data_1, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    fsm.imu_data_1.set_filter_params(param.filter_param_.sample_freq_quad_acc, param.filter_param_.cutoff_freq_quad_acc,
                                     param.filter_param_.sample_freq_quad_omg, param.filter_param_.cutoff_freq_quad_omg);
    
    ros::Subscriber imu_2_sub =
        nh.subscribe<sensor_msgs::Imu>("/uav2/imu_data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data_2, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber cable_1_info_data_sub =
        nh.subscribe<sensor_msgs::Imu>("/cable_1_info/data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.cable_1_info_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    fsm.cable_1_info_data.set_filter_params(param.filter_param_.sample_freq_dcable, param.filter_param_.cutoff_freq_dcable,
                                            param.filter_param_.sample_freq_cable, param.filter_param_.cutoff_freq_cable);
    
    ros::Subscriber cable_2_info_data_sub =
        nh.subscribe<sensor_msgs::Imu>("/cable_2_info/data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.cable_2_info_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    fsm.cable_2_info_data.set_filter_params(param.filter_param_.sample_freq_dcable, param.filter_param_.cutoff_freq_dcable,
                                            param.filter_param_.sample_freq_cable, param.filter_param_.cutoff_freq_cable);

    ros::Subscriber payload_imu_data_sub =
        nh.subscribe<sensor_msgs::Imu>("load_imu/data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.payload_imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    fsm.payload_imu_data.set_filter_params(param.filter_param_.sample_freq_load_acc, param.filter_param_.cutoff_freq_load_acc,
                                           param.filter_param_.sample_freq_load_omg, param.filter_param_.cutoff_freq_load_omg);
    ros::Subscriber payload_odom_sub =
        nh.subscribe<nav_msgs::Odometry>("/payloadodom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.payload_odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_1_sub;
    {
        rc_1_sub = nh.subscribe<mavros_msgs::RCIn>("/uav1/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data_1, _1));
    }

    ros::Subscriber rc_2_sub;
    {
        rc_2_sub = nh.subscribe<mavros_msgs::RCIn>("/uav2/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data_2, _1));
    }


    ros::Subscriber bat_1_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/uav1/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data_1, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber bat_2_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/uav2/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data_2, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    ros::Subscriber esc_1_sub =
        nh.subscribe<mavros_msgs::ESCTelemetry>("/uav1/mavros/esc_telemetry",
                                                100,
                                                boost::bind(&Rpm_Data_t::feed, &fsm.rpm_data_1, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    fsm.rpm_data_1.set_filter_params(param.filter_param_.sample_freq_rpm, param.filter_param_.cutoff_freq_rpm);

    ros::Subscriber esc_2_sub =
        nh.subscribe<mavros_msgs::ESCTelemetry>("/uav2/mavros/esc_telemetry",
                                                100,
                                                boost::bind(&Rpm_Data_t::feed, &fsm.rpm_data_2, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    fsm.rpm_data_2.set_filter_params(param.filter_param_.sample_freq_rpm, param.filter_param_.cutoff_freq_rpm);

    fsm.planning_stop_pub_ = nh.advertise<std_msgs::Empty>("/planning_stop_trigger", 10);
    fsm.planning_restart_pub_ = nh.advertise<std_msgs::Empty>("/planning_restart_trigger", 10);
    fsm.ctrl_FCU_1_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/attitude", 10);
    fsm.ctrl_FCU_2_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav2/mavros/setpoint_raw/attitude", 10);

    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    fsm.des_yaw_pub = nh.advertise<nav_msgs::Odometry>("/des_yaw_pub", 10);

    // fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv_1 = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    fsm.arming_client_srv_1 = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    fsm.reboot_FCU_srv_1 = nh.serviceClient<mavros_msgs::CommandLong>("/uav1/mavros/cmd/command");


    fsm.set_FCU_mode_srv_1 = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    fsm.arming_client_srv_1 = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    fsm.reboot_FCU_srv_1 = nh.serviceClient<mavros_msgs::CommandLong>("/uav2/mavros/cmd/command");

    ros::Duration(0.5).sleep();
    if (!param.use_simulation_)
    {
        ROS_INFO("[MPCCTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_1_is_received(ros::Time::now()) && fsm.rc_2_is_received(ros::Time::now()))
            {
                ROS_INFO("[MPCCTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
        int trials = 0;
        while (ros::ok() && !fsm.state_data_1.current_state.connected && !fsm.state_data_2.current_state.connected)
        {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (trials++ > 5)
                ROS_ERROR("Unable to connnect to PX4!!!");
        }
    }
    else
    {
        ROS_WARN("[MPCCTRL] Remote controller disabled, be careful!");
    }

    // Create a ROS timer for force observer
    ros::Timer force_state_timer = nh.createTimer(ros::Duration(1.0 / param.force_estimator_param_.force_observer_freq), system_state_update_main);
    // Create a ROS timer for controller
    ros::Timer MPC_controller_main_timer = nh.createTimer(ros::Duration(1.0 / param.ctrl_freq_max_), MPC_controller_main);
    // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.

    ros::spin();

    return 0;
}
