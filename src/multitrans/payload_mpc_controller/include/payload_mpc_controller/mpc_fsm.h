#ifndef __MPCFSM_H
#define __MPCFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "mpc_params.h"
#include "mpc_input.h"
#include "mpc_controller.h"
#include "polynomial_trajectory.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ESCStatus.h>
#include "multi_optimization_based_force_estimator.hpp"
#include "std_msgs/Float64.h"


namespace PayloadMPC
{
    class MPCFSM
    {
        public:
            //***************PX4CTRL Real *****************
            RC_Data_t rc_data_1;
            State_Data_t state_data_1;
            ExtendedState_Data_t extended_state_data_1;
            Odom_Data_t odom_data_1;
            Imu_Data_t imu_data_1;
            Command_Data_t cmd_data_1;
            Battery_Data_t bat_data_1;
            Rpm_Data_t rpm_data_1;

            RC_Data_t rc_data_2;
            State_Data_t state_data_2;
            ExtendedState_Data_t extended_state_data_2;
            Odom_Data_t odom_data_2;
            Imu_Data_t imu_data_2;
            Command_Data_t cmd_data_2;
            Battery_Data_t bat_data_2;
            Rpm_Data_t rpm_data_2;

            Odom_Data_t payload_odom_data;
            Imu_Data_t payload_imu_data;
            Imu_Data_t cable_1_info_data;
            Imu_Data_t cable_2_info_data;

            Start_Trigger_Data_t start_trigger_data;
            Cmd_Trigger_Data_t cmd_trigger_data;
            Trajectory_Data_t trajectory_data;

            ros::Publisher traj_start_trigger_pub;
            ros::Publisher ctrl_FCU_1_pub, ctrl_FCU_2_pub;
            ros::Publisher pub_force_marker_, pub_force_;

            ros::Publisher debug_pub; // debug
            ros::ServiceClient set_FCU_mode_srv_1;
            ros::ServiceClient arming_client_srv_1;
            ros::ServiceClient reboot_FCU_srv_1;

            ros::ServiceClient set_FCU_mode_srv_2;
            ros::ServiceClient arming_client_srv_2;
            ros::ServiceClient reboot_FCU_srv_2;

            ros::Publisher des_yaw_pub;
            ros::Publisher planning_stop_pub_;
            ros::Publisher planning_restart_pub_;

            ros::Publisher  uav1_path_pub_;
            ros::Publisher  uav2_path_pub_;
            ros::Publisher  payload_path_pub_;
            ros::Subscriber uav_2_odom_sub_;
            ros::Subscriber uav_1_odom_sub_;
            ros::Subscriber payload_odom_sub_;

            nav_msgs::Path uav1_path_;
            nav_msgs::Path uav2_path_;
            nav_msgs::Path payload_path_;

            Eigen::Vector3d hover_pose_;
            double hover_yaw_;
            ros::Time last_set_hover_pose_time;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            enum State_t
            {
                MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
                AUTO_HOVER,		 // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
                CMD_CTRL,		 // px4ctrl is actived, and controling the drone.
                AUTO_TAKEOFF,
                AUTO_LAND,
            };

            enum Exec_Traj_State_t
            {
                HOVER = 10,		// execute the hover trajectory
                POLY_TRAJ = 11, // execute the polynomial trajectory
                POINTS = 12,	// execute the point trajectory
            };

            MPCFSM(const ros::NodeHandle &nh, MpcParams &params, MpcController &controller);

            void process();
            void CMD_CTRL_process();

            bool rc_1_is_received(const ros::Time &now_time);
            bool odom_1_is_received(const ros::Time &now_time);
            bool imu_1_is_received(const ros::Time &now_time);
            bool bat_1_is_received(const ros::Time &now_time);
            bool recv_new_odom_1();
            bool rc_2_is_received(const ros::Time &now_time);
            bool odom_2_is_received(const ros::Time &now_time);
            bool imu_2_is_received(const ros::Time &now_time);
            bool bat_2_is_received(const ros::Time &now_time);
            bool recv_new_odom_2();
            void addNewForceObseverState();


            void Uav1OdomCallback(const nav_msgs::OdometryConstPtr &msg);
            void Uav2OdomCallback(const nav_msgs::OdometryConstPtr &msg);
            void PayLoadCallback(const nav_msgs::OdometryConstPtr &msg);
        private:

            // Subscribers and publisher.
            ros::Publisher pub_control_command, pub_predicted_trajectory_1, pub_predicted_trajectory_2, pub_payload_predicted_trajectory_, pub_reference_trajectory_1, pub_reference_trajectory_2, pub_payload_reference_trajectory_, pub_cable_1, pub_cable_2;
            ros::Publisher pub_all_ref_data_, pub_rmse_info_;
            State_t fsm_state; // Should only be changed in PX4CtrlFSM::process() function!

            Exec_Traj_State_t exec_traj_state_;

            // Handles
            ros::NodeHandle nh_;

            MpcParams &params_;
            MpcController &controller_;
            MultiOptForceEstimator force_estimator_;

            long int rmse_cnt_ = 0;
            double rmse_sum_ = 0, rmse_payload_sum_ = 0;
            double rmse_xy_sum_ = 0, rmse_payload_xy_sum_ = 0;
            double payload_max_ = 0, drone_max_ = 0;
            double payload_max_xy_ = 0, drone_max_xy_ = 0;

            double len, last_len;
            Eigen::Matrix<real_t, kStateSize, 1> est_state_;
            Eigen::Matrix<real_t, kStateSize, kSamples + 1> mpc_predicted_states_;
            Eigen::Matrix<real_t, kInputSize, kSamples> mpc_predicted_inputs_;
            Eigen::Vector3d fq_, fl_;

            void setEstimateState(const Odom_Data_t &odom_1_est_state, const Odom_Data_t &odom_2_est_state, const Odom_Data_t &odom_payload_state, 
								  const Imu_Data_t &cable_1_info_data, const Imu_Data_t &cable_2_info_data);
            void setForceEstimation();

            void publishPrediction(const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
                                const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> predicted_traj,
                                ros::Time &time, double dt);

            void publish_bodyrate_ctrl_1(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
                                    const ros::Time &stamp);

            void publish_bodyrate_ctrl_2(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
                                    const ros::Time &stamp);
            // ---- tools ----
            void printandresetRMSE();
            void addRMSE();
            void update_hover_pose();
            void update_hover_with_rc();
            void publish_trigger(const nav_msgs::Odometry &odom_msg);

            bool toggle_offboard_mode_1(bool on_off); // It will only try to toggle once, so not blocked.
            bool toggle_offboard_mode_2(bool on_off); // It will only try to toggle once, so not blocked.

            bool toggle_arm_disarm_1(bool arm);		// It will only try to toggle once, so not blocked.
            bool toggle_arm_disarm_2(bool arm);     // It will only try to toggle once, so not blocked. 
            
            void reboot_FCU_1();
            void reboot_FCU_2();
            
    };
}


#endif