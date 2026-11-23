#include "so3_quadrotor/two_quadrotor_dynamics.hpp"
#include <quadrotor_msgs/SO3Command.h>
#include <mavros_msgs/ESCTelemetry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <random>


namespace so3_quadrotor
{

  class Nodelet : public nodelet::Nodelet
  {
    private:
      std::shared_ptr<TwoQuadrotor> twoquadrotorPtr_;
      Control control_1, control_2;
      Cmd     cmd_1, cmd_2;

      ros::Publisher uav1_odom_pub_, uav2_odom_pub_, uav1_imu_pub_, uav2_imu_pub_, vis_pub_, payload_odom_pub_, uav1_cable_info_pub_, uav2_cable_info_pub_;
      ros::Publisher uav1_rpm_pub_, uav2_rpm_pub_, load_imu_pub_;
      ros::Subscriber uav1_cmd_sub_, uav2_cmd_sub_;

      ros::Timer simulation_timer;
      tf::TransformBroadcaster tf_br_;

      double noise_uav_, noise_rpm_, noise_payload_, noise_acc_uav_, noise_acc_payload_, noise_vel_uav_, noise_vel_payload_;
      std::default_random_engine generator;
      std::normal_distribution<double> distribution{0.0,1.0};

      // parameters
      int simulation_rate_ = 1e3;
      int odom_rate_ = 400;

      // msgs 
      nav_msgs::Odometry uav1_odom_msg_, uav2_odom_msg_;
      nav_msgs::Odometry odom_payload_msg_;
      sensor_msgs::Imu uav1_imu_msg_, uav2_imu_msg_;
      sensor_msgs::Imu load_imu_msg_;
      sensor_msgs::Imu uav1_cable_info_msg_, uav2_cable_info_msg_;
      mavros_msgs::ESCTelemetry uav1_rpm_msg_, uav2_rpm_msg_;
      visualization_msgs::MarkerArray vis_msg_;
      double payload_size_;
      double motor_yaw_1[4] = {0,0,0,0};
      double motor_yaw_2[4] = {0,0,0,0};

      Eigen::Vector3d guass_random(double std)
      {
        Eigen::Vector3d noise(distribution(generator),distribution(generator),distribution(generator));
        return noise*std;
      }

      Eigen::Vector4d guass_random4d(double std)
      {
        Eigen::Vector4d noise(distribution(generator),distribution(generator),distribution(generator),distribution(generator));
        return noise*std;
      }

      void uav1_cmd_callback(const mavros_msgs::AttitudeTarget::ConstPtr& cmd_msg)
      {
        cmd_1.thrust = cmd_msg->thrust;
        cmd_1.body_rates[0] = cmd_msg->body_rate.x;
        cmd_1.body_rates[1] = cmd_msg->body_rate.y;
        cmd_1.body_rates[2] = cmd_msg->body_rate.z;
      }

      void uav2_cmd_callback(const mavros_msgs::AttitudeTarget::ConstPtr& cmd_msg)
      {
        cmd_2.thrust = cmd_msg->thrust;
        cmd_2.body_rates[0] = cmd_msg->body_rate.x;
        cmd_2.body_rates[1] = cmd_msg->body_rate.y;
        cmd_2.body_rates[2] = cmd_msg->body_rate.z;
      }

      Eigen::Quaterniond vector_to_quaternion(Eigen::Vector3d roteq)
      {
        Eigen::Vector3d q_ref(0.0, 1.0, 0.0);
        double y = q_ref.dot(roteq);
        Eigen::Vector3d cross = q_ref.cross(roteq);
        if(y > 0.999999)
        {
          return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        }
        if (y < -0.999999)
        {
          return Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);
        }
        Eigen::Vector3d axis = cross / cross.norm();
        double theta = std::acos(y);
        double q_w = std::cos(theta / 2);
        Eigen::Vector3d q_xyz = axis * std::sin(theta / 2);
        return Eigen::Quaterniond(q_w, q_xyz[0], q_xyz[1], q_xyz[2]);
      }

      void timer_callback(const ros::TimerEvent& event)
      {
        auto last_control_1 = control_1;
        auto last_control_2 = control_2;
        control_1 = twoquadrotorPtr_->getControl_1(cmd_1);
        control_2 = twoquadrotorPtr_->getControl_2(cmd_2);
        // std::cout << control_1.rpm[0] << " " << control_1.rpm[1] << " " << control_1.rpm[2] << " " << control_1.rpm[3] << std::endl;
        // std::cout << cmd_1.thrust << " " << cmd_1.body_rates[0] << " " << cmd_1.body_rates[1] << " " << cmd_1.body_rates[2] << std::endl;

        for (size_t i = 0; i < 4; i++)
        {
          if (std::isnan(control_1.rpm[i]))
          {
            control_1.rpm[i] = last_control_1.rpm[i];
          }
          if (std::isnan(control_2.rpm[i]))
          {
            control_2.rpm[i] = last_control_2.rpm[i];
          }
        }

        twoquadrotorPtr_->setInput_1(control_1.rpm[0], control_1.rpm[1], control_1.rpm[2], control_1.rpm[3]);
        twoquadrotorPtr_->setInput_2(control_2.rpm[0], control_2.rpm[1], control_2.rpm[2], control_2.rpm[3]);
        twoquadrotorPtr_->step(1.0/simulation_rate_);
        
        const Eigen::Vector3d&      uav1pos   = twoquadrotorPtr_->getUAV1Pos() + guass_random(noise_uav_);
        const Eigen::Vector3d&      uav2pos   = twoquadrotorPtr_->getUAV2Pos() + guass_random(noise_uav_);
        const Eigen::Vector3d&      uav1vel   = twoquadrotorPtr_->getUAV1Vel() + guass_random(noise_vel_uav_);
        const Eigen::Vector3d&      uav2vel   = twoquadrotorPtr_->getUAV2Vel() + guass_random(noise_vel_uav_);
              Eigen::Vector3d       uav1acc   = twoquadrotorPtr_->getUAV1Acc() + guass_random(noise_acc_uav_);
              Eigen::Vector3d       uav2acc   = twoquadrotorPtr_->getUAV2Acc() + guass_random(noise_acc_uav_);
        const Eigen::Quaterniond&   uav1quat  = twoquadrotorPtr_->getUAV1Quat();
        const Eigen::Quaterniond&   uav2quat  = twoquadrotorPtr_->getUAV2Quat();
        const Eigen::Vector3d&      uav1omega = twoquadrotorPtr_->getUAV1Omega();
        const Eigen::Vector3d&      uav2omega = twoquadrotorPtr_->getUAV2Omega();
        const Eigen::Vector3d&      rotepos = twoquadrotorPtr_->getLoadPos() + guass_random(noise_payload_);
        const Eigen::Vector3d&      rotevel = twoquadrotorPtr_->getLoadVel() + guass_random(noise_vel_payload_);
              Eigen::Vector3d       roteacc = twoquadrotorPtr_->getLoadAcc() + guass_random(noise_acc_payload_);
        const Eigen::Vector3d       uav1q   = twoquadrotorPtr_->getUAV1q();
        const Eigen::Vector3d       uav1dq  = twoquadrotorPtr_->getUAV1dq();
        const Eigen::Vector3d       uav2q   = twoquadrotorPtr_->getUAV2q();
        const Eigen::Vector3d       uav2dq  = twoquadrotorPtr_->getUAV2dq();
        const Eigen::Vector3d       roteq   = twoquadrotorPtr_->getLoadq();
        const Eigen::Vector3d       rotedq  = twoquadrotorPtr_->getLoaddq();
        const Eigen::Vector4d       uav1rpm = twoquadrotorPtr_->getUAV1Rpm() + guass_random4d(noise_rpm_);
        const Eigen::Vector4d       uav2rpm = twoquadrotorPtr_->getUAV2Rpm() + guass_random4d(noise_rpm_);
        
        const Eigen::Vector3d       uav1attachPos = twoquadrotorPtr_->getUAV1attachPos();
        const Eigen::Vector3d       uav2attachPos = twoquadrotorPtr_->getUAV2attachPos();
        //Eigen::Vector3d fake_q_1 = uav1p
        uav1acc = uav1acc + Eigen::Vector3d(0,0,twoquadrotorPtr_->config.g);
        uav1acc = uav1quat.toRotationMatrix().transpose() * uav1acc; //Convert to inertial frame
        uav2acc = uav2acc + Eigen::Vector3d(0,0,twoquadrotorPtr_->config.g);
        uav2acc = uav2quat.toRotationMatrix().transpose() * uav1acc; //Convert to inertial frame
        roteacc = roteacc + Eigen::Vector3d(0,0,twoquadrotorPtr_->config.g); 

        static ros::Time next_odom_pub_time = ros::Time::now();
        ros::Time tnow = ros::Time::now();
        if (tnow >= next_odom_pub_time)
        {
          next_odom_pub_time += ros::Duration(1.0/odom_rate_);

          // tf
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(uav1pos.x(), uav1pos.y(), uav1pos.z()) );
          transform.setRotation( tf::Quaternion(uav1quat.x(), uav1quat.y(), uav1quat.z(), uav1quat.w()) );
          tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav1"));
          transform.setOrigin( tf::Vector3(uav2pos.x(), uav2pos.y(), uav2pos.z()) );
          transform.setRotation( tf::Quaternion(uav2quat.x(), uav2quat.y(), uav2quat.z(), uav2quat.w()) );
          tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav2"));

          // uav1 and uav2 odom
          uav1_odom_msg_.header.stamp = tnow;
          uav1_odom_msg_.pose.pose.position.x = uav1pos(0) ;
          uav1_odom_msg_.pose.pose.position.y = uav1pos(1) ;
          uav1_odom_msg_.pose.pose.position.z = uav1pos(2) ;
          uav1_odom_msg_.pose.pose.orientation.x = uav1quat.x();
          uav1_odom_msg_.pose.pose.orientation.y = uav1quat.y();
          uav1_odom_msg_.pose.pose.orientation.z = uav1quat.z();
          uav1_odom_msg_.pose.pose.orientation.w = uav1quat.w();

          uav1_odom_msg_.twist.twist.linear.x = uav1vel(0);
          uav1_odom_msg_.twist.twist.linear.y = uav1vel(1);
          uav1_odom_msg_.twist.twist.linear.z = uav1vel(2);

          uav1_odom_msg_.twist.twist.angular.x = uav1omega(0);
          uav1_odom_msg_.twist.twist.angular.y = uav1omega(1);
          uav1_odom_msg_.twist.twist.angular.z = uav1omega(2);
          uav1_odom_pub_.publish(uav1_odom_msg_);

          uav2_odom_msg_.header.stamp = tnow;
          uav2_odom_msg_.pose.pose.position.x = uav2pos(0) ;
          uav2_odom_msg_.pose.pose.position.y = uav2pos(1) ;
          uav2_odom_msg_.pose.pose.position.z = uav2pos(2) ;
          uav2_odom_msg_.pose.pose.orientation.x = uav2quat.x();
          uav2_odom_msg_.pose.pose.orientation.y = uav2quat.y();
          uav2_odom_msg_.pose.pose.orientation.z = uav2quat.z();
          uav2_odom_msg_.pose.pose.orientation.w = uav2quat.w();

          uav2_odom_msg_.twist.twist.linear.x = uav2vel(0);
          uav2_odom_msg_.twist.twist.linear.y = uav2vel(1);
          uav2_odom_msg_.twist.twist.linear.z = uav2vel(2);

          uav2_odom_msg_.twist.twist.angular.x = uav2omega(0);
          uav2_odom_msg_.twist.twist.angular.y = uav2omega(1);
          uav2_odom_msg_.twist.twist.angular.z = uav2omega(2);
          uav2_odom_pub_.publish(uav2_odom_msg_);

          // payload odom 
          odom_payload_msg_.header.stamp = tnow;
          odom_payload_msg_.pose.pose.position.x = rotepos(0) ;
          odom_payload_msg_.pose.pose.position.y = rotepos(1) ;
          odom_payload_msg_.pose.pose.position.z = rotepos(2) ;
          Eigen::Quaterniond payload_quad =  vector_to_quaternion(roteq);
          odom_payload_msg_.pose.pose.orientation.x = payload_quad.x();
          odom_payload_msg_.pose.pose.orientation.y = payload_quad.y();
          odom_payload_msg_.pose.pose.orientation.z = payload_quad.z();
          odom_payload_msg_.pose.pose.orientation.w = payload_quad.w();

          odom_payload_msg_.twist.twist.linear.x = rotevel(0);
          odom_payload_msg_.twist.twist.linear.y = rotevel(1);
          odom_payload_msg_.twist.twist.linear.z = rotevel(2);

          odom_payload_msg_.twist.twist.angular.x = rotedq(0);
          odom_payload_msg_.twist.twist.angular.y = rotedq(1);
          odom_payload_msg_.twist.twist.angular.z = rotedq(2);
          payload_odom_pub_.publish(odom_payload_msg_);


          // uav1 and uav2 imu
          uav1_imu_msg_.header.stamp = tnow;
          uav1_imu_msg_.orientation.x = uav1quat.x();
          uav1_imu_msg_.orientation.y = uav1quat.y();
          uav1_imu_msg_.orientation.z = uav1quat.z();
          uav1_imu_msg_.orientation.w = uav1quat.w();

          uav1_imu_msg_.angular_velocity.x = uav1omega(0);
          uav1_imu_msg_.angular_velocity.y = uav1omega(1);
          uav1_imu_msg_.angular_velocity.z = uav1omega(2);

          uav1_imu_msg_.linear_acceleration.x = uav1acc[0];
          uav1_imu_msg_.linear_acceleration.y = uav1acc[1];
          uav1_imu_msg_.linear_acceleration.z = uav1acc[2];
          uav1_imu_pub_.publish(uav1_imu_msg_);

          uav2_imu_msg_.header.stamp = tnow;
          uav2_imu_msg_.orientation.x = uav2quat.x();
          uav2_imu_msg_.orientation.y = uav2quat.y();
          uav2_imu_msg_.orientation.z = uav2quat.z();
          uav2_imu_msg_.orientation.w = uav2quat.w();

          uav2_imu_msg_.angular_velocity.x = uav2omega(0);
          uav2_imu_msg_.angular_velocity.y = uav2omega(1);
          uav2_imu_msg_.angular_velocity.z = uav2omega(2);

          uav2_imu_msg_.linear_acceleration.x = uav2acc[0];
          uav2_imu_msg_.linear_acceleration.y = uav2acc[1];
          uav2_imu_msg_.linear_acceleration.z = uav2acc[2];
          uav2_imu_pub_.publish(uav2_imu_msg_);

          // uav1 and uav2 cable info
          uav1_cable_info_msg_.header.stamp = tnow;
          uav1_cable_info_msg_.orientation.x = 0;
          uav1_cable_info_msg_.orientation.y = 0;
          uav1_cable_info_msg_.orientation.z = 0;
          uav1_cable_info_msg_.orientation.w = 1;

          uav1_cable_info_msg_.angular_velocity.x = uav1q(0);
          uav1_cable_info_msg_.angular_velocity.y = uav1q(1);
          uav1_cable_info_msg_.angular_velocity.z = uav1q(2);

          uav1_cable_info_msg_.linear_acceleration.x = uav1dq[0];
          uav1_cable_info_msg_.linear_acceleration.y = uav1dq[1];
          uav1_cable_info_msg_.linear_acceleration.z = uav1dq[2];
          uav1_cable_info_pub_.publish(uav1_cable_info_msg_);

          uav2_cable_info_msg_.header.stamp = tnow;
          uav2_cable_info_msg_.orientation.x = 0;
          uav2_cable_info_msg_.orientation.y = 0;
          uav2_cable_info_msg_.orientation.z = 0;
          uav2_cable_info_msg_.orientation.w = 1;

          uav2_cable_info_msg_.angular_velocity.x = uav2q(0);
          uav2_cable_info_msg_.angular_velocity.y = uav2q(1);
          uav2_cable_info_msg_.angular_velocity.z = uav2q(2);

          uav2_cable_info_msg_.linear_acceleration.x = uav2dq[0];
          uav2_cable_info_msg_.linear_acceleration.y = uav2dq[1];
          uav2_cable_info_msg_.linear_acceleration.z = uav2dq[2];
          uav2_cable_info_pub_.publish(uav2_cable_info_msg_);

          //RPM 
          uav1_rpm_msg_.header = uav1_imu_msg_.header;
          uav1_rpm_msg_.esc_telemetry[0].header = uav1_imu_msg_.header;
          uav1_rpm_msg_.esc_telemetry[0].rpm = uav1rpm[0];
          uav1_rpm_msg_.esc_telemetry[1].header = uav1_imu_msg_.header;
          uav1_rpm_msg_.esc_telemetry[1].rpm = uav1rpm[1];
          uav1_rpm_msg_.esc_telemetry[2].header = uav1_imu_msg_.header;
          uav1_rpm_msg_.esc_telemetry[2].rpm = uav1rpm[2];
          uav1_rpm_msg_.esc_telemetry[3].header = uav1_imu_msg_.header;
          uav1_rpm_msg_.esc_telemetry[3].rpm = uav1rpm[3];
          uav1_rpm_pub_.publish(uav1_rpm_msg_);

          uav2_rpm_msg_.header = uav2_imu_msg_.header;
          uav2_rpm_msg_.esc_telemetry[0].header = uav2_imu_msg_.header;
          uav2_rpm_msg_.esc_telemetry[0].rpm = uav2rpm[0];
          uav2_rpm_msg_.esc_telemetry[1].header = uav2_imu_msg_.header;
          uav2_rpm_msg_.esc_telemetry[1].rpm = uav2rpm[1];
          uav2_rpm_msg_.esc_telemetry[2].header = uav2_imu_msg_.header;
          uav2_rpm_msg_.esc_telemetry[2].rpm = uav2rpm[2];
          uav2_rpm_msg_.esc_telemetry[3].header = uav2_imu_msg_.header;
          uav2_rpm_msg_.esc_telemetry[3].rpm = uav2rpm[3];
          uav2_rpm_pub_.publish(uav2_rpm_msg_);


          // IMU payload
          load_imu_msg_.header = uav1_imu_msg_.header;
          load_imu_msg_.orientation.x = 0;
          load_imu_msg_.orientation.y = 0;
          load_imu_msg_.orientation.z = 0;
          load_imu_msg_.orientation.w = 1;
          load_imu_msg_.angular_velocity.x = 0;
          load_imu_msg_.angular_velocity.y = 0;
          load_imu_msg_.angular_velocity.z = 0;
          load_imu_msg_.linear_acceleration.x = roteacc[0];
          load_imu_msg_.linear_acceleration.y = roteacc[1];
          load_imu_msg_.linear_acceleration.z = roteacc[2];
          load_imu_msg_.angular_velocity.x = rotedq[0];
          load_imu_msg_.angular_velocity.y = rotedq[1];
          load_imu_msg_.angular_velocity.z = rotedq[2]; 
          load_imu_pub_.publish(load_imu_msg_);

          // drone visualization
          std::vector<Eigen::Vector3d> propellers;
          propellers.emplace_back(0, +twoquadrotorPtr_->config.arm_length/2, 0.02);
          propellers.emplace_back(0, -twoquadrotorPtr_->config.arm_length/2, 0.02);
          propellers.emplace_back(-twoquadrotorPtr_->config.arm_length/2, 0, 0.02);
          propellers.emplace_back(+twoquadrotorPtr_->config.arm_length/2, 0, 0.02);

          for (size_t i = 0; i < 4; i++)
          {
            double rpm_1 = twoquadrotorPtr_->state.motor_rpm_1.coeff(i);
            double rpm_2 = twoquadrotorPtr_->state.motor_rpm_2.coeff(i);
            if (i/2) {
              motor_yaw_1[i] += rpm_1*60 / odom_rate_;
              motor_yaw_2[i] += rpm_2*60 / odom_rate_;
              
            } else {
              motor_yaw_1[i] -= rpm_1*60 / odom_rate_;
              motor_yaw_2[i] -= rpm_2*60 / odom_rate_;
            } 
            motor_yaw_1[i] = std::fmod(motor_yaw_1[i], 2*M_PI);
            motor_yaw_2[i] = std::fmod(motor_yaw_2[i], 2*M_PI);

            Eigen::Quaterniond quat_propeller1 = uav1quat * uav_utils::ypr_to_quaternion(Eigen::Vector3d(motor_yaw_1[i], 0, 0));
            Eigen::Quaterniond quat_propeller2 = uav2quat * uav_utils::ypr_to_quaternion(Eigen::Vector3d(motor_yaw_2[i], 0, 0));
            Eigen::Vector3d pos_propeller1 = uav1pos + uav1quat.toRotationMatrix() * propellers[i];
            Eigen::Vector3d pos_propeller2 = uav2pos + uav2quat.toRotationMatrix() * propellers[i];
            vis_msg_.markers[i].pose.position.x = pos_propeller1(0);
            vis_msg_.markers[i].pose.position.y = pos_propeller1(1);
            vis_msg_.markers[i].pose.position.z = pos_propeller1(2);
            vis_msg_.markers[i].pose.orientation.x = quat_propeller1.x();
            vis_msg_.markers[i].pose.orientation.y = quat_propeller1.y();
            vis_msg_.markers[i].pose.orientation.z = quat_propeller1.z();
            vis_msg_.markers[i].pose.orientation.w = quat_propeller1.w();
            vis_msg_.markers[i+4].pose.position.x = pos_propeller2(0);
            vis_msg_.markers[i+4].pose.position.y = pos_propeller2(1);
            vis_msg_.markers[i+4].pose.position.z = pos_propeller2(2);
            vis_msg_.markers[i+4].pose.orientation.x = quat_propeller2.x();
            vis_msg_.markers[i+4].pose.orientation.y = quat_propeller2.y();
            vis_msg_.markers[i+4].pose.orientation.z = quat_propeller2.z();
            vis_msg_.markers[i+4].pose.orientation.w = quat_propeller2.w();
          }
          for (size_t i=8; i<10; ++i) {
            vis_msg_.markers[i].pose.position.x = uav1pos(0);
            vis_msg_.markers[i].pose.position.y = uav1pos(1);
            vis_msg_.markers[i].pose.position.z = uav1pos(2);
            vis_msg_.markers[i].pose.orientation.x = uav1quat.x();
            vis_msg_.markers[i].pose.orientation.y = uav1quat.y();
            vis_msg_.markers[i].pose.orientation.z = uav1quat.z();
            vis_msg_.markers[i].pose.orientation.w = uav1quat.w();
            vis_msg_.markers[i].id = i;
            vis_msg_.markers[i+2].pose.position.x = uav2pos(0);
            vis_msg_.markers[i+2].pose.position.y = uav2pos(1);
            vis_msg_.markers[i+2].pose.position.z = uav2pos(2);
            vis_msg_.markers[i+2].pose.orientation.x = uav2quat.x();
            vis_msg_.markers[i+2].pose.orientation.y = uav2quat.y();
            vis_msg_.markers[i+2].pose.orientation.z = uav2quat.z();
            vis_msg_.markers[i+2].pose.orientation.w = uav2quat.w();
            vis_msg_.markers[i+2].id = i+2;
          }
          vis_msg_.markers[12].points.clear();
          vis_msg_.markers[12].pose.orientation.w = 1;
          geometry_msgs::Point p;
          p.x = uav1attachPos(0);
          p.y = uav1attachPos(1);
          p.z = uav1attachPos(2);
          vis_msg_.markers[12].points.push_back(p);
          p.x = uav2attachPos(0);
          p.y = uav2attachPos(1);
          p.z = uav2attachPos(2);
          vis_msg_.markers[12].points.push_back(p);
          vis_msg_.markers[13].points.clear();
          vis_msg_.markers[13].pose.orientation.w = 1;
          p.x = uav1pos(0);
          p.y = uav1pos(1);
          p.z = uav1pos(2);
          vis_msg_.markers[13].points.push_back(p);
          p.x = uav1attachPos(0) ;
          p.y = uav1attachPos(1);
          p.z = uav1attachPos(2);
          vis_msg_.markers[13].points.push_back(p);

          vis_msg_.markers[14].points.clear();
          vis_msg_.markers[14].pose.orientation.w = 1;
          
          p.x = uav2pos(0);
          p.y = uav2pos(1);
          p.z = uav2pos(2);
          vis_msg_.markers[14].points.push_back(p);
          p.x = uav2attachPos(0);
          p.y = uav2attachPos(1);
          p.z = uav2attachPos(2);
          vis_msg_.markers[14].points.push_back(p);

          vis_pub_.publish(vis_msg_);

        } 
      }

    public:

      void onInit(void)
      {
        ros::NodeHandle nh(getPrivateNodeHandle());
        // parameters
        double init_uav1_x, init_uav1_y, init_uav1_z, init_uav1_yaw;
        double init_uav2_x, init_uav2_y, init_uav2_z, init_uav2_yaw;
        double init_rote_x, init_rote_y, init_rote_z;
        nh.getParam("init_uav1_x", init_uav1_x);
        nh.getParam("init_uav1_y", init_uav1_y);
        nh.getParam("init_uav1_z", init_uav1_z);
        nh.getParam("init_uav1_yaw", init_uav1_yaw);

        nh.getParam("init_rote_x", init_rote_x);
        nh.getParam("init_rote_y", init_rote_y);
        nh.getParam("init_rote_z", init_rote_z);

        nh.getParam("init_uav2_x", init_uav2_x);
        nh.getParam("init_uav2_y", init_uav2_y);
        nh.getParam("init_uav2_z", init_uav2_z);
        nh.getParam("init_uav2_yaw", init_uav2_yaw);
        // nh.param("init_q1", init_q1,0.0);
        // nh.param("init_q2", init_q2,0.0);
        // nh.param("init_q3", init_q3,-1.0);
        // config of quadrotor
        Config config;
        nh.getParam("g",       config.g);
        nh.getParam("mass_q", config.mass_q); 
        nh.getParam("mass_r", config.mass_r);
        nh.getParam("cable_length", config.cable_length);
        nh.getParam("rode_length", config.rode_length);
        // cable_length = config.cable_length;
        double Ixx, Iyy, Izz;
        nh.getParam("Ixx", Ixx);
        nh.getParam("Iyy", Iyy);
        nh.getParam("Izz", Izz);
        config.J_q = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();
        nh.getParam("kf", config.kf);
        double prop_radius;
        nh.getParam("prop_radius", prop_radius);
        config.km = 0.07 * (3 * prop_radius) * config.kf;
        nh.getParam("arm_length", config.arm_length);
        nh.getParam("motor_time_constant", config.motor_time_constant);
        nh.getParam("max_rpm", config.max_rpm);
        nh.getParam("min_rpm", config.min_rpm);
        nh.getParam("simulation_rate", simulation_rate_);
        nh.getParam("odom_rate", odom_rate_);
        nh.getParam("payload_size", payload_size_);
        
        nh.param("noise_uav", noise_uav_,0.0);
        nh.param("noise_payload", noise_payload_,0.0);
        nh.param("noise_vel_uav", noise_vel_uav_,0.0);
        nh.param("noise_vel_payload", noise_vel_payload_,0.0);
        nh.param("noise_acc_uav", noise_acc_uav_,0.0);
        nh.param("noise_acc_payload", noise_acc_payload_,0.0);
        nh.param("noise_rpm", noise_rpm_,0.0);

        std::vector<double> kom;
        nh.getParam("kOmega", kom);
        for (size_t i = 0; i < 3; i++)
        {
          config.kOmega[i] = kom[i];
        }
        std::vector<double> kdom;
        nh.getParam("kdOmega", kdom);
        for (size_t i = 0; i < 3; i++)
        {
          config.kdOmega[i] = kdom[i];
        }

        uav1_rpm_msg_.esc_telemetry.resize(4);
        uav2_rpm_msg_.esc_telemetry.resize(4);

        twoquadrotorPtr_ = std::make_shared<TwoQuadrotor>(config);
        twoquadrotorPtr_->setUAV1Ypr(Eigen::Vector3d(init_uav1_yaw, 0, 0));
        twoquadrotorPtr_->setUAV2Ypr(Eigen::Vector3d(init_uav2_yaw, 0, 0));
        double rpm = sqrt((config.mass_q + 0.5 * config.mass_r) * config.g / 4 / config.kf);
        twoquadrotorPtr_->setUAV1Rpm(Eigen::Vector4d(rpm, rpm, rpm, rpm));
        twoquadrotorPtr_->setUAV2Rpm(Eigen::Vector4d(rpm, rpm, rpm, rpm));
        twoquadrotorPtr_->setLoadPos(Eigen::Vector3d(init_rote_x, init_rote_y, init_rote_z));
        twoquadrotorPtr_->setUAV1Pos(Eigen::Vector3d(init_uav1_x, init_uav1_y, init_uav1_z));
        twoquadrotorPtr_->setUAV2Pos(Eigen::Vector3d(init_uav2_x, init_uav2_y, init_uav2_z));
        twoquadrotorPtr_->setLoadqr(Eigen::Vector3d(0, 1, 0));
        twoquadrotorPtr_->setUAV1ql(Eigen::Vector3d(0, 0, -1));
        twoquadrotorPtr_->setUAV2ql(Eigen::Vector3d(0, 0, -1));
        cmd_1.thrust =  (config.mass_q+0.5 *config.mass_r) * config.g;
        cmd_1.body_rates.setZero();
        cmd_2.thrust =  (config.mass_q+0.5 *config.mass_r) * config.g;
        cmd_2.body_rates.setZero();

        uav1_odom_pub_ = nh.advertise<nav_msgs::Odometry>("uav1odom", 100);
        uav2_odom_pub_ = nh.advertise<nav_msgs::Odometry>("uav2odom", 100);
        payload_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_payload", 100);

        uav1_imu_pub_  = nh.advertise<sensor_msgs::Imu>("uav1imu", 10);
        uav2_imu_pub_  = nh.advertise<sensor_msgs::Imu>("uav2imu", 10);

        uav1_cable_info_pub_  = nh.advertise<sensor_msgs::Imu>("uav1cable_info", 10);
        uav2_cable_info_pub_  = nh.advertise<sensor_msgs::Imu>("uav2cable_info", 10);

        load_imu_pub_ = nh.advertise<sensor_msgs::Imu>("load_imu", 10);
        vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vis", 10);
        uav1_cmd_sub_ = nh.subscribe<mavros_msgs::AttitudeTarget>("uav1so3cmd", 10, &Nodelet::uav1_cmd_callback, this, ros::TransportHints().tcpNoDelay());
        uav2_cmd_sub_ = nh.subscribe<mavros_msgs::AttitudeTarget>("uav2so3cmd", 10, &Nodelet::uav2_cmd_callback, this, ros::TransportHints().tcpNoDelay());
        uav1_rpm_pub_ = nh.advertise<mavros_msgs::ESCTelemetry>("uav1rpm", 10);
        uav2_rpm_pub_ = nh.advertise<mavros_msgs::ESCTelemetry>("uav2rpm", 10);
        simulation_timer = nh.createTimer(ros::Duration(1.0/simulation_rate_), &Nodelet::timer_callback, this);


        uav1_odom_msg_.header.frame_id = "world";
        uav2_odom_msg_.header.frame_id = "world";
        odom_payload_msg_.header.frame_id = "world";
        uav1_imu_msg_ .header.frame_id = "world";
        uav2_imu_msg_ .header.frame_id = "world";
        vis_msg_.markers.resize(8);
        visualization_msgs::Marker propeller;
        propeller.header.frame_id = "world";
        propeller.type = visualization_msgs::Marker::CYLINDER;
        propeller.action = visualization_msgs::Marker::ADD;
        propeller.scale.x = 2 * prop_radius;
        propeller.scale.y = 0.1 * prop_radius;
        propeller.scale.z = 0.05 * prop_radius;
        propeller.color.a = 0.7;
        propeller.color.r = 0.6;
        propeller.color.g = 0.7;
        propeller.color.b = 0.8;
        for (size_t i=0; i<8; ++i) {
          vis_msg_.markers[i] = propeller;
          vis_msg_.markers[i].id = i;
        }
        visualization_msgs::Marker drone_body = propeller;
        drone_body.type = visualization_msgs::Marker::SPHERE;
        drone_body.color.a = 1.0;
        drone_body.scale.x = config.arm_length;
        drone_body.scale.y = 0.1 * config.arm_length;
        drone_body.scale.z = 0.05 * config.arm_length;
        vis_msg_.markers.push_back(drone_body);
        drone_body.scale.y = config.arm_length;
        drone_body.scale.x = 0.1 * config.arm_length;
        vis_msg_.markers.push_back(drone_body);
        drone_body.color.a = 1.0;
        drone_body.scale.x = config.arm_length;
        drone_body.scale.y = 0.1 * config.arm_length;
        drone_body.scale.z = 0.05 * config.arm_length;
        vis_msg_.markers.push_back(drone_body);
        drone_body.scale.y = config.arm_length;
        drone_body.scale.x = 0.1 * config.arm_length;
        vis_msg_.markers.push_back(drone_body);


        visualization_msgs::Marker payload = propeller;
        payload.type = visualization_msgs::Marker::LINE_STRIP;
        payload.color.a = 1.0;
        payload.color.r = 0.3;
        payload.color.g = 0.4;
        payload.color.b = 0.5;
        payload.scale.x = 0.02;
        payload.scale.y = 0.02;
        payload.scale.z = 0.02;
        payload.id = (int32_t)vis_msg_.markers.size();
        vis_msg_.markers.push_back(payload);
        visualization_msgs::Marker payload_line = payload;
        payload_line.type = visualization_msgs::Marker::LINE_STRIP;
        payload_line.scale.x = 0.02;
        payload_line.id = (int32_t)vis_msg_.markers.size();
        vis_msg_.markers.push_back(payload_line);
        payload_line.type = visualization_msgs::Marker::LINE_STRIP;
        payload_line.scale.x = 0.02;
        payload_line.id = (int32_t)vis_msg_.markers.size();
        vis_msg_.markers.push_back(payload_line);
    // drone_body.scale.y = config.arm_length;
    // drone_body.scale.x = 0.1 * config.arm_length;
    // vis_msg_.markers.push_back(payload);
      };
  };
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(so3_quadrotor::Nodelet, nodelet::Nodelet);
