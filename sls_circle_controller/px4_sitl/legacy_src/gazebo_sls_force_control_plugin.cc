#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "QSFGeometricController.h"
#include "rtwtypes.h"

#include <ros/ros.h>
#include "controller_msgs/SlsState.h"
#include "controller_msgs/SlsForce.h"
#include "iostream"
#include <Eigen/Dense>
#include "math.h"

#include "mrotor_controller/common.h"
#include "mrotor_controller/control.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PointStamped.h"

#ifndef MATH_PI
#define MATH_PI		3.141592653589793238462643383280
#endif

namespace gazebo
{

class SlsForceControlPlugin : public ModelPlugin
{
private:
    // >>> Pointers
    physics::ModelPtr model;
    physics::LinkPtr baseLink, pendulum, load;
    physics::JointPtr pendulumJoint, loadJoint;
    event::ConnectionPtr updateConnection;

    // >>> Link States
    ignition::math::Vector3d mavPos_;
    ignition::math::Vector3d mavVel_;
    ignition::math::Quaterniond mavAtt_;
    ignition::math::Vector3d mavAttEuler_;
    ignition::math::Vector3d mavRate_;
    ignition::math::Vector3d mavAngularAcc_;
    ignition::math::Vector3d loadPos_;
    ignition::math::Vector3d loadVel_;
    ignition::math::Vector3d pendAngle_;
    ignition::math::Vector3d pendRate_;

    // >>> Controller Parameters
    
    double K_[10] = {24, 50, 35, 10, 24, 50, 35, 10, 2, 3};
    double param_[4] = {0.25, 1.51, 0.85, 9.80665};
    double max_fb_force_;
    double target_force_ned_[3] = {};
    bool use_ref_att_ = false;
    bool use_ref_rate_ = true;
    // bool use_ref_rate_ = false;
    bool rate_pid_enabled_ = true;
    
    // >>> Attitude Controller
    bool init_complete_ = false;
    Eigen::Vector4d q_des_;
    double mavYaw_ = 0;
    double attctrl_tau_ = 0.05;
    Eigen::Vector4d cmdBodyRate_;
    Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_thrust_{Eigen::Vector3d::Zero()};

    // >>> Rate PID Controller
    double diff_t_, last_t_;
    Eigen::Vector3d lim_int_;
    Eigen::Vector3d last_rate_;
    double angular_imax_ = 123456789.0;
    double maxTorque_ = 100;
    Eigen::Vector3d rate_pid_gain_k_;
    Eigen::Vector3d rate_pid_gain_p_;
    Eigen::Vector3d rate_pid_gain_i_;
    Eigen::Vector3d rate_pid_gain_d_;
    Eigen::Vector3d rate_pid_gain_ff_;
    Eigen::Vector3d rate_pid_int_;

    Eigen::Vector3d previousRateState_;
    double timeConstantDownRate_;
    double timeConstantUpRate_;
    double fcRate_ = 100;

    Eigen::Vector3d previousRateDState_;
    double timeConstantDownRateD_;
    double timeConstantUpRateD_;
    double fcRateD_ = 30;

    Eigen::Vector4d previousAttState_;
    double timeConstantDownAtt_;
    double timeConstantUpAtt_;
    double fcAtt_ = 30;

    // >>> Mission Stages
    int mission_stage_ = 0;

    // >>> Reference
    // reference
    double target_center_enu_[3] = {0, 0, 1};
    double target_radium_enu_[3] = {1, 1, 0};
    double target_frequency_enu_[3] = {1, 1, 0};
    double target_phase_enu_[3] = {1.57, 0, 0};
 
    // mission setpoints
    double target_center_enu_1_[3] = {0, 0, 1};
    double target_center_enu_2_[3] = {1.5, 0, 1};
    double target_center_enu_3_[3] = {0, 1.5, 1};

    double ref_trajectory_ned_[12] = {
        target_radium_enu_[0], target_frequency_enu_[0], target_center_enu_[0], target_phase_enu_[0],  // X: 北向 (NED X = ENU X)
        target_radium_enu_[1], target_frequency_enu_[1], target_center_enu_[1], target_phase_enu_[1],  // Y: 东向 (NED Y = ENU Y)
        target_radium_enu_[2], target_frequency_enu_[2], -target_center_enu_[2], target_phase_enu_[2]  // Z: 天向负 (NED Z = -ENU Z)
    };

    double ref_trajectory_ned_1_[12] = {
        0, 0, target_center_enu_1_[0], 0,  // X: 北向 (NED X = ENU X)
        0, 0, target_center_enu_1_[1], 0,  // Y: 东向 (NED Y = ENU Y)
        0, 0, -target_center_enu_1_[2], 0  // Z: 天向负 (NED Z = -ENU Z)
    };

    double ref_trajectory_ned_2_[12] = {
        0, 0, target_center_enu_2_[0], 0,
        0, 0, target_center_enu_2_[1], 0,
        0, 0, -target_center_enu_2_[2], 0
    };

    double ref_trajectory_ned_3_[12] = {
        0, 0, target_center_enu_3_[0], 0,
        0, 0, target_center_enu_3_[1], 0,
        0, 0, -target_center_enu_3_[2], 0
    };

    Eigen::Vector3d targetJerk_ = Eigen::Vector3d::Zero();


    // >>> Mission
    ros::Time mission_last_called_;
    ros::Time plugin_loaded_ = ros::Time::now();
    bool mission_initialized_ = false;
    


    // >>> ROS Interface
    ros::Publisher sls_state_pub_;
    ros::Publisher sls_force_pub_;
    ros::Publisher mav_att_pub_;
    ros::Publisher mav_att_sp_pub_;
    ros::Publisher mav_rate_pub_;
    ros::Publisher mav_rate_sp_pub_;
    ros::Publisher wind_actual_pub_;
    ros::Publisher wind_estimated_pub_;
    controller_msgs::SlsState sls_state_; 
    controller_msgs::SlsForce sls_force_; 
    geometry_msgs::QuaternionStamped mav_att_sp_, mav_att_; 
    geometry_msgs::Vector3Stamped mav_rate_sp_, mav_rate_;
    //
    ros::Publisher desired_pos_pub_;

    // >>> add wind
    // >>> Wind disturbance
    bool enable_wind_ = true;
    // Wind speed in world frame (ENU: X north, Y east, Z up) - [m/s]
    ignition::math::Vector3d wind_speed_;  // 大幅降低风速便于调试
    // Simple linear drag coefficient (N/(m/s))
    ignition::math::Vector3d drag_coeff_;  // 大幅降低阻力系数 Random wind parameters
    double wind_turbulence_ = 0.5;      // 湍流强度 (m/s)
    double wind_change_rate_ = 0.2;     // 变化速率 (m/s per second)
    ignition::math::Vector3d wind_noise_;
    ros::Time last_wind_update_;

    // 线性扩张状态观测器 (LESO)
    bool enable_leso_ = true;                 // LESO 使能开关
    const double leso_wo_ = 35.0;                 // 观测器带宽 (rad/s)，越大估计越快但噪声敏感
    double leso_beta1_, leso_beta2_, leso_beta3_; // 观测器增益 (自动计算)
    Eigen::Vector3d leso_z_x_;              // [位置估计, 速度估计, 扰动加速度估计] for X
    Eigen::Vector3d leso_z_y_;              // for Y
    Eigen::Vector3d leso_z_z_;              // for Z
    double mass_load_ = param_[0];          // 负载质量 (用于将力转换为加速度)

    double ref_pos[3],ref_pos_last_[3];
    bool ref_pos_last_tag_ = true;
    ros::Time ref_pos_last_time_ = ros::Time::now();
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
        
        ROS_INFO_STREAM("ROS Plugin");
        
        int argc = 0; 
        char **argv = nullptr; 
        ros::init(argc, argv, "gazebo_force_control_node", ros::init_options::NoSigintHandler); 
        ROS_INFO_STREAM("Gazebo Force Control Plugin Initialized");

        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        sls_state_pub_ = nh.advertise<controller_msgs::SlsState> ("/drone1/mrotor_sls_controller/sls_state", 1);
        sls_force_pub_ = nh.advertise<controller_msgs::SlsForce> ("/drone1/mrotor_sls_controller/sls_force", 1);
        mav_att_pub_ = nh.advertise<geometry_msgs::QuaternionStamped> ("/drone1/mrotor_sls_controller/mav_att", 1);
        mav_att_sp_pub_ = nh.advertise<geometry_msgs::QuaternionStamped> ("/drone1/mrotor_sls_controller/mav_att_sp", 1);
        mav_rate_pub_ = nh.advertise<geometry_msgs::Vector3Stamped> ("/drone1/mrotor_sls_controller/mav_rate", 1);
        mav_rate_sp_pub_ = nh.advertise<geometry_msgs::Vector3Stamped> ("/drone1/mrotor_sls_controller/mav_rate_sp", 1);
        //add
        desired_pos_pub_ = nh.advertise<geometry_msgs::PointStamped>("/drone1/desired_position", 1);
        wind_actual_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/drone1/wind_actual", 1);
        wind_estimated_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/drone1/wind_estimated", 1);

        model = _model;

        // Get links
        baseLink = model->GetLink("base_link");
        baseLink = model->GetLink("base_link");
        if (!baseLink) {
            // 尝试带命名空间的名称
            baseLink = model->GetLink("px4vision_ancl::base_link");
        }
        if (!baseLink) {
            gzerr << "Failed to find base_link or px4vision_ancl::base_link\n";
            return;
        }
        pendulum = model->GetLink("pendulum");
        if (!pendulum) pendulum = model->GetLink("px4vision_sls::pendulum");
        load = model->GetLink("load");
        if (!load) load = model->GetLink("px4vision_sls::load");

        // Get joints
        pendulumJoint = model->GetJoint("pendulum_joint");
        if (!pendulumJoint) pendulumJoint = model->GetJoint("px4vision_sls::pendulum_joint");
        loadJoint = model->GetJoint("load_joint");
        if (!loadJoint) loadJoint = model->GetJoint("px4vision_sls::load_joint");


        if (!baseLink || !pendulum || !load || !pendulumJoint || !loadJoint)
        {
            gzerr << "Required links or joints not found in the model!\n";
            return;
        }

        for (int i = 0; i < 10; i++) {
            std::cout << "K_[" << i << "]: " << K_[i] << std::endl;
        }

        std::cout << "Load Mass: " << param_[0] << std::endl;
        std::cout << "Drone Mass: " << param_[1] << std::endl;
        std::cout << "Cable Length: " << param_[2] << std::endl;
        std::cout << "Gravity: " << param_[3] << std::endl;

        max_fb_force_ = 3 * param_[1] * param_[3];
        std::cout << "Max Force: " << max_fb_force_ << std::endl;

        // Rate PID Controller
        lim_int_ << 1.5, 1.5, 1.5;

        rate_pid_gain_k_ << 0.85, 0.85, 1;
        rate_pid_gain_p_ << 2.0, 2.0, 4.0;
        rate_pid_gain_i_ << 0.5, 0.5, 0.5;
        rate_pid_gain_d_ << 0.0004, 0.0004, 0.0;

        rate_pid_gain_k_ << 1.0, 1.0, 1.0;
        rate_pid_gain_p_ << 1.5, 1.5, 3.0;
        rate_pid_gain_i_ << 0.1, 0.1, 0.1;
        rate_pid_gain_d_ << 0.004, 0.004, 0.0;

        rate_pid_gain_p_ = rate_pid_gain_p_.cwiseProduct(rate_pid_gain_k_);
        rate_pid_gain_i_ = rate_pid_gain_i_.cwiseProduct(rate_pid_gain_k_);
        rate_pid_gain_d_ = rate_pid_gain_d_.cwiseProduct(rate_pid_gain_k_);


        ROS_INFO_STREAM("rate_pid_gain_p_" << rate_pid_gain_p_(0) << " " << rate_pid_gain_p_(1) << " " << rate_pid_gain_p_(2));
        ROS_INFO_STREAM("rate_pid_gain_i_" << rate_pid_gain_i_(0) << " " << rate_pid_gain_i_(1) << " " << rate_pid_gain_i_(2));
        ROS_INFO_STREAM("rate_pid_gain_d_" << rate_pid_gain_d_(0) << " " << rate_pid_gain_d_(1) << " " << rate_pid_gain_d_(2));

        timeConstantDownRate_ = 1.d / (2*MATH_PI*fcRate_);
        timeConstantUpRate_ = 1.d / (2*MATH_PI*fcRate_);
        timeConstantDownRateD_ = 1.d / (2*MATH_PI*fcRateD_);
        timeConstantUpRateD_ = 1.d / (2*MATH_PI*fcRateD_);        
        timeConstantDownAtt_ = 1.d / (2*MATH_PI*fcAtt_);
        timeConstantUpAtt_ = 1.d / (2*MATH_PI*fcAtt_);   

        // Connect to the update event
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SlsForceControlPlugin::OnUpdate, this));

        init_complete_ = true;
        last_t_ = ros::Time::now().toSec();
        ROS_INFO_STREAM("Init Complete");

        // >>> add wind
        // Initialize wind parameters from ROS param server
        // nh_private.param<bool>("enable_wind", enable_wind_, false);
        nh_private.param<double>("wind_speed_x", wind_speed_.X(), 2.0);
        nh_private.param<double>("wind_speed_y", wind_speed_.Y(), 1.5);
        nh_private.param<double>("wind_speed_z", wind_speed_.Z(), 0.0);
        nh_private.param<double>("wind_drag_coeff", drag_coeff_.X(), 0.7);
        drag_coeff_.Y() = drag_coeff_.X();
        drag_coeff_.Z() = 0.0;
        nh_private.param<double>("wind_turbulence", wind_turbulence_, 0.5);
        last_wind_update_ = ros::Time::now();
        wind_noise_.Set(0,0,0);

        // 初始化 LESO 状态
        leso_z_x_.setZero();
        leso_z_y_.setZero();
        leso_z_z_.setZero();
        // 根据带宽计算增益 (特征多项式 (s+wo)^3)
        leso_beta1_ = 3.0 * leso_wo_;
        leso_beta2_ = 3.0 * leso_wo_ * leso_wo_;
        leso_beta3_ = leso_wo_ * leso_wo_ * leso_wo_;

    }

    void OnUpdate() {
        // Time step
        diff_t_ = ros::Time::now().toSec() - last_t_;
        last_t_ = ros::Time::now().toSec();
        // Retrieve base link (quadrotor) states
        mavPos_ = baseLink->WorldPose().Pos();
        mavAtt_ = baseLink->WorldPose().Rot();
        mavAttEuler_ = mavAtt_.Euler();
        // ROS_INFO_STREAM(mavAttEuler_.X() << " " << mavAttEuler_.Y() << " " << mavAttEuler_.Z());
        // mavYaw_ = mavAttEuler_.Z();
        mavVel_ = baseLink->WorldLinearVel();
        mavRate_ = baseLink->WorldAngularVel();
        mavAngularAcc_ = baseLink->RelativeAngularAccel();
        // ignition::math::Quaterniond quadOrientation = baseLink->WorldPose().Rot();  

        // Retrieve load states
        loadPos_ = load->WorldPose().Pos();
        loadVel_ = load->WorldLinearVel();

        // Calculate pendulum angles (alpha, beta)
        pendAngle_ = (loadPos_ - mavPos_);
        pendAngle_ = pendAngle_.Normalize();
        pendRate_ = pendAngle_.Cross(loadVel_ - mavVel_);
        

        sls_state_.header.stamp = ros::Time::now();
        sls_state_.sls_state[0] = loadPos_.Y();
        sls_state_.sls_state[1] = loadPos_.X();
        sls_state_.sls_state[2] = -loadPos_.Z();
        sls_state_.sls_state[3] = pendAngle_.Y();
        sls_state_.sls_state[4] = pendAngle_.X();
        sls_state_.sls_state[5] = -pendAngle_.Z();
        sls_state_.sls_state[6] = loadVel_.Y();
        sls_state_.sls_state[7] = loadVel_.X();
        sls_state_.sls_state[8] = -loadVel_.Z();
        sls_state_.sls_state[9] = pendRate_.Y();
        sls_state_.sls_state[10] = pendRate_.X();
        sls_state_.sls_state[11] = -pendRate_.Z();
        sls_state_pub_.publish(sls_state_);

        double sls_state_array[12];
        for(int i=0; i<12;i++){
            sls_state_array[i] = sls_state_.sls_state[i];
        }    
        
        if(ros::Time::now().toSec() - plugin_loaded_.toSec() < 15) {
            QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
            getRefPosition(ref_trajectory_ned_1_, 0, ref_pos,ref_pos_last_);
            ref_pos_last_[0] = ref_pos[0];
            ref_pos_last_[1] = ref_pos[1];
            ref_pos_last_[2] = ref_pos[2];
            ref_pos_last_time_ = ros::Time::now();
        }

        else {
            switch(mission_stage_) {
            case 0:
                if(!mission_initialized_){
                    ROS_INFO("[exeMission] Mission started at case 0");
                    mission_initialized_ = true;
                    mission_last_called_ = ros::Time::now();
                }
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                getRefPosition(ref_trajectory_ned_1_, 0, ref_pos,ref_pos_last_);
                checkMissionStage(15);
                break;

            case 1:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                getRefPosition(ref_trajectory_ned_1_, 0, ref_pos,ref_pos_last_);
                checkMissionStage(15);
                break;

            case 2:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_2_, 0, target_force_ned_);
                getRefPosition(ref_trajectory_ned_2_, 0, ref_pos,ref_pos_last_);
                checkMissionStage(15);
                break;
            
            case 3:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_3_, 0, target_force_ned_);
                getRefPosition(ref_trajectory_ned_3_, 0, ref_pos,ref_pos_last_);
                checkMissionStage(15);  
                break;

            case 4:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                getRefPosition(ref_trajectory_ned_1_, 0, ref_pos,ref_pos_last_);
                checkMissionStage(15);
                break;

            case 5:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_, ros::Time::now().toSec() - mission_last_called_.toSec(), target_force_ned_);
                getRefPosition(ref_trajectory_ned_, ros::Time::now().toSec() - mission_last_called_.toSec(), ref_pos,ref_pos_last_);
                checkMissionStage(30);
                break;                

            default:
                QSFGeometricController(sls_state_array, K_, param_, ref_trajectory_ned_1_, 0, target_force_ned_);
                getRefPosition(ref_trajectory_ned_1_, 0, ref_pos,ref_pos_last_);
                if(ros::Time::now().toSec() - mission_last_called_.toSec() >= 10){
                    ROS_INFO("[exeMission] Mission Accomplished");
                    mission_last_called_ = ros::Time::now();
                }
                break;
            }
        }
        geometry_msgs::PointStamped desired_pos_msg;
        desired_pos_msg.header.stamp = ros::Time::now();
        desired_pos_msg.point.x = ref_pos[0]; 
        desired_pos_msg.point.y = ref_pos[1];
        desired_pos_msg.point.z = ref_pos[2];
        desired_pos_pub_.publish(desired_pos_msg);

        // ========== LESO + 残差积分 ==========
        double est_force_x;
        double est_force_y;
        double est_force_z;
        if (enable_leso_) {
            double load_x_ned = loadPos_.X();   // 北向位置
            double load_y_ned = loadPos_.Y();   // 东向位置
            double load_z_ned = loadPos_.Z();   // 天向位置
            double u_x = target_force_ned_[1] / param_[1];   // 北向加速度输入
            double u_y = target_force_ned_[0] / param_[1];   // 东向加速度输入
            double u_z = target_force_ned_[2] / param_[1];   // 天向加速度输入
            double e_x = load_x_ned - leso_z_x_(0);
            double e_y = load_y_ned - leso_z_y_(0);
            double e_z = load_z_ned - leso_z_z_(0);
            
            leso_z_x_(0) += (leso_z_x_(1) + leso_beta1_ * e_x) * diff_t_;
            leso_z_x_(1) += (leso_z_x_(2) + leso_beta2_ * e_x + u_x) * diff_t_;
            leso_z_x_(2) += (leso_beta3_ * e_x) * diff_t_;
            
            leso_z_y_(0) += (leso_z_y_(1) + leso_beta1_ * e_y) * diff_t_;
            leso_z_y_(1) += (leso_z_y_(2) + leso_beta2_ * e_y + u_y) * diff_t_;
            leso_z_y_(2) += (leso_beta3_ * e_y) * diff_t_;
            
            leso_z_z_(0) += (leso_z_z_(1) + leso_beta1_ * e_z) * diff_t_;
            leso_z_z_(1) += (leso_z_z_(2) + leso_beta2_ * e_z + u_z) * diff_t_;
            leso_z_z_(2) += (leso_beta3_ * e_z) * diff_t_;
            
            // 残差积分
            static Eigen::Vector3d integral_res = Eigen::Vector3d::Zero();
            double Ki_res = 1.5;   // 可适当增大
            double _theta = std::min((ros::Time::now().toSec() - ref_pos_last_time_.toSec()) / 10.0,1.0);
            ref_pos[1] = ref_pos_last_[1] * (1 - _theta) + ref_pos[1] * _theta;
            ref_pos[0] = ref_pos_last_[0] * (1 - _theta) + ref_pos[0] * _theta;
            integral_res(1) += (ref_pos[1] - load_x_ned) * diff_t_;
            integral_res(0) += (ref_pos[0] - load_y_ned) * diff_t_;
            integral_res(2) += (ref_pos[2] - load_z_ned) * diff_t_;
            integral_res = integral_res.cwiseMax(-5.0).cwiseMin(5.0);

            // 增加积分抗饱和逻辑
            double integral_limit = 10.0;  // 增大积分限幅
            integral_res = integral_res.cwiseMax(-integral_limit).cwiseMin(integral_limit);
            
            est_force_x = param_[1] * leso_z_x_(2) - Ki_res * integral_res(1);
            est_force_y = param_[1] * leso_z_y_(2) - Ki_res * integral_res(0);

            target_force_ned_[0] -= est_force_y;
            target_force_ned_[1] -= est_force_x; 
            // 发布估计扰动
            geometry_msgs::Vector3Stamped est_msg;
            est_msg.header.stamp = ros::Time::now();
            est_msg.vector.x = mass_load_ * leso_z_x_(2) + Ki_res * integral_res(1);
            est_msg.vector.y = mass_load_ * leso_z_y_(2) + Ki_res * integral_res(0);
            est_msg.vector.z = mass_load_ * leso_z_z_(2) + Ki_res * integral_res(2);
            wind_estimated_pub_.publish(est_msg);
        }

        sls_force_.header.stamp = ros::Time::now();
        sls_force_.sls_force[0] = target_force_ned_[0];
        sls_force_.sls_force[1] = target_force_ned_[1];
        sls_force_.sls_force[2] = target_force_ned_[2];
        sls_force_pub_.publish(sls_force_);

        ignition::math::Vector3d controlForce(target_force_ned_[1], target_force_ned_[0], -target_force_ned_[2]);

        // 先限幅控制力，再添加风扰（避免风扰导致控制力饱和）
        if (controlForce.Length() > max_fb_force_)
            controlForce = (max_fb_force_ / controlForce.Length()) * controlForce;
        
        

        // >>> Attitude Control
        Eigen::Vector3d targetForce(controlForce.X(), controlForce.Y(), controlForce.Z());
        // ROS_INFO_STREAM(targetForce(0) << " " << targetForce(1) << " " << targetForce(2));
        Eigen::Vector3d targetAcc = targetForce / param_[1];
        q_des_ = acc2quaternion(targetAcc, mavYaw_);
        Eigen::Vector4d mavAtt(mavAtt_.W(), mavAtt_.X(), mavAtt_.Y(), mavAtt_.Z());
        // mavAtt = updateAttLPF(mavAtt, diff_t_);
        updateAttitudeCtrl(mavAtt, q_des_, targetAcc, targetJerk_);
        cmdBodyRate_.head(3) = getDesiredRate();
        // double thrust_command = getDesiredThrust().z();
        cmdBodyRate_(3) = getDesiredThrust().z() * param_[1];

        // >>> Rate Control
        Eigen::Vector3d mavRate(mavRate_.X(), mavRate_.Y(), mavRate_.Z());
        // mavRate = updateRateLPF(mavRate, diff_t_);
        Eigen::Vector3d desired_torque;
        desired_torque = updateRateCtrl(mavRate, cmdBodyRate_.head(3));
        

        ignition::math::Vector3d actual_wind = computeWindForce();
        geometry_msgs::Vector3Stamped actual_msg;
        actual_msg.header.stamp = ros::Time::now();
        actual_msg.vector.x = actual_wind.X();
        actual_msg.vector.y = actual_wind.Y();
        actual_msg.vector.z = actual_wind.Z();
        wind_actual_pub_.publish(actual_msg);
        this->baseLink->AddRelativeForce(actual_wind);  

        static int print_cnt = 0;
        if (print_cnt++ % 1000 == 0) {
            ignition::math::Vector3d actual_wind = computeWindForce();
            // double est_force_x = -mass_load_ * leso_z_x_(2) - Ki_res * integral_res(1);
            // double est_force_y = -mass_load_ * leso_z_y_(2) - Ki_res * integral_res(0);
            ROS_INFO_STREAM("=== Wind Comparison ===");
            ROS_INFO_STREAM("Actual wind force (N): x=" << actual_wind.X() << ", y=" << actual_wind.Y());
            ROS_INFO_STREAM("LESO estimated disturbance (N): x=" << est_force_x << ", y=" << est_force_y);
            ROS_INFO_STREAM("Difference: dx=" << (est_force_x / actual_wind.X()) << ", dy=" << (est_force_y / actual_wind.Y()));
        }

        if(use_ref_rate_){
            this->baseLink->AddRelativeForce(ignition::math::Vector3d(0, 0, cmdBodyRate_(3)));  
            
            if(!rate_pid_enabled_) {
                ROS_INFO_STREAM("Instantaneous Rate Control");
                ROS_INFO_STREAM(cmdBodyRate_(0) << " " << cmdBodyRate_(1) << " " << cmdBodyRate_(2) << " " << cmdBodyRate_(3));  
                this->baseLink->SetAngularVel({cmdBodyRate_(0), cmdBodyRate_(1), cmdBodyRate_(2)}); 
            }        

            else {
                // ROS_INFO_STREAM("PID Rate Control");
                // ROS_INFO_STREAM("diff_t_:" << diff_t_);
                // ROS_INFO_STREAM("int:" << rate_pid_int_(0) << " " << rate_pid_int_(1) << " " << rate_pid_int_(2));
                // ROS_INFO_STREAM("torque:" << desired_torque(0) << " " << desired_torque(1) << " " << desired_torque(2));
                this->baseLink->AddRelativeTorque(ignition::math::Vector3d(desired_torque(0), desired_torque(1), desired_torque(2)));
            }
        }

        else if(use_ref_att_) {

        }

        else { // outer-loop control
            ROS_INFO_STREAM("Force Control");
            // this->baseLink->AddForce(controlForce);

            // >>> add wind
            this->baseLink->AddForce(controlForce);
        }

        mav_att_.header.stamp = ros::Time::now();
        mav_att_.quaternion.x = mavAtt(1);
        mav_att_.quaternion.y = mavAtt(2);
        mav_att_.quaternion.z = mavAtt(3);
        mav_att_.quaternion.w = mavAtt(0);; 
        mav_att_pub_.publish(mav_att_);

        mav_att_sp_.header.stamp = ros::Time::now();
        mav_att_sp_.quaternion.x = q_des_(1);
        mav_att_sp_.quaternion.y = q_des_(2);
        mav_att_sp_.quaternion.z = q_des_(3);
        mav_att_sp_.quaternion.w = q_des_(0);
        mav_att_sp_pub_.publish(mav_att_sp_);

        mav_rate_.header.stamp = ros::Time::now();
        mav_rate_.vector.x = mavRate_.X();
        mav_rate_.vector.y = mavRate_.Y();
        mav_rate_.vector.z = mavRate_.Z();
        mav_rate_pub_.publish(mav_rate_);

        mav_rate_sp_.header.stamp = ros::Time::now();
        mav_rate_sp_.vector.x = cmdBodyRate_(0);
        mav_rate_sp_.vector.y = cmdBodyRate_(1);
        mav_rate_sp_.vector.z = cmdBodyRate_(2);
        mav_rate_sp_pub_.publish(mav_rate_sp_);
    }

    void checkMissionStage(double mission_time_span) {
        if(ros::Time::now().toSec() - mission_last_called_.toSec() >= mission_time_span) {
            mission_last_called_ = ros::Time::now();
            mission_stage_ += 1;
            ROS_INFO_STREAM("[exeMission] Stage " << mission_stage_-1 << " ended, switching to stage " << mission_stage_);
            ref_pos_last_tag_ = false;
        }
    }

    void updateAttitudeCtrl(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                      const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
        // Geometric attitude controller
        // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
        // attitude control: Technical report. ETH Zurich, 2013.

        const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
        const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
        const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
        desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
        desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
        desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
        const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);
        const Eigen::Vector3d zb = rotmat.col(2);
        desired_thrust_(0) = 0.0;
        desired_thrust_(1) = 0.0;
        desired_thrust_(2) = ref_acc.dot(zb);
    }
// #ifdef USE_OLD
    Eigen::Vector3d updateRateCtrl(Eigen::Vector3d &curr_rate, const Eigen::Vector3d &ref_rate) {
        // Compute rate error
        Eigen::Vector3d rate_error = ref_rate - curr_rate;

        // Compute angular acceleration
        Eigen::Vector3d mavAngularAcc(mavAngularAcc_.X(), mavAngularAcc_.Y(), mavAngularAcc_.Z());
        // if(diff_t_ > 0.0){
        //     mavAngularAcc = (curr_rate - last_rate_) / diff_t_;
        //     last_rate_ = curr_rate;
        //     // mavAngularAcc = updateRateDLPF(mavAngularAcc, diff_t_);
        // }

        // Update integral
        for(int i = 0; i < 3; i++) {
            // double i_factor = rate_error(i) / (400.d * MATH_PI / 180);
            // i_factor = std::max(0.0d, 1.d - i_factor * i_factor);
            // double rate_i = rate_pid_int_(i) + i_factor * rate_pid_gain_i_(i) * rate_error(i) * diff_t_;
            double rate_i = rate_pid_int_(i) + 1.0 * rate_pid_gain_i_(i) * rate_error(i) * diff_t_;

            if (std::isfinite(rate_i)) {
			    rate_pid_int_(i) = (rate_i < -lim_int_(i)) ? -lim_int_(i) : ((rate_i > lim_int_(i)) ? lim_int_(i) : rate_i);
		    }
        }

        // Compute reference torque
        Eigen::Vector3d desired_torque = rate_pid_gain_p_.cwiseProduct(rate_error) + rate_pid_int_ - rate_pid_gain_d_.cwiseProduct(mavAngularAcc) + rate_pid_gain_ff_.cwiseProduct(ref_rate);
        return desired_torque;
    }

    Eigen::Vector3d getDesiredThrust() { return desired_thrust_; };
    Eigen::Vector3d getDesiredRate() { return desired_rate_; };

    ~SlsForceControlPlugin() {

    }


    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
        Eigen::Vector4d quat;
        Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
        Eigen::Matrix3d rotmat;

        proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

        zb_des = vector_acc / vector_acc.norm();
        yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
        xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

        rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
        quat = rot2Quaternion(rotmat);
        return quat;
    }


    Eigen::Vector4d updateAttLPF(Eigen::Vector4d inputState, double samplingTime) {
        /*
        This method will apply a first order filter on the inputState.
        */
        Eigen::Vector4d outputState;

        for(int i = 0; i < 4; i ++) {
            if(inputState(i) > previousAttState_(i)){
                // Calcuate the outputState if accelerating.
                double alphaUp = exp(- samplingTime / timeConstantUpAtt_);
                // x(k+1) = Ad*x(k) + Bd*u(k)
                outputState(i) = alphaUp * previousAttState_(i) + (1 - alphaUp) * inputState(i);
            }
            
            else{
                // Calculate the outputState if decelerating.
                double alphaDown = exp(- samplingTime / timeConstantDownAtt_);
                outputState(i) = alphaDown * previousAttState_(i) + (1 - alphaDown) * inputState(i);
            }
        }

        previousAttState_ = outputState;
        return outputState;
    }

    Eigen::Vector3d updateRateLPF(Eigen::Vector3d inputState, double samplingTime) {
        /*
        This method will apply a first order filter on the inputState.
        */
        Eigen::Vector3d outputState;

        for(int i = 0; i < 3; i ++) {
            if(inputState(i) > previousRateState_(i)){
                // Calcuate the outputState if accelerating.
                double alphaUp = exp(- samplingTime / timeConstantUpRate_);
                // x(k+1) = Ad*x(k) + Bd*u(k)
                outputState(i) = alphaUp * previousRateState_(i) + (1 - alphaUp) * inputState(i);
            }
            
            else{
                // Calculate the outputState if decelerating.
                double alphaDown = exp(- samplingTime / timeConstantDownRate_);
                outputState(i) = alphaDown * previousRateState_(i) + (1 - alphaDown) * inputState(i);
            }
        }

        previousRateState_ = outputState;
        return outputState;
    }

    Eigen::Vector3d updateRateDLPF(Eigen::Vector3d inputState, double samplingTime) {
        /*
        This method will apply a first order filter on the inputState.
        */
        Eigen::Vector3d outputState;

        for(int i = 0; i < 3; i ++) {
            if(inputState(i) > previousRateState_(i)){
                // Calcuate the outputState if accelerating.
                double alphaUp = exp(- samplingTime / timeConstantUpRateD_);
                // x(k+1) = Ad*x(k) + Bd*u(k)
                outputState(i) = alphaUp * previousRateDState_(i) + (1 - alphaUp) * inputState(i);
            }
            
            else{
                // Calculate the outputState if decelerating.
                double alphaDown = exp(- samplingTime / timeConstantDownRateD_);
                outputState(i) = alphaDown * previousRateDState_(i) + (1 - alphaDown) * inputState(i);
            }
        }

        previousRateState_ = outputState;
        return outputState;

    }
    // >>> add pos5
    void getRefPosition(const double ref[12], double t, double pos[3], double pos_last[3]) {
        if(!ref_pos_last_tag_ && mission_stage_ < 5){
            pos_last[0] = pos[0];
            pos_last[1] = pos[1];
            pos_last[2] = pos[2];
            ref_pos_last_time_ = ros::Time::now();
            ref_pos_last_tag_ = true;
        }else if(mission_stage_ >= 5){
            pos_last[0] = pos[0];
            pos_last[1] = pos[1];
            pos_last[2] = pos[2];
            ref_pos_last_time_ = ros::Time::now();
            ref_pos_last_tag_ = true;
        }
        // X轴: ref[0]=振幅, ref[1]=角频率, ref[2]=偏置, ref[3]=相位
        pos[0] = ref[0] * std::sin(ref[1] * t + ref[3]) + ref[2];
        // Y轴: ref[4~7]
        pos[1] = ref[4] * std::sin(ref[5] * t + ref[7]) + ref[6];
        // Z轴: ref[8~11]
        pos[2] = ref[8] * std::sin(ref[9] * t + ref[11]) + ref[10];
    }

    // >>> add wind
    ignition::math::Vector3d computeWindForce() {
        if (!enable_wind_) return ignition::math::Vector3d::Zero;
        
        //可选：添加时间变化的随机风（布朗运动）
        ros::Time now = ros::Time::now();
        double dt = (now - last_wind_update_).toSec();
        if (dt > 0.02) {  // 限制更新频率
            // 简单的随机游走模拟湍流
            wind_noise_.X() += (2.0 * ((double)rand() / RAND_MAX) - 1.0) * wind_turbulence_ * sqrt(dt);
            wind_noise_.Y() += (2.0 * ((double)rand() / RAND_MAX) - 1.0) * wind_turbulence_ * sqrt(dt);
            wind_noise_.Z() += (2.0 * ((double)rand() / RAND_MAX) - 1.0) * wind_turbulence_ * sqrt(dt);
            // 限幅，避免过大
            wind_noise_.X() = std::min(std::max(wind_noise_.X(), -1.5), 1.5);
            wind_noise_.Y() = std::min(std::max(wind_noise_.Y(), -1.5), 1.5);
            wind_noise_.Z() = std::min(std::max(wind_noise_.Z(), -1.5), 1.5);
            last_wind_update_ = now;
        }
        
        ignition::math::Vector3d current_wind = wind_speed_ + wind_noise_;
        // 相对速度 (风 - 无人机速度)
        ignition::math::Vector3d rel_vel = current_wind - mavVel_;
        // 线性阻力模型: F_drag = drag_coeff .* rel_vel
        ignition::math::Vector3d drag_force;
        drag_force.X() = drag_coeff_.X() * rel_vel.X();
        drag_force.Y() = drag_coeff_.Y() * rel_vel.Y();
        drag_force.Z() = drag_coeff_.Z() * rel_vel.Z();
        return drag_force;
    }

    double sgn(double x) {
        if (x > 0) return 1.0;
        if (x < 0) return -1.0;
        return 0.0;
    }
    
    double sat(double x, double boundary) {
        if (x > boundary) return 1.0;
        if (x < -boundary) return -1.0;
        return x / boundary;
    }
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(SlsForceControlPlugin)


}
// ROS1 原版 Gazebo SLS force 控制插件归档。
// 当前 ROS2 主路径不直接编译此文件，只用于对照原仿真施力方式。
