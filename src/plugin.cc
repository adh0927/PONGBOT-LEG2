#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "Eigen/Dense"
#include <stdlib.h>

//#include <stdio.h>

//#include </root/Downloads/eigen-eigen-5a0156e40feb/Eigen/Dense>
//#include <rbdl/rbdl.h>

#define PI      3.141592
#define D2R     PI/180
#define R2D     180/PI
#define MAX_DEBUG_INDEX 60000

using Eigen::MatrixXd;

namespace gazebo
{
  class PIDJoints : public ModelPlugin
  {
    double hip_pos_error, knee_pos_error;
    double HR_err,HP_err,KN_err;
    double time = 0;
    double dt;
    double save[MAX_DEBUG_INDEX][2];
    unsigned int debug_index = 0;

    // DH PARA
    double q[3], q_dot[3];
    double desired_vel = 0.5;//0.8;//1.0;//0.8;//0.5;//0.5;
    double init_upper_deg = 0, real_upper_deg = 0;
    double g = 9.81;

    // ================================ time setting
    double stance_time = 0.3;//0.2;
    double flight_time = 0.3;//0.15;
    double wait_time = 0.000;//0.05;
    double take_off_time = 0.8;
    double landing_time = 0.6;
    double take_off_landing_time = 0.3;//1.0;//0.6;
    double t1 = 1.0; // walk ready
    double t2 = t1 + 0.0;//0.5; // waiting
    double t3 = t2 + take_off_time; //stance_time;//1.5; //stance_time;
    double t4 = t3 + flight_time;
    double t5 = t4 + take_off_landing_time; //stance_time;//1.5; //stance_time;
    double t6 = t5 + flight_time;
    double t7 = t6 + landing_time;//stance_time;

// ===================================== //
    double landing_height = 0.35 + 0.05;
    double to_height;// = landing_height - 0.10;
    double take_off_speed = 1.2;//1.154;//-flight_time * g / 2.0;
    double landing_vel = -0.6;//-0.3;
    double stair_height = 0;//0.05; //0.1
    double flight_time_offset = 0;//0.1;//0.05;//0.12; //0.10;

    double zmp_x, zmp_y, lpf_zmp_x, old_lpf_zmp_x;
    double UBC_output = 0;
    double F_RF_Z,M_RF_X,M_RF_Y,F_LF_Z,M_LF_X,M_LF_Y,RF_POS_Y,LF_POS_Y;
    double zmp_gain = 0, zmp_gain_cnt = 0,dsp_xzmp_con = 0,dsp_yzmp_con = 0, dsp_xzmp_con_dot = 0, dsp_yzmp_con_dot = 0;
    double old_dsp_xzmp_con = 0,old_dsp_yzmp_con = 0;

    double pv_Gd[1501],pv_Gx[4],pv_Gi[2];
    double pv_A[3][3],pv_B[3][1],pv_C[1][3];
    double com_x_take_off_pos[800],com_x_take_off_vel[800],com_x_take_off_acc[800];
    double com_x_landing_pos[600],com_x_landing_vel[600],com_x_landing_acc[600];
    double com_x_take_off_landing_pos[300],com_x_take_off_landing_vel[300],com_x_take_off_landing_acc[300];

    double cal_zmp_x = 0, cal_zmp_x2[1500];
    double zmp_ref[2];
    double init_x = 0;

    enum{
      flight_phase = 0,
      stance_phase,
      take_off,
      landing,
      continuous
    };

    double ref_hip_pos;
    double ref_knee_pos;
    double ref_RHR_deg;
    double x,z;
    int cnt = 0;
    double theta1, d_theta1, d2_theta1;
    double theta2, d_theta2, d2_theta2;
    double old_theta2, old_d_theta2;
    double thigh = 0.305, calf = 0.305;
    double P_, D_, I_;

//  DH Pointers
    physics::JointPtr HR_JOINT;
    physics::JointPtr HP_JOINT;
    physics::JointPtr KN_JOINT;

    physics::JointWrench HR_JOINT_W;// = this->RKN_joint->GetForceTorque(0);
    physics::JointWrench HP_JOINT_W;// = this->RAR_joint->GetForceTorque(0);
    physics::JointWrench KN_JOINT_W;// = this->RAP_joint->GetForceTorque(0);

    physics::LinkPtr FOOT;

//  DH PID
    common::PID pid_HIP_ROLL;
    common::PID pid_HIP_PITCH;
    common::PID pid_KNEE;

// ===================================== //

// ** Pointers for each joints

    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;

// ============= node handler setting ==============
    ros::NodeHandle n;

// ============= subscriber setting ==============
    ros::Subscriber SUB; // 주체 : 가제보 (ros --> gazebo)
    ros::Subscriber S_HIP_PITCH_P;    
    ros::Subscriber S_HIP_PITCH_I;
    ros::Subscriber S_HIP_PITCH_D;
   
    ros::Subscriber S_HIP_ROLL_P;    
    ros::Subscriber S_HIP_ROLL_I;
    ros::Subscriber S_HIP_ROLL_D;

    ros::Subscriber S_KNEE_P;    
    ros::Subscriber S_KNEE_I;
    ros::Subscriber S_KNEE_D;

//    ros::Subscriber S_Knee_P_Gain_sub;
//    ros::Subscriber S_Knee_I_Gain_sub;
//    ros::Subscriber S_Knee_D_Gain_sub;

    ros::Subscriber S_Data_Save_Flag;

// ============= publisher setting ==============
    ros::Publisher P_Times;
    ros::Publisher P_RKN_torque_ref;
    ros::Publisher P_RKN_torque_real;
    ros::Publisher P_RAP_torque_ref;
    ros::Publisher P_RAP_torque_real;
    ros::Publisher P_LHP_ref_pos;
    ros::Publisher P_LKN_ref_pos;
    ros::Publisher P_LAP_ref_pos;
    ros::Publisher P_LHP_real_pos;
    ros::Publisher P_LKN_real_pos;
    ros::Publisher P_LAP_real_pos;
    ros::Publisher P_dh_plot1;
    ros::Publisher P_dh_plot2;
    ros::Publisher P_dh_plot3;
    ros::Publisher P_dh_plot4;
    ros::Publisher P_dh_plot5;
    ros::Publisher P_dh_plot6;
    ros::Publisher P_dh_plot7;
    ros::Publisher P_dh_plot8;
    ros::Publisher P_dh_plot9;

// ============= topic setting ==============
    std_msgs::Float64 m_Times;
    std_msgs::Float64 m_RKN_torque_ref;
    std_msgs::Float64 m_RKN_torque_real;
    std_msgs::Float64 m_RAP_torque_ref;
    std_msgs::Float64 m_RAP_torque_real;
    std_msgs::Float64 m_LHP_ref_pos;
    std_msgs::Float64 m_LKN_ref_pos;
    std_msgs::Float64 m_LAP_ref_pos;
    std_msgs::Float64 m_LHP_real_pos;
    std_msgs::Float64 m_LKN_real_pos;
    std_msgs::Float64 m_LAP_real_pos;
    std_msgs::Float64 m_dh_plot1;
    std_msgs::Float64 m_dh_plot2;
    std_msgs::Float64 m_dh_plot3;
    std_msgs::Float64 m_dh_plot4;
    std_msgs::Float64 m_dh_plot5;
    std_msgs::Float64 m_dh_plot6;
    std_msgs::Float64 m_dh_plot7;
    std_msgs::Float64 m_dh_plot8;
    std_msgs::Float64 m_dh_plot9;

    math::Vector3 FOOT_POS;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
        this->model_ = _model;
        // initialize a PID class

        printf("[!! Hopping Robot was Loaded !!!]\n");

        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"PIDJoints");

        ROS_INFO("[DH] ==== PLUGIN_LOADED ====");

        S_Data_Save_Flag = n.subscribe("Data_save_flag", 1, &PIDJoints::S_Data_Save_Flag_func, this);
//        S_HIP_PITCH_P = n.subscribe("HIP_PITCH_P_GAIN", 1, &PIDJoints::HIP_PITCH_P_Gain, this);
//        S_HIP_PITCH_I = n.subscribe("HIP_PITCH_I_GAIN", 1, &PIDJoints::HIP_PITCH_I_Gain, this);
//        S_HIP_PITCH_D = n.subscribe("RAP_PITCH_D_GAIN", 1, &PIDJoints::HIP_PITCH_D_Gain, this);

//        S_HIP_ROLL_P = n.subscribe("HIP_ROLL_P_GAIN", 1, &PIDJoints::RKN_P_Gain, this);
//        S_HIP_ROLL_I = n.subscribe("HIP_ROLL_I_GAIN", 1, &PIDJoints::RKN_I_Gain, this);
//        S_HIP_ROLL_D = n.subscribe("HIP_ROLL_D_GAIN", 1, &PIDJoints::RKN_D_Gain, this);

//      Hip_P_Gain_sub = n.subscribe("Hip_P_Gain_setup", 1, &PIDJoints::Hip_P_Gain_sub_f, this);
//      Hip_I_Gain_sub = n.subscribe("Hip_I_Gain_setup", 1, &PIDJoints::Hip_I_Gain_sub_f, this);
//      Hip_D_Gain_sub = n.subscribe("Hip_D_Gain_setup", 1, &PIDJoints::Hip_D_Gain_sub_f, this);

//      Knee_P_Gain_sub = n.subscribe("Knee_P_Gain_setup", 1, &PIDJoints::Knee_P_Gain_sub_f, this);
//      Knee_I_Gain_sub = n.subscribe("Knee_I_Gain_setup", 1, &PIDJoints::Knee_I_Gain_sub_f, this);
//      Knee_D_Gain_sub = n.subscribe("Knee_D_Gain_setup", 1, &PIDJoints::Knee_D_Gain_sub_f, this);

//        P_Times = n.advertise<std_msgs::Float64>("times",1);
//        P_LHP_ref_pos = n.advertise<std_msgs::Float64>("ref_LHP_pos",1);
//        P_LKN_ref_pos = n.advertise<std_msgs::Float64>("ref_LKN_pos",1);
//        P_LAP_ref_pos = n.advertise<std_msgs::Float64>("ref_LAP_pos",1);
//        P_LHP_real_pos = n.advertise<std_msgs::Float64>("real_LHP_pos",1);
//        P_LKN_real_pos = n.advertise<std_msgs::Float64>("real_LKN_pos",1);
//        P_LAP_real_pos = n.advertise<std_msgs::Float64>("real_LAP_pos",1);

//        P_RKN_torque_ref = n.advertise<std_msgs::Float64>("ref_RKN_torque",1);
//        P_RKN_torque_real = n.advertise<std_msgs::Float64>("real_RKN_torque",1);
//        P_RAP_torque_ref = n.advertise<std_msgs::Float64>("ref_RAP_torque",1);
//        P_RAP_torque_real = n.advertise<std_msgs::Float64>("real_RAP_torque",1);

//        P_dh_plot1 = n.advertise<std_msgs::Float64>("real_RAP_pos",1);
//        P_dh_plot2 = n.advertise<std_msgs::Float64>("ref_RAP_pos",1);
//        P_dh_plot3 = n.advertise<std_msgs::Float64>("real_RKN_pos",1);
//        P_dh_plot4 = n.advertise<std_msgs::Float64>("ref_RKN_pos",1);
//        P_dh_plot5 = n.advertise<std_msgs::Float64>("LM_X",1);
//        P_dh_plot6 = n.advertise<std_msgs::Float64>("LM_Z",1);
//        P_dh_plot7 = n.advertise<std_msgs::Float64>("AM_Y",1);
//        P_dh_plot8 = n.advertise<std_msgs::Float64>("Right_force_z",1);
//        P_dh_plot9 = n.advertise<std_msgs::Float64>("Right_torque_x",1);

        ros::Rate loop_rate(1000); // rqt에서 쏴주는 녀석의 시간

        this->model_ = _model;

        // ================================================== PID GAIN TUNNING ============================================== //
    // RIGHT SIDE
        this->pid_HIP_ROLL.Init(150, 0.1, 1, 200, -200, 1000, -1000);
        this->HR_JOINT = this->model_->GetJoint("HIP_ROLL_JOINT");
        this->pid_HIP_PITCH.Init(200, 0.2, 1, 200, -200, 1000, -1000);
        this->HP_JOINT = this->model_->GetJoint("HIP_PITCH_JOINT");
        this->pid_KNEE.Init(200, 0.2, 1, 200, -200, 1000, -1000);
        this->KN_JOINT = this->model_->GetJoint("KNEE_JOINT");

        this->FOOT = this->model_->GetLink("CALF");

        this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PIDJoints::UpdatePID, this));

    }
// ============= Callback function setting ==============
    public:void S_Data_Save_Flag_func(const std_msgs::Float64Ptr &msg)
    {
//        data_save_flag = msg->data;
//        printf("data_save_flag = %f\n",data_save_flag);
    }

//    public: void RAP_P_Gain(const std_msgs::Float64Ptr &msg)
//    {
//      P_ = msg->data;
//      this->pid_RAP.SetPGain(P_);
//      this->pid_LAP.SetPGain(P_);
////      printf("P_ = %f\n",P_);
//    }
//    public: void RAP_I_Gain(const std_msgs::Float64Ptr &msg)
//    {
//      I_ = msg->data;
//      this->pid_RAP.SetPGain(I_);
//      this->pid_LAP.SetPGain(I_);
////      printf("I_ = %f\n",I_);
//    }
//    public: void RAP_D_Gain(const std_msgs::Float64Ptr &msg)
//    {
//      D_ = msg->data;
//      this->pid_RAP.SetPGain(D_);
//      this->pid_LAP.SetPGain(D_);
////      printf("D_ = %f\n",D_);
//    }

//  public: void RKN_P_Gain(const std_msgs::Float64Ptr &msg)
//  {
//    P_ = msg->data;
//    this->pid_RKN.SetPGain(P_);
//    this->pid_LKN.SetPGain(P_);
////      printf("P_ = %f\n",P_);
//  }
//  public: void RKN_I_Gain(const std_msgs::Float64Ptr &msg)
//  {
//    I_ = msg->data;
//    this->pid_RKN.SetPGain(I_);
//    this->pid_LKN.SetPGain(I_);
////      printf("I_ = %f\n",I_);
//  }
//  public: void RKN_D_Gain(const std_msgs::Float64Ptr &msg)
//  {
//    D_ = msg->data;
//    this->pid_RKN.SetPGain(D_);
//    this->pid_LKN.SetPGain(D_);
////      printf("D_ = %f\n",D_);
//  }

    void UpdatePID()
    {
        common::Time current_time = this->model_->GetWorld()->GetSimTime();
        HR_JOINT_W = this->HR_JOINT->GetForceTorque(0);
        HP_JOINT_W = this->HP_JOINT->GetForceTorque(0);
        KN_JOINT_W = this->KN_JOINT->GetForceTorque(0);

        dt = current_time.Double() - this->last_update_time_.Double();

        FOOT_POS = this->FOOT->GetWorldPose().pos;

      HR_err = this->HR_JOINT->GetAngle(0).Radian() - (0*D2R);//output_pos[4];
      this->pid_HIP_ROLL.Update(HR_err, dt);
      this->HR_JOINT->SetForce(1, this->pid_HIP_ROLL.GetCmd());
      if(this->pid_HIP_ROLL.GetCmd() >= 1000 || this->pid_HIP_ROLL.GetCmd() <= -1000){
        printf("======= this->pid_HIP_ROLL.GetCmd() = %f\n",this->pid_HIP_ROLL.GetCmd());
      }
      HP_err = this->HP_JOINT->GetAngle(0).Radian() - (-45*D2R);
      this->pid_HIP_PITCH.Update(HP_err, dt);
      this->HP_JOINT->SetForce(1, this->pid_HIP_PITCH.GetCmd());
      if(this->pid_HIP_PITCH.GetCmd() >= 1000 || this->pid_HIP_PITCH.GetCmd() <= -1000){
        printf("======= this->pid_HIP_PITCH.GetCmd() = %f\n",this->pid_HIP_PITCH.GetCmd());
      }

      KN_err = this->KN_JOINT->GetAngle(0).Radian() - (90*D2R);
      this->pid_KNEE.Update(KN_err, dt);
      this->KN_JOINT->SetForce(1, this->pid_KNEE.GetCmd());
      if(this->pid_KNEE.GetCmd() >= 1000 || this->pid_KNEE.GetCmd() <= -1000){
        printf("======= this->pid_KNEE.GetCmd() = %f\n",this->pid_KNEE.GetCmd());
      }

      this->last_update_time_ = current_time;      

// ================= publish data setting ==================

      m_Times.data = 0;//time;

//      m_RKN_torque_ref.data = this->pid_RKN.GetCmd();
//      m_RKN_torque_real.data = -RKN_joint_W.body2Torque.y;
//      m_RAP_torque_ref.data = this->pid_RAP.GetCmd();
//      m_RAP_torque_real.data = RAP_joint_W.body2Torque.y;

////      Right_Fz = RAR_joint_W.body1Force.y
////      Right_My = RAR_joint_W.body1Torque.x
////      Right_Mx = RAR_joint_W.body1Torque.y

////      printf("body1Force.x = %f,body1Force.y = %f,body1Force.z = %f\n",RAR_joint_W.body1Force.x,RAR_joint_W.body1Force.y,RAR_joint_W.body1Force.z);
////      printf("body2Force.x = %f,body2Force.y = %f,body2Force.z = %f\n",RAR_joint_W.body2Force.x,RAR_joint_W.body2Force.y,RAR_joint_W.body2Force.z);

////      printf("body1Torque.x = %f,body1Torque.y = %f,body1Torque.z = %f\n",RAR_joint_W.body1Torque.x,RAR_joint_W.body1Torque.y,RAR_joint_W.body1Torque.z);
////      printf("body2Torque.x = %f,body2Torque.y = %f,body2Torque.z = %f\n",RAR_joint_W.body2Torque.x,RAR_joint_W.body2Torque.y,RAR_joint_W.body2Torque.z);


////      printf("RAP torque = %f\n",RAP_joint_W.body2Torque.y);

////      F_RF_Z = RAR_joint_W.body1Force.y;
////      M_RF_X = RAR_joint_W.body1Torque.y;
////      M_RF_Y = RAR_joint_W.body1Torque.x;

////      F_LF_Z = LAR_joint_W.body1Force.y;
////      M_LF_X = LAR_joint_W.body1Torque.y;
////      M_LF_Y = LAR_joint_W.body1Torque.x;

////      LF_POS_Y =  0.15375;
////      RF_POS_Y = -0.15375;

//      m_dh_plot1.data = -this->RAP_joint->GetAngle(0).Degree();
//      m_dh_plot2.data = output_pos[1]*R2D;
//      m_dh_plot3.data = -this->RKN_joint->GetAngle(0).Degree();
//      m_dh_plot4.data = output_pos[2]*R2D;
//      m_dh_plot5.data = tmp_F[0];
//      m_dh_plot6.data = tmp_F[1];
//      m_dh_plot7.data = tmp_F[2];
//      m_dh_plot8.data = F_RF_Z;//RAR_joint_W.body1Force.y;//ref_F2[0];
//      m_dh_plot9.data = M_RF_X;//RAR_joint_W.body1Torque.x;//0;//ref_F2[1];
//      m_dh_plot10.data = M_RF_Y;//RAR_joint_W.body1Torque.y;//0;//ref_F2[2];
//      m_dh_plot11.data = com_pos[2];//this->COM_POS[2];
//      m_dh_plot12.data = com_vel[2];
//      m_dh_plot13.data = F_LF_Z;//LAR_joint_W.body1Force.y;//ref_F2[0];
//      m_dh_plot14.data = M_LF_X;//LAR_joint_W.body1Torque.x;//0;//ref_F2[1];
//      m_dh_plot15.data = M_LF_Y;//LAR_joint_W.body1Torque.y;//0;//ref_F2[2];
//      m_dh_plot16.data = com_pos[0];
//      m_dh_plot17.data = com_vel[0];
//      m_dh_plot18.data = zmp_x;
//      m_dh_plot19.data = zmp_y;
//      m_dh_plot20.data = x;
//      m_dh_plot21.data = dsp_xzmp_con;

//      // velocity
////      m_dh_plot22.data = q_dot[0]*60/(2.0*PI); // rad/s -> rpm
////      m_dh_plot23.data = q_dot[1]*60/(2.0*PI);
////      m_dh_plot24.data = q_dot[2]*60/(2.0*PI);
//      m_dh_plot22.data = actual_vel[1]*60/(2.0*PI)*50; // rad/s -> rpm
//      m_dh_plot23.data = actual_vel[2]*60/(2.0*PI)*50;
//      m_dh_plot24.data = actual_vel[3]*60/(2.0*PI)*50;
//      // torque
////      m_dh_plot25.data = this->pid_LAP.GetCmd();
////      m_dh_plot26.data = this->pid_LKN.GetCmd();
////      m_dh_plot27.data = this->pid_LHP.GetCmd();
//      m_dh_plot25.data = LAP_joint_W.body2Torque.y/50.0 ;
//      m_dh_plot26.data = LKN_joint_W.body2Torque.y/50.0;
//      m_dh_plot27.data = LHP_joint_W.body2Torque.y/50.0;

//      // power
////      m_dh_plot28.data = q_dot[0]*this->pid_LAP.GetCmd();
////      m_dh_plot29.data = q_dot[1]*this->pid_LKN.GetCmd();
////      m_dh_plot30.data = q_dot[2]*this->pid_LHP.GetCmd();
//      m_dh_plot28.data = actual_vel[1]*LAP_joint_W.body2Torque.y;
//      m_dh_plot29.data = actual_vel[2]*LKN_joint_W.body2Torque.y;
//      m_dh_plot30.data = actual_vel[3]*LHP_joint_W.body2Torque.y;

//      m_dh_plot31.data = RFOOT_POS.x;
//      m_dh_plot32.data = phase;

//      m_dh_plot33.data = real_com_x;
//      m_dh_plot34.data = real_com_z;
//      m_dh_plot35.data = real_com_x_dot;
//      m_dh_plot36.data = real_com_z_dot;
//      m_dh_plot37.data = output_pos[6]*R2D + output_pos[7]*R2D + output_pos[8]*R2D;
//      m_dh_plot38.data = UPPER_BODY_POS.x;
//      m_dh_plot39.data = q_dot[0]*(60/(2*PI))*50;
//      m_dh_plot40.data = q_dot[1]*(60/(2*PI))*50;
//      m_dh_plot41.data = q_dot[2]*(60/(2*PI))*50;
//      m_dh_plot42.data = zmp_ref[0];
//      m_dh_plot43.data = UBC_output*180/PI;

//      if(time >= t2)
//        integral_AM_Y = integral_AM_Y + tmp_F[2]*dt;
//      m_dh_plot44.data = integral_AM_Y;

//      m_dh_plot45.data = q[0]*R2D;
//      m_dh_plot46.data = q[1]*R2D;
//      m_dh_plot47.data = q[2]*R2D;
//      m_dh_plot48.data = x_dot;



////      printf("m_Times = %f,m_RAP_torque_ref = %f,m_RAP_torque_real = %f\n",m_Times.data,m_RAP_torque_ref.data,m_RAP_torque_real.data);

//      m_LHP_ref_pos.data = output_pos[6]*R2D;//ref_hip_pos*180/PI;
//      m_LKN_ref_pos.data = output_pos[7]*R2D;//ref_knee_pos*180/PI;
//      m_LAP_ref_pos.data = output_pos[8]*R2D;//ref_knee_pos*180/PI;
//      m_LHP_real_pos.data = -this->LHP_joint->GetAngle(0).Degree();
//      m_LKN_real_pos.data = -this->LKN_joint->GetAngle(0).Degree();
//      m_LAP_real_pos.data = -this->LAP_joint->GetAngle(0).Degree();

////      printf("LHP = %f,LKN = %f,LAP = %f\n",-this->LHP_joint->GetAngle(0).Degree(), - this->LHP_joint->GetAngle(0).Degree(), - this->LHP_joint->GetAngle(0).Degree());
////      printf("[real] upper body degree = %f\n",-this->LHP_joint->GetAngle(0).Degree() - this->LKN_joint->GetAngle(0).Degree() - this->LAP_joint->GetAngle(0).Degree());
////      printf("[ref]  upper body degree = %f\n",output_pos[6]*R2D + output_pos[7]*R2D + output_pos[8]*R2D);

//      P_Times.publish(m_Times);
////      P_LHP_torque.publish(m_LHP_torque);
////      P_LKN_torque.publish(m_LKN_torque);
//      P_LHP_ref_pos.publish(m_LHP_ref_pos);
//      P_LKN_ref_pos.publish(m_LKN_ref_pos);
//      P_LAP_ref_pos.publish(m_LAP_ref_pos);
//      P_LHP_real_pos.publish(m_LHP_real_pos);
//      P_LKN_real_pos.publish(m_LKN_real_pos);
//      P_LAP_real_pos.publish(m_LAP_real_pos);

//      P_RKN_torque_ref.publish(m_RKN_torque_ref);
//      P_RKN_torque_real.publish(m_RKN_torque_real);
//      P_RAP_torque_ref.publish(m_RAP_torque_ref);
//      P_RAP_torque_real.publish(m_RAP_torque_real);

//      P_dh_plot1.publish(m_dh_plot1);
//      P_dh_plot2.publish(m_dh_plot2);
//      P_dh_plot3.publish(m_dh_plot3);
//      P_dh_plot4.publish(m_dh_plot4);
//      P_dh_plot5.publish(m_dh_plot5);
//      P_dh_plot6.publish(m_dh_plot6);
//      P_dh_plot7.publish(m_dh_plot7);
//      P_dh_plot8.publish(m_dh_plot8);
//      P_dh_plot9.publish(m_dh_plot9);


    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PIDJoints)
}
