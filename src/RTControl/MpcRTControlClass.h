/*****************************************************************************
MpcRTControlClass.h

Description:	Header file of MpcRTControlClass

@Version:	1.0
@Author:	Jiatao Ding
@Release:	Tue 27 Jun 2017 09:31:28 AM CEST
@Update:	Tue 27 Jun 2017 09:31:24 AM CEST
*****************************************************************************/
#pragma once
#include "MPC/MPCClass.h"
#include "KMP/kmp.h"
// #include "Stateestimator/Dob.h"
// 
// #include "Stateestimator/ekf.h"

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>

const double dt_mpc = 0.1;   // definition of sampling time of MPC solover
const double _height_offsetx = 0.05;  
const double _height_offset_time = 2;  
const double _Tstep = 0.6; 

using namespace Eigen;
using namespace std;

class MpcRTControlClass
{
public:
	MpcRTControlClass();
	~MpcRTControlClass(){};


    MPCClass mpc;
    
    double RobotPara_Z_C, RobotPara_G, RobotPara_FOOT_LENGTH, RobotPara_FOOT_WIDTH, RobotPara_HIP_TO_ANKLE_X_OFFSET, RobotParaClass_HALF_HIP_WIDTH, RobotParaClass_PreviewT, RobotPara_totalmass;  
    std::string Robot_name;
    
    bool IsStartWalk;
    
    Matrix<double,12,1> WalkingReactStepping(int walkdtime);

    Matrix<double,12,1> com_foot;  
    
    std::string rt_file1;    
    
private:
//     void StartWalking();
//     void StopWalking();
    
    /// step parameters reference
    double stepwidthinput,steplengthinput,stepheightinput;

    int _t_int;
    double _dtx;
    

    
    int _t_walkdtime_flag, _t_walkdtime_restart_flag;
    bool _stop_walking, _start_walking_again;
    
    double _ppx, _pix, _pdx,  _ppy,_piy, _pdy, _ppz,_piz,  _pdz, _ppthetax, _pdthetax, _pithetax,  _ppthetay, _pithetay,  _pdthetay,  _ppthetaz, _pithetaz,  _pdthetaz; 
    
    Eigen::VectorXd _error_com_position, _error_torso_angle;
    
    
    Eigen::VectorXd _flag_walkdtime;
    
//     Eigen::VectorXd _stop_flag_walkdtime;
    
    int _walkdtime_max, _wal_max;	
    int _walkdtime1;	
    
    Eigen::MatrixXd _COM_IN;
    Eigen::MatrixXd _body_IN;	
    Eigen::MatrixXd _FootR_IN;	
    Eigen::MatrixXd _FootL_IN;	

    Eigen::Matrix<double,18,1> _estimated_state;	

    Eigen::Vector3d _Rfoot_location_feedback,_Lfoot_location_feedback;
    
    
    Eigen::MatrixXd _rtcontrol_state_global, _mpc_state;    
    
//     Eigen::MatrixXd _state_generate_interpo;
    



//     void InternalLoggerLoop();


    Vector3d PelvisPos,body_thetax,LeftFootPosx,RightFootPosx;
    Vector3d ZMPxy_realx,zmp_ref,F_R,F_L,M_R,M_L;
    
    int j_count,bjx1;
    double tx,td;
    
    
protected:
	
    double _feedback_lamda;	
    
    
    Vector3d _F_r_mpc, _F_l_mpc,_M_r_mpc,_M_l_mpc;
    Vector4d _ZMP_relax_mpc;
    
    Vector3d dob_comx, dob_comy, dob_comz,dob_thetax,dob_theaty, dob_hip_pos_ac;
    Eigen::Matrix<double,5,1> dob_dist;


    Vector3d ekf_comx, ekf_comy, ekf_comz, ekf_dist;	
    /****KMP based trajectory***********************/
    Eigen::Matrix<double,6,1> _kmp_leg_traje;	
	
	
};

