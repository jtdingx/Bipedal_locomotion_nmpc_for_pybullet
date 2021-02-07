/*****************************************************************************
MpcRTControlClass.cpp

Description:    source file of MpcRTControlClass

@Version:   1.0
@Author:    Jiatao Ding
@Release:   Tue 27 Jun 2017 09:33:32 AM CEST
@Update:    Tue 27 Jun 2017 09:33:37 AM CEST
*****************************************************************************/
#include "MpcRTControlClass.h"
// #include "MPC/MPCClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>

using namespace Eigen;
using namespace std;

MpcRTControlClass::MpcRTControlClass()
{
// #ifdef USE_XBOT_LOGGER
//   int interleave = 1;
//   int buffer_size = logger_len;
// 	xbot_logger->createVectorVariable("mpc_hipPos", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("mpc_torso_angle", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("mpc_LeftFootPos", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("mpc_RightFootPos", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("mpc_Fr_ref", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("mpc_Fl_ref", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("mpc_Mr_ref", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("mpc_Ml_ref", 3, interleave, buffer_size);	
// 	xbot_logger->createVectorVariable("mpc_zmp_relax", 4, interleave, buffer_size);
// /*	xbot_logger->createVectorVariable("dob_est_comx", 3, interleave, buffer_size);
//     xbot_logger->createVectorVariable("dob_est_comy", 3, interleave, buffer_size);
//     xbot_logger->createVectorVariable("dob_est_comz", 3, interleave, buffer_size);
//     xbot_logger->createVectorVariable("dob_est_thetax", 3, interleave, buffer_size);
//     xbot_logger->createVectorVariable("dob_est_thetay", 3, interleave, buffer_size);
// 	xbot_logger->createVectorVariable("dob_hip_meanvalue", 3, interleave, buffer_size);	
// 	xbot_logger->createVectorVariable("dob_est_dist", 5, interleave, buffer_size);
// 	
// 	xbot_logger->createVectorVariable("ekf_est_comx", 3, interleave, buffer_size);
//     xbot_logger->createVectorVariable("ekf_est_comy", 3, interleave, buffer_size);
//     xbot_logger->createVectorVariable("ekf_est_comz", 3, interleave, buffer_size);	
// 	xbot_logger->createVectorVariable("ekf_est_dist", 3, interleave, buffer_size);*/	
// #endif
	
    Robot_name = "Talos";
    RobotPara_totalmass = 81;
	
    RobotPara_Z_C = 0.899;
    RobotPara_G = 9.8; 
    RobotPara_FOOT_LENGTH = 0.2; 
    RobotPara_FOOT_WIDTH = 0.08;
    RobotPara_HIP_TO_ANKLE_X_OFFSET = 0.03; 
    RobotParaClass_HALF_HIP_WIDTH = 0.085;

    
    mpc._RobotPara_Z_C = RobotPara_Z_C;
    mpc._RobotPara_G = RobotPara_G;
    mpc._RobotPara_FOOT_LENGTH = RobotPara_FOOT_LENGTH;
    mpc._RobotPara_FOOT_WIDTH = RobotPara_FOOT_WIDTH;
    mpc._RobotPara_HIP_TO_ANKLE_X_OFFSET = RobotPara_HIP_TO_ANKLE_X_OFFSET;
    mpc._RobotParaClass_HALF_HIP_WIDTH = RobotParaClass_HALF_HIP_WIDTH;
    
    mpc._robot_name = Robot_name;
    mpc._robot_mass = RobotPara_totalmass;

    mpc._method_flag = 2;//strategy: 0: reactive step; 1: reactive step+ body inclination; 2: reactive step+ body inclination+height variation;	

    //   mpc._lift_height = RobotPara().LIFT_HEIGHT;
    mpc._lift_height = 0.05;
    //  mpc._lift_height = 0.1;
    //  mpc._tstep = _Tstep;
    
    /*  dob._rbot_mas = RobotPara().totalmass;
    dob._g = RobotParaClass::G();
    dob._dt = RobotParaClass::dT();
    dob._z_c = RobotParaClass::Z_C()-_height_offsetx;
    
    ekf._rbot_mas = RobotPara().totalmass;
    ekf._g = RobotParaClass::G();
    ekf._dt = RobotParaClass::dT();
    ekf._z_c = RobotParaClass::Z_C()-_height_offsetx; */ 
    
    
    // initialization
    // input step parameters
    stepwidthinput = RobotParaClass_HALF_HIP_WIDTH*2; 
    if (Robot_name == "coman")
    {
        steplengthinput = 0.1;
    }
    else if (Robot_name == "Talos")
    {
        steplengthinput = 0.1;
    }
    else if (Robot_name == "cogimon")
    {
        steplengthinput = 0.15;
    } 
    //   else
    //   {DPRINTF("Errorrrrrrrr for IK\n");}

    stepheightinput = 0.00;  	  
    mpc.FootStepInputs(stepwidthinput, steplengthinput, stepheightinput);
    
    // offline initialization
    mpc.Initialize();
    
    _dtx = 0.005;
    
    
    
    _t_int = 0;
    _t_walkdtime_flag = 0;
    _t_walkdtime_restart_flag = 0;
    _walkdtime1 =0;
    
    _stop_walking = false;
    _start_walking_again = false;
    
    // loop numbern generation
    
    _walkdtime_max = mpc.Get_maximal_number(_dtx)+1;
    _wal_max = _walkdtime_max;
    
    _flag_walkdtime.setZero(_walkdtime_max);
    _stop_flag_walkdtime.setZero(_walkdtime_max);
        
    _estimated_state.setZero();
    _estimated_state_global.setZero(19,_walkdtime_max);
    
    _Rfoot_location_feedback.setZero();
    _Lfoot_location_feedback.setZero(); 
    
    _state_generate_interpo.setZero(12,_walkdtime_max);
    
    _COM_IN.setZero(3,_walkdtime_max);
    _COM_IN(2,0) = RobotPara_Z_C-_height_offsetx;
    _COM_IN(2,1) = RobotPara_Z_C-_height_offsetx; 
    _body_IN.setZero(3,_walkdtime_max);
    _FootR_IN.setZero(3,_walkdtime_max);
    _FootR_IN(1,0) = -RobotParaClass_HALF_HIP_WIDTH;
    _FootR_IN(1,1) = -RobotParaClass_HALF_HIP_WIDTH;
    _FootL_IN.setZero(3,_walkdtime_max);	
    _FootL_IN(1,0) = RobotParaClass_HALF_HIP_WIDTH;
    _FootL_IN(1,1) = RobotParaClass_HALF_HIP_WIDTH; 
    

    //// parameters for local coordinate  
    _ppx = 0.01; _pdx = 0.0001;     _pix = 0.000001;
    _ppy = 0.01;  _pdy = 0.00001;    _piy = 0.0000001;
    _ppz = 0.01;  _pdz = 0.00001;   _piz = 0.00000001; 
    
    _ppthetax= 0.01; _pdthetax =0; _pithetax =0.0001;
    _ppthetay= 0.01; _pdthetay = 0;_pithetax =0.0001; 
    _ppthetaz= 0.1;  _pdthetaz = 0.00001;_pithetaz =0.001;     
    
    _error_com_position.setZero(3);
    _error_torso_angle.setZero(3);
    
    _feedback_lamda = 0;
    
    /////*************leg***************************//////
    _kmp_leg_traje.setZero();  
    
    
    _F_r_mpc.setZero();
    _F_l_mpc.setZero();
    _M_r_mpc.setZero();
    _M_l_mpc.setZero();
    _ZMP_relax_mpc.setZero();
  
 		
    ///DoB estimation          
    dob_comx.setZero();
    dob_comy.setZero();
    dob_comz.setZero();
    dob_thetax.setZero();
    dob_theaty.setZero();
    dob_dist.setZero();
    dob_hip_pos_ac.setZero();
    
    //ekf estimation
    ekf_comx.setZero(); 
    ekf_comy.setZero(); 
    ekf_comz.setZero();
    ekf_dist.setZero();
    
    ////// ***********  end-effectors ********************////
    PelvisPos.setZero();
    body_thetax.setZero();
    LeftFootPosx.setZero();
    RightFootPosx.setZero();
    ZMPxy_realx.setZero();
    zmp_ref.setZero();
    
    F_R.setZero();
    F_L.setZero(); 
    F_R(2) = _F_r_mpc(2) = RobotPara_G/2*RobotPara_totalmass;
    F_L(2) = _F_l_mpc(2) = RobotPara_G/2*RobotPara_totalmass;
    M_R.setZero();
    M_L.setZero();  
    
    tx = 1.0;
    td = 0;
    
    IsStartWalk = true;
    _start_walking_again = false;
    
    com_foot.setZero();
    
}






Matrix<double,12,1> MpcRTControlClass::WalkingReactStepping(int walkdtime)
{
  
  // this is the loop for normal walking
    _walkdtime1 = walkdtime - _t_walkdtime_restart_flag;

    if(IsStartWalk)
    {
      if (!_start_walking_again)
      {
    // 	cout<<"normal walking_first time"<<endl;
        if (_walkdtime1*_dtx<=_height_offset_time)
        {

            PelvisPos = mpc.X_CoM_position_squat(_walkdtime1, _dtx);              
            body_thetax.setZero();	  
            LeftFootPosx(0) =0;    LeftFootPosx(1) = RobotParaClass_HALF_HIP_WIDTH;   LeftFootPosx(2) =0;
            RightFootPosx(0) = 0;  RightFootPosx(1) = -RobotParaClass_HALF_HIP_WIDTH; RightFootPosx(2) = 0;
            
            com_foot(0,0) = PelvisPos(0);       com_foot(1,0) = PelvisPos(1);       com_foot(2,0) = PelvisPos(2);
            com_foot(3,0) = body_thetax(0);     com_foot(4,0) = body_thetax(1);     com_foot(5,0) = body_thetax(2);
            com_foot(6,0) = RightFootPosx(0);   com_foot(7,0) = RightFootPosx(1);   com_foot(8,0) = RightFootPosx(2);
            com_foot(9,0) = LeftFootPosx(0);    com_foot(10,0) = LeftFootPosx(1);   com_foot(11,0)= LeftFootPosx(2);		
            
            ZMPxy_realx = zmp_ref = (LeftFootPosx+RightFootPosx)/2;
            F_R.setZero();
            _F_r_mpc.setZero();
            F_L.setZero(); 
            _F_l_mpc.setZero();
            F_R(2) = _F_r_mpc(2) = RobotPara_G/2*RobotPara_totalmass;
            F_L(2) = _F_l_mpc(2) = RobotPara_G/2*RobotPara_totalmass;
            M_R.setZero();
            M_L.setZero();
            _ZMP_relax_mpc.setZero();
            
            j_count = 1;
            bjx1 = 1;
            
            tx = 1;
            td = 0;	
            
    /*		///DoB estimation
            const RobotStateClass& irobot = _WBS.getRobotState();		
            dob.hosomo_dob(bjx1,irobot);	          
            dob_comx = dob._comxk_est;
            dob_comy = dob._comyk_est;
            dob_comz = dob._comzk_est;
            dob_thetax = dob._thetaxk_est;
            dob_theaty = dob._thetayk_est;
            dob_dist = dob._dis_est;
            dob_hip_pos_ac = dob._hip_pos_mean;
            
            ///ekf estimation
            ekf.ekf_est(bjx1,irobot);	          
            ekf_comx = ekf._comxk_est;
            ekf_comy = ekf._comyk_est;
            ekf_comz = ekf._comzk_est;
            ekf_dist = ekf._dis_est;*/		
                
        }
        else
        {      
        _walkdtime1 -= (int)_height_offset_time/_dtx;
        if(_walkdtime1 < _walkdtime_max)
        {
            _t_walkdtime_flag = _walkdtime1;	  		
            _t_int = floor(_walkdtime1 * _dtx / dt_mpc);		
            
            if (_t_int >=1)
            {
            _flag_walkdtime(_walkdtime1) = _t_int;
            
            if (_flag_walkdtime(_walkdtime1 -1) < _t_int)
            {		
            mpc.CoM_foot_trajection_generation_local(_t_int, _estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);		
            mpc.Foot_trajectory_solve(_t_int, _stop_walking);		      
            }
            }
    // 	  cout << "generation complete!!"<<endl;
    // // get the optimized footsteps: second method: use the generated reference trajectory at each 0.05s and intepolated trajectory : used in real time
            Eigen::Vector3d COM_in1, COM_in2, COM_in3;
            Eigen::Vector3d body_in1, body_in2, body_in3;
            Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
            Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;
            
            if (_walkdtime1>=2){
            COM_in1 = _COM_IN.col(_walkdtime1-2);
            COM_in2 = _COM_IN.col(_walkdtime1-1);
            COM_in3 = _COM_IN.col(_walkdtime1);

            body_in1 = _body_IN.col(_walkdtime1-2);
            body_in2 = _body_IN.col(_walkdtime1-1);
            body_in3 = _body_IN.col(_walkdtime1);

            FootL_in1 = _FootL_IN.col(_walkdtime1-2);
            FootL_in2 = _FootL_IN.col(_walkdtime1-1);
            FootL_in3 = _FootL_IN.col(_walkdtime1);	
            
            FootR_in1 = _FootR_IN.col(_walkdtime1-2);
            FootR_in2 = _FootR_IN.col(_walkdtime1-1);
            FootR_in3 = _FootR_IN.col(_walkdtime1);		  
            
            }
            else{
            COM_in1.setZero();
            COM_in2.setZero();
            COM_in3 = _COM_IN.col(_walkdtime1);	
            
            body_in1.setZero();
            body_in2.setZero();
            body_in3 = _body_IN.col(_walkdtime1);	
            
            FootL_in1.setZero();
            FootL_in2.setZero();
            FootL_in3 = _FootL_IN.col(_walkdtime1);
            
            FootR_in1.setZero();
            FootR_in2.setZero();
            FootR_in3 = _FootR_IN.col(_walkdtime1);			  
            
            }
            
            PelvisPos = mpc.XGetSolution_CoM_position(_walkdtime1, _dtx,COM_in1,COM_in2,COM_in3);              
            body_thetax = mpc.XGetSolution_body_inclination(_walkdtime1, _dtx, body_in1,body_in2,body_in3);	  
    // 		LeftFootPosx = mpc.XGetSolution_Foot_positionL(_walkdtime1, _dtx, FootL_in1,FootL_in2,FootL_in3);
    // 		RightFootPosx = mpc.XGetSolution_Foot_positionR(_walkdtime1, _dtx, FootR_in1,FootR_in2,FootR_in3);

    // 		/////*****************leg trajectory generated by KMP********************************/////
            _kmp_leg_traje = mpc.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int);
            
            RightFootPosx(0) = _kmp_leg_traje(0,0);  RightFootPosx(1) = _kmp_leg_traje(1,0); RightFootPosx(2) = _kmp_leg_traje(2,0);	
            LeftFootPosx(0) = _kmp_leg_traje(3,0);   LeftFootPosx(1) = _kmp_leg_traje(4,0);  LeftFootPosx(2) = _kmp_leg_traje(5,0);
        
            com_foot(0,0) = PelvisPos(0);       com_foot(1,0) = PelvisPos(1);       com_foot(2,0) = PelvisPos(2);
            com_foot(3,0) = body_thetax(0);     com_foot(4,0) = body_thetax(1);     com_foot(5,0) = body_thetax(2);
            com_foot(6,0) = RightFootPosx(0);   com_foot(7,0) = RightFootPosx(1);   com_foot(8,0) = RightFootPosx(2);
            com_foot(9,0) = LeftFootPosx(0);    com_foot(10,0) = LeftFootPosx(1);   com_foot(11,0)= LeftFootPosx(2);		
            
            
            mpc.Zmp_distributor(_walkdtime1,_dtx);
            
            ZMPxy_realx = zmp_ref = mpc._ZMPxy_realx;
            F_R = _F_r_mpc = mpc._F_R;
            F_L = _F_l_mpc = mpc._F_L;
            M_R = _M_r_mpc = mpc._M_R;
            M_L = _M_l_mpc = mpc._M_L;
            
            _ZMP_relax_mpc = mpc._zmp_rela_vari;
            
            j_count = mpc._j_count;
            bjx1 = mpc._bjx1;
            if (bjx1>=1)
            {		    
            tx = mpc._tx(bjx1-1);
            td = mpc._td(bjx1-1);			  
            }
            else
            {		    
            tx = 0;
            td = 0;			  
            }

            // store
            _COM_IN(0,_walkdtime1) = PelvisPos(0);
            _COM_IN(1,_walkdtime1) = PelvisPos(1);
            _COM_IN(2,_walkdtime1) = PelvisPos(2);
            
            
            _body_IN(0,_walkdtime1) = body_thetax(0);
            _body_IN(1,_walkdtime1) = body_thetax(1);
            _body_IN(2,_walkdtime1) = body_thetax(2);	  
            
            _FootL_IN(0,_walkdtime1) = LeftFootPosx(0);
            _FootL_IN(1,_walkdtime1) = LeftFootPosx(1);
            _FootL_IN(2,_walkdtime1) = LeftFootPosx(2);	

            _FootR_IN(0,_walkdtime1) = RightFootPosx(0);
            _FootR_IN(1,_walkdtime1) = RightFootPosx(1);
            _FootR_IN(2,_walkdtime1) = RightFootPosx(2);
            

            ////// reference state storage		
            _state_generate_interpo(0,_walkdtime1) = PelvisPos(0);
            _state_generate_interpo(1,_walkdtime1) = PelvisPos(1);
            _state_generate_interpo(2,_walkdtime1) = PelvisPos(2);
            _state_generate_interpo(3,_walkdtime1) = body_thetax(0);
            _state_generate_interpo(4,_walkdtime1) = body_thetax(1);
            _state_generate_interpo(5,_walkdtime1) = body_thetax(2);
            _state_generate_interpo(6,_walkdtime1) = LeftFootPosx(0);
            _state_generate_interpo(7,_walkdtime1) = LeftFootPosx(1);
            _state_generate_interpo(8,_walkdtime1) = LeftFootPosx(2);
            _state_generate_interpo(9,_walkdtime1) = RightFootPosx(0);
            _state_generate_interpo(10,_walkdtime1) = RightFootPosx(1);
            _state_generate_interpo(11,_walkdtime1) = RightFootPosx(2);	    
                
            
            ////// simulator: CoM pelvis_position && velocity:	
            _Rfoot_location_feedback = RightFootPosx;
            _Lfoot_location_feedback = LeftFootPosx;
            
            
    // 		///DoB estimation
    // 		dob.hosomo_dob(bjx1,irobot);	          
    // 		dob_comx = dob._comxk_est;
    // 		dob_comy = dob._comyk_est;
    // 		dob_comz = dob._comzk_est;
    // 		dob_thetax = dob._thetaxk_est;
    // 		dob_theaty = dob._thetayk_est;
    // 		dob_dist = dob._dis_est;
    // 		dob_hip_pos_ac = dob._hip_pos_mean;
    // 
    // 		///ekf estimation
    // 		ekf.ekf_est(bjx1,irobot);	          
    // 		ekf_comx = ekf._comxk_est;
    // 		ekf_comy = ekf._comyk_est;
    // 		ekf_comz = ekf._comzk_est;
    // 		ekf_dist = ekf._dis_est;
            

    /*		_estimated_state(0,0) = irobot.gcom[0];  
            _estimated_state(1,0) = irobot.gdcom[0];
            _estimated_state(2,0) = irobot.gddcom[0];	    
            
            _estimated_state(3,0) = irobot.gcom[1];
            _estimated_state(4,0) = irobot.gdcom[1];
            _estimated_state(5,0) = irobot.gddcom[1];	    
            _estimated_state(6,0) = irobot.gcom[2];
            _estimated_state(7,0) = irobot.gdcom[2];
            _estimated_state(8,0) = irobot.gddcom[2];
            
            _estimated_state(9,0) = IMU_Euler[0];
            _estimated_state(10,0) = IMU_AngularVel[0];
            _estimated_state(11,0) = IMU_AngularAcc[0];
            
            _estimated_state(12,0) = IMU_Euler[1];
            _estimated_state(13,0) = IMU_AngularVel[1];	   
            _estimated_state(14,0) = IMU_AngularAcc[1];	    
            
            _estimated_state(15,0) = IMU_Euler[2];
            _estimated_state(16,0) = IMU_AngularVel[2];	   
            _estimated_state(17,0) = IMU_AngularAcc[2];		    
            
            
            /// CoM position: using the hip position  	    
            _estimated_state(6,0) = hip_pos[2];	*/    	   
            
            
    /*	      /// PD control of the reference trajectory
            if (_walkdtime1 >1)
            {
            _error_com_position(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0));
            _error_com_position(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0));
            _error_com_position(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0));
            
            _error_torso_angle(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0));
            _error_torso_angle(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0));
            _error_torso_angle(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0));
            //body_thetax(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))*_ppthetax + (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))/_dtx*_pdthetax  + _error_torso_angle(0)*_pithetax;  
            // body_thetax(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))*_ppthetay + (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))/_dtx*_pdthetay+ _error_torso_angle(1)*_pithetay;  
            body_thetax(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))*_ppthetaz + (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))/_dtx*_pdthetaz+ _error_torso_angle(2)*_pithetaz;  
        
            }
    // 		*/			
            
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	     	
            /// save data: logging 
            double pitch_max, roll_max;
            pitch_max=roll_max = 30*3.1415926/180;
            if ((abs(_estimated_state(9,0))>pitch_max)||(abs(_estimated_state(12,0))>pitch_max))
            {
            //savedata();  ////save .mat file
            }		
            
        }
        else  //walking beyond time counter
        {		
            IsStartWalk = false;
            _t_walkdtime_restart_flag = walkdtime;
            
            if (_walkdtime1 == _walkdtime_max)
            {
            //////=============////data-save==================================
            mpc.File_wl(); 
            
            std::string fileName1 = "NMPC_optimal_trajectory_interpo.txt" ;
            std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.    
            
            for(int i=0; i<_state_generate_interpo.rows(); i++){
            for(int j=0; j<_state_generate_interpo.cols(); j++){
                outfile1 << (double) _state_generate_interpo(i,j) << " " ; 
            }
            outfile1 << std::endl;       // a   newline
            }
            outfile1.close();  
            
            // savedata();

    // 		  cout<<"========= Finish normal walking. ============="<<endl;
    // 		  cout<<"========= data saving. ============="<<endl;
            
            }		
            else
            {

            int _t_walkdtime_restart_flagxxx = walkdtime;
    /*		  cout<< "_t_walkdtime_restart_flag"<<_t_walkdtime_restart_flag<<endl;*/		  
            }		
        }            
        }

      }
      else 
      {
// 	    cout<<"start_walking again"<<endl;
	//////// stop walking 
	if (_stop_walking)
	{
//               cout<<"stop_walking"<<endl;
	  if (_t_int>=10)  ////stop
	  {
      /// the last four step      
	    _stop_flag_walkdtime(_t_int) = 1;
	    if (_stop_flag_walkdtime(_t_int)>_stop_flag_walkdtime(_t_int-1))
	    {
	      mpc.Indexfind((_t_int-1)*dt_mpc,0);		
	      _wal_max = round(mpc._tx(mpc._j_period)/_dtx)+ 4 * round(_Tstep/_dtx);	
	      mpc._j_period = 0;	       
	    }
			      
	    if (_wal_max > _walkdtime_max){
	      _wal_max = _walkdtime_max; 	      
	    }
	    
	    /// stop walking///////////////////// 	      
	    if(_walkdtime1 < _wal_max)
	    {	        	      
	      _t_int = floor(_walkdtime1 * _dtx / dt_mpc);		  
	      if (_t_int >=1)
	      {
		_flag_walkdtime(_walkdtime1) = _t_int;
		
		if (_flag_walkdtime(_walkdtime1 -1) < _t_int)
		{		
		  mpc.CoM_foot_trajection_generation_local(_t_int, _estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);		
		  mpc.Foot_trajectory_solve(_t_int, _stop_walking);	
		  // cout << "walking ref generation"<<endl;	      
		}
	      }
    // 	  cout << "generation complete!!"<<endl;
    // // get the optimized footsteps: second method: use the generated reference trajectory at each 0.05s and intepolated trajectory : used in real time
	      Eigen::Vector3d COM_in1, COM_in2, COM_in3;
	      Eigen::Vector3d body_in1, body_in2, body_in3;
	      Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
	      Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;
	      
	      if (_walkdtime1>=2){
		COM_in1 = _COM_IN.col(_walkdtime1-2);
		COM_in2 = _COM_IN.col(_walkdtime1-1);
		COM_in3 = _COM_IN.col(_walkdtime1);

		body_in1 = _body_IN.col(_walkdtime1-2);
		body_in2 = _body_IN.col(_walkdtime1-1);
		body_in3 = _body_IN.col(_walkdtime1);

		FootL_in1 = _FootL_IN.col(_walkdtime1-2);
		FootL_in2 = _FootL_IN.col(_walkdtime1-1);
		FootL_in3 = _FootL_IN.col(_walkdtime1);	
		
		FootR_in1 = _FootR_IN.col(_walkdtime1-2);
		FootR_in2 = _FootR_IN.col(_walkdtime1-1);
		FootR_in3 = _FootR_IN.col(_walkdtime1);		  
		
	      }
	      else{
		COM_in1.setZero();
		COM_in2.setZero();
		COM_in3 = _COM_IN.col(_walkdtime1);	
		
		body_in1.setZero();
		body_in2.setZero();
		body_in3 = _body_IN.col(_walkdtime1);	
		
		FootL_in1.setZero();
		FootL_in2.setZero();
		FootL_in3 = _FootL_IN.col(_walkdtime1);
		
		FootR_in1.setZero();
		FootR_in2.setZero();
		FootR_in3 = _FootR_IN.col(_walkdtime1);			  
		
	      }
	      
	      PelvisPos = mpc.XGetSolution_CoM_position(_walkdtime1, _dtx,COM_in1,COM_in2,COM_in3);     		
	      body_thetax = mpc.XGetSolution_body_inclination(_walkdtime1, _dtx, body_in1,body_in2,body_in3);	  
// 		  LeftFootPosx = mpc.XGetSolution_Foot_positionL(_walkdtime1, _dtx, FootL_in1,FootL_in2,FootL_in3);
// 		  RightFootPosx = mpc.XGetSolution_Foot_positionR(_walkdtime1, _dtx, FootR_in1,FootR_in2,FootR_in3);
		
// 		  /////*****************leg trajectory generated by KMP********************************/////
	      _kmp_leg_traje = mpc.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int);
	      
	      RightFootPosx(0) = _kmp_leg_traje(0,0);  RightFootPosx(1) = _kmp_leg_traje(1,0); RightFootPosx(2) = _kmp_leg_traje(2,0);	
	      LeftFootPosx(0) = _kmp_leg_traje(3,0);   LeftFootPosx(1) = _kmp_leg_traje(4,0);  LeftFootPosx(2) = _kmp_leg_traje(5,0);	  
		
	      com_foot(0,0) = PelvisPos(0);       com_foot(1,0) = PelvisPos(1);       com_foot(2,0) = PelvisPos(2);
	      com_foot(3,0) = body_thetax(0);     com_foot(4,0) = body_thetax(1);     com_foot(5,0) = body_thetax(2);
	      com_foot(6,0) = RightFootPosx(0);   com_foot(7,0) = RightFootPosx(1);   com_foot(8,0) = RightFootPosx(2);
	      com_foot(9,0) = LeftFootPosx(0);    com_foot(10,0) = LeftFootPosx(1);   com_foot(11,0)= LeftFootPosx(2);		  
	      
	      // store
	      _COM_IN(0,_walkdtime1) = PelvisPos(0);
	      _COM_IN(1,_walkdtime1) = PelvisPos(1);
	      _COM_IN(2,_walkdtime1) = PelvisPos(2);
	      
	      _body_IN(0,_walkdtime1) = body_thetax(0);
	      _body_IN(1,_walkdtime1) = body_thetax(1);
	      _body_IN(2,_walkdtime1) = body_thetax(2);	  
	      
	      _FootL_IN(0,_walkdtime1) = LeftFootPosx(0);
	      _FootL_IN(1,_walkdtime1) = LeftFootPosx(1);
	      _FootL_IN(2,_walkdtime1) = LeftFootPosx(2);	

	      _FootR_IN(0,_walkdtime1) = RightFootPosx(0);
	      _FootR_IN(1,_walkdtime1) = RightFootPosx(1);
	      _FootR_IN(2,_walkdtime1) = RightFootPosx(2);
	      
	      ///////////////////////////////////////////////////////////////////////
	      ////// reference state storage
	      
	      _state_generate_interpo(0,_walkdtime1) = PelvisPos(0);
	      _state_generate_interpo(1,_walkdtime1) = PelvisPos(1);
	      _state_generate_interpo(2,_walkdtime1) = PelvisPos(2);
	      _state_generate_interpo(3,_walkdtime1) = body_thetax(0);
	      _state_generate_interpo(4,_walkdtime1) = body_thetax(1);
	      _state_generate_interpo(5,_walkdtime1) = body_thetax(2);
	      _state_generate_interpo(6,_walkdtime1) = LeftFootPosx(0);
	      _state_generate_interpo(7,_walkdtime1) = LeftFootPosx(1);
	      _state_generate_interpo(8,_walkdtime1) = LeftFootPosx(2);
	      _state_generate_interpo(9,_walkdtime1) = RightFootPosx(0);
	      _state_generate_interpo(10,_walkdtime1) = RightFootPosx(1);
	      _state_generate_interpo(11,_walkdtime1) = RightFootPosx(2);	    

	      _Rfoot_location_feedback = RightFootPosx;
	      _Lfoot_location_feedback = LeftFootPosx;		  
	      
/*		  ////// simulator: CoM pelvis_position && velocity:		  		
	      ///DoB estimation
	      dob.hosomo_dob(bjx1,irobot);	          
	      dob_comx = dob._comxk_est;
	      dob_comy = dob._comyk_est;
	      dob_comz = dob._comzk_est;
	      dob_thetax = dob._thetaxk_est;
	      dob_theaty = dob._thetayk_est;
	      dob_dist = dob._dis_est;
	      dob_hip_pos_ac = dob._hip_pos_mean;
	      
	      ///ekf estimation
	      ekf.ekf_est(bjx1,irobot);	          
	      ekf_comx = ekf._comxk_est;
	      ekf_comy = ekf._comyk_est;
	      ekf_comz = ekf._comzk_est;
	      ekf_dist = ekf._dis_est;*/		  

/*		  _estimated_state(0,0) = irobot.gcom[0];  
	      _estimated_state(1,0) = irobot.gdcom[0];
	      _estimated_state(2,0) = irobot.gddcom[0];	    
	      
	      _estimated_state(3,0) = irobot.gcom[1];
	      _estimated_state(4,0) = irobot.gdcom[1];
	      _estimated_state(5,0) = irobot.gddcom[1];	    
	      _estimated_state(6,0) = irobot.gcom[2];
	      _estimated_state(7,0) = irobot.gdcom[2];
	      _estimated_state(8,0) = irobot.gddcom[2];
	      
	      _estimated_state(9,0) = IMU_Euler[0];
	      _estimated_state(10,0) = IMU_AngularVel[0];
	      _estimated_state(11,0) = IMU_AngularAcc[0];
	      
	      _estimated_state(12,0) = IMU_Euler[1];
	      _estimated_state(13,0) = IMU_AngularVel[1];	   
	      _estimated_state(14,0) = IMU_AngularAcc[1];	    
	      
	      _estimated_state(15,0) = IMU_Euler[2];
	      _estimated_state(16,0) = IMU_AngularVel[2];	   
	      _estimated_state(17,0) = IMU_AngularAcc[2];		    
			
	      _estimated_state(6,0) = hip_pos[2];	*/    	   
	      
	      
	  /// PD control of the reference trajectory
//   		if (_walkdtime1 >1)
//   		{
//   		  _error_com_position(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0));
//   		  _error_com_position(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0));
//   		  _error_com_position(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0));
//   		  
//   		  _error_torso_angle(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0));
//   		  _error_torso_angle(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0));
//   		  _error_torso_angle(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0));
//   		  
//   		  
// //   		  PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx + _error_com_position(0)*_pix;  
// //   		  PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
// //   		  PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;
//   		     
// //   		  body_thetax(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))*_ppthetax + (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))/_dtx*_pdthetax  + _error_torso_angle(0)*_pithetax;  
// //   		  body_thetax(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))*_ppthetay + (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))/_dtx*_pdthetay+ _error_torso_angle(1)*_pithetay;  
//   		  body_thetax(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))*_ppthetaz + (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))/_dtx*_pdthetaz+ _error_torso_angle(2)*_pithetaz;  	    
//   		}
	      
	      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	     	
	      /// save data: logging 
	      double pitch_max, roll_max;
	      pitch_max=roll_max = 30*3.1415926/180;
	      if ((abs(_estimated_state(9,0))>pitch_max)||(abs(_estimated_state(12,0))>pitch_max))
	      {
		//savedata();
	      }		  		  
	      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      	      
	    }
	    else
	    {
	      _t_walkdtime_restart_flag = walkdtime;	
	    }		      
	  } 
	  else
	  {
		_t_walkdtime_restart_flag = walkdtime;    
	  }	      
	}	  	  
	else   ///////normal walking ===============================
	{
	    if(_walkdtime1 < _walkdtime_max)
	    {
	      _t_walkdtime_flag = _walkdtime1;	  		
	      _t_int = floor(_walkdtime1 * _dtx / dt_mpc);		
	      
	      if (_t_int >=1)
	      {
		_flag_walkdtime(_walkdtime1) = _t_int;
		
		if (_flag_walkdtime(_walkdtime1 -1) < _t_int)
		{		
		  mpc.CoM_foot_trajection_generation_local(_t_int, _estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);		
		  mpc.Foot_trajectory_solve(_t_int, _stop_walking);	
    // 	      cout << "walking ref generation"<<endl;	      
		}
	      }
    // 	  cout << "generation complete!!"<<endl;
    // // get the optimized footsteps: second method: use the generated reference trajectory at each 0.05s and intepolated trajectory : used in real time
	      Eigen::Vector3d COM_in1, COM_in2, COM_in3;
	      Eigen::Vector3d body_in1, body_in2, body_in3;
	      Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
	      Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;
	      
	      if (_walkdtime1>=2){
		COM_in1 = _COM_IN.col(_walkdtime1-2);
		COM_in2 = _COM_IN.col(_walkdtime1-1);
		COM_in3 = _COM_IN.col(_walkdtime1);

		body_in1 = _body_IN.col(_walkdtime1-2);
		body_in2 = _body_IN.col(_walkdtime1-1);
		body_in3 = _body_IN.col(_walkdtime1);

		FootL_in1 = _FootL_IN.col(_walkdtime1-2);
		FootL_in2 = _FootL_IN.col(_walkdtime1-1);
		FootL_in3 = _FootL_IN.col(_walkdtime1);	
		
		FootR_in1 = _FootR_IN.col(_walkdtime1-2);
		FootR_in2 = _FootR_IN.col(_walkdtime1-1);
		FootR_in3 = _FootR_IN.col(_walkdtime1);		  
		
	      }
	      else{
		COM_in1.setZero();
		COM_in2.setZero();
		COM_in3 = _COM_IN.col(_walkdtime1);	
		
		body_in1.setZero();
		body_in2.setZero();
		body_in3 = _body_IN.col(_walkdtime1);	
		
		FootL_in1.setZero();
		FootL_in2.setZero();
		FootL_in3 = _FootL_IN.col(_walkdtime1);
		
		FootR_in1.setZero();
		FootR_in2.setZero();
		FootR_in3 = _FootR_IN.col(_walkdtime1);			  
		
	      }
	      
	      PelvisPos = mpc.XGetSolution_CoM_position(_walkdtime1, _dtx,COM_in1,COM_in2,COM_in3);              
	      body_thetax = mpc.XGetSolution_body_inclination(_walkdtime1, _dtx, body_in1,body_in2,body_in3);	  
// 		  LeftFootPosx = mpc.XGetSolution_Foot_positionL(_walkdtime1, _dtx, FootL_in1,FootL_in2,FootL_in3);
// 		  RightFootPosx = mpc.XGetSolution_Foot_positionR(_walkdtime1, _dtx, FootR_in1,FootR_in2,FootR_in3);

// 		/////*****************leg trajectory generated by KMP********************************/////
	      _kmp_leg_traje = mpc.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int);
	      
	      RightFootPosx(0) = _kmp_leg_traje(0,0);  RightFootPosx(1) = _kmp_leg_traje(1,0); RightFootPosx(2) = _kmp_leg_traje(2,0);	
	      LeftFootPosx(0) = _kmp_leg_traje(3,0);   LeftFootPosx(1) = _kmp_leg_traje(4,0);  LeftFootPosx(2) = _kmp_leg_traje(5,0);		  
	      
	      com_foot(0,0) = PelvisPos(0);       com_foot(1,0) = PelvisPos(1);       com_foot(2,0) = PelvisPos(2);
	      com_foot(3,0) = body_thetax(0);     com_foot(4,0) = body_thetax(1);     com_foot(5,0) = body_thetax(2);
	      com_foot(6,0) = RightFootPosx(0);   com_foot(7,0) = RightFootPosx(1);   com_foot(8,0) = RightFootPosx(2);
	      com_foot(9,0) = LeftFootPosx(0);    com_foot(10,0) = LeftFootPosx(1);   com_foot(11,0)= LeftFootPosx(2);		  
	      
	      // store
	      _COM_IN(0,_walkdtime1) = PelvisPos(0);
	      _COM_IN(1,_walkdtime1) = PelvisPos(1);
	      _COM_IN(2,_walkdtime1) = PelvisPos(2);
	      
	      _body_IN(0,_walkdtime1) = body_thetax(0);
	      _body_IN(1,_walkdtime1) = body_thetax(1);
	      _body_IN(2,_walkdtime1) = body_thetax(2);	  
	      
	      _FootL_IN(0,_walkdtime1) = LeftFootPosx(0);
	      _FootL_IN(1,_walkdtime1) = LeftFootPosx(1);
	      _FootL_IN(2,_walkdtime1) = LeftFootPosx(2);	

	      _FootR_IN(0,_walkdtime1) = RightFootPosx(0);
	      _FootR_IN(1,_walkdtime1) = RightFootPosx(1);
	      _FootR_IN(2,_walkdtime1) = RightFootPosx(2);
	      
	      ///////////////////////////////////////////////////////////////////////
	      ////// reference state storage
	      
	      _state_generate_interpo(0,_walkdtime1) = PelvisPos(0);
	      _state_generate_interpo(1,_walkdtime1) = PelvisPos(1);
	      _state_generate_interpo(2,_walkdtime1) = PelvisPos(2);
	      _state_generate_interpo(3,_walkdtime1) = body_thetax(0);
	      _state_generate_interpo(4,_walkdtime1) = body_thetax(1);
	      _state_generate_interpo(5,_walkdtime1) = body_thetax(2);
	      _state_generate_interpo(6,_walkdtime1) = LeftFootPosx(0);
	      _state_generate_interpo(7,_walkdtime1) = LeftFootPosx(1);
	      _state_generate_interpo(8,_walkdtime1) = LeftFootPosx(2);
	      _state_generate_interpo(9,_walkdtime1) = RightFootPosx(0);
	      _state_generate_interpo(10,_walkdtime1) = RightFootPosx(1);
	      _state_generate_interpo(11,_walkdtime1) = RightFootPosx(2);
	      
	      _Rfoot_location_feedback = RightFootPosx;
	      _Lfoot_location_feedback = LeftFootPosx;
	      
		      
	      
	      ////// simulator: CoM pelvis_position && velocity:		  
/*		  ///DoB estimation
	      dob.hosomo_dob(bjx1,irobot);	          
	      dob_comx = dob._comxk_est;
	      dob_comy = dob._comyk_est;
	      dob_comz = dob._comzk_est;
	      dob_thetax = dob._thetaxk_est;
	      dob_theaty = dob._thetayk_est;
	      dob_dist = dob._dis_est;			  
	      dob_hip_pos_ac = dob._hip_pos_mean;  
		
	      ///ekf estimation
	      ekf.ekf_est(bjx1,irobot);	          
	      ekf_comx = ekf._comxk_est;
	      ekf_comy = ekf._comyk_est;
	      ekf_comz = ekf._comzk_est;
	      ekf_dist = ekf._dis_est;*/		  

/*		  _estimated_state(0,0) = irobot.gcom[0];  
	      _estimated_state(1,0) = irobot.gdcom[0];
	      _estimated_state(2,0) = irobot.gddcom[0];	    
	      
	      _estimated_state(3,0) = irobot.gcom[1];
	      _estimated_state(4,0) = irobot.gdcom[1];
	      _estimated_state(5,0) = irobot.gddcom[1];	    
	      _estimated_state(6,0) = irobot.gcom[2];
	      _estimated_state(7,0) = irobot.gdcom[2];
	      _estimated_state(8,0) = irobot.gddcom[2];
	      
	      _estimated_state(9,0) = IMU_Euler[0];
	      _estimated_state(10,0) = IMU_AngularVel[0];
	      _estimated_state(11,0) = IMU_AngularAcc[0];
	      
	      _estimated_state(12,0) = IMU_Euler[1];
	      _estimated_state(13,0) = IMU_AngularVel[1];	   
	      _estimated_state(14,0) = IMU_AngularAcc[1];	    
	      
	      _estimated_state(15,0) = IMU_Euler[2];
	      _estimated_state(16,0) = IMU_AngularVel[2];	   
	      _estimated_state(17,0) = IMU_AngularAcc[2];		    
		  
	      _estimated_state(6,0) = hip_pos[2];*/	    	   
	      
	      
/*  	      /// PD control of the reference trajectory
	    if (_walkdtime1 >1)
	    {
	      _error_com_position(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0));
	      _error_com_position(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0));
	      _error_com_position(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0));
	      
	      _error_torso_angle(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0));
	      _error_torso_angle(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0));
	      _error_torso_angle(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0));
		      
	      PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx + _error_com_position(0)*_pix;  
	      PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
	      PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;		    

	      body_thetax(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))*_ppthetax + (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))/_dtx*_pdthetax  + _error_torso_angle(0)*_pithetax;  
	      body_thetax(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))*_ppthetay + (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))/_dtx*_pdthetay+ _error_torso_angle(1)*_pithetay;  
	      body_thetax(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))*_ppthetaz + (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))/_dtx*_pdthetaz+ _error_torso_angle(2)*_pithetaz;  	    
	    }*/		  
	      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	     	
	      /// save data: logging 
	      double pitch_max, roll_max;
	      pitch_max=roll_max = 30*3.1415926/180;
	      if ((abs(_estimated_state(9,0))>pitch_max)||(abs(_estimated_state(12,0))>pitch_max))
	      {
		//savedata();
	      }		  
	    }
	    else  //walking beyond time counter
	    {
	      _t_walkdtime_restart_flag = walkdtime;
	      
	      IsStartWalk = false;	 		  
	      _t_walkdtime_restart_flag = walkdtime;
	      
	      if (_walkdtime1 == _walkdtime_max)
	      {
	      //////=============////data-save==================================
            mpc.File_wl(); 
        
            std::string fileName1 = "NMPC_optimal_trajectory_interpo.txt" ;
            std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.    		    
            for(int i=0; i<_state_generate_interpo.rows(); i++){
                for(int j=0; j<_state_generate_interpo.cols(); j++){
                outfile1 << (double) _state_generate_interpo(i,j) << " " ; 
                }
                outfile1 << std::endl;       // a   newline
            }
            outfile1.close();  
		
		//savedata();	    
	      }		  
	      else
	      {
		    int _t_walkdtime_restart_flagxxx = walkdtime;
/*		    cout<< "_t_walkdtime_restart_flag"<<_t_walkdtime_restart_flag<<endl;*/		  
	      }				  
	    }	    	    
	}    	    
      }


    }
    else
    { 
      /////keep the current state:double support phase		
      ZMPxy_realx = zmp_ref =(LeftFootPosx+RightFootPosx)/2;
      F_R.setZero();
      _F_r_mpc.setZero();
      F_L.setZero(); 
      _F_l_mpc.setZero();
      F_R(2) = _F_r_mpc(2) = RobotPara_G/2*RobotPara_totalmass;
      F_L(2) = _F_l_mpc(2) = RobotPara_G/2*RobotPara_totalmass;
      M_R.setZero();
      M_L.setZero();
      
      j_count = 1;
      bjx1 = 1;
      
      tx = 1;
      td = 0;	
      
/*		///DoB estimation
      const RobotStateClass& irobot = _WBS.getRobotState();
      dob.hosomo_dob(bjx1,irobot);	          
      dob_comx = dob._comxk_est;
      dob_comy = dob._comyk_est;
      dob_comz = dob._comzk_est;
      dob_thetax = dob._thetaxk_est;
      dob_theaty = dob._thetayk_est;
      dob_dist = dob._dis_est;
      dob_hip_pos_ac = dob._hip_pos_mean;
      
      ///ekf estimation
      ekf.ekf_est(bjx1,irobot);	          
      ekf_comx = ekf._comxk_est;
      ekf_comy = ekf._comyk_est;
      ekf_comz = ekf._comzk_est;
      ekf_dist = ekf._dis_est;*/	
      
//    savedata();
    }
    
    return com_foot;	
}



// void MpcRTControlClass::StartWalking()
// {
//   IsStartWalk = true;
//   if (_stop_walking)
//   {
//     _start_walking_again = true;
//   }
//    _stop_walking = false;
//   
// }
// 
// void MpcRTControlClass::StopWalking()
// {
//   if (_t_int <10)
//   {
//     IsStartWalk = false;  
//   }
//   else
//   {     
//     IsStartWalk = true;
//     _stop_walking = true;        
//   }      
// }



// void MpcRTControlClass::InternalLoggerLoop()
// {
// #ifdef USE_XBOT_LOGGER
// 	xbot_logger->add("mpc_hipPos", PelvisPos);
// 	xbot_logger->add("mpc_torso_angle", body_thetax);
// 	xbot_logger->add("mpc_LeftFootPos", LeftFootPosx);
// 	xbot_logger->add("mpc_RightFootPos", RightFootPosx);
// 	xbot_logger->add("mpc_Fr_ref", _F_r_mpc);
// 	xbot_logger->add("mpc_Fl_ref", _F_l_mpc);
// 	xbot_logger->add("mpc_Mr_ref", _M_r_mpc);
// 	xbot_logger->add("mpc_Ml_ref", _M_l_mpc);
//     xbot_logger->add("mpc_zmp_relax", _ZMP_relax_mpc); 
// 	
// /*	xbot_logger->add("dob_est_comx", dob_comx);	
// 	xbot_logger->add("dob_est_comy", dob_comy);		
// 	xbot_logger->add("dob_est_comz", dob_comz);
// 	xbot_logger->add("dob_est_thetax", dob_thetax);	
// 	xbot_logger->add("dob_est_thetay", dob_theaty);		
// 	xbot_logger->add("dob_est_dist", dob_dist);	
// 	xbot_logger->add("dob_hip_meanvalue", dob_hip_pos_ac);	
// 
// 
// 	xbot_logger->add("ekf_est_comx", ekf_comx);	
// 	xbot_logger->add("ekf_est_comy", ekf_comy);		
// 	xbot_logger->add("ekf_est_comz", ekf_comz);		
// 	xbot_logger->add("ekf_est_dist", ekf_dist);	*/	
// 	
// #endif
// }

