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
#include <string>  
#include <cassert>
#include <vector>


#define GET_ARRAY_LEN(array, len) { len = sizeof(array)/sizeof(array[0]);}

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
    RobotPara_totalmass = 95;
	
    RobotPara_Z_C = 0.994-0.0324;
    RobotPara_G = 9.8; 
    RobotPara_FOOT_LENGTH = 0.15; 
    RobotPara_FOOT_WIDTH = 0.08;
    RobotPara_HIP_TO_ANKLE_X_OFFSET = 0.01; 
    RobotParaClass_HALF_HIP_WIDTH = 0.084999;

    
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
        steplengthinput = 0.2;
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
    
    int xx = 1;
    mpc.Initialize(xx);
    
    _dtx = 0.005;
    
    
    
    _t_int = 0;
    _t_walkdtime_flag = 0;
    _t_walkdtime_restart_flag = 0;
    _walkdtime1 =0;
    
    _stop_walking = false;
    _start_walking_again = false;
    
    
    _walkdtime_max = mpc.Get_maximal_number(_dtx)+1;
    _wal_max = _walkdtime_max;
    
    _flag_walkdtime.setZero(_walkdtime_max);
//     _stop_flag_walkdtime.setZero(_walkdtime_max);
        
    _estimated_state.setZero();
    _rtcontrol_state_global.setZero(3,12);
    
    _Rfoot_location_feedback.setZero();
    _Lfoot_location_feedback.setZero(); 
    
 //   _state_generate_interpo.setZero(12,_walkdtime_max);
    
    _COM_IN.setZero(3,5);
    _COM_IN(2,0) = RobotPara_Z_C-_height_offsetx;
    _COM_IN(2,1) = RobotPara_Z_C-_height_offsetx; 
    _body_IN.setZero(3,5);
    _FootR_IN.setZero(3,5);
    _FootR_IN(1,0) = -RobotParaClass_HALF_HIP_WIDTH;
    _FootR_IN(1,1) = -RobotParaClass_HALF_HIP_WIDTH;
    _FootL_IN.setZero(3,5);	
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
    
    rt_file1 = "/home/jiatao/Dropbox/nmpc_pybullet/rtcontr_states.txt";    
    
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
//             printf("walkdtime:%d",_walkdtime1);
            PelvisPos = mpc.X_CoM_position_squat(_walkdtime1, _dtx);              
            body_thetax.setZero();	  
            LeftFootPosx(0) =0;    LeftFootPosx(1) = RobotParaClass_HALF_HIP_WIDTH;   LeftFootPosx(2) =0;
            RightFootPosx(0) = 0;  RightFootPosx(1) = -RobotParaClass_HALF_HIP_WIDTH; RightFootPosx(2) = 0;
            
            com_foot(0,0) = PelvisPos(0)-0.066;       com_foot(1,0) = PelvisPos(1)+0.00162;       com_foot(2,0) = PelvisPos(2)+0.0324;
            com_foot(3,0) = body_thetax(0);     com_foot(4,0) = body_thetax(1);     com_foot(5,0) = body_thetax(2);
            com_foot(6,0) = RightFootPosx(0)-0.075;   com_foot(7,0) = RightFootPosx(1)+0.00162;   com_foot(8,0) = RightFootPosx(2)+0.0324;
            com_foot(9,0) = LeftFootPosx(0)-0.075;    com_foot(10,0) = LeftFootPosx(1)+0.0016;   com_foot(11,0)= LeftFootPosx(2)+0.0324;		
            
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
    
    
            
            mpc.File_wl(rt_file1,_rtcontrol_state_global);        
        }
        else
        {      
            cout <<"xx"<<endl;
            _walkdtime1 -= (int)_height_offset_time/_dtx;
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////// Gait generation block///////////////////////////////////////////////////////            
            if(_walkdtime1 < _walkdtime_max)
            {
                
                Eigen::Vector3d COM_in1, COM_in2, COM_in3;
                Eigen::Vector3d body_in1, body_in2, body_in3;
                Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
                Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;
              
                int rowNUM = _rtcontrol_state_global.rows();
                MatrixXd feed_data = mpc.rea_save(rt_file1,rowNUM);             
                
                if (_walkdtime1>=2){
                    COM_in1 = (feed_data.block(0,0,1,3)).transpose();                  
                    body_in1 = (feed_data.block(0,3,1,3)).transpose();                    
                    FootL_in1 = (feed_data.block(0,6,1,3)).transpose();                   
                    FootR_in1 = (feed_data.block(0,9,1,3)).transpose();
                    
                    COM_in2 = (feed_data.block(1,0,1,3)).transpose();
                    body_in2 = (feed_data.block(1,3,1,3)).transpose();
                    FootL_in2 = (feed_data.block(1,6,1,3)).transpose();
                    FootR_in2 = (feed_data.block(1,9,1,3)).transpose();
                }
                else{
                    COM_in1.setZero();COM_in2.setZero();
                    COM_in3 = _COM_IN.col(_walkdtime1);	
                    COM_in1(2) = COM_in1(2) = COM_in1(2) = RobotPara_Z_C-_height_offsetx;
                    
                    body_in1.setZero();body_in2.setZero();
                    body_in3 = _body_IN.col(_walkdtime1);	
                    
                    FootL_in1.setZero();FootL_in2.setZero();
                    FootL_in3 = _FootL_IN.col(_walkdtime1);
                    FootL_in1(1)=FootL_in2(1)=FootL_in3(1)=RobotParaClass_HALF_HIP_WIDTH;
                    
                    FootR_in1.setZero();FootR_in2.setZero();
                    FootR_in3 = _FootR_IN.col(_walkdtime1);	
                    FootR_in1(1)=FootR_in2(1)=FootR_in3(1)=-RobotParaClass_HALF_HIP_WIDTH;


                    feed_data.block(0,0,1,3) = COM_in1.transpose();
                    feed_data.block(0,3,1,3) = body_in1.transpose();
                    feed_data.block(0,6,1,3) = FootL_in1.transpose();
                    feed_data.block(0,9,1,3) = FootR_in1.transpose(); 
                    feed_data.block(1,0,1,3) = COM_in3.transpose();
                    feed_data.block(1,3,1,3) = body_in3.transpose();
                    feed_data.block(1,6,1,3) = FootL_in3.transpose();
                    feed_data.block(1,9,1,3) = FootR_in3.transpose();                    
                    
                }
                //////////////should be replace by the estimated values:
                _Rfoot_location_feedback = FootL_in2;
                _Lfoot_location_feedback = FootL_in2;                
                
                
                _t_walkdtime_flag = _walkdtime1;	  		
                _t_int = floor(_walkdtime1 * _dtx / dt_mpc);
                
                mpc.mpc_loop_flag = 0;
                if (_t_int >=1)
                {
                    _flag_walkdtime(_walkdtime1) = _t_int;
//                     if (_flag_walkdtime(_walkdtime1-1) < _t_int)
                    if (feed_data(2,0) < _t_int)
                    {		
                        mpc.CoM_foot_trajection_generation_local(_t_int,_estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);	
                        mpc.Foot_trajectory_solve(_t_int, _stop_walking);
			            mpc.mpc_loop_flag = 1;			
                    }
                }

                //////////////////// End-effector traje generation-intepolation/////////////////////////////////////////
                PelvisPos = mpc.XGetSolution_CoM_position(_walkdtime1, _dtx,_t_int,COM_in1,COM_in2,COM_in3);              
                body_thetax = mpc.XGetSolution_body_inclination(_walkdtime1, _dtx, _t_int,body_in1,body_in2,body_in3);	  
                LeftFootPosx = mpc.XGetSolution_Foot_positionL(_walkdtime1, _dtx, _t_int,FootL_in1,FootL_in2,FootL_in3);
                RightFootPosx = mpc.XGetSolution_Foot_positionR(_walkdtime1, _dtx, _t_int,FootR_in1,FootR_in2,FootR_in3);

         		/////*****************leg trajectory generated by KMP********************************/////
//                 _kmp_leg_traje = mpc.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int);
//                 
//                 RightFootPosx(0) = _kmp_leg_traje(0,0);  RightFootPosx(1) = _kmp_leg_traje(1,0); RightFootPosx(2) = _kmp_leg_traje(2,0);	
//                 LeftFootPosx(0) = _kmp_leg_traje(3,0);   LeftFootPosx(1) = _kmp_leg_traje(4,0);  LeftFootPosx(2) = _kmp_leg_traje(5,0);
            

                com_foot(0,0) = PelvisPos(0)-0.066;       com_foot(1,0) = PelvisPos(1)+0.00162;       com_foot(2,0) = PelvisPos(2)+0.0324;
                com_foot(3,0) = body_thetax(0);     com_foot(4,0) = body_thetax(1);     com_foot(5,0) = body_thetax(2);
                com_foot(6,0) = RightFootPosx(0)-0.075;   com_foot(7,0) = RightFootPosx(1)+0.00162;   com_foot(8,0) = RightFootPosx(2)+0.0324;
                com_foot(9,0) = LeftFootPosx(0)-0.075;    com_foot(10,0) = LeftFootPosx(1)+0.00162;   com_foot(11,0)= LeftFootPosx(2)+0.0324;	                
                    
                    
                
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////// Admittance control preparation///////////////////////////////////////////////////////
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
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                
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
  
        	      /// PD control of the reference trajectory
/*                if (_walkdtime1 >1)
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
                }*/					
                
                //////=========================== convert_data&save-to-file:
                if (_walkdtime1>=2){
                    _rtcontrol_state_global.block(0,0,1,12) = feed_data.block(1,0,1,12);

                    _rtcontrol_state_global.block(1,0,1,3) = PelvisPos.transpose();
                    _rtcontrol_state_global.block(1,3,1,3) = body_thetax.transpose();
                    _rtcontrol_state_global.block(1,6,1,3) = LeftFootPosx.transpose();
                    _rtcontrol_state_global.block(1,9,1,3) = RightFootPosx.transpose();  
                    _rtcontrol_state_global(2,0) = _t_int;
                    mpc.File_wl(rt_file1,_rtcontrol_state_global);     
                }
                else{
                    mpc.File_wl(rt_file1,feed_data);                   
                }        

                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
/*                /// save data: logging 
                double pitch_max, roll_max;
                pitch_max=roll_max = 30*3.1415926/180;
                if ((abs(_estimated_state(9,0))>pitch_max)||(abs(_estimated_state(12,0))>pitch_max))
                {
                //savedata();  ////save .mat file
                }*/		
                
            }
            else  //walking beyond time counter
            {	
                if  (_walkdtime1 >= _walkdtime_max)
                    {
                        IsStartWalk = false;
                        int rowNUM = _rtcontrol_state_global.rows();
                        MatrixXd feed_data = mpc.rea_save(rt_file1,rowNUM);  
                        com_foot = feed_data.row(1);
                        
                        _rtcontrol_state_global.block(0,0,1,12) = feed_data.block(0,0,1,12);
                        _rtcontrol_state_global.block(1,0,1,12) = feed_data.block(1,0,1,12);
                        mpc.File_wl(rt_file1,_rtcontrol_state_global); 
                    }	
            }            
        }

      }
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

