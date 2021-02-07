/*****************************************************************************
MPCClass.cpp

Description:    source file of MPCClass

@Version:   1.0
@Author:    Jiatao Ding
@Release:   Thu 02 Aug 2018 12:33:23 PM CEST
@Update:    Thu 02 Aug 2018 12:33:19 PM CEST
*****************************************************************************/
#include <cstdio>
#include <cstdlib>

#include <iostream>
// #include "/usr/local/include/eigen3/Eigen/Dense"
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include <vector>
#include "MPCClass.h"
#include <string>  
#include <cassert>



using namespace Eigen;
using namespace std;


MPCClass::MPCClass()                    ///declaration function
	: QPBaseClass()
    , _RobotPara_Z_C(0.899)
    , _RobotPara_G(9.8)
    , _RobotPara_FOOT_LENGTH(0.15)
    , _RobotPara_FOOT_WIDTH(0.1)
    , _RobotPara_HIP_TO_ANKLE_X_OFFSET(0.01)
    , _RobotParaClass_HALF_HIP_WIDTH(0.0)    
	, _robot_name("")
	, _robot_mass(0.0)
	, _lift_height(0.0)
	, _method_flag(0)
	, _n_end_walking(1)
	, _j_period(0)
	, _F_R(0,0,0)
	, _F_L(0,0,0)
	, _M_R(0,0,0)
	, _M_L(0,0,0)   
{
  
}

////////////////step parameters input============================================================
void MPCClass::FootStepInputs(double stepwidth, double steplength, double stepheight)
{	
	_steplength.setConstant(steplength);
    _steplength(_footstepsnumber-1) = 0;
    _steplength(_footstepsnumber-2) = 0;
    _steplength(_footstepsnumber-3) = 0;
    _steplength(_footstepsnumber-4) = 0;	
    _steplength(_footstepsnumber-5) = steplength/4;
	_steplength(0) = 0;
	_steplength(1) = steplength/2;
	
	_stepwidth.setConstant(stepwidth);
	_stepwidth(0) = _stepwidth(0)/2;
	
	_stepheight.setConstant(stepheight);     


	_lift_height_ref.setConstant(_lift_height);
    _lift_height_ref(_footstepsnumber-1) = 0;
    _lift_height_ref(_footstepsnumber-2) = 0;	
    _lift_height_ref(_footstepsnumber-3) = 0.02; 
    _lift_height_ref(_footstepsnumber-4) = 0.04; 	
}

/////////////////////// initialize all the variables============================================================
void MPCClass::Initialize(int j_loop)
{
     ///initialize
	
	_query_kmp = zeros<vec>(1);
	_mean_kmp = zeros<vec>(6);
	
  	_inDim_kmp  = 1; 	      		    //input dimension
	_outDim_kmp = 3+3; 	      		    //output dimension
	_pvFlag_kmp = 1;			    // output: pos (and vel)
	///////////// adjust KMP parameters
	_lamda_kmp  = 5, _kh_kmp = 0.75;	    	    //set kmp parameters 


	//static char fileName1[]="/home/jiatao/Dropbox/NP_IL_ODE_humanoids2019/T_V_O_IL_height/src/walking/MPC/referdata_swing.txt";  
	static char fileName1[]="/home/jiatao/Dropbox/nmpc_pybullet/src/KMP/referdata_swing.txt"; 
	
	_data_kmp.load(fileName1);         	    // load original data
	kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp); // initialize kmp
	kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp); // initialize kmp
	
	
	/////// swing leg trajectory generation using kmp_initialize
	_Lfootx_kmp.setZero();  _Lfooty_kmp.setConstant(_stepwidth(0)); _Lfootz_kmp.setZero(); 
	_Lfootvx_kmp.setZero(); _Lfootvy_kmp.setZero();                 _Lfootvz_kmp.setZero(); 

	_Rfootx_kmp.setZero();  _Rfooty_kmp.setConstant(-_stepwidth(0));_Rfootz_kmp.setZero(); 
	_Rfootvx_kmp.setZero(); _Rfootvy_kmp.setZero();                 _Rfootvz_kmp.setZero(); 
	

///// stair climbing after height variance: 5cm:	
	_steplength(6) = 0.15;	
	_steplength(7) = 0.05;
	_steplength(8) = 0.0; 		
	_steplength(9) = -0.05; 
	_steplength(10) = -0.1; 	
	_steplength(11) = -0.1;    
	_steplength(12) = -0.1; 
 	_steplength(13) = 0; 
 	_steplength(14) = 0.1;       
	
// 	_stepheight(11) = 0.05;			
// 	_stepheight(12) =0.001; 	
//         _stepheight(14) =-0.05; 
//         _stepheight(13) =-0.05; 
	
 	// ==step loctions setup==
 	
/*    _RobotPara_Z_C = 0.899;
    _RobotPara_G = 9.8; 
    _RobotPara_FOOT_LENGTH = 0.2; 
    _RobotPara_FOOT_WIDTH = 0.08;
    _RobotPara_HIP_TO_ANKLE_X_OFFSET = 0.03; 
    _RobotParaClass_HALF_HIP_WIDTH = 0.085;
    RobotParaClass_PreviewT  = 0;*/	
 	
	
    _footx_ref.setZero();
	_footy_ref.setZero();
	_footz_ref.setZero();	
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
	  _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);   
	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
	}	
//	cout<<"_footx_ref" <<_footx_ref<<endl;
        
//  sampling time & step cycle	
	_ts.setConstant(_tstep);
	_td = 0.2*_ts;                                // dsp time
	
	_tx.setZero();   // start time for each step cycle	
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	  _tx(i) = round(_tx(i)/_dt)*_dt -0.00001;	  
	}	
	   

	_t_whole.setLinSpaced(_nsum,_dt,_tx(_footstepsnumber-1));  ///sampling time sequence for entire step period		
    // ==initial parameters & matrix for MPC==
    _hcom = _RobotPara_Z_C-_height_offset;                                                    /// intial comz
	_Hcom.setConstant(_hcom); /// for comz reference during the predictive window: matrix operations: height variance between support feet and body center	
	_ggg.setConstant(_RobotPara_G); //// acceleration of gravity: 1*1 matrix: matrix operations

    _zmpx_real.setZero(); _zmpy_real.setZero();                              
	_comx.setZero(); _comvx.setZero(); _comax.setZero();
	_comy.setZero(); _comvy.setZero(); _comay.setZero();
	_comz.setZero(); _comvz.setZero(); _comaz.setZero();	
	_thetax.setZero(); _thetavx.setZero(); _thetaax.setZero();
	_thetay.setZero(); _thetavy.setZero(); _thetaay.setZero();
	_thetaz.setZero(); _thetavz.setZero(); _thetaaz.setZero();	
	_torquex_real.setZero(); _torquey_real.setZero();  
	
	//// state variable for mpc
	_xk.setZero(); 
	_yk.setZero(); 
	_zk.setZero();
	_thetaxk.setZero(); 
	_thetayk.setZero();
	_x_vacc_k.setZero(); 
	_y_vacc_k.setZero();
	_z_vacc_k.setZero(); 
	_thetax_vacc_k.setZero(); 
	_thetay_vacc_k.setZero(); 	
///////================================================================================================			
	_a << 1, _dt,    
	      0,   1;
	_b << pow(_dt,2)/2,
	               _dt;		
	
	_cp.setZero();
	_cp(0,0) = 1;
	_cv.setZero();
	_cv(0,1) = 1;	
		
	//predictive model matrixs: just calculated once
	_pps.setZero(); _ppu.setZero();
	_pvs.setZero(); _pvu.setZero();
	
    _pps = Matrix_ps(_a,_nh,_cp);
	_pvs = Matrix_ps(_a,_nh,_cv);
      
	_ppu = Matrix_pu(_a,_b,_nh,_cp);
	_pvu = Matrix_pu(_a,_b,_nh,_cv);
	
	_pvu_2 = _pvu.transpose()*_pvu;
	_ppu_2 = _ppu.transpose()*_ppu;
	
	xyz1 = 0;  //flag for Indexfind function: 
	xyz2 = 1;
	_j_period = 0; // the number for step cycle indefind
	
        //footz refer: 
	_Zsc.setZero();		
  	for (int i = 0; i < _nsum-1; i++) {	  
          Indexfind(_t_whole(i),xyz1);	  
	  _Zsc(i) = _footz_ref(_j_period);   
	  _j_period = 0; 
	}		

    _yk.topRows(1).setConstant(_footy_ref(0)); 
	_zk.topRows(1).setConstant(_hcom);
		
	/// for footstep location reference generation: the foot-ref = _v_i*fx + _VV_i*_Lx_ref	
	_v_i.setZero();                    ///current step cycle
	_VV_i.setZero();	      /// next step cycles: 2 cycles maximal
	
	// optimized footstep location
	_footx_real.setZero();  
	_footy_real.setZero(); 
	_footz_real.setZero();	
	_footxyz_real.setZero();	
	_footx_real_next.setZero(); 
	_footy_real_next.setZero(); 
	_footz_real_next.setZero();
	_footx_real_next1.setZero();  
	_footy_real_next1.setZero(); 
	_footz_real_next1.setZero();
	
	/// for foot trajectory generation

    _Lfootx.setZero();
    _Lfooty.setZero(); _Lfooty.setConstant(_stepwidth(0)); _Lfootz.setZero(); 
	_Lfootvx.setZero(); _Lfootvy.setZero();_Lfootvz.setZero(); 
	_Lfootax.setZero(); _Lfootay.setZero();_Lfootaz.setZero();
	_Rfootx.setZero(); 
    _Rfooty.setZero(); _Rfooty.setConstant(-_stepwidth(0));_Rfootz.setZero(); 
	_Rfootvx.setZero(); _Rfootvy.setZero();_Rfootvz.setZero(); 
	_Rfootax.setZero(); _Rfootay.setZero();_Rfootaz.setZero();	
	_ry_left_right = 0;
/////=========================================constraints initialize========================================
	_ZMP_ratio = 0.8;	
	  //vertical height constraints	
	_z_max=0.1;
	_z_min=-0.1;	
	
	if(_robot_name == "coman"){
	  _rad = 0.1; 	  
	  _footx_max=0.3;
	  _footx_min=-0.2;	  	  
	  /// zmp-constraints	
	  _zmpx_ub=0.07;  
	  _zmpx_lb=-0.03;
	  _zmpy_ub=0.05; 
	  _zmpy_lb=-0.05;		  	  
	}
	else if (_robot_name == "Talos")
	{
	  _rad = 0.2; 	  
	  _footx_max=0.4;
	  _footx_min=-0.2;	  	  
	  /// zmp-constraints	
	  _zmpx_ub=(_RobotPara_FOOT_LENGTH/2+_RobotPara_HIP_TO_ANKLE_X_OFFSET)*_ZMP_ratio;  
	  _zmpx_lb=(-(_RobotPara_FOOT_LENGTH/2-_RobotPara_HIP_TO_ANKLE_X_OFFSET)*_ZMP_ratio);
	  _zmpy_ub=(_RobotPara_FOOT_WIDTH/2*_ZMP_ratio); 
	  _zmpy_lb=(-_RobotPara_FOOT_WIDTH/2*_ZMP_ratio);		
	}
	else if (_robot_name == "cogimon")
        {	  
	  _rad = 0.2; 	  
	  _footx_max=0.4;
	  _footx_min=-0.2;	  	  
	  /// zmp-constraints	
	  _zmpx_ub=(_RobotPara_FOOT_LENGTH/2+_RobotPara_HIP_TO_ANKLE_X_OFFSET)*_ZMP_ratio;  
	  _zmpx_lb=(-(_RobotPara_FOOT_LENGTH/2-_RobotPara_HIP_TO_ANKLE_X_OFFSET)*_ZMP_ratio);
	  _zmpy_ub=(_RobotPara_FOOT_WIDTH/2*_ZMP_ratio); 
	  _zmpy_lb=(-_RobotPara_FOOT_WIDTH/2*_ZMP_ratio);	
        } 
        /*else {
	  DPRINTF("Errorrrrrrrr for IK\n");}*/		
	  
	_mass = _robot_mass; 		
	_j_ini = _mass* pow(_rad,2);		

	
	_footy_max=2*_RobotParaClass_HALF_HIP_WIDTH + 0.2; 
	_footy_min=_RobotParaClass_HALF_HIP_WIDTH - 0.03;
	
	// angle range
	_thetax_max=10*M_PI/180;  
	_thetax_min=-5*M_PI/180;
	_thetay_max=10*M_PI/180;  
	_thetay_min=-10*M_PI/180;
	
	// torque range
	_torquex_max=80/_j_ini; 
	_torquex_min=-60/_j_ini;
	_torquey_max=80/_j_ini;  
	_torquey_min=-80/_j_ini;	

	// swing foot velocity constraints	
	_footx_vmax=3;
	_footx_vmin=-2;
	_footy_vmax=2; 
	_footy_vmin=-2;	

	_fx= 0;    //current step location in local coordinate
	_fy= 0;
	_fxx_global= 0; //global footstep location in local coordinate
	_fyy_global= 0;		

///===========initiallize: preparation for MPC solution	
	// sulotion preparation	

	_V_ini.setZero();                                /// initialize optimal variable
    _V_inix.setZero();	
// 	_V_optimal.setZero();	      ///optimization det_V
	
	_flag.setZero();	        // 	 store n_vis: actual following step numbers under predictive window
	_flag_global.setZero();    // store step cycle sequence	
	
	_Lx_ref.setZero();             ///reference following footstep locations during the predictive window
	_Ly_ref.setZero(); 
	_Lz_ref.setZero();                    
	_comx_center_ref.setZero();
	_comy_center_ref.setZero();
	_comz_center_ref.setZero();	
	_thetax_center_ref.setZero(); 
	_thetay_center_ref.setZero();	
		  
	if(_robot_name == "coman"){
	/// push recovery when sepping in stepstones 
/*	 _Rx = 10;            _Ry = 10;              _Rz =100;                 _Rthetax = 10;       _Rthetay = 10;          //acceleration
	_alphax = 100;        _alphay = 10;          _alphaz = 1000; 	       _alphathetax =100;   _alphathetay = 100;          //velocity
	_beltax = 10000;      _beltay = 10000;        _beltaz = 100000000;      _beltathetax = 5000000;      _beltathetay = 5000000;         //position
	_gamax =  55000000;  _gamay = 200000000;     _gamaz = 200;                                                        //footstep location                                     
        _gama_zmpx1 =  1000000000; _gama_zmpx2 = 1000000000;  _gama_zmpy1 =  1000000000; _gama_zmpy2 = 1000000000;     //ZMP constraint relaxation;*/	  
	 _Rx = 10;            _Ry = 10;              _Rz =100;                 _Rthetax = 10;       _Rthetay = 10;          //acceleration
	_alphax = 100;        _alphay = 10;          _alphaz = 1000; 	       _alphathetax =100;   _alphathetay = 100;          //velocity
	_beltax = 10000;      _beltay = 10000;        _beltaz = 100000000;      _beltathetax = 5000000;      _beltathetay = 5000000;         //position
	_gamax =  55000000;  _gamay = 200000000;     _gamaz = 200;                                                        //footstep location                                     
    _gama_zmpx1 =  1000000000; _gama_zmpx2 = 1000000000;  _gama_zmpy1 =  1000000000; _gama_zmpy2 = 1000000000;     //ZMP constraint relaxation;	  
	  	  
	}
	else if(_robot_name  == "Talos"){
         //////// for methx ==2:reactive step + body inclination + height variance: for flat ground walking and up-down stairs: offline
	 _Rx = 10;           _Ry = 10;            _Rz =10;         	   _Rthetax = 1;      _Rthetay = 1;          //acceleration
	_alphax = 10;       _alphay = 100;        _alphaz = 100; 	   _alphathetax =1;   _alphathetay = 1;          //velocity
	_beltax = 5000;     _beltay = 1000;        _beltaz = 20000000;     _beltathetax = 1000; _beltathetay = 1000;         //position
	_gamax =  10000000; _gamay = 10000000;  _gamaz = 200;                                                        //footstep location
                                       
    _gama_zmpx1 =  1000000000; _gama_zmpx2 = 1000000000;  _gama_zmpy1 =  1000000000; _gama_zmpy2 = 1000000000;     //ZMP constraint relaxation;
	}
	else if (_robot_name == "cogimon"){
	  //////// for methx ==2:reactive step + body inclination + height variance: for flat ground walking and up-down stairs: offline
	 _Rx = 1;           _Ry = 1;            _Rz =1;         	   _Rthetax = 1;      _Rthetay = 1;          //acceleration
	_alphax = 1;       _alphay = 1;        _alphaz = 100; 	       _alphathetax =1;   _alphathetay = 1;          //velocity
	_beltax = 5000;   _beltay = 10;        _beltaz = 20000000;     _beltathetax = 10; _beltathetay = 10;         //position
	_gamax =  10000000; _gamay = 10000000;  _gamaz = 200;                                                        //footstep location
                                       
        _gama_zmpx1 =  1000000000; _gama_zmpx2 = 1000000000;  _gama_zmpy1 =  1000000000; _gama_zmpy2 = 1000000000;     //ZMP constraint relaxation;  
        } 
// 	else
// 	{DPRINTF("Errorrrrrrrr for IK\n");}
	///////////////////=====================================//////////////////////////
	/// for offline calculation	
	_loop = 2;   /// loop number for SQP
///// next code just run once	
	A_unit.setIdentity(_nh,_nh);
	C_unit.setIdentity(_nstep,_nstep);		
	
  /////////// initialize each variable
	_bjxx = 0; 
	_t_f.setZero();     ///predictive window time-period
	_bjx1 = 0;
	_bjx2 = 0;
    _mx = 0;
    _tnx.setZero();	  
	  
	_n_vis =0; 
	xxx = 0; 
	xxx1=0; 
	xxx2=0;	 
	
	// optimization objective function 	
	_WX.setZero();
	_WY.setZero();
	_WZ.setZero();
	_WthetaX.setZero();
	_WthetaY.setZero();
	_PHIX.setZero();
	_PHIY.setZero();
	_PHIZ.setZero();
	_Q_goal.setZero();
	_q_goal.setZero();
	_Q_goal1.setZero();
	_q_goal1.setZero();	
	
	_WX = _Rx*0.5 * A_unit + _alphax*0.5 * _pvu_2 + _beltax*0.5 * _ppu_2;	  
	_WY = _Ry/2 * A_unit + _alphay/2 * _pvu_2 + _beltay/2 * _ppu_2;
	_WZ = _Rz/2 * A_unit + _alphaz/2 * _pvu_2 + _beltaz/2 * _ppu_2;  
	_WthetaX = _Rthetax/2 * A_unit + _alphathetax/2 * _pvu_2 + _beltathetax/2 * _ppu_2;
	_WthetaY = _Rthetay/2 * A_unit + _alphathetay/2 * _pvu_2 + _beltathetay/2 * _ppu_2;
	_PHIX  = _gamax/2 * C_unit;
	_PHIY  = _gamay/2 * C_unit;
	_PHIZ  = _gamaz/2 * C_unit;
	_PHIzmpx1(0,0)  = _gama_zmpx1/2;
	_PHIzmpx2(0,0)  = _gama_zmpx2/2;
	_PHIzmpy1(0,0)  = _gama_zmpy1/2;
	_PHIzmpy2(0,0)  = _gama_zmpy2/2;	
	
	_Q_goal.block<_nh, _nh>(0, 0) = _WX;
	_Q_goal.block<_nh, _nh>(_nh, _nh) = _WY;
	_Q_goal.block<_nh, _nh>(2*_nh, 2*_nh) = _WZ;
	_Q_goal.block<_nh, _nh>(3*_nh, 3*_nh) = _WthetaX;
	_Q_goal.block<_nh, _nh>(4*_nh, 4*_nh) = _WthetaY;
	_Q_goal.block<_nstep,_nstep>(5*_nh, 5*_nh) = _PHIX;
	_Q_goal.block<_nstep,_nstep>(5*_nh+_nstep, 5*_nh+_nstep) = _PHIY;
	_Q_goal.block<_nstep,_nstep>(5*_nh+2*_nstep, 5*_nh+2*_nstep) = _PHIZ;
	_Q_goal(5*_nh+3*_nstep, 5*_nh+3*_nstep) = _PHIzmpx1(0,0);
	_Q_goal(5*_nh+3*_nstep+1, 5*_nh+3*_nstep+1) = _PHIzmpx2(0,0);
	_Q_goal(5*_nh+3*_nstep+2, 5*_nh+3*_nstep+2) = _PHIzmpy1(0,0);	
	_Q_goal(5*_nh+3*_nstep+3, 5*_nh+3*_nstep+3) = _PHIzmpy2(0,0);	
      
	_Q_goal1 = 2 * _Q_goal;	
      
      
	// constraints
	_Sjx.setZero();
	_Sjy.setZero();
	_Sjz.setZero();
	_Sjthetax.setZero();
	_Sjthetay.setZero();
	_Sjzmpx1.setZero();
	_Sjzmpx2.setZero();
	_Sjzmpy1.setZero();
	_Sjzmpy2.setZero();
	
	_Sjx.block<_nh, _nh>(0, 0) = A_unit;
	_Sjy.block<_nh, _nh>(0, _nh) = A_unit;
	_Sjz.block<_nh, _nh>(0, 2*_nh) = A_unit;	
	_Sjthetax.block<_nh, _nh>(0, 3*_nh) = A_unit;
	_Sjthetay.block<_nh, _nh>(0, 4*_nh) = A_unit;
	_Sjzmpx1(0, 5*_nh+3*_nstep) = 1;
	_Sjzmpx2(0, 5*_nh+3*_nstep+1) = 1;	
	_Sjzmpy1(0, 5*_nh+3*_nstep+2) = 1;
	_Sjzmpy2(0, 5*_nh+3*_nstep+3) = 1;	
	
	
	
	_Sfx.setZero();
	_Sfy.setZero();
	_Sfz.setZero();	 	
	

	// ZMP boundary preparation
	_H_q_upx.setZero();
	_F_zmp_upx.setZero();
	_H_q_lowx.setZero();
	_F_zmp_lowx.setZero();
	_H_q_upy.setZero();
	_F_zmp_upy.setZero();
	_H_q_lowy.setZero();
	_F_zmp_lowy.setZero();

	_phi_i_x_up.setZero();
	_p_i_x_t_up.setZero();
	_del_i_x_up.setZero();
	_phi_i_x_low.setZero();
	_p_i_x_t_low.setZero();
	_del_i_x_low.setZero();
	_phi_i_y_up.setZero();
	_p_i_y_t_up.setZero();
	_del_i_y_up.setZero();
	_phi_i_y_low.setZero();
	_p_i_y_t_low.setZero();
	_del_i_y_low.setZero();	  

	// angle boundary preparation
	_q_upx.setZero();
	_qq_upx.setZero();
	_q_lowx.setZero();
	_qq_lowx.setZero();
	_q_upy.setZero();
	_qq_upy.setZero();
	_q_lowy.setZero();
	_qq_lowy.setZero();

	_qq1_upx.setZero();
	_qq1_lowx.setZero();
	_qq1_upy.setZero();
	_qq1_lowy.setZero();	  

	// torque bondary preparation
	_t_upx.setZero();
	_tt_upx.setZero();
	_t_lowx.setZero();
	_tt_lowx.setZero();
	_t_upy.setZero();
	_tt_upy.setZero();
	_t_lowy.setZero();
	_tt_lowy.setZero();

	_tt1_upx.setZero();
	_tt1_lowx.setZero();
	_tt1_upy.setZero();
	_tt1_lowy.setZero();

	// CoM height boundary preparation
	_H_h_upz.setZero();
	_F_h_upz.setZero();
	_H_h_lowz.setZero();
	_F_h_lowz.setZero();
	_delta_footz_up.setZero();
	_delta_footz_low.setZero();

	// CoM height acceleration boundary preparation
	_H_hacc_lowz.setZero();
	_F_hacc_lowz.setZero();
	_delta_footzacc_up.setZero();	  


	//swing foot velocity constraints
	_Footvx_max.setZero();
	_Footvx_min.setZero();
	_Footvy_max.setZero();
	_Footvy_min.setZero();
	_footubxv.setZero();
	_footlbxv.setZero();
	_footubyv.setZero();
	_footlbyv.setZero();
	
	// foot location constraints: be careful that the step number is change: so should be intialized in each whole loop
	_H_q_footx_up.setZero();
	_F_foot_upx.setZero();
	_H_q_footx_low.setZero();
	_F_foot_lowx.setZero();
	_H_q_footy_up.setZero();
	_F_foot_upy.setZero();
	_H_q_footy_low.setZero();
	_F_foot_lowy.setZero();	
	
	

	// foot vertical location-equality constraints
	_H_q_footz.setZero(1, _Nt);
	_F_footz.setZero(1, 1);

	// CoMZ height-equality constraints
	_h_h.setZero();
	_hhhx.setZero();	  

	// body inclination-equality constraints
	_a_hx.setZero();
	_a_hxx.setZero();
	_a_hy.setZero();
	_a_hyy.setZero();


	// foot location constraints
	_Sfoot.setZero();
	_Sfoot(0) = -1;
	_Sfoot(1) = 1;
	  
	// offline calulated the ZMP constraints coefficient==================================
	  //////////initiallize vector
	vector <Eigen::Matrix<double,_Nt, _Nt>> x_offline1(_nh)  ;
	for (int j=0;j<_nh; j++)
	{
	  x_offline1[j]= Eigen::Matrix<double,_Nt, _Nt>::Zero();
	}

	ZMPx_constraints_offfline = x_offline1;
	ZMPy_constraints_offfline = x_offline1;		
	_phi_i_x_up_est = x_offline1;
	_phi_i_y_up_est = x_offline1;
        _phi_i_x_low_est = x_offline1;
	_phi_i_y_low_est = x_offline1;	
	
	vector <Eigen::Matrix<double,_nh, _Nt>> x_offline2(_nh)  ;
	for (int j=0;j<_nh; j++)
	{
	  x_offline2[j]= Eigen::Matrix<double,_nh, _Nt>::Zero();
	}	

	ZMPx_constraints_half = x_offline2;	
	ZMPy_constraints_half = x_offline2;
		
	for(int jxx=1; jxx<=_nh; jxx++)
	{
	  _Si.setZero();
	  _Si(0,jxx-1) = 1;
	  // ZMP constraints	      		 
         ZMPx_constraints_offfline[jxx-1] = -(_Sjzmpx1).transpose() * _Si * _Sjz;
	 ZMPx_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _Sjz;
	  	  
         ZMPy_constraints_offfline[jxx-1] = -(_Sjzmpy1).transpose() * _Si * _Sjz;
	 ZMPy_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _Sjz;
	      
	}
        

    _Footx_global_relative =0;
    _Footy_global_relative =0;	

	// boundary initialzation	  
	_Si.setZero();	  

	_ZMPx_constraints_half2.setZero();	 
	_ZMPy_constraints_half2.setZero();
	_phi_i_x_up1.setZero();
	_phi_i_y_up1.setZero();	
	_phi_i_x_low1.setZero(); 
	_phi_i_y_low1.setZero();
	
	CoMMM_ZMP_foot.setZero();
	
	///QP initiallize
	if (_method_flag ==0)
	{
	  
	  int nVars = _Nt;
	  int nEqCon = 1+3*_nh;
	  int nIneqCon = 5*_nh + 4*_nstep+4;  
	  resizeQP(nVars, nEqCon, nIneqCon);		   
        }
	else if (_method_flag ==1)
	{
	  int nVars = _Nt;
	  int nEqCon = 1+_nh;
	  int nIneqCon = 13*_nh + 4*_nstep +4;
	  resizeQP(nVars, nEqCon, nIneqCon);	   
	}
	else
	{
	  int nVars = _Nt;
	  int nEqCon = 1;
	  int nIneqCon = 15*_nh + 4*_nstep +4;
	  resizeQP(nVars, nEqCon, nIneqCon);		   
	}

	_t_end_walking = _tx(_footstepsnumber-1)- 9*_tstep/4;  // ending time when normal walking ends
	_n_end_walking = round(_t_end_walking/_dt);	
	_comy_matrix_inv.setZero();
	
	
	
	
	//////polynomial intepolation for lower level interpolation
	_AAA_inv.setZero();

	_j_count = 0;
	
	/////////////for ZMP distribution
	_F_R(2) = _F_L(2) = 0.5 * _mass * _RobotPara_G;
	_Co_L.setZero(); _Co_R.setZero();
	
	_comxyzx.setZero(); _comvxyzx.setZero(); _comaxyzx.setZero(); 
	_thetaxyx.setZero(); _thetavxyx.setZero(); _thetaaxyx.setZero();
	_Lfootxyzx.setZero(); _Rfootxyzx.setZero();
	_ZMPxy_realx.setZero();
	
	_zmp_rela_vari.setZero();
	
	_comxyzx(2) = _RobotPara_Z_C;	
	
	/// mpc state reservation:
    _mpc_starowNum = 7;
    _mpc_sta_colNum = _footstepsnumber;
    if (_mpc_sta_colNum<=18)
    {
       _mpc_sta_colNum=18; 
    }
    _mpccontrol_state.setZero(_mpc_starowNum,_mpc_sta_colNum);	
    
    file_mpc = "/home/jiatao/Dropbox/nmpc_pybullet/mpccontr_states.txt";
	
    _mpc_starowNum1 = 13;
    _mpccontrol_com.setZero(_mpc_starowNum1,_nsum);
    
    file_mpcx = "/home/jiatao/Dropbox/nmpc_pybullet/mpccontr_comzmp.txt";
    
    _mpc_starowNum2 = 1;
    _mpccontrol_com1.setZero(_mpc_starowNum2,18);
    file_mpcy = "/home/jiatao/Dropbox/nmpc_pybullet/mpccontr_kmp.txt";
    
    mpc_loop_flag = 0;
	
}

/////////////////////////////////////////////////////////////============================================================================================
/////////////////////// local coordinate CoM solution---modified---------------------------------
void MPCClass::CoM_foot_trajection_generation_local(int i, Eigen::Matrix<double,18,1> estimated_state, Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,double lamda, bool _stopwalking)
{
  _j_count = i;
  
  if (i>1)
  {
     MatrixXd mpc_state = rea_save(file_mpc,_mpc_starowNum); 
    _footx_real = mpc_state.block(0,0,1,_footstepsnumber).transpose();
    _footy_real = mpc_state.block(1,0,1,_footstepsnumber).transpose();
    _footz_real = mpc_state.block(2,0,1,_footstepsnumber).transpose();
    _lift_height_ref = mpc_state.block(3,0,1,_footstepsnumber).transpose();
    _xk.col(i-1) = mpc_state.block(4,0,1,2).transpose();
    _yk.col(i-1) = mpc_state.block(4,2,1,2).transpose();
    _zk.col(i-1) = mpc_state.block(4,4,1,2).transpose();
    _thetaxk.col(i-1) = mpc_state.block(4,6,1,2).transpose();
    _thetayk.col(i-1) = mpc_state.block(4,8,1,2).transpose(); 
    _fxx_global = mpc_state(4,10); 
    _fyy_global = mpc_state(4,11);
    _footx_real_next(i+_nT-2,0) = mpc_state(4,12);
    _footy_real_next(i+_nT-2,0) = mpc_state(4,13);
    
    _Lfootx(0,i-1) = mpc_state(5,0);  _Lfootvx(0,i-1) = mpc_state(5,1);  _Lfootax(0,i-1) = mpc_state(5,2); 
    _Lfooty(0,i-1) = mpc_state(5,3);  _Lfootvy(0,i-1) = mpc_state(5,4);  _Lfootay(0,i-1) = mpc_state(5,5);  
    _Lfootz(0,i-1) = mpc_state(5,6);  _Lfootvz(0,i-1) = mpc_state(5,7);  _Lfootaz(0,i-1) = mpc_state(5,8); 
    _Rfootx(0,i-1) = mpc_state(5,9);  _Rfootvx(0,i-1) = mpc_state(5,10);  _Rfootax(0,i-1)= mpc_state(5,11); 
    _Rfooty(0,i-1) = mpc_state(5,12); _Rfootvy(0,i-1) = mpc_state(5,13);  _Rfootay(0,i-1)= mpc_state(5,14);  
    _Rfootz(0,i-1) = mpc_state(5,15); _Rfootvz(0,i-1) = mpc_state(5,16);  _Rfootaz(0,i-1)= mpc_state(5,17);  
    
    _comx(0,i-1) = mpc_state(6,0);    _comvx(0,i-1) = mpc_state(6,1);    _comax(0,i-1) =mpc_state(6,2); 
    _comy(0,i-1) = mpc_state(6,3);    _comvy(0,i-1) = mpc_state(6,4);    _comay(0,i-1) =mpc_state(6,5);  
    _comz(0,i-1) = mpc_state(6,6);    _comvz(0,i-1) = mpc_state(6,7);    _comaz(0,i-1) = mpc_state(6,8); 
    _thetax(0,i-1) = mpc_state(6,9);  _thetavx(0,i-1) = mpc_state(6,10);  _thetaax(0,i-1) = mpc_state(6,11); 
    _thetay(0,i-1) = mpc_state(6,12); _thetavy(0,i-1) =  mpc_state(6,13); _thetaay(0,i-1) = mpc_state(6,14); 
    
  

    MatrixXd mpc_com = rea_save(file_mpcx,_mpc_starowNum1);
    _comx.block(0,0,1,i-1) = mpc_com.block(0,0,1,i-1);
    _comy.block(0,0,1,i-1) = mpc_com.block(1,0,1,i-1);
    _comz.block(0,0,1,i-1) = mpc_com.block(2,0,1,i-1);
    _thetax.block(0,0,1,i-1) = mpc_com.block(3,0,1,i-1);
    _thetay.block(0,0,1,i-1) = mpc_com.block(4,0,1,i-1);
    _zmpx_real.block(0,0,1,i-1) = mpc_com.block(5,0,1,i-1);
    _zmpy_real.block(0,0,1,i-1) = mpc_com.block(6,0,1,i-1);
    _Lfootx.block(0,0,1,i-1) = mpc_com.block(7,0,1,i-1);
    _Lfooty.block(0,0,1,i-1) = mpc_com.block(8,0,1,i-1);
    _Lfootz.block(0,0,1,i-1) = mpc_com.block(9,0,1,i-1); 
    _Rfootx.block(0,0,1,i-1) = mpc_com.block(10,0,1,i-1);
    _Rfooty.block(0,0,1,i-1) = mpc_com.block(11,0,1,i-1);
    _Rfootz.block(0,0,1,i-1) = mpc_com.block(12,0,1,i-1);
  }
  
  
  if (i<_n_end_walking)   ///////normal walking
  {
    /// modified the footy_min
    if (i==(round(2*_ts(1)/_dt))+1) ///update the footy_limit
    {
	  _footy_min = _RobotPara_FOOT_WIDTH+0.01;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
//	================ iterative calulcation: predictive control_tracking with time-varying height+angular momentum	  	  	    
    // run once
    /////////////////////////////////////////////////////	        
      
    Indexfind(i*_dt,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
    _bjxx = _j_period+1;  //coincidence with matlab 
    _j_period = 0;	  
    
    // com_center_ref = ZMP_center_ref = v_i*f + V_i*L_ref
    //solve the following steps: 1.5s may couve 2 or three  steps, so  one/two following steps	    
    _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
    
    Indexfind(_t_f(0),xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
    _bjx1 = _j_period+1;
    _j_period = 0;
    
    Indexfind(_t_f(_nh-1),xyz1);           /// step cycle number when (i+_nh)*dt fall into : current sampling time
    _bjx2 = _j_period+1;
    _j_period = 0;	  
	    
    ////================================================================
    /// judge if stop walking enable: if (from _bjx1 step, reference step length and step height will be set to be zero)
    if(_stopwalking)  
    {    
      for (int i_t = _bjx1; i_t < _footstepsnumber; i_t++) {
	_steplength(i_t) = 0;	
	_footx_ref(i_t) = _footx_ref(i_t-1) + _steplength(i_t-1); 	      
      }	  
    }
    
    _mx = _bjx2 - _bjx1 +1;
    /// find out the relative postion of step cycle switching time to the predictive window 	    
    for (int j=1;j<_mx; j++)
    {
      Indexfind(_tx(_bjx1+j-1),xyz2);
      _tnx(j-1) = _j_period; 
      _j_period = 0;	      
    }

    _v_i.setZero();
    _VV_i.setZero();
    xxx = _tnx(0);
    // be careful that the position is from 0;;;;;         
    if (fabs(_tnx(0) - _nT) <=0.00001)
    {
      _n_vis =2;
      for (int jjj = 1; jjj <= _mx; jjj++)
      {
	
	if (jjj == 1)
	{
	  _VV_i.block(0, 0, xxx, 1).setOnes();
	}
	else
	{	
	  xxx1 = _nh-_tnx(0);
	  _VV_i.block(xxx, 1, xxx1, 1).setOnes();		
	}
      }	    
    }
    else
    {	
      _v_i.segment(0, xxx).setOnes();	      
      if (abs(_mx - 2) <=0.00001)
      {
        _n_vis = 1;
        _VV_i.block(xxx, 0, _nh -xxx, 1).setOnes();
      }
      else
      {
        _n_vis = 2;
        xxx2 = _nh -_tnx(1);
//         _VV_i.block<_nT, 1>(xxx, 0).setOnes();
        _VV_i.block(xxx, 0,_nT, 1).setOnes();        
        _VV_i.block(_tnx(1), 1, xxx2, 1).setOnes();	      	      
      }
    }
    

    _flag(i-1,0)= _n_vis;	    
    _flag_global(i-1,0) = _bjxx;
    _flag_global(i,0) = _bjx1;	  

    
    ////////////////////////////////////////////// check /////////////////////////////////////////////////
    //////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////	  
//============================================================//	  
//////////////////// relative state: 	  
    ///// pass the actual stage into the control loop	  	  	      
    if (i>1)
    {
      _Footx_global_relative = _xk(0,i-1) + _fxx_global;  //// read _xk,_yk,_zk,_thetaxk,_thetayk,_fxx_global,_fyy_global,_footx_real,_footy_real,_footz_real
      _Footy_global_relative = _yk(0,i-1) + _fyy_global;	    
    }
    // current foot location
    _fx =0;
    _fy = 0;	  	    
    _fxx_global = _footx_real(_bjxx-1);
    _fyy_global = _footy_real(_bjxx-1);	    

    if (_n_vis ==1)
    {
      _Lx_ref(0) = _footx_ref(_bjx2-1) - _fxx_global;
      _Ly_ref(0) = _footy_ref(_bjx2-1) - _fyy_global;
      _Lz_ref(0) = _footz_ref(_bjx2-1);
      _Lx_ref(1) = 0;
      _Ly_ref(1) = 0;
      _Lz_ref(1) = 0;	    
    }
    else
    {
      _Lx_ref(0) = _footx_ref(_bjx2-2) - _fxx_global;
      _Ly_ref(0) = _footy_ref(_bjx2-2) - _fyy_global;
      _Lz_ref(0) = _footz_ref(_bjx2-2);
      _Lx_ref(1) = _footx_ref(_bjx2-1) - _fxx_global;
      _Ly_ref(1) = _footy_ref(_bjx2-1) - _fyy_global;
      _Lz_ref(1) = _footz_ref(_bjx2-1);	    
    }	    
  // com_center_ref
    _comx_center_ref = _v_i*_fx + _VV_i*_Lx_ref;
    _comy_center_ref = _v_i*_fy + _VV_i*_Ly_ref;
    _comz_center_ref = _Zsc.segment<_nh>(i) + _Hcom;
    
    ///////////////////////////////////////////////////////////////////////////////
    ////limitation test
    
    //// height variance & pitch 
//     Eigen::MatrixXd heig_vari;
//     heig_vari.setConstant(_nh,1,0.04);
//     
//     _thetay_center_ref.setZero(_nh,1);
//     
//     if ((i>36)&&(i<71))
//     {
//       _comz_center_ref = _Zsc.block< Dynamic,1>(i, 0,_nh, 1) + _Hcom - heig_vari;
//       _thetay_center_ref.setConstant(_nh,1,0.1);
//     }   
//     
    
/*    //// height reduction without pitch: z1 = 1.1
    Eigen::MatrixXd heig_vari;
    heig_vari.setConstant(_nh,1,0.1);
    
    _thetay_center_ref.setZero(_nh,1);
    
    if ((i>36)&&(i<71))
    {
      _comz_center_ref = _Zsc.block< Dynamic,1>(i, 0,_nh, 1) + _Hcom - heig_vari;
//       _thetay_center_ref.setConstant(_nh,1,0.15);
      _thetay_center_ref.setConstant(_nh,1,0.0);
    }*/      
    
     
// relative state switch	      
    if (i>1)
    {
      if (_flag_global(i-2,0) < _flag_global(i-1,0) )
      {
        /// reference relative state switch     
        _xk(0,i-1) = _Footx_global_relative - _fxx_global; 
        _yk(0,i-1) = _Footy_global_relative - _fyy_global;	
      }
    }
    //////////////////////////////////////////////////////////////////////////============================================================//////////////////////////////////////////
    // foot location constraints: be careful that the step number is change: so should be intialized in each whole loop
    _H_q_footx_up.setZero();
    _F_foot_upx.setZero();
    _H_q_footx_low.setZero();
    _F_foot_lowx.setZero();
    _H_q_footy_up.setZero();
    _F_foot_upy.setZero();
    _H_q_footy_low.setZero();
    _F_foot_lowy.setZero();
    

    // boundary initialzation
    _Sfx.setZero();
    _Sfy.setZero();
    _Sfz.setZero();
    
    if (_n_vis ==1)
    {
      _Sfx(0,5*_nh) = 1;
      _Sfy(0,5*_nh+_nstep) = 1;
      _Sfz(0,5*_nh+2*_nstep) = 1;
    }  
    else
    {
      _Sfx(0,5*_nh) = 1;
      _Sfx(1,5*_nh+1) = 1;
      _Sfy(0,5*_nh+_nstep) = 1;
      _Sfy(1,5*_nh+_nstep+1) = 1;
      _Sfz(0,5*_nh+2*_nstep) = 1;	 
      _Sfz(1,5*_nh+2*_nstep+1) = 1;	
      
    }
	

  //SQP MOdels	 
    _q_goal.block<_nh, 1>(0, 0) = _alphax * _pvu.transpose() * _pvs * _xk.col(i-1) + _beltax * _ppu.transpose() * _pps * _xk.col(i-1) - _beltax * _ppu.transpose() * _comx_center_ref;
    _q_goal.block<_nh, 1>(_nh, 0) = _alphay * _pvu.transpose() * _pvs * _yk.col(i-1) + _beltay * _ppu.transpose() * _pps * _yk.col(i-1) - _beltay * _ppu.transpose() * _comy_center_ref;
    _q_goal.block<_nh, 1>(2*_nh, 0) = _alphaz * _pvu.transpose() * _pvs * _zk.col(i-1) + _beltaz * _ppu.transpose() * _pps * _zk.col(i-1) - _beltaz * _ppu.transpose() * _comz_center_ref;
    _q_goal.block<_nh, 1>(3*_nh, 0) = _alphathetax * _pvu.transpose() * _pvs * _thetaxk.col(i-1) + _beltathetax * _ppu.transpose() * _pps * _thetaxk.col(i-1) - _beltathetax * _ppu.transpose() * _thetax_center_ref;
    _q_goal.block<_nh, 1>(4*_nh, 0) = _alphathetay * _pvu.transpose() * _pvs * _thetayk.col(i-1) + _beltathetay * _ppu.transpose() * _pps * _thetayk.col(i-1) - _beltathetay * _ppu.transpose() * _thetay_center_ref;
    _q_goal.block<_nstep, 1>(5*_nh, 0) = -_gamax * _Lx_ref;
    _q_goal.block<_nstep, 1>(5*_nh+_nstep, 0) = -_gamay * _Ly_ref;
    _q_goal.block<_nstep, 1>(5*_nh+2*_nstep, 0) = -_gamaz * _Lz_ref;
    
  ///// the following code only run once in each loop   
    for(int jxx=1; jxx<=_nh; jxx++)
    {
      _Si.setZero();
      _Si(0,jxx-1) = 1;
      
      // ZMP constraints
      // x-ZMP upper boundary                                      
      _p_i_x_t_up.col(jxx-1) = _mass * (((_Si * _pps * _xk.col(i-1)).transpose() *_Si*_Sjz + _ggg*_Si*_ppu*_Sjx - (_Si * _pps * _zk.col(i-1)).transpose() *_Si*_Sjx + _Zsc.row(i+jxx-1)*_Si*_Sjx - (_Si * _v_i * _fx).transpose() *_Si*_Sjz - _ggg*_Si*_VV_i*_Sfx - _zmpx_ub*_Si*_Sjz - _ggg*_Sjzmpx1).transpose()) - (_j_ini * _Si * _Sjthetay).transpose();		

      _del_i_x_up.col(jxx-1) = _mass * ( _ggg*_Si * _pps * _xk.col(i-1)  - _ggg *_Si * _v_i * _fx - _ggg *_zmpx_ub);


      // x-ZMP low boundary
      _p_i_x_t_low.col(jxx-1) = (_p_i_x_t_up.col(jxx-1).transpose() + _mass * (_zmpx_ub - _zmpx_lb)*_Si*_Sjz + _mass * _ggg*(_Sjzmpx1 - _Sjzmpx2)).transpose();	      
      _del_i_x_low.col(jxx-1) = _del_i_x_up.col(jxx-1) +  _mass * _ggg*(_zmpx_ub -_zmpx_lb);
      
      // y-ZMP upper boundary
      _p_i_y_t_up.col(jxx-1) = _mass * (((_Si * _pps * _yk.col(i-1)).transpose() *_Si*_Sjz + _ggg*_Si*_ppu*_Sjy - (_Si * _pps * _zk.col(i-1)).transpose() *_Si*_Sjy + _Zsc.row(i+jxx-1)*_Si*_Sjy - (_Si * _v_i * _fy).transpose() *_Si*_Sjz - _ggg*_Si*_VV_i*_Sfy - _zmpy_ub*_Si*_Sjz - _ggg*_Sjzmpy1).transpose()) + (_j_ini * _Si * _Sjthetax).transpose();
      _del_i_y_up.col(jxx-1) = _mass * ( _ggg*_Si * _pps * _yk.col(i-1)  - _ggg *_Si * _v_i * _fy - _ggg *_zmpy_ub);	      
    
      // y-ZMP low boundary
      _phi_i_y_low = _phi_i_y_up;  
      _p_i_y_t_low.col(jxx-1) = (_p_i_y_t_up.col(jxx-1).transpose() + _mass * (_zmpy_ub - _zmpy_lb)*_Si*_Sjz + _mass * _ggg*(_Sjzmpy1 - _Sjzmpy2)).transpose();	      
      _del_i_y_low.col(jxx-1) = _del_i_y_up.col(jxx-1) +  _mass * _ggg*(_zmpy_ub -_zmpy_lb);	      	      	     

      
      
      //angle range constraints
      _q_upx.row(jxx-1) = _Si* _ppu* _Sjthetax;
      _q_lowx.row(jxx-1) = -_q_upx.row(jxx-1);	         

      _qq1_upx.row(jxx-1) = _Si* _pps* _thetaxk.col(i-1);
      _qq1_upx(jxx-1,0) = _qq1_upx(jxx-1,0)-_thetax_max;
      
      _q_upy.row(jxx-1) = _Si* _ppu* _Sjthetay;
      _q_lowy.row(jxx-1) = -_q_upy.row(jxx-1);	
      
      _qq1_upy.row(jxx-1) = _Si* _pps* _thetayk.col(i-1);
      _qq1_upy(jxx-1,0) = _qq1_upy(jxx-1,0)-_thetay_max;    

      //torque range constraints
      _t_upx.row(jxx-1) = _Si* _Sjthetax;
      _t_lowx.row(jxx-1) = -_t_upx.row(jxx-1);
      
      _tt1_upx(jxx-1,0) = -_torquex_max;
      
      _t_upy.row(jxx-1) = _Si* _Sjthetay;
      _t_lowy.row(jxx-1) = -_t_upy.row(jxx-1);	 
      
      _tt1_upy(jxx-1,0) = -_torquey_max;		
      
      // body height constraints
      _H_h_upz.row(jxx-1) = _Si* _ppu* _Sjz;
      _H_h_lowz.row(jxx-1) = -_Si* _ppu* _Sjz;   	
      _delta_footz_up.row(jxx-1) = _Si*_pps*_zk.col(i-1) - _comz_center_ref.row(jxx-1) ;
      _delta_footz_up(jxx-1,0) = _delta_footz_up(jxx-1,0) - _z_max;
      
      // body height acceleration constraints	      
      _H_hacc_lowz.row(jxx-1) = -_Si* _Sjz;   
      _delta_footzacc_up.row(jxx-1) = _ggg;
    }
// 
///       only one-time caluation	  
    _ZMPx_constraints_half2 = (_VV_i* _Sfx).transpose();
    _ZMPy_constraints_half2 = (_VV_i* _Sfy).transpose();	    

    for(int jxx=1; jxx<=_nh; jxx++)
    {
      _Si.setZero();
      _Si(0,jxx-1) = 1;
      // ZMP constraints
      // x-ZMP upper boundary
      _phi_i_x_up1 = ZMPx_constraints_offfline[jxx-1] + _ZMPx_constraints_half2 * ZMPx_constraints_half[jxx-1];	      
      _phi_i_x_up_est[jxx-1] = _mass * (_phi_i_x_up1 + _phi_i_x_up1.transpose())/2;
          
      // x-ZMP lower boundary
      _phi_i_x_low1 = _phi_i_x_up1 + ((_Sjzmpx1).transpose()-(_Sjzmpx2).transpose()) * _Si * _Sjz;	      
      _phi_i_x_low_est[jxx-1] = _mass * (_phi_i_x_low1 + _phi_i_x_low1.transpose())/2;      

      // y-ZMP upper boundary
      _phi_i_y_up1 = ZMPy_constraints_offfline[jxx-1] + _ZMPy_constraints_half2 * ZMPy_constraints_half[jxx-1];
      _phi_i_y_up_est[jxx-1] = _mass * (_phi_i_y_up1 + _phi_i_y_up1.transpose())/2;   
      // y-ZMP upper boundary
      _phi_i_y_low1 = _phi_i_y_up1 + ((_Sjzmpy1).transpose()-(_Sjzmpy2).transpose()) * _Si * _Sjz;	      
      _phi_i_y_low_est[jxx-1] = _mass * (_phi_i_y_low1 + _phi_i_y_low1.transpose())/2;           
    }

    // constraints: only once 
    _Footvx_max = _Sfx.row(0);
    _Footvx_min = -_Sfx.row(0);
    _Footvy_max = _Sfy.row(0);
    _Footvy_min = -_Sfy.row(0);	  		    
    ///////////// equality equation	    
    //equality constraints
    _H_q_footz = _Sfz.row(0);	    
    _h_h = _ppu * _Sjz;
    
    _a_hx = _ppu * _Sjthetax;
    _a_hy = _ppu * _Sjthetay;
    
    // SEQUENCE QUADARTIC PROGRAMMING: lOOP_until the maximal loops reaches	

    ////////////////////////////////////////////////////SQP inner loop for fast solution///////////////////////////////////////////////////////////////////////  
    for (int xxxx=1; xxxx <= _loop; xxxx++)
    {	

      _q_goal1 = _Q_goal1 * _V_ini + _q_goal;	  
	      
        ///////////// inequality equation #  time consuming process	    
        
        for(int jxx=1; jxx<=_nh; jxx++)
        {
            // ZMP constraints
            _phi_i_x_up = _phi_i_x_up_est[jxx-1];
            _H_q_upx.row(jxx-1) = (2*_phi_i_x_up*_V_ini + _p_i_x_t_up.col(jxx-1)).transpose(); 
            _F_zmp_upx.row(jxx-1) = -((_V_ini.transpose() * _phi_i_x_up + _p_i_x_t_up.col(jxx-1).transpose()) * _V_ini + _del_i_x_up.col(jxx-1)); 

            // x-ZMP low boundary
            _phi_i_x_low = _phi_i_x_low_est[jxx-1];	      	      
            _H_q_lowx.row(jxx-1) = -(2*_phi_i_x_low*_V_ini + _p_i_x_t_low.col(jxx-1)).transpose();  
            _F_zmp_lowx.row(jxx-1) = (_V_ini.transpose() * _phi_i_x_low + _p_i_x_t_low.col(jxx-1).transpose()) * _V_ini + _del_i_x_low.col(jxx-1); 
            
            
            // y-ZMP upper boundary	      
            _phi_i_y_up = _phi_i_y_up_est[jxx-1];	      
            _H_q_upy.row(jxx-1) = (2*_phi_i_y_up*_V_ini + _p_i_y_t_up.col(jxx-1)).transpose(); 
            _F_zmp_upy.row(jxx-1) = -((_V_ini.transpose() * _phi_i_y_up + _p_i_y_t_up.col(jxx-1).transpose()) * _V_ini + _del_i_y_up.col(jxx-1)); 
            
            // y-ZMP low boundary
            _phi_i_y_low = _phi_i_y_low_est[jxx-1];	      	      
            _H_q_lowy.row(jxx-1) = -(2*_phi_i_y_low*_V_ini + _p_i_y_t_low.col(jxx-1)).transpose();  
            _F_zmp_lowy.row(jxx-1) = (_V_ini.transpose() * _phi_i_y_low + _p_i_y_t_low.col(jxx-1).transpose()) * _V_ini + _del_i_y_low.col(jxx-1); 
            
            
            //angle range constraints
            _qq_upx.row(jxx-1) = -(_q_upx.row(jxx-1)* _V_ini + _qq1_upx.row(jxx-1));
            _qq_lowx.row(jxx-1) = - _qq_upx.row(jxx-1);	 
            _qq_lowx(jxx-1,0) = _thetax_max - _thetax_min + _qq_lowx(jxx-1,0);	 
                    
            _qq_upy.row(jxx-1) = -(_q_upy.row(jxx-1)* _V_ini + _qq1_upy.row(jxx-1));
            _qq_lowy.row(jxx-1) = - _qq_upy.row(jxx-1);
            _qq_lowy(jxx-1,0) = _thetay_max - _thetay_min + _qq_lowy(jxx-1,0);

            //torque range constraints	      
            _tt_upx.row(jxx-1) = -(_t_upx.row(jxx-1)* _V_ini +  _tt1_upx.row(jxx-1));
            _tt_lowx.row(jxx-1) = - _tt_upx.row(jxx-1);	
            _tt_lowx(jxx-1,0) = _torquex_max - _torquex_min + _tt_lowx(jxx-1,0);	
            
            _tt_upy.row(jxx-1) = -(_t_upy.row(jxx-1)* _V_ini +  _tt1_upy.row(jxx-1));
            _tt_lowy.row(jxx-1) = - _tt_upy.row(jxx-1);		      
            _tt_lowy(jxx-1,0) = _torquey_max - _torquey_min + _tt_lowy(jxx-1,0);	
            
            // body height constraints	      
            _F_h_upz.row(jxx-1) = -(_H_h_upz.row(jxx-1)*_V_ini + _delta_footz_up.row(jxx-1));
            _F_h_lowz.row(jxx-1) = -_F_h_upz.row(jxx-1);	      
            _F_h_lowz(jxx-1,0) = _z_max - _z_min +_F_h_lowz(jxx-1,0);
            
            // body height acceleration constraints	      
            _F_hacc_lowz.row(jxx-1) = (-_H_hacc_lowz.row(jxx-1)*_V_ini + _delta_footzacc_up.row(jxx-1));	      	      
        }

    
        // foot location constraints
        if (_n_vis == 1)  //one next step
        {
        _H_q_footx_up.row(0) = _Sfx.row(0);
        _F_foot_upx.row(0) = -(_H_q_footx_up.row(0) * _V_ini); 
        _F_foot_upx(0,0) = _F_foot_upx(0,0) +_fx + _footx_max; 
        
        _H_q_footx_low.row(0) = -_Sfx.row(0);
        _F_foot_lowx.row(0) = (_H_q_footx_up.row(0) * _V_ini);
        _F_foot_lowx(0,0) = _F_foot_lowx(0,0)-_fx- _footx_min;
        
        
        // footy location constraints
        if (_bjxx % 2 == 0) //odd
        {
            _H_q_footy_up.row(0) = _Sfy.row(0);
            _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
            _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy - _footy_min; 
            
            _H_q_footy_low.row(0) = -_Sfy.row(0);
            _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini);	
            _F_foot_lowy(0,0) = _F_foot_lowy(0,0) -_fy + _footy_max;	
        }
        else
        {
            _H_q_footy_up.row(0) = _Sfy.row(0);
            _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
            _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy + _footy_max; 
            _H_q_footy_low.row(0) = -_Sfy.row(0);
            _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini );
            _F_foot_lowy(0,0) = _F_foot_lowy(0,0)-_fy - _footy_min;
        }	 
        }
        else   //two next steps
        {
        _H_q_footx_up.row(0) = _Sfx.row(0);
        _F_foot_upx.row(0) = -(_H_q_footx_up.row(0) * _V_ini); 
        _F_foot_upx(0,0) = _F_foot_upx(0,0) +_fx + _footx_max;
        _H_q_footx_low.row(0) = -_Sfx.row(0);
        _F_foot_lowx.row(0) = (_H_q_footx_up.row(0) * _V_ini);
        _F_foot_lowx(0,0) = _F_foot_lowx(0,0)-_fx - _footx_min;
        
        // footy location constraints
        if (_bjxx % 2 == 0) //odd
        {
            _H_q_footy_up.row(0) = _Sfy.row(0);
            _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
            _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy - _footy_min;
            _H_q_footy_low.row(0) = -_Sfy.row(0);
            _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini);
            _F_foot_lowy(0,0) = _F_foot_lowy(0,0)-_fy + _footy_max;
        }
        else
        {
            _H_q_footy_up.row(0) = _Sfy.row(0);
            _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
            _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy + _footy_max;
            _H_q_footy_low.row(0) = -_Sfy.row(0);
            _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini);
            _F_foot_lowy(0,0) = _F_foot_lowy(0,0)-_fy - _footy_min;
        }
        
        // the next two steps 
        _H_q_footx_up.row(1) = _Sfoot* _Sfx;
        _F_foot_upx.row(1) = -(_H_q_footx_up.row(1) * _V_ini); 	 
        _F_foot_upx(1,0) = _F_foot_upx(1,0) +_footx_max; 
        
        _H_q_footx_low.row(1) = -_Sfoot* _Sfx;
        _F_foot_lowx.row(1) = (_H_q_footx_up.row(1) * _V_ini);
        _F_foot_lowx(1,0) = (_F_foot_lowx(1,0) - _footx_min);
        
        // footy location constraints
        if (_bjxx % 2 == 0) //odd
        {
            _H_q_footy_up.row(1) = _Sfoot* _Sfy;
            _F_foot_upy.row(1) = -(_H_q_footy_up.row(1) * _V_ini); 
            _F_foot_upy(1,0) = _F_foot_upy(1,0) + _footy_max; 
            
            _H_q_footy_low.row(1) = -_H_q_footy_up.row(1);
            _F_foot_lowy.row(1) = (_H_q_footy_up.row(1) * _V_ini);
            _F_foot_lowy(1,0) = _F_foot_lowy(1,0) - _footy_min;		  
        }
        else
        {
            _H_q_footy_up.row(1) = _Sfoot* _Sfy;
            _F_foot_upy.row(1) = -(_H_q_footy_up.row(1) * _V_ini); 
            _F_foot_upy(1,0) = _F_foot_upy(1,0) - _footy_min; 
            
            _H_q_footy_low.row(1) = -_H_q_footy_up.row(1);
            _F_foot_lowy.row(1) = (_H_q_footy_up.row(1) * _V_ini);
            _F_foot_lowy(1,0) = _F_foot_lowy(1,0) + _footy_max;		  
        }	      	     	      
        }

    
    //swing foot veloctiy boundary
    if (i ==1)
    {
      _footubxv = -(_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));  ///// readline _footx_real_next， 
      _footubxv(0,0) = _footubxv(0,0)  + _footx_max;
      
      _footlbxv = (_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));
      _footlbxv(0,0) = _footlbxv(0,0) - _footx_min;		
      
      _footubyv = -(_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));
      _footubyv(0,0) = _footubyv(0,0) +_footy_max;		
      
      _footlbyv = (_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));
      _footlbyv(0,0) = _footlbyv(0,0) - _footy_min;		
      
    }
    else
    {
      if (fabs(i*_dt - _tx(_bjxx-1))<=0.01)
      {		
        _Footvx_max.setZero();  _Footvx_min.setZero();  _Footvy_max.setZero();  _Footvy_min.setZero();		  
        _footubxv.setZero(); _footlbxv.setZero(); _footubyv.setZero(); _footlbyv.setZero();			
      }
      else
      {
        _footubxv = -(_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));
        _footubxv(0,0) = _footubxv(0,0) + _footx_vmax*_dt;
        _footlbxv = (_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));
        _footlbxv(0,0) = _footlbxv(0,0) - _footx_vmin*_dt;		  
        _footubyv = -(_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));
        _footubyv(0,0) = _footubyv(0,0) + _footy_vmax*_dt;		  
        _footlbyv = (_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));	
        _footlbyv(0,0) = _footlbyv(0,0) - _footy_vmin*_dt;		  
      }
    }


    ///////////// equality equation	    
    //equality constraints
    _F_footz = _Sfz.row(0)*_V_ini - _Lz_ref.row(0);	    
    _hhhx = _h_h*_V_ini + _pps * _zk.col(i-1) - _Hcom;	

    _a_hxx = _a_hx * _V_ini + _pps * _thetaxk.col(i-1);
    _a_hyy = _a_hy * _V_ini + _pps * _thetayk.col(i-1);		    	      
  //////////////////////////////===========////////////////////////////////////////////////////	    
	// quadratic program GetSolution
    
    if (_method_flag ==0)
    {
      solve_reactive_step(); ///merely the reactive steps
    }
    else
    {
      if (_method_flag ==1)
      {
        solve_reactive_step_body_inclination(); /// the reactive steps + body inclination
      }
      else
      {
        solve_reactive_step_body_inclination_CoMz(); /// the reactive steps + body inclination + height variance
      }
    }    
    }
 
// /////////////////////////////////////////////////////////
    //next step location
    _comax.col(i) = _V_ini.row(0);
    _footx_real.row(_bjxx) = _V_ini.row(5*_nh);
    _footx_real_next.row(i+_nT -1) = _V_ini.row(5*_nh);
    _xk.col(i) = _a * _xk.col(i-1)  + _b* _comax.col(i);
    _comx(0,i)=_xk(0,i); _comvx(0,i) = _xk(1,i);	  
    
    _comay.col(i) = _V_ini.row(0+_nh);
    _footy_real.row(_bjxx) = _V_ini.row(5*_nh + _nstep);
    _footy_real_next.row(i+_nT -1) = _V_ini.row(5*_nh + _nstep);
    _yk.col(i) = _a * _yk.col(i-1)  + _b* _comay.col(i);
    _comy(0,i)=_yk(0,i); _comvy(0,i) = _yk(1,i); 
    
    
    
    _comx(0,i) +=  _fxx_global;
    _comy(0,i) +=  _fyy_global;	  
    _footx_real(_bjxx) = _footx_real(_bjxx) + _fxx_global;
    _footy_real(_bjxx) = _footy_real(_bjxx) + _fyy_global;	  	  
    _footx_real_next1(i+_nT -1) = _footx_real_next(i+_nT -1) + _fxx_global;	  
    _footy_real_next1(i+_nT -1) = _footy_real_next(i+_nT -1) + _fyy_global;

    
    
    _comaz.col(i) = _V_ini.row(0+2*_nh);
    _footz_real.row(_bjxx) = _V_ini.row(5*_nh + 2*_nstep);
    _footz_real_next.row(i+_nT -1) = _V_ini.row(5*_nh + 2*_nstep);	  
    _zk.col(i) = _a * _zk.col(i-1)  + _b* _comaz.col(i);
    _comz(0,i)=_zk(0,i); _comvz(0,i) = _zk(1,i); 

    _thetaax.col(i) = _V_ini.row(0+3*_nh);
    _thetaxk.col(i) = _a * _thetaxk.col(i-1)  + _b* _thetaax.col(i);
    _thetax(0,i) = _thetaxk(0,i); _thetavx(0,i) = _thetaxk(1,i);  	


    _thetaay.col(i) = _V_ini.row(0+4*_nh);
    _thetayk.col(i) = _a * _thetayk.col(i-1)  + _b* _thetaay.col(i);
    _thetay(0,i) = _thetayk(0,i); _thetavy(0,i) = _thetayk(1,i); 	
    
    _zmp_rela_vari(0) = _V_ini(5*_nh + 3*_nstep,0);
    _zmp_rela_vari(1) = _V_ini(5*_nh + 3*_nstep+1,0);
    _zmp_rela_vari(2) = _V_ini(5*_nh + 3*_nstep+2,0);
    _zmp_rela_vari(3) = _V_ini(5*_nh + 3*_nstep+3,0);
    
    
    // reference relative state    
    /// /// relative state to the actual foot lcoation: very good
    if (_bjxx % 2 == 0)  // odd : left support
    {
      estimated_state(0,0) =  estimated_state(0,0) - _Lfoot_location_feedback(0);
      estimated_state(3,0) =  estimated_state(3,0) - _Lfoot_location_feedback(1);
      estimated_state(6,0) =  estimated_state(6,0) - _Lfoot_location_feedback(2);
    }
    else
    {
      estimated_state(0,0) =  estimated_state(0,0) - _Rfoot_location_feedback(0);
      estimated_state(3,0) =  estimated_state(3,0) - _Rfoot_location_feedback(1);
      estimated_state(6,0) =  estimated_state(6,0) - _Rfoot_location_feedback(2);
    }
//////============================================================================================================================	      
////////////////////////////// state modified:====================================================================================
     ///////////////================state feedback: determined by ratio parameter: lamda==============================////
     double lamdax, lamday,lamdaz,lamdavx,lamdavy,lamdavz;
      lamda = 0.0;
      lamdax = 0.0;
      lamdavx = 0.0;
      lamday = 0;
      lamdavy = 0;
      lamdaz = 0;
      lamdavz = 0;  
     
/*    if (i<round(2*_ts(1)/_dt))
    {
      lamda = 0.0;
      lamdax = 0.1;
      lamdavx = 0.1;
      lamday = 0;
      lamdavy = 0;
      lamdaz = 0;
      lamdavz = 0;      
    }
    else
    {
      lamda = 0.2;
      lamdax = 0.01;
      lamdavx = 0.01;
      lamday = 0.1;
      lamdavy = 0.1; 
      lamdaz = 0.01;
      lamdavz = 0.01;   

    }
    
 */   
 

      
    _xk(0,i) = lamdax*estimated_state(0,0)+(1-lamdax)*_xk(0,i);             
    _xk(1,i) = lamdavx*estimated_state(1,0)+(1-lamdavx)*_xk(1,i); 	
    _yk(0,i) = (lamday*estimated_state(3,0)+(1-lamday)*_yk(0,i));
    _yk(1,i) = (lamdavy*estimated_state(4,0)+(1-lamdavy)*_yk(1,i));	
    _zk(0,i) = (lamdaz*estimated_state(6,0)+(1-lamdaz)*_zk(0,i)); 
    _zk(1,i) = (lamdavz*estimated_state(7,0)+(1-lamdavz)*_zk(1,i)); 	
    

    _thetaxk(0,i) = (lamda*estimated_state(9,0)+(1-lamda)*_thetaxk(0,i));
    _thetaxk(1,i) = (lamda*estimated_state(10,0)+(1-lamda)*_thetaxk(1,i));   
    _thetayk(0,i) = (lamda*estimated_state(12,0)+(1-lamda)*_thetayk(0,i)); 
    _thetayk(1,i) = (lamda*estimated_state(13,0)+(1-lamda)*_thetayk(1,i)); 
    


    
////////////////===============================================================================================	  
  /// next two sample time:	actually the preictive value is not reliable  
    _comx(0,i+1) = _comx(0,i) + _dt * _comvx(0,i); 	  	  
    _comy(0,i+1) = _comy(0,i) + _dt * _comvy(0,i); 	 
    _comz(0,i+1) = _comz(0,i) + _dt * _comvz(0,i); 	 
    _thetax(0,i+1) = _thetax(0,i)+ _dt * _thetavx(0,i); 	
    _thetay(0,i+1) = _thetay(0,i)+ _dt * _thetavy(0,i); 	
    
    
    _torquex_real.col(i) = _j_ini * _thetaax.col(i);
    _torquey_real.col(i) = _j_ini * _thetaay.col(i);
    
    _zmpx_real(0,i) = _comx(0,i) - (_comz(0,i) - _Zsc(i))/(_comaz(0,i)+_ggg(0))*_comax(0,i) - _j_ini * _thetaay(0,i)/(_mass * (_ggg(0) + _comaz(0,i)));
    _zmpy_real(0,i) = _comy(0,i) - (_comz(0,i) - _Zsc(i))/(_comaz(0,i)+_ggg(0))*_comay(0,i) + _j_ini * _thetaax(0,i)/(_mass * (_ggg(0) + _comaz(0,i)));

    
    _footxyz_real.row(0) = _footx_real.transpose();
    _footxyz_real.row(1) = _footy_real.transpose();	  
    _footxyz_real.row(2) = _footz_real.transpose();

    
    /////  generate the trajectory during the double support phase'
    _nTdx = round(_td(1)/_dt)+2;
    for (int jxx=2; jxx <=_nTdx; jxx++)
    {
      _comax.col(i+jxx-1) = _V_ini.row(jxx-1);
      _xk.col(i+jxx-1) = _a * _xk.col(i+jxx-2)  + _b* _comax.col(i+jxx-1);
      _comx(0,i+jxx-1)=_xk(0,i+jxx-1); _comvx(0,i+jxx-1) = _xk(1,i+jxx-1); 	  
      
      _comay.col(i+jxx-1) = _V_ini.row(jxx-1+_nh);
      _yk.col(i+jxx-1) = _a * _yk.col(i+jxx-2)  + _b* _comay.col(i+jxx-1);
      _comy(0,i+jxx-1)=_yk(0,i+jxx-1); _comvy(0,i+jxx-1) = _yk(1,i+jxx-1); 	     	      
      
      _comx(0,i+jxx-1) +=  _fxx_global;
      _comy(0,i+jxx-1) +=  _fyy_global;	  	      
      
      _comaz.col(i+jxx-1) = _V_ini.row(jxx-1+2*_nh);	  
      _zk.col(i+jxx-1) = _a * _zk.col(i+jxx-2)  + _b* _comaz.col(i+jxx-1);
      _comz(0,i+jxx-1)=_zk(0,i+jxx-1); _comvz(0,i+jxx-1) = _zk(1,i+jxx-1);

      _thetaax.col(i+jxx-1) = _V_ini.row(jxx-1+3*_nh);
      _thetaxk.col(i+jxx-1) = _a * _thetaxk.col(i+jxx-2)  + _b* _thetaax.col(i+jxx-1);
      _thetax(0,i+jxx-1) = _thetaxk(0,i+jxx-1); _thetavx(0,i+jxx-1) = _thetaxk(1,i+jxx-1);	


      _thetaay.col(i+jxx-1) = _V_ini.row(jxx-1+4*_nh);
      _thetayk.col(i+jxx-1) = _a * _thetayk.col(i+jxx-2)  + _b* _thetaay.col(i+jxx-1);
      _thetay(0,i+jxx-1) = _thetayk(0,i+jxx-1); _thetavy(0,i+jxx-1) = _thetayk(1,i+jxx-1);		      
    
      _torquex_real.col(i+jxx-1) = _j_ini * _thetaax.col(i+jxx-1);
      _torquey_real.col(i+jxx-1) = _j_ini * _thetaay.col(i+jxx-1);
      
      _zmpx_real(0,i+jxx-1) = _comx(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comax(0,i+jxx-1) - _j_ini * _thetaay(0,i+jxx-1)/(_mass * (_ggg(0) + _comaz(0,i+jxx-1)));
      _zmpy_real(0,i+jxx-1) = _comy(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comay(0,i+jxx-1) + _j_ini * _thetaax(0,i+jxx-1)/(_mass * (_ggg(0) + _comaz(0,i+jxx-1)));
      
    }	    
    
    if (i>=1)
    {	  
      _Rfootx(0) = _Rfootx(1);
      _Lfootx(0) = _Lfootx(1);
      _Rfooty(0) = _Rfooty(1);
      _Lfooty(0) = _Lfooty(1);
      _comx(0) = _comx(1);	
      _comy(0) = _comy(1);
      _comz(0) = _comz(1);	  
    }	 
	    
  }
  else  /////stop walking stage
  {
    if(i <= _n_end_walking+round(_tstep/_dt/2)){
        CoM_foot_gen_end(i);   
    }
    else
    {
        CoM_foot_gen_stopx(i);
    }
	
  }
  
//   _mpccontrol_com.block(0,0,1,_nsum) = _comx;
//   _mpccontrol_com.block(1,0,1,_nsum) = _comy;
//   _mpccontrol_com.block(2,0,1,_nsum) = _comz;
//   _mpccontrol_com.block(3,0,1,_nsum) = _thetax;
//   _mpccontrol_com.block(4,0,1,_nsum) = _thetay;
//   _mpccontrol_com.block(5,0,1,_nsum) = _zmpx_real;
//   _mpccontrol_com.block(6,0,1,_nsum) = _zmpy_real;
//   
//   File_wl(file_mpcx,_mpccontrol_com);
  
}


void MPCClass::CoM_foot_gen_end(int i)
{
      Indexfind(i*_dt,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
      _bjxx = _j_period+1;  //coincidence with matlab 
      _j_period = 0;		   
      
      _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
      
      Indexfind(_t_f(0),xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
      _bjx1 = _j_period+1;
      _j_period = 0;
      
      Eigen::Matrix<double, 4, 1> _comy_temp;
      _comy_temp.setZero();
      _comy_temp(0) = _comy(0,_n_end_walking-2);
      _comy_temp(1) = _comy(0,_n_end_walking-1);
      _comy_temp(2) = 0;
      _comy_temp(3) = 0;

      
      Eigen::Matrix<double, 4, 4> _comy_matrix;
      
      int ix_temp1 = -1;
      int ix_temp2 = 0;	   
      int ix_temp3 = round(_tstep/_dt)/2+1;
      
    
      Eigen::Matrix4d AAA_inv;
      
      double abx1, abx2, abx3, abx4;
      abx1 = ((ix_temp1 - ix_temp2)*pow(ix_temp1 - ix_temp3, 2));
      abx2 = ((ix_temp1 - ix_temp2)*pow(ix_temp2 - ix_temp3, 2));
      abx3 =(pow(ix_temp1 - ix_temp3, 2)*pow(ix_temp2 - ix_temp3, 2));
      abx4 = ((ix_temp1 - ix_temp3)*(ix_temp2 - ix_temp3));
      

      AAA_inv(0,0) = 1/ abx1;
      AAA_inv(0,1) =  -1/ abx2;
      AAA_inv(0,2) = (ix_temp1 + ix_temp2 - 2*ix_temp3)/ abx3;
      AAA_inv(0,3) = 1/ abx4;
      
      AAA_inv(1,0) = -(ix_temp2 + 2*ix_temp3)/ abx1;
      AAA_inv(1,1) = (ix_temp1 + 2*ix_temp3)/ abx2;
      AAA_inv(1,2) = -(pow(ix_temp1, 2) + ix_temp1*ix_temp2 + pow(ix_temp2, 2) - 3*pow(ix_temp3, 2))/ abx3;
      AAA_inv(1,3) = -(ix_temp1 + ix_temp2 + ix_temp3)/ abx4;
      
      AAA_inv(2,0) = (ix_temp3*(2*ix_temp2 + ix_temp3))/ abx1;
      AAA_inv(2,1) = -(ix_temp3*(2*ix_temp1 + ix_temp3))/ abx2;
      AAA_inv(2,2) = (ix_temp3*(2*pow(ix_temp1, 2) + 2*ix_temp1*ix_temp2 - 3*ix_temp3*ix_temp1 + 2*pow(ix_temp2, 2) - 3*ix_temp3*ix_temp2))/ abx3;
      AAA_inv(2,3) = (ix_temp1*ix_temp2 + ix_temp1*ix_temp3 + ix_temp2*ix_temp3)/ abx4;
      
      AAA_inv(3,0) = -(ix_temp2*pow(ix_temp3, 2))/ abx1;
      AAA_inv(3,1) = (ix_temp1*pow(ix_temp3, 2))/ abx2;
      AAA_inv(3,2) = (ix_temp1*ix_temp2*(ix_temp1*ix_temp2 - 2*ix_temp1*ix_temp3 - 2*ix_temp2*ix_temp3 + 3*pow(ix_temp3, 2)))/ abx3;
      AAA_inv(3,3) = -(ix_temp1*ix_temp2*ix_temp3)/ abx4;
	
      _comy_matrix_inv = AAA_inv * _comy_temp;


      /////  generate the trajectory during the double support phase'
      _nTdx = round(_ts(1)/_dt);
      for (int jxx=1; jxx <=_nTdx; jxx++)
      {
        if (i+jxx-1 <= _n_end_walking+round(_tstep/_dt/2))
        {
        Eigen::Matrix<double, 1, 4> _t_temp;
        
        _t_temp(0) = pow(i+jxx-1-_n_end_walking+1, 3);
        _t_temp(1) = pow(i+jxx-1-_n_end_walking+1, 2);
        _t_temp(2) = pow(i+jxx-1-_n_end_walking+1, 1);
        _t_temp(3) = pow(i+jxx-1-_n_end_walking+1, 0);	
        
        _comy.col(i+jxx-1) = _t_temp* _comy_matrix_inv;
        
        Eigen::Matrix<double, 1, 4> _t_tempv;
        
        _t_tempv(0) = 3*pow(i+jxx-1-_n_end_walking+1, 2);
        _t_tempv(1) = 2*pow(i+jxx-1-_n_end_walking+1, 1);
        _t_tempv(2) = 1;
        _t_tempv(3) = 0;	
        
        _comvy.col(i+jxx-1) = _t_tempv* _comy_matrix_inv;

        Eigen::Matrix<double, 1, 4> _t_tempa;
        
        _t_tempa(0) = 6*pow(i+jxx-1-_n_end_walking+1, 1);
        _t_tempa(1) = 2;
        _t_tempa(2) = 0;
        _t_tempa(3) = 0;	
        
        _comay.col(i+jxx-1) = _t_tempa* _comy_matrix_inv;				
        }
        else
        {
        _comy(0,i+jxx-1)=_comy(0,i+jxx-2); _comvy(0,i+jxx-1) = 0; _comay(0,i+jxx-1)= 0;		
        }
        
        _comx(0,i+jxx-1)=_comx(0,_n_end_walking-1); _comvx(0,i+jxx-1) = 0; _comax(0,i+jxx-1)= 0; 	  
                
        _comz(0,i+jxx-1)=_comz(0,_n_end_walking-1); _comvz(0,i+jxx-1) = 0; _comaz(0,i+jxx-1)= 0; 

        _thetax(0,i+jxx-1) = _thetax(0,_n_end_walking-1); _thetavx(0,i+jxx-1) = 0; _thetaax(0,i+jxx-1)= 0; 	

        _thetay(0,i+jxx-1) = _thetay(0,_n_end_walking-1); _thetavy(0,i+jxx-1) = 0; _thetaay(0,i+jxx-1)= 0; 		      
        
        _torquex_real.col(i+jxx-1) = _j_ini * _thetaax.col(i+jxx-1);
        _torquey_real.col(i+jxx-1) = _j_ini * _thetaay.col(i+jxx-1);
        
        _zmpx_real(0,i+jxx-1) = _comx(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comax(0,i+jxx-1) - _j_ini * _thetaay(0,i+jxx-1)/(_mass * (_ggg(0) + _comaz(0,i+jxx-1)));
        _zmpy_real(0,i+jxx-1) = _comy(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comay(0,i+jxx-1) + _j_ini * _thetaax(0,i+jxx-1)/(_mass * (_ggg(0) + _comaz(0,i+jxx-1)));
        
        }	    
        
        _footx_real_next.row(i+_nT -1)=_footx_real_next.row(_n_end_walking-1+_nT -1);
        _footy_real_next.row(i+_nT -1)=_footy_real_next.row(_n_end_walking-1-_nT -1);	   
        
        for (int jxxx = _bjxx+1; jxxx<_footstepsnumber; jxxx++){
            _footx_real(jxxx) = _footx_real(_bjxx) ;
            _footy_real(jxxx) = _footy_real(_bjxx-1);		     
        
        }
        _footxyz_real.row(0) = _footx_real.transpose();
        _footxyz_real.row(1) = _footy_real.transpose();	  
        _footxyz_real.row(2) = _footz_real.transpose();
    // 	    cout<<"_footy_real:"<<_footy_real<<endl;
    
}


void MPCClass::CoM_foot_gen_stopx(int i)
{
     
      Indexfind(i*_dt,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
      _bjxx = _j_period+1;  //coincidence with matlab 
      _j_period = 0;	
      
      _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
      
      Indexfind(_t_f(0),xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
      _bjx1 = _j_period+1;
      _j_period = 0;	
		
      _comy(0,i) = _comy(0,i-1);
      _comy(0,i+1) = _comy(0,i);	    
      
      _comx(0,i) = _comx(0,_n_end_walking-1);
      _comx(0,i+1) = _comx(0,_n_end_walking-1);
      _comz(0,i) = _comz(0,_n_end_walking-1);
      _comz(0,i+1) = _comz(0,_n_end_walking-1);	   
      _thetax(0,i) = _thetax(0,_n_end_walking-1);
      _thetax(0,i+1) = _thetax(0,_n_end_walking-1);	   
      _thetay(0,i) = _thetay(0,_n_end_walking-1);
      _thetay(0,i+1) = _thetay(0,_n_end_walking-1);
	      
      /////  generate the trajectory during the double support phase'
      _nTdx = round(_ts(1)/_dt)+2;
      for (int jxx=2; jxx <=_nTdx; jxx++)
      {
        _comx(0,i+jxx-1)=_comx(0,i+jxx-2); _comvx(0,i+jxx-1) = 0; _comax(0,i+jxx-1)= 0; 	  

        _comy(0,i+jxx-1)=_comy(0,i+jxx-2); _comvy(0,i+jxx-1) = 0; _comay(0,i+jxx-1)= 0; 	     	      

        _comz(0,i+jxx-1)=_comz(0,i+jxx-2); _comvz(0,i+jxx-1) = 0; _comaz(0,i+jxx-1)= 0; 

        _thetax(0,i+jxx-1) = _thetax(0,i+jxx-2); _thetavx(0,i+jxx-1) = 0; _thetaax(0,i+jxx-1)= 0; 	

        _thetay(0,i+jxx-1) = _thetay(0,i+jxx-2); _thetavy(0,i+jxx-1) = 0; _thetaay(0,i+jxx-1)= 0; 		      
        
        _torquex_real.col(i+jxx-1) = _j_ini * _thetaax.col(i+jxx-1);
        _torquey_real.col(i+jxx-1) = _j_ini * _thetaay.col(i+jxx-1);
        
        _zmpx_real(0,i+jxx-1) = _comx(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comax(0,i+jxx-1) - _j_ini * _thetaay(0,i+jxx-1)/(_mass * (_ggg(0) + _comaz(0,i+jxx-1)));
        _zmpy_real(0,i+jxx-1) = _comy(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comay(0,i+jxx-1) + _j_ini * _thetaax(0,i+jxx-1)/(_mass * (_ggg(0) + _comaz(0,i+jxx-1)));
	
      }	    
			
      _footx_real_next.row(i+_nT -1)=_footx_real_next.row(_n_end_walking-1+_nT -1);
      _footy_real_next.row(i+_nT -1)=_footy_real_next.row(_n_end_walking-1-_nT -1);	   
	
      for (int jxxx = _bjxx+1; jxxx<_footstepsnumber; jxxx++){
	      _footx_real(jxxx) = _footx_real(_bjxx) ;
	      _footy_real(jxxx) = _footy_real(_bjxx-1);		     
	  
      }
	_footxyz_real.row(0) = _footx_real.transpose();
	_footxyz_real.row(1) = _footy_real.transpose();	  
	_footxyz_real.row(2) = _footz_real.transpose();     
}




//////////////////////////// modified
int MPCClass::Indexfind(double goalvari, int xyz)
{
  _j_period = 0;
  if (xyz<0.05)
  {
    while (goalvari >= _tx(_j_period))
    {
      _j_period++;
    }
    
    _j_period = _j_period-1;	  
  }
  else
  {
    while ( fabs(goalvari - _t_f(_j_period)) >0.0001 )
    {
      _j_period++;
    }
	    
  }	  


}


///// only walking once when initialize
Eigen::MatrixXd  MPCClass::Matrix_ps(Eigen::Matrix<double,2,2> a, int nh,Eigen::RowVector2d cxps)
{
//   Eigen::MatrixXd matrixps(nh,3);
  Eigen::MatrixXd matrixps;
  matrixps.setZero(nh,2);  
  
  
  
  Eigen::MatrixXd A;
//   A.setIdentity(a.rows(),a.cols());
  
  for (int i = 0; i < nh; i++) {
    A.setIdentity(a.rows(),a.cols());
    for (int j = 1; j < i+2; j++)
    {
      A = A*a;
    }  
    
     matrixps.middleRows(i, 1)= cxps * A;      
  }
    
  return matrixps;
}


Eigen::MatrixXd MPCClass::Matrix_pu(Eigen::Matrix<double,2,2> a, Eigen::Matrix<double,2,1> b, int nh, Eigen::RowVector2d cxpu)
{
  Eigen::MatrixXd matrixpu;
  matrixpu.setZero(nh,nh);
  
  Eigen::MatrixXd A;
  Eigen::MatrixXd Tempxx;
  
  
  for (int i = 1; i < nh+1; i++) {
    for (int j = 1; j < i+1; j++)
    { 
      A.setIdentity(a.rows(),a.cols());      
      if (j==i)
      {
	Tempxx = cxpu * A * b;
	matrixpu(i-1,j-1) = Tempxx(0,0);
      }
      else
      {	
	for (int k = 1; k < i-j+1; k++)
	{
	  A = A*a;
	}
	Tempxx = cxpu * A * b;
	matrixpu(i-1,j-1) = Tempxx(0,0);
      }          
    }       
  }
    
  return matrixpu;  
}

// ==================================================================================================
///// three model MPC solution :modified================================================================
void MPCClass::solve_reactive_step()
{
/*  int nVars = _Nt;
  int nEqCon = 1+3*_nh;
  int nIneqCon = 5*_nh + 4*_nstep+4;  
  resizeQP(nVars, nEqCon, nIneqCon);	*/    

  _G = _Q_goal1;
  _g0 = _q_goal1;
  _X = _V_ini;
	    
  _CI.block<_Nt,_nh>(0,0) = _H_q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,_nh) = _H_q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,2*_nh) = _H_q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,3*_nh) = _H_q_lowy.transpose() * (-1); 
  _CI.block<_Nt,_nh>(0,4*_nh) = _H_hacc_lowz.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh) = _H_q_footx_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+_nstep) = _H_q_footx_low.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+2*_nstep) = _H_q_footy_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+3*_nstep) = _H_q_footy_low.transpose() * (-1);  

  
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep) = _Footvx_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep+1) = _Footvx_min.transpose() * (-1);
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep+2) = _Footvy_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep+3) = _Footvy_min.transpose() * (-1);

  

  
  _ci0.block(0, 0,_nh,1) = _F_zmp_upx;
  _ci0.block(_nh, 0,_nh,1) = _F_zmp_lowx;
  _ci0.block(2*_nh, 0,_nh,1) = _F_zmp_upy;
  _ci0.block(3*_nh, 0,_nh,1) = _F_zmp_lowy;
  _ci0.block(4*_nh, 0,_nh,1) = _F_hacc_lowz;
   
  _ci0.block(5*_nh, 0,_nstep,1) = _F_foot_upx;
  _ci0.block(5*_nh+_nstep, 0,_nstep,1) = _F_foot_lowx;
  _ci0.block(5*_nh+2*_nstep, 0,_nstep,1) = _F_foot_upy;
  _ci0.block(5*_nh+3*_nstep, 0,_nstep,1) = _F_foot_lowy;
   
  _ci0.block(5*_nh+4*_nstep, 0,1,1) = _footubxv;
  _ci0.block(5*_nh+4*_nstep+1, 0,1,1) = _footlbxv;
  _ci0.block(5*_nh+4*_nstep+2, 0,1,1) = _footubyv;
  _ci0.block(5*_nh+4*_nstep+3, 0,1,1) = _footlbyv;

  
  _CE.block(0,0, _Nt,1) = _H_q_footz.transpose();
  _CE.block(0,1, _Nt,_nh) = _h_h.transpose();
  _CE.block(0,_nh+1, _Nt,_nh) = _a_hx.transpose();
  _CE.block(0,2*_nh+1, _Nt,_nh) = _a_hy.transpose();
  
  _ce0.block(0,0, 1,1) = _F_footz;
  _ce0.block(1,0, _nh,1) = _hhhx;  
  _ce0.block(1+_nh,0, _nh,1) = _a_hxx;  
  _ce0.block(1+2*_nh,0, _nh,1) = _a_hyy;    
  
  
  Solve();  

}

void MPCClass::solve_reactive_step_body_inclination()
{
/*  int nVars = _Nt;
  int nEqCon = 1+_nh;
  int nIneqCon = 13*_nh + 4*_nstep +4;
  resizeQP(nVars, nEqCon, nIneqCon);*/	    

  _G = _Q_goal1;
  _g0 = _q_goal1;
  _X = _V_ini;

  
  _CI.block<_Nt,_nh>(0,0) = _H_q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,_nh) = _H_q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,2*_nh) = _H_q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,3*_nh) = _H_q_lowy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,4*_nh) = _H_hacc_lowz.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh) = _H_q_footx_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+_nstep) = _H_q_footx_low.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+2*_nstep) = _H_q_footy_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+3*_nstep) = _H_q_footy_low.transpose() * (-1);	    

  _CI.block<_Nt,_nh>(0,5*_nh+4*_nstep) = _q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,6*_nh+4*_nstep) = _q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,7*_nh+4*_nstep) = _q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,8*_nh+4*_nstep) = _q_lowy.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,9*_nh+4*_nstep) = _t_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,10*_nh+4*_nstep) = _t_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,11*_nh+4*_nstep) = _t_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,12*_nh+4*_nstep) = _t_lowy.transpose() * (-1);
  
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep) = _Footvx_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep+1) = _Footvx_min.transpose() * (-1);
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep+2) = _Footvy_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep+3) = _Footvy_min.transpose() * (-1);

  

  
  _ci0.block(0, 0,_nh,1) = _F_zmp_upx;
  _ci0.block(_nh, 0,_nh,1) = _F_zmp_lowx;
  _ci0.block(2*_nh, 0,_nh,1) = _F_zmp_upy;
  _ci0.block(3*_nh, 0,_nh,1) = _F_zmp_lowy; 
  _ci0.block(4*_nh, 0,_nh,1) = _F_hacc_lowz;
  _ci0.block(5*_nh, 0,_nstep,1) = _F_foot_upx;
  _ci0.block(5*_nh+_nstep, 0,_nstep,1) = _F_foot_lowx;
  _ci0.block(5*_nh+2*_nstep, 0,_nstep,1) = _F_foot_upy;
  _ci0.block(5*_nh+3*_nstep, 0,_nstep,1) = _F_foot_lowy;	    
  _ci0.block(5*_nh+4*_nstep, 0,_nh,1) = _qq_upx;
  _ci0.block(6*_nh+4*_nstep, 0,_nh,1) = _qq_lowx;
  _ci0.block(7*_nh+4*_nstep, 0,_nh,1) = _qq_upy;
  _ci0.block(8*_nh+4*_nstep, 0,_nh,1) = _qq_lowy;
  
  _ci0.block(9*_nh+4*_nstep, 0,_nh,1) = _tt_upx;
  _ci0.block(10*_nh+4*_nstep, 0,_nh,1) = _tt_lowx;
  _ci0.block(11*_nh+4*_nstep, 0,_nh,1) = _tt_upy;
  _ci0.block(12*_nh+4*_nstep, 0,_nh,1) = _tt_lowy;
  
  _ci0.block(13*_nh+4*_nstep, 0,1,1) = _footubxv;
  _ci0.block(13*_nh+4*_nstep+1, 0,1,1) = _footlbxv;
  _ci0.block(13*_nh+4*_nstep+2, 0,1,1) = _footubyv;
  _ci0.block(13*_nh+4*_nstep+3, 0,1,1) = _footlbyv;

  
  _CE.block(0,0, _Nt,1) = _H_q_footz.transpose();
  _CE.block(0,1, _Nt,_nh) = _h_h.transpose();
  
  _ce0.block(0,0, 1,1) = _F_footz;
  _ce0.block(1,0, _nh,1) = _hhhx;  

  Solve();  

}

void MPCClass::solve_reactive_step_body_inclination_CoMz()
{
/*  int nVars = _Nt;
  int nEqCon = 1;
  int nIneqCon = 15*_nh + 4*_nstep +4;
  resizeQP(nVars, nEqCon, nIneqCon);	*/    

  _G = _Q_goal1;
  _g0 = _q_goal1;
  _X = _V_ini;

  
  _CI.block<_Nt,_nh>(0,0) = _H_q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,_nh) = _H_q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,2*_nh) = _H_q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,3*_nh) = _H_q_lowy.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,4*_nh) = _H_h_upz.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,5*_nh) = _H_h_lowz.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,6*_nh) = _H_hacc_lowz.transpose() * (-1);
  
  _CI.block<_Nt,_nstep>(0,7*_nh) = _H_q_footx_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,7*_nh+_nstep) = _H_q_footx_low.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,7*_nh+2*_nstep) = _H_q_footy_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,7*_nh+3*_nstep) = _H_q_footy_low.transpose() * (-1);	    

  _CI.block<_Nt,_nh>(0,7*_nh+4*_nstep) = _q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,8*_nh+4*_nstep) = _q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,9*_nh+4*_nstep) = _q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,10*_nh+4*_nstep,_Nt,_nh) = _q_lowy.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,11*_nh+4*_nstep) = _t_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,12*_nh+4*_nstep) = _t_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,13*_nh+4*_nstep) = _t_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,14*_nh+4*_nstep) = _t_lowy.transpose() * (-1);
  
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep) = _Footvx_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep+1) = _Footvx_min.transpose() * (-1);
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep+2) = _Footvy_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep+3) = _Footvy_min.transpose() * (-1);

  

  
  _ci0.block(0, 0,_nh,1) = _F_zmp_upx;
  _ci0.block(_nh, 0,_nh,1) = _F_zmp_lowx;
  _ci0.block(2*_nh, 0,_nh,1) = _F_zmp_upy;
  _ci0.block(3*_nh, 0,_nh,1) = _F_zmp_lowy;
  
  _ci0.block(4*_nh, 0,_nh,1) = _F_h_upz;
  _ci0.block(5*_nh, 0,_nh,1) = _F_h_lowz;
  
  _ci0.block(6*_nh, 0,_nh,1) = _F_hacc_lowz;
  
  _ci0.block(7*_nh, 0,_nstep,1) = _F_foot_upx;
  _ci0.block(7*_nh+_nstep, 0,_nstep,1) = _F_foot_lowx;
  _ci0.block(7*_nh+2*_nstep, 0,_nstep,1) = _F_foot_upy;
  _ci0.block(7*_nh+3*_nstep, 0,_nstep,1) = _F_foot_lowy;	    

  _ci0.block(7*_nh+4*_nstep, 0,_nh,1) = _qq_upx;
  _ci0.block(8*_nh+4*_nstep, 0,_nh,1) = _qq_lowx;
  _ci0.block(9*_nh+4*_nstep, 0,_nh,1) = _qq_upy;
  _ci0.block(10*_nh+4*_nstep, 0,_nh,1) = _qq_lowy;
  
  _ci0.block(11*_nh+4*_nstep, 0,_nh,1) = _tt_upx;
  _ci0.block(12*_nh+4*_nstep, 0,_nh,1) = _tt_lowx;
  _ci0.block(13*_nh+4*_nstep, 0,_nh,1) = _tt_upy;
  _ci0.block(14*_nh+4*_nstep, 0,_nh,1) = _tt_lowy;
  
  _ci0.block(15*_nh+4*_nstep, 0,1,1) = _footubxv;
  _ci0.block(15*_nh+4*_nstep+1, 0,1,1) = _footlbxv;
  _ci0.block(15*_nh+4*_nstep+2, 0,1,1) = _footubyv;
  _ci0.block(15*_nh+4*_nstep+3, 0,1,1) = _footlbyv;

  
  
  
  
  
  _CE = _H_q_footz.transpose();
  _ce0 = _F_footz;
  
  
  
  Solve();  

}

void MPCClass::Solve()
{
// min 0.5 * x G x + g0 x
// _s.t.
// 		CE^T x + ce0 = 0
// 		CI^T x + ci0 >= 0
		solveQP();
		if (_X.rows() == _Nt)
		{
		  _V_ini += _X;
		}

}



////////////////////////////////=================swing foot trajectory==============================================////////////////////////
/////////////==================================================////////////////////////////////////////////////
//// foot trajectory solve--------polynomial ================================================
void MPCClass::Foot_trajectory_solve(int j_index,bool _stopwalking)
{   
  /// need to fetch the the rfootxyz, rfootvxyz,rfootaxyz, lfootxyz, lfootvxyz,lfootaxyz,_footxyz_real,_lift_height_ref;
  // maximal swing foot height: 
//   double  Footz_ref = _lift_height;
  //////////////external push mode==========
//   double  Footz_ref = 0.1;  
  ///// stairs :============================
//    double  Footz_ref = 0.07;  //0.02m
//     double  Footz_ref = 0.09;  //0.05m  
//    double  Footz_ref = 0.1;      //0.06m 
 
//    double  Footz_ref = 0.12;      //0.07m 
//    double  Footz_ref = 0.13;      //0.08m,0.09m 
//    double  Footz_ref = 0.14;      //0.1m,
   
//// judge if stop  
  if(_stopwalking)  
  {
    
    for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
      _lift_height_ref(i_t) = 0;  
    }	  

  }  
 
/* if (_bjx1<=7)
 {
   _lift_height_ref(_bjx1-1) = 0.1;
}
 else
 {
//    if (_bjx1<=16)
//    {
//      _lift_height_ref(_bjx1-1)  = 0.1;
//   }
//    else
//    {
//      _lift_height_ref(_bjx1-1) = 0.02;
//   }
   
  
} */ 
  

  _footxyz_real(1,0) = -_stepwidth(0);
  
//   foot trajectory generation:
  if (_bjx1 >= 2)
  {
      if (_bjx1 % 2 == 0)           //odd:left support
      {
    //     no change on the left support location

//         _Lfootx(j_index+1) = _Lfootx(j_index) = _Lfootx(j_index-1);
//         _Lfooty(j_index+1) = _Lfooty(j_index) = _Lfooty(j_index-1);
//         _Lfootz(j_index+1) = _Lfootz(j_index) = _Lfootz(j_index-1);
            _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
            
            _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);          

        /// right swing
        if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double support
        {
//             _Rfootx(j_index+1) = _Rfootx(j_index) = _Rfootx(j_index-1);
//             _Rfooty(j_index+1) = _Rfooty(j_index) = _Rfooty(j_index-1);
//             _Rfootz(j_index+1) = _Rfootz(j_index) = _Rfootz(j_index-1);
            _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
            _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
            _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
            
            _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
            _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
            _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);            
	  
        }
        else
        {
            double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
            Eigen::Vector3d t_plan(0,0,0);
            t_plan(0) = t_des - _dt;
            t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
            t_plan(2) = _ts(_bjx1-1) - 0.0045;
            
            if (abs(t_des - _ts(_bjx1-1)) <= ( + 0.0005))
            {
                _Rfootx(j_index) = _footxyz_real(0,_bjxx); 
                _Rfooty(j_index) = _footxyz_real(1,_bjxx);
                _Rfootz(j_index) = _footxyz_real(2,_bjxx); 	
                
                _Rfootx(j_index+1) = _footxyz_real(0,_bjxx); 
                _Rfooty(j_index+1) = _footxyz_real(1,_bjxx);
                _Rfootz(j_index+1) = _footxyz_real(2,_bjxx); 	  	    
            }
            else
            {
                Eigen::Matrix<double,7,7> AAA_inv = solve_AAA_inv_x(t_plan);
                    
                Eigen::Matrix<double, 1, 7> t_a_plan;
                t_a_plan.setZero();
                t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
                t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
                

                Eigen::Matrix<double, 1, 7> t_a_planv;
                t_a_planv.setZero();
                t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
                t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
                
                
                Eigen::Matrix<double, 1, 7> t_a_plana;
                t_a_plana.setZero();
                t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
                t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
                

                
                ////////////////////////////////////////////////////////////////////////////
                Eigen::Matrix<double, 7, 1> Rfootx_plan;
                Rfootx_plan.setZero();	
                Rfootx_plan(0) = _Rfootvx(j_index-1);     Rfootx_plan(1) = _Rfootax(j_index-1); Rfootx_plan(2) = _Rfootx(j_index-1); Rfootx_plan(3) = _Lfootx(j_index);
                Rfootx_plan(4) = _footxyz_real(0,_bjxx);  Rfootx_plan(5) = 0;                   Rfootx_plan(6) = 0;
                
                
                Eigen::Matrix<double, 7, 1> Rfootx_co;
                Rfootx_co.setZero();
                Rfootx_co = AAA_inv * Rfootx_plan;
                
                _Rfootx(j_index) = t_a_plan * Rfootx_co;
                _Rfootvx(j_index) = t_a_planv * Rfootx_co;
                _Rfootax(j_index) = t_a_plana * Rfootx_co;
                
                /////////////////////////////////////////////////////////////////////////////
//                 if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
//                 {
                   _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
//                 }
                
                Eigen::Matrix<double, 7, 1> Rfooty_plan;
                Rfooty_plan.setZero();	
                Rfooty_plan(0) = _Rfootvy(j_index-1);     Rfooty_plan(1) = _Rfootay(j_index-1); Rfooty_plan(2) = _Rfooty(j_index-1); Rfooty_plan(3) = _ry_left_right;
                Rfooty_plan(4) = _footxyz_real(1,_bjxx);  Rfooty_plan(5) = 0;                   Rfooty_plan(6) = 0;	  
//                 cout<<"Rfooty_plan:"<<Rfooty_plan.transpose()<<endl;
                
                Eigen::Matrix<double, 7, 1> Rfooty_co;
                Rfooty_co.setZero();
                Rfooty_co = AAA_inv * Rfooty_plan;
                
                _Rfooty(j_index) = t_a_plan * Rfooty_co;
                _Rfootvy(j_index) = t_a_planv * Rfooty_co;
                _Rfootay(j_index) = t_a_plana * Rfooty_co;	
                
                
                //////////////////////////////////////////////////////////
                Eigen::Matrix<double, 7, 1> Rfootz_plan;
                Rfootz_plan.setZero();	
                Rfootz_plan(0) = _Rfootvz(j_index-1);     Rfootz_plan(1) = _Rfootaz(j_index-1); Rfootz_plan(2) = _Rfootz(j_index-1); Rfootz_plan(3) = _Lfootz(j_index)+_lift_height_ref(_bjx1-1);
                Rfootz_plan(4) = _footxyz_real(2,_bjxx);  Rfootz_plan(5) = 0;                   Rfootz_plan(6) = 0.000;	
                
                Eigen::Matrix<double, 7, 1> Rfootz_co;
                Rfootz_co.setZero();
                Rfootz_co = AAA_inv * Rfootz_plan;
                
                _Rfootz(j_index) = t_a_plan * Rfootz_co;
                _Rfootvz(j_index) = t_a_planv * Rfootz_co;
                _Rfootaz(j_index) = t_a_plana * Rfootz_co;	
                    
                
                _Rfootx(j_index+1) = _Rfootx(j_index)+_dt * _Rfootvx(j_index);
                _Rfooty(j_index+1) = _Rfooty(j_index)+_dt * _Rfootvy(j_index);
                _Rfootz(j_index+1) = _Rfootz(j_index)+_dt * _Rfootvz(j_index);	    
            }
        }
      }
      else                       //right support
      {
    //       no change on right support
/*	  _Rfootx(j_index+1) = _Rfootx(j_index) = _Rfootx(j_index-1);
	  _Rfooty(j_index+1) = _Rfooty(j_index) = _Rfooty(j_index-1);
	  _Rfootz(j_index+1) = _Rfootz(j_index) = _Rfootz(j_index-1);   */

        _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);
        
        _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);
        
        /// left swing
        if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
        {
// 	  _Lfootx(j_index+1) = _Lfootx(j_index) = _Lfootx(j_index-1);
// 	  _Lfooty(j_index+1) = _Lfooty(j_index) = _Lfooty(j_index-1);
// 	  _Lfootz(j_index+1) = _Lfootz(j_index) = _Lfootz(j_index-1);
        _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);

        _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);            
        
        }
        else
        {
        // 	cout << "ssp"<<endl;
            //initial state and final state and the middle state
            double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
            Eigen::Vector3d t_plan(0,0,0);
            t_plan(0) = t_des - _dt;
            t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
            t_plan(2) = _ts(_bjx1-1) - 0.0045;
            
            if (abs(t_des - _ts(_bjx1-1)) <= ( + 0.0005))
            {
                
                _Lfootx(j_index) = _footxyz_real(0,_bjxx); 
                _Lfooty(j_index) = _footxyz_real(1,_bjxx);
                _Lfootz(j_index) = _footxyz_real(2,_bjxx); 

                _Lfootx(j_index+1) = _footxyz_real(0,_bjxx); 
                _Lfooty(j_index+1) = _footxyz_real(1,_bjxx);
                _Lfootz(j_index+1) = _footxyz_real(2,_bjxx); 
                
            }
            else
            {
                Eigen::Matrix<double,7,7> AAA_inv = solve_AAA_inv_x(t_plan);
                
                Eigen::Matrix<double, 1, 7> t_a_plan;
                t_a_plan.setZero();
                t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
                t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
                

                Eigen::Matrix<double, 1, 7> t_a_planv;
                t_a_planv.setZero();
                t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
                t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
                
                
                Eigen::Matrix<double, 1, 7> t_a_plana;
                t_a_plana.setZero();
                t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
                t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
                
                
                ////////////////////////////////////////////////////////////////////////////
                Eigen::Matrix<double, 7, 1> Lfootx_plan;
                Lfootx_plan.setZero();	
                Lfootx_plan(0) = _Lfootvx(j_index-1);     Lfootx_plan(1) = _Lfootax(j_index-1); Lfootx_plan(2) = _Lfootx(j_index-1); Lfootx_plan(3) = _Rfootx(j_index);
                Lfootx_plan(4) = _footxyz_real(0,_bjxx);  Lfootx_plan(5) = 0;                   Lfootx_plan(6) = 0;	  
                
                
                Eigen::Matrix<double, 7, 1> Lfootx_co;
                Lfootx_co.setZero();
                Lfootx_co = AAA_inv * Lfootx_plan;
                
                _Lfootx(j_index) = t_a_plan * Lfootx_co;
                _Lfootvx(j_index) = t_a_planv * Lfootx_co;
                _Lfootax(j_index) = t_a_plana * Lfootx_co;
                
                /////////////////////////////////////////////////////////////////////////////
//                 if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
//                 {
                _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
//                 }
                
                Eigen::Matrix<double, 7, 1> Lfooty_plan;
                Lfooty_plan.setZero();	
                Lfooty_plan(0) = _Lfootvy(j_index-1);     Lfooty_plan(1) = _Lfootay(j_index-1); Lfooty_plan(2) = _Lfooty(j_index-1); Lfooty_plan(3) = _ry_left_right;
                Lfooty_plan(4) = _footxyz_real(1,_bjxx);  Lfooty_plan(5) = 0;                   Lfooty_plan(6) = 0;		
/*                cout<<"Lfooty_plan:"<<Lfooty_plan.transpose()<<endl;*/                
                
                
                
                Eigen::Matrix<double, 7, 1> Lfooty_co;
                Lfooty_co.setZero();
                Lfooty_co = AAA_inv * Lfooty_plan;
                
                _Lfooty(j_index) = t_a_plan * Lfooty_co;
                _Lfootvy(j_index) = t_a_planv * Lfooty_co;
                _Lfootay(j_index) = t_a_plana * Lfooty_co;	
                
                
                //////////////////////////////////////////////////////////
                Eigen::Matrix<double, 7, 1> Lfootz_plan;
                Lfootz_plan.setZero();		
                Lfootz_plan(0) = _Lfootvz(j_index-1);     Lfootz_plan(1) = _Lfootaz(j_index-1); Lfootz_plan(2) = _Lfootz(j_index-1); Lfootz_plan(3) = _Rfootz(j_index)+_lift_height_ref(_bjx1-1);
                Lfootz_plan(4) = _footxyz_real(2,_bjxx);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0.000;		  
                
                
                Eigen::Matrix<double, 7, 1> Lfootz_co;
                Lfootz_co.setZero();
                Lfootz_co = AAA_inv * Lfootz_plan;
                
                _Lfootz(j_index) = t_a_plan * Lfootz_co;
                _Lfootvz(j_index) = t_a_planv * Lfootz_co;
                _Lfootaz(j_index) = t_a_plana * Lfootz_co;
                
            
                _Lfootx(j_index+1) = _Lfootx(j_index)+_dt * _Lfootvx(j_index);
                _Lfooty(j_index+1) = _Lfooty(j_index)+_dt * _Lfootvy(j_index);
                _Lfootz(j_index+1) = _Lfootz(j_index)+_dt * _Lfootvz(j_index);

            }
        }

      }

  }
  else
  {
    _Rfooty(j_index) = -_stepwidth(0);
    _Lfooty(j_index) = _stepwidth(0); 
  }
  
  
  /////////data saving///////////////////////////////////////////////
  _mpccontrol_state.block(0,0,1,_footstepsnumber) = _footx_real.transpose();
  _mpccontrol_state.block(1,0,1,_footstepsnumber) = _footy_real.transpose();
  _mpccontrol_state.block(2,0,1,_footstepsnumber) = _footz_real.transpose();
  _mpccontrol_state.block(3,0,1,_footstepsnumber) = _lift_height_ref.transpose();
  _mpccontrol_state.block(4,0,1,2) = _xk.col(j_index).transpose();
  _mpccontrol_state.block(4,2,1,2) = _yk.col(j_index).transpose();
  _mpccontrol_state.block(4,4,1,2) = _zk.col(j_index).transpose();
  _mpccontrol_state.block(4,6,1,2) = _thetaxk.col(j_index).transpose();
  _mpccontrol_state.block(4,8,1,2) = _thetayk.col(j_index).transpose(); 
  _mpccontrol_state(4,10) = _fxx_global; 
  _mpccontrol_state(4,11) = _fyy_global;
  _mpccontrol_state(4,12) = _footx_real_next(j_index+_nT -1,0);
  _mpccontrol_state(4,13) = _footy_real_next(j_index+_nT -1,0);
  
  _mpccontrol_state(5,0) = _Lfootx(0,j_index);  _mpccontrol_state(5,1) = _Lfootvx(0,j_index);  _mpccontrol_state(5,2) = _Lfootax(0,j_index); 
  _mpccontrol_state(5,3) = _Lfooty(0,j_index);  _mpccontrol_state(5,4) = _Lfootvy(0,j_index);  _mpccontrol_state(5,5) = _Lfootay(0,j_index);  
  _mpccontrol_state(5,6) = _Lfootz(0,j_index);  _mpccontrol_state(5,7) = _Lfootvz(0,j_index);  _mpccontrol_state(5,8) = _Lfootaz(0,j_index); 
  _mpccontrol_state(5,9) = _Rfootx(0,j_index);  _mpccontrol_state(5,10) = _Rfootvx(0,j_index);  _mpccontrol_state(5,11) = _Rfootax(0,j_index); 
  _mpccontrol_state(5,12) = _Rfooty(0,j_index);  _mpccontrol_state(5,13) = _Rfootvy(0,j_index);  _mpccontrol_state(5,14) = _Rfootay(0,j_index);  
  _mpccontrol_state(5,15) = _Rfootz(0,j_index);  _mpccontrol_state(5,16) = _Rfootvz(0,j_index);  _mpccontrol_state(5,17) = _Rfootaz(0,j_index);  
  
  _mpccontrol_state(6,0) = _comx(0,j_index);    _mpccontrol_state(6,1) = _comvx(0,j_index);    _mpccontrol_state(6,2) = _comax(0,j_index); 
  _mpccontrol_state(6,3) = _comy(0,j_index);    _mpccontrol_state(6,4) = _comvy(0,j_index);    _mpccontrol_state(6,5) = _comay(0,j_index);  
  _mpccontrol_state(6,6) = _comz(0,j_index);    _mpccontrol_state(6,7) = _comvz(0,j_index);    _mpccontrol_state(6,8) = _comaz(0,j_index); 
  _mpccontrol_state(6,9) = _thetax(0,j_index);  _mpccontrol_state(6,10) = _thetavx(0,j_index);  _mpccontrol_state(6,11) = _thetaax(0,j_index); 
  _mpccontrol_state(6,12) = _thetay(0,j_index);  _mpccontrol_state(6,13) = _thetavy(0,j_index); _mpccontrol_state(6,14) = _thetaay(0,j_index);  
/*  _mpccontrol_state(6,15) = _Rfootz(0,j_index);  _mpccontrol_state(6,16) = _Rfootvz(0,j_index); _mpccontrol_state(6,17) = _Rfootaz(0,j_index); */ 
  
  File_wl(file_mpc,_mpccontrol_state);
  
  
  ///
  _mpccontrol_com.block(0,0,1,_nsum) = _comx;
  _mpccontrol_com.block(1,0,1,_nsum) = _comy;
  _mpccontrol_com.block(2,0,1,_nsum) = _comz;
  _mpccontrol_com.block(3,0,1,_nsum) = _thetax;
  _mpccontrol_com.block(4,0,1,_nsum) = _thetay;
  _mpccontrol_com.block(5,0,1,_nsum) = _zmpx_real;
  _mpccontrol_com.block(6,0,1,_nsum) = _zmpy_real;
  _mpccontrol_com.block(7,0,1,_nsum) = _Lfootx;
  _mpccontrol_com.block(8,0,1,_nsum) = _Lfooty;
  _mpccontrol_com.block(9,0,1,_nsum) = _Lfootz; 
  _mpccontrol_com.block(10,0,1,_nsum) = _Rfootx;
  _mpccontrol_com.block(11,0,1,_nsum) = _Rfooty;
  _mpccontrol_com.block(12,0,1,_nsum) = _Rfootz;   
  
  File_wl(file_mpcx,_mpccontrol_com);  
  
}

Vector3d MPCClass::X_CoM_position_squat(int walktime, double dt_sample)
{
  Vector3d com_inte(0,0,0);

  double t_des  = walktime * dt_sample;

  Eigen::Vector3d t_plan;
  t_plan(0) = 0.00001;
  t_plan(1) = _height_squat_time/2+0.0001;
  t_plan(2) = _height_squat_time+0.0001;

  if (t_des<=_height_squat_time)
  {   
    Eigen::Matrix<double,7,7> AAA_inv = solve_AAA_inv_x(t_plan);
	    
    Eigen::Matrix<double, 1, 7> t_a_plan;
    t_a_plan.setZero();
    t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
    t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;

    ////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix<double, 7, 1> com_squat_plan;
    com_squat_plan.setZero();	
    com_squat_plan(0) = 0;     com_squat_plan(1) = 0; 
    com_squat_plan(2) = _RobotPara_Z_C; 
    com_squat_plan(3) = _RobotPara_Z_C-_height_offset/2;
    com_squat_plan(4) = _RobotPara_Z_C-_height_offset;  
    com_squat_plan(5) = 0;                   
    com_squat_plan(6) = 0;


    Eigen::Matrix<double, 7, 1> com_co;
    com_co.setZero();
    com_co = AAA_inv * com_squat_plan;

    com_inte(2) = t_a_plan * com_co;
    
  }
  else
  {
    com_inte(2) = _RobotPara_Z_C-_height_offset;   
  }
  
  return com_inte; 
}

///////////////////////// ODE - sampling time maximal ========================================
int MPCClass::Get_maximal_number_reference()
{
  return (_nsum -_nh-1);
}

int MPCClass::Get_maximal_number(double dtx)
{
  
  return (_nsum -_nh-1)*floor(_dt/dtx);
}
////====================================================================================================================
/////////////////////////// using the lower-level control-loop  sampling time as the reference: every 5ms;  at the same time: just using the next one position + next one velocity

Vector3d MPCClass::XGetSolution_CoM_position(int walktime, double dt_sample, int t_int, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{ 
  if (t_int>=1)
  {
     MatrixXd mpc_state = rea_save(file_mpc,_mpc_starowNum); 
     if (mpc_loop_flag==1) ///current state is generated by the current loop: the storage data is the last time
     {
      _Lfootx(0,t_int-1) = mpc_state(5,0);  _Lfootvx(0,t_int-1) = mpc_state(5,1);  _Lfootax(0,t_int-1) = mpc_state(5,2); 
      _Lfooty(0,t_int-1) = mpc_state(5,3);  _Lfootvy(0,t_int-1) = mpc_state(5,4);  _Lfootay(0,t_int-1) = mpc_state(5,5);  
      _Lfootz(0,t_int-1) = mpc_state(5,6);  _Lfootvz(0,t_int-1) = mpc_state(5,7);  _Lfootaz(0,t_int-1) = mpc_state(5,8); 
      _Rfootx(0,t_int-1) = mpc_state(5,9);  _Rfootvx(0,t_int-1) = mpc_state(5,10);  _Rfootax(0,t_int-1)= mpc_state(5,11); 
      _Rfooty(0,t_int-1) = mpc_state(5,12); _Rfootvy(0,t_int-1) = mpc_state(5,13);  _Rfootay(0,t_int-1)= mpc_state(5,14);  
      _Rfootz(0,t_int-1) = mpc_state(5,15); _Rfootvz(0,t_int-1) = mpc_state(5,16);  _Rfootaz(0,t_int-1)= mpc_state(5,17);  
      
      _comx(0,t_int-1) = mpc_state(6,0);    _comvx(0,t_int-1) = mpc_state(6,1);    _comax(0,t_int-1) =mpc_state(6,2); 
      _comy(0,t_int-1) = mpc_state(6,3);    _comvy(0,t_int-1) = mpc_state(6,4);    _comay(0,t_int-1) =mpc_state(6,5);  
      _comz(0,t_int-1) = mpc_state(6,6);    _comvz(0,t_int-1) = mpc_state(6,7);    _comaz(0,t_int-1) = mpc_state(6,8); 
      _thetax(0,t_int-1) = mpc_state(6,9);  _thetavx(0,t_int-1) = mpc_state(6,10);  _thetaax(0,t_int-1) = mpc_state(6,11); 
      _thetay(0,t_int-1) = mpc_state(6,12); _thetavy(0,t_int-1) =  mpc_state(6,13); _thetaay(0,t_int-1) = mpc_state(6,14);       
    }
    else /////the storage data is the last time
    {
      _Lfootx(0,t_int) = mpc_state(5,0);  _Lfootvx(0,t_int) = mpc_state(5,1);  _Lfootax(0,t_int) = mpc_state(5,2); 
      _Lfooty(0,t_int) = mpc_state(5,3);  _Lfootvy(0,t_int) = mpc_state(5,4);  _Lfootay(0,t_int) = mpc_state(5,5);  
      _Lfootz(0,t_int) = mpc_state(5,6);  _Lfootvz(0,t_int) = mpc_state(5,7);  _Lfootaz(0,t_int) = mpc_state(5,8); 
      _Rfootx(0,t_int) = mpc_state(5,9);  _Rfootvx(0,t_int) = mpc_state(5,10);  _Rfootax(0,t_int)= mpc_state(5,11); 
      _Rfooty(0,t_int) = mpc_state(5,12); _Rfootvy(0,t_int) = mpc_state(5,13);  _Rfootay(0,t_int)= mpc_state(5,14);  
      _Rfootz(0,t_int) = mpc_state(5,15); _Rfootvz(0,t_int) = mpc_state(5,16);  _Rfootaz(0,t_int)= mpc_state(5,17);  
      
      _comx(0,t_int) = mpc_state(6,0);    _comvx(0,t_int) = mpc_state(6,1);    _comax(0,t_int) =mpc_state(6,2); 
      _comy(0,t_int) = mpc_state(6,3);    _comvy(0,t_int) = mpc_state(6,4);    _comay(0,t_int) =mpc_state(6,5);  
      _comz(0,t_int) = mpc_state(6,6);    _comvz(0,t_int) = mpc_state(6,7);    _comaz(0,t_int) = mpc_state(6,8); 
      _thetax(0,t_int) = mpc_state(6,9);  _thetavx(0,t_int) = mpc_state(6,10);  _thetaax(0,t_int) = mpc_state(6,11); 
      _thetay(0,t_int) = mpc_state(6,12); _thetavy(0,t_int) =  mpc_state(6,13); _thetaay(0,t_int) = mpc_state(6,14);        
    }

    MatrixXd mpc_com = rea_save(file_mpcx,_mpc_starowNum1);
    if (mpc_loop_flag==1)
    {
        _comx.block(0,0,1,t_int-1) = mpc_com.block(0,0,1,t_int-1);
        _comy.block(0,0,1,t_int-1) = mpc_com.block(1,0,1,t_int-1);
        _comz.block(0,0,1,t_int-1) = mpc_com.block(2,0,1,t_int-1);
        _thetax.block(0,0,1,t_int-1) = mpc_com.block(3,0,1,t_int-1);
        _thetay.block(0,0,1,t_int-1) = mpc_com.block(4,0,1,t_int-1);
        _zmpx_real.block(0,0,1,t_int-1) = mpc_com.block(5,0,1,t_int-1);
        _zmpy_real.block(0,0,1,t_int-1) = mpc_com.block(6,0,1,t_int-1);
        _Lfootx.block(0,0,1,t_int-1) = mpc_com.block(7,0,1,t_int-1);
        _Lfooty.block(0,0,1,t_int-1) = mpc_com.block(8,0,1,t_int-1);
        _Lfootz.block(0,0,1,t_int-1) = mpc_com.block(9,0,1,t_int-1); 
        _Rfootx.block(0,0,1,t_int-1) = mpc_com.block(10,0,1,t_int-1);
        _Rfooty.block(0,0,1,t_int-1) = mpc_com.block(11,0,1,t_int-1);
        _Rfootz.block(0,0,1,t_int-1) = mpc_com.block(12,0,1,t_int-1);  

    }
    else
    {
        _comx.block(0,0,1,_nsum) = mpc_com.block(0,0,1,_nsum);
        _comy.block(0,0,1,_nsum) = mpc_com.block(1,0,1,_nsum);
        _comz.block(0,0,1,_nsum) = mpc_com.block(2,0,1,_nsum);
        _thetax.block(0,0,1,_nsum) = mpc_com.block(3,0,1,_nsum);
        _thetay.block(0,0,1,_nsum) = mpc_com.block(4,0,1,_nsum);
        _zmpx_real.block(0,0,1,_nsum) = mpc_com.block(5,0,1,_nsum);
        _zmpy_real.block(0,0,1,_nsum) = mpc_com.block(6,0,1,_nsum); 
        _Lfootx = mpc_com.block(7,0,1,_nsum);
        _Lfooty = mpc_com.block(8,0,1,_nsum);
        _Lfootz = mpc_com.block(9,0,1,_nsum); 
        _Rfootx = mpc_com.block(10,0,1,_nsum);
        _Rfooty = mpc_com.block(11,0,1,_nsum);
        _Rfootz = mpc_com.block(12,0,1,_nsum);        
        
    }
    
    
  }  
  
  
  //reference com position
	_comz(0) = _RobotPara_Z_C-_height_offset;
	_comz(1) = _RobotPara_Z_C-_height_offset;
	_comz(2) = _RobotPara_Z_C-_height_offset;
	_comz(3) = _RobotPara_Z_C-_height_offset;
	_comz(4) = _RobotPara_Z_C-_height_offset;
	
	Vector3d com_inte(0,0,0);	
	
	if (walktime>=2)
	{
// 	  int t_int= floor(walktime / (_dt / dt_sample) );

	  ///// chage to be relative time
	  double t_cur = walktime * dt_sample ;
	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = 0;
	  t_plan(1) = dt_sample;
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);
	  
	  solve_AAA_inv(t_plan);
	  	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(2*dt_sample, 3);   t_a_plan(1) = pow(2*dt_sample, 2);   
	  t_a_plan(2) = pow(2*dt_sample, 1);   t_a_plan(3) = pow(2*dt_sample, 0); 
	  
	  Eigen::Matrix<double, 1, 4> t_a_planv;
	  t_a_planv.setZero();
	  t_a_planv(0) = 3*pow(2*dt_sample, 2);   t_a_planv(1) = 2*pow(2*dt_sample, 1);   
	  t_a_planv(2) = 1;   t_a_planv(3) = 0; 	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plana;
	  t_a_plana.setZero();
	  t_a_plana(0) = 6*pow(2*dt_sample, 1);   t_a_plana(1) = 2;   
	  t_a_plana(2) = 0;   t_a_plana(3) = pow(2*dt_sample, 0); 	  
	  	  
	  
	  // COM&&foot trajectory interpolation	   	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = _comx(t_int); temp(3) = _comvx(t_int);	  
	  com_inte(0) = t_a_plan * (_AAA_inv)*temp;
	  _comxyzx(0) = com_inte(0);
	  _comvxyzx(0) = t_a_planv * (_AAA_inv)*temp;
	  _comaxyzx(0) = t_a_plana * (_AAA_inv)*temp;
	  
	  
	  temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = _comy(t_int); temp(3) = _comvy(t_int);	  
	  com_inte(1) = t_a_plan * (_AAA_inv)*temp;
	  _comxyzx(1) = com_inte(1);
	  _comvxyzx(1) = t_a_planv * (_AAA_inv)*temp;
	  _comaxyzx(1) = t_a_plana * (_AAA_inv)*temp;	  
	  
	  temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = _comz(t_int); temp(3) = _comvz(t_int);	  
	  com_inte(2) = t_a_plan *(_AAA_inv)*temp;
	  _comxyzx(2) = com_inte(2);
	  _comvxyzx(2) = t_a_planv * (_AAA_inv)*temp;
	  _comaxyzx(2) = t_a_plana * (_AAA_inv)*temp;	 
	  
	  /////be careful, the polynomial may cause overfitting
	  double t_des = t_cur-t_int*_dt;
	  if (t_des<=0){
	    t_des =0.00001;
	  }
	    
// 	  if (t_int>=1)
// 	  {
// 	    _comaxyzx(0) = (_comax(t_int)-_comax(t_int-1))/_dt*t_des+_comax(t_int-1);
// 	    _comaxyzx(1) = (_comay(t_int)-_comay(t_int-1))/_dt*t_des+_comay(t_int-1);
// 	    _comaxyzx(2) = (_comaz(t_int)-_comaz(t_int-1))/_dt*t_des+_comaz(t_int-1);
// 	  }
// 	  else
// 	  {
// 	    _comaxyzx(0) = (_comax(t_int)-0)/_dt*t_des+0;
// 	    _comaxyzx(1) = (_comay(t_int)-0)/_dt*t_des+0;
// 	    _comaxyzx(2) = (_comaz(t_int)-0)/_dt*t_des+0;	    
// 	  } 
	}
	else
	{
	  com_inte = body_in3;	
	  _comxyzx = com_inte;
	}

 	return com_inte;
	
}

Vector3d MPCClass::XGetSolution_body_inclination(int walktime, double dt_sample, int t_int, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{		
	Vector3d com_inte(0,0,0);		
	if (walktime>=2)
	{
// 	  int t_int = floor(walktime / (_dt / dt_sample) );
	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(2*dt_sample, 3);   t_a_plan(1) = pow(2*dt_sample, 2);   
	  t_a_plan(2) = pow(2*dt_sample, 1);   t_a_plan(3) = pow(2*dt_sample, 0); 
	  
	  Eigen::Matrix<double, 1, 4> t_a_planv;
	  t_a_planv.setZero();
	  t_a_planv(0) = 3*pow(2*dt_sample, 2);   t_a_planv(1) = 2*pow(2*dt_sample, 1);   
	  t_a_planv(2) = 1;   t_a_planv(3) = 0; 	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plana;
	  t_a_plana.setZero();
	  t_a_plana(0) = 6*pow(2*dt_sample, 1);   t_a_plana(1) = 2;   
	  t_a_plana(2) = 0;   t_a_plana(3) = pow(2*dt_sample, 0); 	  
	  
	  //body inclination interpolation	  	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = _thetax(t_int); temp(3) = _thetavx(t_int);	  
	  com_inte(0) = t_a_plan * (_AAA_inv)*temp;
	  _thetaxyx(0) = com_inte(0);
	  _thetavxyx(0) = t_a_planv * (_AAA_inv)*temp;
	  _thetaaxyx(0) = t_a_plana * (_AAA_inv)*temp;	  
	  
	  temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = _thetay(t_int); temp(3) = _thetavy(t_int);	  
	  com_inte(1) = t_a_plan * (_AAA_inv)*temp;
	  _thetaxyx(1) = com_inte(1);
	  _thetavxyx(1) = t_a_planv * (_AAA_inv)*temp;
	  _thetaaxyx(1) = t_a_plana * (_AAA_inv)*temp;		  
	  
	  temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = _thetaz(t_int); temp(3) = _thetavz(t_int);	  
	  com_inte(2) = t_a_plan *(_AAA_inv)*temp;
	  _thetaxyx(2) = com_inte(2);
	  _thetavxyx(2) = t_a_planv * (_AAA_inv)*temp;
	  _thetaaxyx(2) = t_a_plana * (_AAA_inv)*temp;
	  
	  /////be careful, the polynomial may cause overfitting
	  double t_des = walktime * dt_sample-t_int*_dt;
	  if (t_des<=0){
	    t_des =0.0001;
	  }	  

/*	  if (t_int>=1)
	  {
	    _thetaxyx(0) = (_thetaax(t_int)-_thetaax(t_int-1))/_dt*t_des+_thetaax(t_int-1);
	    _thetaxyx(1) = (_thetaay(t_int)-_thetaay(t_int-1))/_dt*t_des+_thetaay(t_int-1);
	    _thetaxyx(2) = (_thetaaz(t_int)-_thetaaz(t_int-1))/_dt*t_des+_thetaaz(t_int-1);
	  }
	  else
	  {
	    _thetaxyx(0) = (_thetaax(t_int)-0)/_dt*t_des+0;
	    _thetaxyx(1) = (_thetaay(t_int)-0)/_dt*t_des+0;
	    _thetaxyx(2) = (_thetaaz(t_int)-0)/_dt*t_des+0;	    
	  }*/ 	  
	}
	else
	{
	  com_inte = body_in3;
	  _thetaxyx = com_inte;
	}
 	return com_inte;
}


Vector3d MPCClass::XGetSolution_Foot_positionR(int walktime, double dt_sample, int t_int, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{	
	Vector3d com_inte(0,0,0);	
	
	if (walktime>=2)
	{
// 	  int t_int = floor(walktime / (_dt / dt_sample) );

	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(2*dt_sample, 3);   t_a_plan(1) = pow(2*dt_sample, 2);   
	  t_a_plan(2) = pow(2*dt_sample, 1);   t_a_plan(3) = pow(2*dt_sample, 0);  
  
	  // foot trajectory interpolation	  
	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = _Rfootx(t_int); temp(3) = _Rfootvx(t_int);	  
	  com_inte(0) = t_a_plan * (_AAA_inv)*temp;
	  temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = _Rfooty(t_int); temp(3) = _Rfootvy(t_int);
	  com_inte(1) = t_a_plan * (_AAA_inv)*temp;
	  temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = _Rfootz(t_int); temp(3) = _Rfootvz(t_int);	  
	  com_inte(2) = t_a_plan *(_AAA_inv)*temp;	    
	}
	else
	{
          com_inte = body_in3;	    
	}
	
	_Rfootxyzx = com_inte;
	
 	return com_inte;
         	
}

Vector3d MPCClass::XGetSolution_Foot_positionL(int walktime, double dt_sample, int t_int, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
	Vector3d com_inte(0,0,0);	
	
	if (walktime>=2)
	{
// 	  int t_int = floor(walktime / (_dt / dt_sample) );
	  	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(2*dt_sample, 3);   t_a_plan(1) = pow(2*dt_sample, 2);   
	  t_a_plan(2) = pow(2*dt_sample, 1);   t_a_plan(3) = pow(2*dt_sample, 0); 
	  
	  // foot trajectory interpolation
	  
	  Eigen::Vector4d  temp;
	  temp.setZero();
	  temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = _Lfootx(t_int); temp(3) = _Lfootvx(t_int);
	  com_inte(0) = t_a_plan * (_AAA_inv)*temp;
	  temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = _Lfooty(t_int); temp(3) = _Lfootvy(t_int);	  
	  com_inte(1) = t_a_plan * (_AAA_inv)*temp;
	  temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = _Lfootz(t_int); temp(3) = _Lfootvz(t_int);		  
	  com_inte(2) = t_a_plan * (_AAA_inv)*temp;
	  
	}
	else
	{
	  com_inte = body_in3;	  
	}

	_Lfootxyzx = com_inte;
	
 	return com_inte;
	
}



////solve the inverse matrix of 4*4 coefficient matrices
void MPCClass::solve_AAA_inv(Eigen::Matrix<double, 4, 1> t_plan)
{
  
// 	  Eigen::MatrixXd AAA1;	
// 
// 	  AAA1.setZero(4,4);	
// 	  AAA1(0,0) = pow(t_plan(0), 3); AAA1(0,1) = pow(t_plan(0), 2); AAA1(0,2) = pow(t_plan(0), 1); AAA1(0,3) = pow(t_plan(0), 0); 
// 	  AAA1(1,0) = pow(t_plan(1), 3); AAA1(1,1) = pow(t_plan(1), 2); AAA1(1,2) = pow(t_plan(1), 1); AAA1(1,3) = pow(t_plan(0), 0); 
// 	  AAA1(2,0) = pow(t_plan(2), 3); AAA1(2,1) = pow(t_plan(2), 2); AAA1(2,2) = pow(t_plan(2), 1); AAA1(2,3) = pow(t_plan(0), 0); 
// 	  AAA1(3,0) = 3*pow(t_plan(2), 2); AAA1(3,1) = 2*pow(t_plan(2), 1); AAA1(3,2) = pow(t_plan(2), 0); AAA1(3,3) = 0;  

  
  
  double abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
  double abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
  double abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
  double abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
  

  _AAA_inv(0,0) = 1/ abx1;
  _AAA_inv(0,1) =  -1/ abx2;
  _AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
  _AAA_inv(0,3) = 1/ abx4;
  
  _AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
  _AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
  _AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
  _AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
  
  _AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
  _AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
  _AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
  _AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
  
  _AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
  _AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
  _AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
  _AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;   
}

////solve the inverse matrix of 7*7 coefficient matrices
Eigen::Matrix<double, 7, 7> MPCClass::solve_AAA_inv_x(Eigen::Vector3d t_plan)
{
  Eigen::Matrix<double,7,7> AAA;
  AAA.setZero();
  Eigen::Matrix<double, 1, 7> aaaa;
  aaaa.setZero();

  aaaa(0) = 6*pow(t_plan(0), 5);   aaaa(1) =  5*pow(t_plan(0), 4);  aaaa(2) =  4*pow(t_plan(0), 3);   aaaa(3) =  3*pow(t_plan(0), 2);
  aaaa(4) = 2*pow(t_plan(0), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
  AAA.row(0) = aaaa;

  aaaa(0) = 30*pow(t_plan(0), 4);  aaaa(1) =  20*pow(t_plan(0), 3);  aaaa(2) =  12*pow(t_plan(0), 2);   aaaa(3) =  6*pow(t_plan(0), 1);
  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
  AAA.row(1) = aaaa;

  aaaa(0) = pow(t_plan(0), 6);     aaaa(1) =  pow(t_plan(0), 5);     aaaa(2) =  pow(t_plan(0), 4);   aaaa(3) =  pow(t_plan(0), 3);
  aaaa(4) = pow(t_plan(0), 2);     aaaa(5) = pow(t_plan(0), 1);      aaaa(6) =  1;	  
  AAA.row(2) = aaaa;

  aaaa(0) = pow(t_plan(1), 6);     aaaa(1) =  pow(t_plan(1), 5);     aaaa(2) =  pow(t_plan(1), 4);   aaaa(3) =  pow(t_plan(1), 3);
  aaaa(4) = pow(t_plan(1), 2);     aaaa(5) = pow(t_plan(1), 1);      aaaa(6) =  1;
  AAA.row(3) = aaaa;

  aaaa(0) = pow(t_plan(2), 6);     aaaa(1) =  pow(t_plan(2), 5);     aaaa(2) =  pow(t_plan(2), 4);   aaaa(3) =  pow(t_plan(2), 3);
  aaaa(4) = pow(t_plan(2), 2);     aaaa(5) = pow(t_plan(2), 1);      aaaa(6) =  1;
  AAA.row(4) = aaaa;	  

  aaaa(0) = 6*pow(t_plan(2), 5);   aaaa(1) =  5*pow(t_plan(2), 4);  aaaa(2) =  4*pow(t_plan(2), 3);   aaaa(3) =  3*pow(t_plan(2), 2);
  aaaa(4) = 2*pow(t_plan(2), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
  AAA.row(5) = aaaa;

  aaaa(0) = 30*pow(t_plan(2), 4);  aaaa(1) =  20*pow(t_plan(2), 3);  aaaa(2) =  12*pow(t_plan(2), 2);   aaaa(3) =  6*pow(t_plan(2), 1);
  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
  AAA.row(6) = aaaa;  
  
  Eigen::Matrix<double,7,7> AAA_inv = AAA.inverse(); 
  
  return AAA_inv;
  
}


/////=============================ZMP optimal distribution for lower level adimittance control=================================
////reference_force_torque_distribution========================================

void MPCClass::Zmp_distributor(int walktime, double dt_sample)
{
// //// judge if stop  
//   if(_stopwalking)  
//   {  
//     for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
//       _lift_height_ref(i_t) = 0;  
//     }	  
// 
//   }  
  
  int j_index = floor(walktime / (_dt / dt_sample));
  
  zmp_interpolation(j_index,walktime,dt_sample);  

// reference_force_torque_distribution 
  if (_bjx1 >= 2)
  {
      if (_bjx1 % 2 == 0)           //odd:left support
      {  
	/// right swing
	if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))
	{
	  int nTx_n = round(_tx(_bjx1-1)/_dt);
	  int nTx_n_dsp = round((_tx(_bjx1-1)+_td(_bjx1-1))/_dt);
	  Vector2d ZMP_init(0,0);
	  ZMP_init(0) = _zmpx_real(0,nTx_n-2);
	  ZMP_init(1) = _zmpy_real(0,nTx_n-2);
	  
	  Vector2d ZMP_end(0,0); 
	  ZMP_end(0) = _zmpx_real(0,nTx_n_dsp-1);
	  ZMP_end(1) = _zmpy_real(0,nTx_n_dsp-1);
	  
// 	  if (abs(ZMP_end(0)-ZMP_init(0))<=0.001) 
// 	  {
// 	      _Co_L(1,1) = abs((_ZMPxy_realx(1)-ZMP_init(1))/(ZMP_end(1)-ZMP_init(1)));
// 	      if (_Co_L(1,1) >1)
// 	      {
// 		_Co_L(1,1)=1;
// 	      }               
// 	      _Co_L(0,0) = _Co_L(1,1);                
// 	      _Co_L(2,2) = sqrt((pow(_Co_L(0,0),2)+pow(_Co_L(0,0),2))/2);  
// 	  }
// 	  else
// 	  {
	    _Co_L(0,0) = abs(((ZMP_end(1)-ZMP_init(1))*(_ZMPxy_realx(1)-ZMP_init(1))+(ZMP_end(0)-ZMP_init(0))*(_ZMPxy_realx(0)-ZMP_init(0)))/(pow(ZMP_end(1)-ZMP_init(1),2)+pow(ZMP_end(0)-ZMP_init(0),2)));
	    _Co_L(1,1) = _Co_L(0,0);
	      
	    if (_Co_L(0,0) >1)
	    {
	      _Co_L(0,0)=1;
	    }
	    if (_Co_L(1,1) >1)
	    {
	      _Co_L(1,1)=1;
	    }
	    _Co_L(2,2) = sqrt((pow(_Co_L(0,0),2)+pow(_Co_L(0,0),2))/2); 
/*	  }  */ 
	  
	  Matrix3d II;
	  II.setIdentity(3,3);
	  _Co_R = II-_Co_L;     

	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);	  
	  
	}
	else
	{
	 //Swing leg with left support
	  _Co_L.setIdentity(3,3);
	  _Co_R.setZero();
	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);  	  
	}	
      }      
      else                       //right support
      {	/// left swing
	if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
	{
	  int nTx_n = round(_tx(_bjx1-1)/_dt);
	  int nTx_n_dsp = round((_tx(_bjx1-1)+_td(_bjx1-1))/_dt);
	  Vector2d ZMP_init(0,0);
	  ZMP_init(0) = _zmpx_real(0,nTx_n-2);
	  ZMP_init(1) = _zmpy_real(0,nTx_n-2);
	  
	  Vector2d ZMP_end(0,0); 
	  ZMP_end(0) = _zmpx_real(0,nTx_n_dsp-1);
	  ZMP_end(1) = _zmpy_real(0,nTx_n_dsp-1);
	  
// 	  if (abs(ZMP_end(0)-ZMP_init(0))<=0.001) 
// 	  {
// 	      _Co_R(1,1) = abs((_ZMPxy_realx(1)-ZMP_init(1))/(ZMP_end(1)-ZMP_init(1)));
// 	      if (_Co_R(1,1) >1)
// 	      {
// 		_Co_R(1,1)=1;
// 	      }               
// 	      _Co_R(0,0) = _Co_R(1,1);                
// 	      _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2);  
// 	  }
// 	  else
// 	  {
	    _Co_R(0,0) = abs(((ZMP_end(1)-ZMP_init(1))*(_ZMPxy_realx(1)-ZMP_init(1))+(ZMP_end(0)-ZMP_init(0))*(_ZMPxy_realx(0)-ZMP_init(0)))/(pow(ZMP_end(1)-ZMP_init(1),2)+pow(ZMP_end(0)-ZMP_init(0),2)));
	    _Co_R(1,1) = _Co_R(0,0);
	      
	    if (_Co_R(0,0) >1)
	    {
	      _Co_R(0,0)=1;
	    }
	    if (_Co_R(1,1) >1)
	    {
	      _Co_R(1,1)=1;
	    }
	    _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2); 
/*	  }*/   
	  
	  Matrix3d II;
	  II.setIdentity(3,3);
	  _Co_L = II-_Co_R;     

	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);	  	 	  
	}
	else
	{
	 //Swing leg with right support
	  _Co_R.setIdentity(3,3);
	  _Co_L.setZero();
	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);  	  
	}
      }
  }
  else
  {
    if (_bjx1==0)   ///stand still:
    {
//       _Co_R = 0.5*Eigen::Vector3d::Zero().setIdentity(3,3);
//       _Co_L.setZero();
      _Co_R(0,0) = _Co_R(1,1) = _Co_R(2,2) = _Co_L(0,0) = _Co_L(1,1) = _Co_L(2,2) = 0.5;
      Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);       
    }
    else   ///right support: double support in the whole walking pattern: assuming that the initial and final ZMP is located at the foot locations 
    {/// left swing
	Vector2d ZMP_init;
	ZMP_init(0) = _footxyz_real(0,_bjx1-1);
	ZMP_init(1) = _footxyz_real(1,_bjx1-1);
	
	Vector2d ZMP_end; 
	ZMP_end(0) = _footxyz_real(0,_bjx1);
	ZMP_end(1) = _footxyz_real(1,_bjx1);
	
// 	if (abs(ZMP_end(0)-ZMP_init(0))<=0.001) 
// 	{
// 	    _Co_R(1,1) = abs((_ZMPxy_realx(1)-ZMP_init(1))/(ZMP_end(1)-ZMP_init(1)));
// 	    if (_Co_R(1,1) >1)
// 	    {
// 	      _Co_R(1,1)=1;
// 	    }               
// 	    _Co_R(0,0) = _Co_R(1,1);                
// 	    _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2);  
// 	}
// 	else
// 	{
	  _Co_R(0,0) = abs(((ZMP_end(1)-ZMP_init(1))*(_ZMPxy_realx(1)-ZMP_init(1))+(ZMP_end(0)-ZMP_init(0))*(_ZMPxy_realx(0)-ZMP_init(0)))/(pow(ZMP_end(1)-ZMP_init(1),2)+pow(ZMP_end(0)-ZMP_init(0),2)));
	  _Co_R(1,1) = _Co_R(0,0);
	    
	  if (_Co_R(0,0) >1)
	  {
	    _Co_R(0,0)=1;
	  }
	  if (_Co_R(1,1) >1)
	  {
	    _Co_R(1,1)=1;
	  }
	  _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2); 
/*	}*/   
	
	Matrix3d II;
	II.setIdentity(3,3);
	_Co_L = II-_Co_R;     

	Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);	  	 	       
    }
  }   
}


void MPCClass::zmp_interpolation(int t_int,int walktime, double dt_sample)
{
  //// calculate by the nonlinear model:
//   if (t_int>=1)
//   {
//     _ZMPxy_realx(0) = _comxyzx(0) - (_comxyzx(2) - _Zsc(t_int-1))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(0) - _j_ini * _thetaaxyx(1)/(_mass * (_ggg(0) + _comaxyzx(2)));
//     _ZMPxy_realx(1) = _comxyzx(1) - (_comxyzx(2) - _Zsc(t_int-1))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(1) + _j_ini * _thetaaxyx(0)/(_mass * (_ggg(0) + _comaxyzx(2)));     
//   }
//   else
//   {
//     _ZMPxy_realx(0) = _comxyzx(0) - (_comxyzx(2))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(0) - _j_ini * _thetaaxyx(1)/(_mass * (_ggg(0) + _comaxyzx(2)));
//     _ZMPxy_realx(1) = _comxyzx(1) - (_comxyzx(2))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(1) + _j_ini * _thetaaxyx(0)/(_mass * (_ggg(0) + _comaxyzx(2)));     
//   }
  
  //// linear interpolation of ZMP reference:  
  double t_des = walktime * dt_sample-t_int*_dt;
  if (t_des<=0){
    t_des =0.0001;
  }	  
//   cout <<"//////////////////////////////////////////"<<endl;
//   cout <<"t_des:"<<t_des<<endl;
//   cout <<"//////////////////////////////////////////"<<endl;
  if (t_int>=1)
  {
    _ZMPxy_realx(0) = (_zmpx_real(0,t_int)-_zmpx_real(0,t_int-1))/_dt*t_des+_zmpx_real(0,t_int-1);
    _ZMPxy_realx(1) = (_zmpy_real(0,t_int)-_zmpy_real(0,t_int-1))/_dt*t_des+_zmpy_real(0,t_int-1);

  }
  else
  {
    _ZMPxy_realx(0) = _zmpx_real(0,t_int)/_dt*t_des+0;
    _ZMPxy_realx(1) = _zmpy_real(0,t_int)/_dt*t_des+0;	    
  }  
  
  
}


void MPCClass::Force_torque_calculate(Vector3d comxyzx1,Vector3d comaxyzx1,Vector3d thetaaxyx1,Vector3d Lfootxyz1,Vector3d Rfootxyz1)
{
  Vector3d gra;
  gra << 0,0, -_ggg;
  
  Vector3d F_total = _mass * (comaxyzx1 - gra);
  
  Vector3d the3a;
  the3a << thetaaxyx1(0),thetaaxyx1(1),0;
  Vector3d L_total = _j_ini * the3a;
  
  _F_R = _Co_R * F_total;
  _F_L = _Co_L * F_total;
  
  Vector3d R_det_foot_com = Rfootxyz1 -  comxyzx1;
  
  Vector3d L_det_foot_com = Lfootxyz1 -  comxyzx1;
  
  Vector3d M_total = L_total - _F_R.cross(R_det_foot_com) - _F_L.cross(L_det_foot_com);
  
  _M_R = _Co_R*M_total;  
  _M_L = _Co_L*M_total;
  
}


//////////////////////////////////////////////////////
Eigen::Matrix<double, 6, 1> MPCClass::XGetSolution_Foot_position_KMP(int walktime, double dt_sample,int j_index)
{
  
  
  if (j_index>=1)
  {
    MatrixXd mpc_com = rea_save(file_mpcy,_mpc_starowNum2);
    _Rfootx_kmp(walktime-1) = mpc_com(0,0);   _Rfooty_kmp(walktime-1) = mpc_com(0,1);    _Rfootz_kmp(walktime-1) = mpc_com(0,2); 
    _Rfootvx_kmp(walktime-1) = mpc_com(0,3);  _Rfootvy_kmp(walktime-1) = mpc_com(0,4);   _Rfootvz_kmp(walktime-1) = mpc_com(0,5);  
    _Lfootx_kmp(walktime-1) = mpc_com(0,6);   _Lfooty_kmp(walktime-1) = mpc_com(0,7);    _Lfootz_kmp(walktime-1) = mpc_com(0,8); 
    _Lfootvx_kmp(walktime-1) = mpc_com(0,9);  _Lfootvy_kmp(walktime-1) = mpc_com(0,10);  _Lfootvz_kmp(walktime-1) = mpc_com(0,11);       
  }  
    
  ///////walktime=====>ij;   int j_index====>i;  dt_sample========>dtx;   
   Eigen::Matrix<double, 6, 1> com_inte;	

  double  Footz_ref = 0.05;
  //0.05-0.06m    
if((_bjx1==12)||(_bjx1 ==15)||(_bjx1 ==16))
{
  Footz_ref = 0.08;
}

if((_bjx1 ==15)||(_bjx1 ==16))
{
  Footz_ref = 0.1;
}

if((_bjx1>(_footstepsnumber-3)))
{
  Footz_ref = 0.005;
}
  
  //// three via_points: time, mean, sigma 
  vec via_point1 =  zeros<vec>(43);
  vec via_point2 =  zeros<vec>(43);
  vec via_point3 =  zeros<vec>(43);
  
  via_point1(7) =0.00000000001; via_point1(14)=0.00000000001; via_point1(21)=0.00000000001;	
  via_point1(28)=0.00000000001; via_point1(35)=0.00000000001; via_point1(42)=0.00000000001;  
  
  via_point2(0) =0.65/2;
  via_point2(7) =0.00000000001; via_point2(14)=0.00000000001; via_point2(21)=0.00000000001;	
  via_point2(28)=0.00000000001; via_point2(35)=0.00000000001; via_point2(42)=0.00000000001;
  
  via_point3(0) =0.65;
  via_point3(7) =0.00000000001; via_point3(14)=0.00000000001; via_point3(21)=0.00000000001;	
  via_point3(28)=0.00000000001; via_point3(35)=0.00000000001; via_point3(42)=0.00000000001;      
  
  
  
  double t_des;      /////////desired time during the current step
  t_des = (walktime+4)*dt_sample - (_tx(_bjx1-1)+_td(_bjx1-1));
  

  if (_bjx1 >= 2)
  {      
    
    if (_bjx1 % 2 == 0)           //odd:left support
    {
      _Lfootx_kmp(walktime) = _footxyz_real(0,_bjx1-1);
      _Lfooty_kmp(walktime) = _footxyz_real(1,_bjx1-1);
      _Lfootz_kmp(walktime) = _footxyz_real(2,_bjx1-1);
      
      _Lfootvx_kmp(walktime) = 0;
      _Lfootvy_kmp(walktime) = 0;
      _Lfootvz_kmp(walktime) = 0;    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {

	_Rfootx_kmp(walktime) = _footxyz_real(0,_bjx1-2);
	_Rfooty_kmp(walktime) = _footxyz_real(1,_bjx1-2);
	_Rfootz_kmp(walktime) = _footxyz_real(2,_bjx1-2);
	
	_Rfootvx_kmp(walktime) = 0;
	_Rfootvy_kmp(walktime) = 0;
	_Rfootvz_kmp(walktime) = 0;  
	
      }
      else
      {
	
	//initial state and final state and the middle state
	double t_des_k;
	t_des_k = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
	
	  /////////////first sampling time of the current walking cycle: initialize the KMP_data
          if (t_des<=dt_sample)
	  {
	    kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
	  //// %%% initial state and final state and the middle state
	  /////%%%%% be careful the x position start from zero, the y
	  //// %%%%% position start from -0.0726
	    ////// add point************ current status****************////////
	    via_point1(0) = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
	    via_point1(1) = _Rfootx_kmp(walktime-1)-_footxyz_real(0,_bjx1-2);
	    via_point1(2) = _Rfooty_kmp(walktime-1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	    via_point1(3) = _Rfootz_kmp(walktime-1)-_footxyz_real(2,_bjx1-2);
	    via_point1(4) = _Rfootvx_kmp(walktime-1);
	    via_point1(5) = _Rfootvy_kmp(walktime-1);
	    via_point1(6) = _Rfootvz_kmp(walktime-1);
	    kmp_leg_R.kmp_insertPoint(via_point1);  // insert point into kmp
	    
	    ////// add point************ middle point***********////////
// 	    via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
	    via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
	    via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	    via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);
	    via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/0.65*1.15;
	    if (_bjx1==2)
	    {
	      via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/0.65*0.8;
	    }	    
	    via_point2(5) = (_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-2))/0.65;
// 	    via_point2(6) = 0;
	    if (_footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)<=0)  ////downstairs
	    {
	      via_point2(6) = -0.1;
// 	      if (_footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)<=0.02)
// 	      {
// 		via_point2(6) = -0.2;
// 	      }
	    }
	    else
	    {
	      via_point2(6) = 0;
	    }	    	    
	    kmp_leg_R.kmp_insertPoint(via_point2);  // insert point into kmp

	    ////// add point************ final status************////////	  
	    via_point3(1) = _footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2)+0.005;
	    via_point3(2) = _footxyz_real(1,_bjx1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	    via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2);
	    via_point3(4) = 0;
	    via_point3(5) = 0;
  // 	   via_point3(6) = 0;
	    if (via_point3(3)<0)  ////downstairs
	    {
	      via_point3(6) = 0.15;
	    }
	    else
	    {
	      via_point3(6) = 0.025;
	    }
	    
	    
	    kmp_leg_R.kmp_insertPoint(via_point3);  // insert point into kmp	
	    
	    kmp_leg_R.kmp_estimateMatrix();
	  }

	  
	  
	    _query_kmp(0) = t_des_k;
	    kmp_leg_R.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values
/*	    if (t_des_k<0.01){
	    cout<<"_query_kmp:"<<endl<<trans(_query_kmp)<<endl;
	    cout<<"kmp:"<<endl<<trans(_mean_kmp)<<endl;
	    cout<<"error:"<<trans(_mean_kmp)-trans(via_point1(span(1,6)))<<endl<<endl;
	      
	      
	    }*/	  
	  


	  
	  _Rfootx_kmp(walktime) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	  _Rfooty_kmp(walktime) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  _Rfootz_kmp(walktime) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);
	  
	  _Rfootvx_kmp(walktime) = _mean_kmp(3);
	  _Rfootvy_kmp(walktime) = _mean_kmp(4);
	  _Rfootvz_kmp(walktime) = _mean_kmp(5); 	  
	  
 	  if (abs(_Rfootz_kmp(walktime)-_Rfootz_kmp(walktime-1))>0.01)
 	  {
// 	    if (_Rfootz_kmp(walktime)-_Rfootz_kmp(walktime-1)>0)
// 	    {
// 	      _Rfootz_kmp(walktime) = _Rfootz_kmp(walktime-1) + 0.01;
// 	    }
// 	    else
// 	    {
// 	      _Rfootz_kmp(walktime) = _Rfootz_kmp(walktime-1) - 0.01;
// 	    }
	    _Rfootz_kmp(walktime)=_Rfootz_kmp(walktime-1);
 	  }
	
	
      }   
    }
    
    else                       //right support
    {
      _Rfootx_kmp(walktime) = _footxyz_real(0,_bjx1-1);
      _Rfooty_kmp(walktime) = _footxyz_real(1,_bjx1-1);
      _Rfootz_kmp(walktime) = _footxyz_real(2,_bjx1-1);
      
      _Rfootvx_kmp(walktime) = 0;
      _Rfootvy_kmp(walktime) = 0;
      _Rfootvz_kmp(walktime) = 0;    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {

	_Lfootx_kmp(walktime) = _footxyz_real(0,_bjx1-2);
	_Lfooty_kmp(walktime) = _footxyz_real(1,_bjx1-2);
	_Lfootz_kmp(walktime) = _footxyz_real(2,_bjx1-2);
	
	_Lfootvx_kmp(walktime) = 0;
	_Lfootvy_kmp(walktime) = 0;
	_Lfootvz_kmp(walktime) = 0;  
	
      }
      else
      {
	
	//initial state and final state and the middle state
	double t_des_k;
	t_des_k = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
	
	if (t_des<=dt_sample)
	{	
	  kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
         //// %%% initial state and final state and the middle state
         /////%%%%% be careful the x position start from zero, the y
         //// %%%%% position start from -0.0726
	  ////// add point************ current status****************////////
	  via_point1(0) = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
	  via_point1(1) = _Lfootx_kmp(walktime-1)-_footxyz_real(0,_bjx1-2);
	  via_point1(2) = _Lfooty_kmp(walktime-1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  via_point1(3) = _Lfootz_kmp(walktime-1)-_footxyz_real(2,_bjx1-2);
	  via_point1(4) = _Lfootvx_kmp(walktime-1);
	  via_point1(5) = _Lfootvy_kmp(walktime-1);
	  via_point1(6) = _Lfootvz_kmp(walktime-1);
 	  kmp_leg_L.kmp_insertPoint(via_point1);  // insert point into kmp
	  
	  ////// add point************ middle point***********////////
// 	  via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
	  via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
	  via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);
	  via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/0.65*1.45;
	  via_point2(5) = (_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-2))/0.65;
// 	  via_point2(6) = 0;
	  if (_footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)<=0)  ////downstairs
	  {
	    via_point2(6) = -0.1;
// 	    if (_footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)<=0.02)
// 	    {
// 	       via_point2(6) = -0.2;
// 	    }
// 	      
	  }
	  else
	  {
	    via_point2(6) = 0;
	  }	  	  	  
 	  kmp_leg_L.kmp_insertPoint(via_point2);  // insert point into kmp

          ////// add point************ final status************////////	  
	  via_point3(1) = _footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2)+0.005;
	  via_point3(2) = _footxyz_real(1,_bjx1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2);
	  via_point3(4) = 0;
	  via_point3(5) = 0;
// 	   via_point3(6) = 0;
	  if (via_point3(3)<0)  ////downstairs
	  {
	     via_point3(6) = 0.15;
	  }
	  else
	  {
	    via_point3(6) = 0.025;
	  }	     	 
	  kmp_leg_L.kmp_insertPoint(via_point3);  // insert point into kmp
	  
	  
	  kmp_leg_L.kmp_estimateMatrix();
	   
	  
	}
	
	  _query_kmp(0) = t_des_k;
	  kmp_leg_L.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values
	  
// 	  if (t_des_k<0.01){
// 	  cout<<"_query_kmp:"<<endl<<trans(_query_kmp)<<endl;
// 	  cout<<"kmp:"<<endl<<trans(_mean_kmp)<<endl;
// 	  cout<<"error:"<<endl<<trans(_mean_kmp)-trans(via_point1(span(1,6)))<<endl;	
// 	  }	


	  
	  _Lfootx_kmp(walktime) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	  _Lfooty_kmp(walktime) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  _Lfootz_kmp(walktime) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);
	  
	  _Lfootvx_kmp(walktime) = _mean_kmp(3);
	  _Lfootvy_kmp(walktime) = _mean_kmp(4);
	  _Lfootvz_kmp(walktime) = _mean_kmp(5); 	  
          if (abs(_Lfootz_kmp(walktime)-_Lfootz_kmp(walktime-1))>0.01)
	  {
// 	    if (_Lfootz_kmp(walktime)-_Lfootz_kmp(walktime-1)>0)
// 	    {
// 	      _Lfootz_kmp(walktime) = _Lfootz_kmp(walktime-1) + 0.01;
// 	    }
// 	    else
// 	    {
// 	      _Lfootz_kmp(walktime) = _Lfootz_kmp(walktime-1) - 0.01;
// 	    }
	    _Lfootz_kmp(walktime) = _Lfootz_kmp(walktime-1);
	  }
	    
	
	
      }   

    }
      
    
  }
  
  
  /////Rfoot,xyz, Lfoot,XYZ
  com_inte(0,0) = _Rfootx_kmp(walktime);
  com_inte(1,0) = _Rfooty_kmp(walktime);
  com_inte(2,0) = _Rfootz_kmp(walktime);

  com_inte(3,0) = _Lfootx_kmp(walktime);
  com_inte(4,0) = _Lfooty_kmp(walktime);
  com_inte(5,0) = _Lfootz_kmp(walktime);

 // cout<<"via_pint3"<<via_point3<<endl;
  
//   cout<"vai"
  
  
  _mpccontrol_com1(0,0) = _Rfootx_kmp(walktime);   _mpccontrol_com1(0,1) = _Rfooty_kmp(walktime);    _mpccontrol_com1(0,2) = _Rfootz_kmp(walktime); 
  _mpccontrol_com1(0,3) = _Rfootvx_kmp(walktime);  _mpccontrol_com1(0,4) = _Rfootvy_kmp(walktime);   _mpccontrol_com1(0,5) = _Rfootvz_kmp(walktime);  
  _mpccontrol_com1(0,6) = _Lfootx_kmp(walktime);   _mpccontrol_com1(0,7) = _Lfooty_kmp(walktime);    _mpccontrol_com1(0,8) = _Lfootz_kmp(walktime); 
  _mpccontrol_com1(0,9) = _Lfootvx_kmp(walktime);  _mpccontrol_com1(0,10) = _Lfootvy_kmp(walktime);  _mpccontrol_com1(0,11) = _Lfootvz_kmp(walktime);  
  
  File_wl(file_mpcy,_mpccontrol_com1);
  
  return com_inte;
  
  
  

}



//////////////////////////////////////////////data save and read====================================================================
///DATA SAVING:modified=========================================================

void MPCClass::File_wl(std::string file_name, MatrixXd data_save)
{
	std::ofstream outfile(file_name.c_str()) ; // file name and the operation type. 
       
        for(int i=0; i<data_save.rows(); i++){
           for(int j=0; j<data_save.cols(); j++){
                 outfile << (double) data_save(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();
		
}


int MPCClass::read_scanf(const string &filename, const int &cols, vector<double *> &_vector)
{
	FILE *fp = fopen(filename.c_str(), "r");
	bool flag = true;
	int i = 0;
	if (!fp) 
	{ 
		//cout << "File open error!\n"; 
		return 0; 
	}
 
	while (flag)
	{
		double *rowArray = new double[cols];
 
		for (i = 0; i < cols; i++) 
		{
			if (EOF == fscanf(fp,"%lf", &rowArray[i]))
			{ 
				flag = false; 
				break; 
			}
		}
		if (cols == i) 
			_vector.push_back(rowArray);
	}
	fclose(fp);
	return 1;
}


MatrixXd MPCClass::rea_save(std::string file_name, int rowNum)
{
	//firstly, read the file in one column: read in arrow
	int columns = 1;
	vector<double *> output_vector;
    MatrixXd feedback_data;
	if (!read_scanf(file_name, columns, output_vector))
	{
		return feedback_data;
	}
	
	///// reset the dimension of the 2D vector
    int data_num = output_vector.size();
    
    int columnNum = round(data_num/rowNum);
	
    feedback_data.setZero(rowNum,columnNum);
    
	for (int i = 0; i < rowNum; i++)
	{
		for (int j = 0; j < columnNum; j++) 
		{   
            int ind = i*columnNum+j;
            feedback_data(i,j) = output_vector[ind][0];
		}
	}    
    
    return feedback_data;
    
}

void MPCClass::File_wlxx()
{
        
// 	CoMMM_ZMP_foot.setZero();
	CoMMM_ZMP_foot.block<1,_nsum>(0, 0) = _comx;
	CoMMM_ZMP_foot.block<1,_nsum>(1, 0) = _comy;	
	CoMMM_ZMP_foot.block<1,_nsum>(2, 0) = _comz;	
	CoMMM_ZMP_foot.block<1,_nsum>(3, 0) = _zmpx_real;	
	CoMMM_ZMP_foot.block<1,_nsum>(4, 0) = _zmpy_real;
	CoMMM_ZMP_foot.block<1,_nsum>(5, 0) = _thetax;	
	CoMMM_ZMP_foot.block<1,_nsum>(6, 0) = _thetay;	
	CoMMM_ZMP_foot.block<1,_nsum>(7, 0) = _torquex_real;
	CoMMM_ZMP_foot.block<1,_nsum>(8, 0) = _torquey_real;
	CoMMM_ZMP_foot.block<1,_nsum>(9, 0) = _footx_real_next1.transpose();	
	CoMMM_ZMP_foot.block<1,_nsum>(10, 0) = _footy_real_next1.transpose();	
	CoMMM_ZMP_foot.block<1,_nsum>(11, 0) = _footz_real_next.transpose();
	
	CoMMM_ZMP_foot.block<1,_nsum>(12, 0) = _Lfootx;	
	CoMMM_ZMP_foot.block<1,_nsum>(13, 0) = _Lfooty;	
	CoMMM_ZMP_foot.block<1,_nsum>(14, 0) = _Lfootz;
	CoMMM_ZMP_foot.block<1,_nsum>(15, 0) = _Rfootx;	
	CoMMM_ZMP_foot.block<1,_nsum>(16, 0) = _Rfooty;	
	CoMMM_ZMP_foot.block<1,_nsum>(17, 0) = _Rfootz;

	CoMMM_ZMP_foot.block<1,_nsum>(18, 0) = _comvx;
	CoMMM_ZMP_foot.block<1,_nsum>(19, 0) = _comax;
	
	CoMMM_ZMP_foot.block<1,_nsum>(20, 0) = _comvy;	
	CoMMM_ZMP_foot.block<1,_nsum>(21, 0) = _comay;
	
	CoMMM_ZMP_foot.block<1,_nsum>(22, 0) = _comvz;	
	CoMMM_ZMP_foot.block<1,_nsum>(23, 0) = _comaz;	

	CoMMM_ZMP_foot.block<1,_nsum>(24, 0) = _thetavx;
	CoMMM_ZMP_foot.block<1,_nsum>(25, 0) = _thetaax;
	
	CoMMM_ZMP_foot.block<1,_nsum>(26, 0) = _thetavy;	
	CoMMM_ZMP_foot.block<1,_nsum>(27, 0) = _thetaay;

	std::string fileName1 = "NMPC_optimal_trajectory.txt" ;
	std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.        
	
        for(int i=0; i<CoMMM_ZMP_foot.rows(); i++){
           for(int j=0; j<CoMMM_ZMP_foot.cols(); j++){
                 outfile1 << (double) CoMMM_ZMP_foot(i,j) << " " ; 
           }
           outfile1 << std::endl;       // a   newline
        }
        outfile1.close();	
	
	
	
}


