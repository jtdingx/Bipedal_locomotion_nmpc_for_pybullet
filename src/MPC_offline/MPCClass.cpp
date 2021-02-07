/*****************************************************************************
MPCClass.cpp

Description:    source file of MPCClass

@Version:   1.0
@Author:    Chengxu Zhou (zhouchengxu@gmail.com)
@Release:   Thu 02 Aug 2018 12:33:23 PM CEST
@Update:    Thu 02 Aug 2018 12:33:19 PM CEST
*****************************************************************************/
#include "MPC/MPCClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include <vector>

using namespace Eigen;
using namespace std;



MPCClass::MPCClass()                    ///declaration function
	: QPBaseClass()
{
  
}


void MPCClass::CoM_foot_trajection_generation()
{
//================= offline block=============================
  //================= offline block=============================
        // ==step parameters initialize==: given by the inputs

	//===step location reconfiguration
	_steplength(4) = 0.7;
	_steplength(5) = 0.7;
	_steplength(6) = 0.7;
	_stepheight.setConstant(_footstepsnumber,0.1);
	_stepheight(3) = 0;
	_stepheight(4) = 0;
	_stepheight(5) = -0.1;	
	_stepheight(6) = -0.1;
	_stepheight(7) = -0.1;
	_stepheight(8) = 0;		
	_stepheight(9) = 0;
	_stepheight(10) = 0;
	_stepheight(11) = 0;	
// 	_footsteps
 	// ==step loctions setup==
        _footx_ref.setZero(_footstepsnumber);
	_footy_ref.setZero(_footstepsnumber);
	_footz_ref.setZero(_footstepsnumber);
		
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
	  _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);   
	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
	}

	// == step cycle setup
	_ts.setConstant(_footstepsnumber,0.8);
// 	_td.setConstant(_footstepsnumber,0);
	_td = 0.2*_ts;
	_tx.setZero(_footstepsnumber);
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	}	
	
	
        _dt =0.05;
	_t.setLinSpaced(floor(_tx(_footstepsnumber-1)/_dt),_dt,_tx(_footstepsnumber-1));
	_nsum = _t.size();
	_nT = floor(_ts(0)/_dt);
	
	
	
	_zmpx_real.setZero(_nsum); _zmpy_real.setZero(_nsum);
	_comx.setZero(_nsum); _comvx.setZero(_nsum); _comax.setZero(_nsum);
	_comy.setZero(_nsum); _comvy.setZero(_nsum); _comay.setZero(_nsum);
	_comz.setZero(_nsum); _comvz.setZero(_nsum); _comaz.setZero(_nsum);	
	_thetax.setZero(_nsum); _thetavx.setZero(_nsum); _thetaax.setZero(_nsum);
	_thetay.setZero(_nsum); _thetavy.setZero(_nsum); _thetaay.setZero(_nsum);	
	_torquex_real.setZero(_nsum); _torquey_real.setZero(_nsum);


	 _CoM_position_optimal.setZero(3,_nsum);
	 _torso_angle_optimal.setZero(2,_nsum);
	 _L_foot_optition_optimal.setZero(3,_nsum);
	 _R_foot_optition_optimal.setZero(3,_nsum);
	 _foot_location_optimal.setZero(3,_footstepsnumber);
	 
	
	_xk.setZero(3,_nsum); _yk.setZero(3,_nsum); _zk.setZero(3,_nsum);
	_thetaxk.setZero(3,_nsum); _thetayk.setZero(3,_nsum);
	_x_vacc_k.setZero(_nsum); _y_vacc_k.setZero(_nsum); _z_vacc_k.setZero(_nsum); 
	_thetax_vacc_k.setZero(_nsum); _thetay_vacc_k.setZero(_nsum); 
	
        // ==initial parameters for MPC==
        _hcom = 0.4668;

	_g = 9.8;
	_ggg.setConstant(1, 9.8);
	_nh = floor(1.051/_dt);
// 	_nh = floor(0.951/_dt);	
	
	_Hcom1.setConstant(_nh,1,_hcom);	
	_a.setZero(3,3);
	_a << 1, _dt, pow(_dt,2)/2,    
	      0,   1,            _dt,
	      0,   0,              1;
	_b.setZero(3,1);
	_b << pow(_dt,3)/6,    
	      pow(_dt,2)/2,
	               _dt;	
	_c.setZero(1,3);
	_c << 1,0,(-1)*_hcom /_g;	
	
	_cp.setZero(1,3);
	_cp(0,0) = 1;
	_cv.setZero(1,3);
	_cv(0,1) = 1;
	_ca.setZero(1,3);
	_ca(0,2) = 1;	
		

	//vertical height constraints
	_z_max.setConstant(_nsum,0.1);
	_z_min.setConstant(_nsum,-0.2);	
	
        //footz refer: height of step
	_Zsc.setZero(_nsum,1);		

	int xyz1 = 0;  //flag for find function 
	int xyz2 = 1;
	

	
  	for (int i = 0; i < _nsum-1; i++) {
	  double goalvari = i*_dt+_dt;	  
	  int goaperiod;
	  goaperiod = Indexfind(goalvari,_tx,xyz1);
	  _Zsc(i,0) = _footz_ref(goaperiod);   
	}		

        _yk.topRows(1).setConstant(_footy_ref(0)); 
	_zk.topRows(1).setConstant(_hcom);
	

	//predictive model
	_pps.setZero(_nh,3); _ppu.setZero(_nh,_nh);
	_pvs.setZero(_nh,3); _pvu.setZero(_nh,_nh);
	_pas.setZero(_nh,3); _pau.setZero(_nh,_nh);

        _pps = Matrix_ps(_a,_nh,_cp);
	_pvs = Matrix_ps(_a,_nh,_cv);
	_pas = Matrix_ps(_a,_nh,_ca);

	_ppu = Matrix_pu(_a,_b,_nh,_cp);
	_pvu = Matrix_pu(_a,_b,_nh,_cv);
	_pau = Matrix_pu(_a,_b,_nh,_ca);
	
//         cout << "_ppu"<<_ppu<<endl;
// 	cout << _pvu<<endl;
// 	cout << _pau<<endl;	
//         cout << _pps<<endl;
// 	cout << _pvs<<endl;
// 	cout << _pas<<endl;	
// 	cout << _Zsc<<endl;
// 	cout << _a<<endl;
// 	cout << _b<<endl;
// 	cout << _c<<endl;
// 	cout <<"_cp"<< _cp<<endl;	
// 	cout << _nsum<<endl;
// 	cout << _z_max<<endl;

	
	_footx_real.setZero(_footstepsnumber);  _footy_real.setZero(_footstepsnumber); _footz_real.setZero(_footstepsnumber);
	
	_footxyz_real.setZero(3,_footstepsnumber);
// 	cout <<"_footxyz_real:"<< _footxyz_real<<endl;
// 	cout <<"_nT:"<< _nT<<endl;
// 	cout <<"_td:"<< _td<<endl;
// 	cout <<"_footx_real:"<< _footx_real<<endl;
	
	_footx_real_next.setZero(_nsum);  _footy_real_next.setZero(_nsum); _footz_real_next.setZero(_nsum);
	
	_Lfootx.setZero(_nsum); _Lfooty.setConstant(_nsum,_stepwidth(0));_Lfootz.setZero(_nsum); _Lfootvx.setZero(_nsum); _Lfootvy.setZero(_nsum);_Lfootvz.setZero(_nsum); 
	_Lfootax.setZero(_nsum); _Lfootay.setZero(_nsum);_Lfootaz.setZero(_nsum);
	_Rfootx.setZero(_nsum); _Rfooty.setConstant(_nsum,-_stepwidth(0));_Rfootz.setZero(_nsum); _Rfootvx.setZero(_nsum); _Rfootvy.setZero(_nsum);_Rfootvz.setZero(_nsum); 
	_Rfootax.setZero(_nsum); _Rfootay.setZero(_nsum);_Rfootaz.setZero(_nsum);
	_ry_left_right = 0;
	
	
	
	
	_footx_max.setConstant(1, 0.5);
	_footx_min.setConstant(1, 0);
	_footy_max.setConstant(1, 0.4); 
	_footy_min.setConstant(1, 0.07);
	
	_fx.setZero(1);
	_fy.setZero(1);
	
	_mass = 100; _rad = 0.4; _j_ini = _mass* pow(_rad,2);
	
	/// zmp-constraints
	_zmpx_ub.setConstant(_nsum,0.05);  _zmpx_lb.setConstant(_nsum,-0.03);
	_zmpy_ub.setConstant(_nsum,0.03); _zmpy_lb.setConstant(_nsum,-0.03);
	
	// com-support range
	_comx_max.setConstant(1,0.06);
	_comx_min.setConstant(1,-0.04);  
	_comy_max.setConstant(1,0.6);  
	_comy_min.setConstant(1,0.02);
	
	// angle range
	_thetax_max.setConstant(1,15*M_PI/180);  
	_thetax_min.setConstant(1,-10*M_PI/180);
	_thetay_max.setConstant(1,10*M_PI/180);  
	_thetay_min.setConstant(1,-10*M_PI/180);
	
	// torque range
	_torquex_max.setConstant(1,160/_j_ini); 
	_torquex_min.setConstant(1,-160/_j_ini);
	_torquey_max.setConstant(1,160/_j_ini);  
	_torquey_min.setConstant(1,-160/_j_ini);	

	// swing foot velocity constraints	
	_footx_vmax.setConstant(1,3);
	_footx_vmin.setConstant(1,-1);
	_footy_vmax.setConstant(1,1); 
	_footy_vmin.setConstant(1,-0.5);	
	
	
///===========initiallize: preparation for MPC solution
	_nstep = 2;
	_Nt = 5*_nh + 3*_nstep;	
	
	// sulotion preparation		
	_V_optimal.setZero(_Nt, _nsum);	
	_Lx_ref.setZero(_nstep); _Ly_ref.setZero(_nstep); _Lz_ref.setZero(_nstep);
	_V_ini.setZero(_Nt,1);
	_comx_center_ref.setZero(_nh,1);
	_comy_center_ref.setZero(_nh,1);
	_comz_center_ref.setZero(_nh,1);
	
	_thetax_center_ref.setZero(_nh,1); 
	_thetay_center_ref.setZero(_nh,1);	
	
        
// 	 store n_vis
	_flag.setZero(_nsum);
	
// 	parameters for objective function======================	
	 _Rx = 1;     _Ry = 1;    _Rz = 1;
	_alphax = 10; _alphay = 10;  _alphaz = 10; 
	_beltax = 500000;  _beltay = 100000;  _beltaz = 20000000;
	_gamax = 10000000;   _gamay = 10000000;  _gamaz = 200;
	_Rthetax = 1; _Rthetay = 1;
	_alphathetax = 10; _alphathetay = 10;
	_beltathetax = 200000; _beltathetay = 200000;

	// time cost consumption
	_tcpu.setZero(_nsum);
	_tcpu_iterative.setZero(_nsum);
	_tcpu_prepara.setZero(_nsum);
	_tcpu_prepara2.setZero(_nsum);
	_tcpu_qp.setZero(_nsum);

	
	_pvu_2 = _pvu.transpose()*_pvu;
	_ppu_2 = _ppu.transpose()*_ppu;

	
	_loop = 2;
	

	// test run time
	clock_t t_start,t_start1, t_start2, t_start3, t_start4,t_finish,t_finish1;
	
	
	
///////////////////////////////////////////////////////////////
//////////// next code block just run once	
	  A_unit.setIdentity(_nh,_nh);
	  C_unit.setIdentity(_nstep,_nstep);		
	

	  // optimization objective function 	
	  _WX.setZero(_nh,_nh);
	  _WY.setZero(_nh,_nh);
	  _WZ.setZero(_nh,_nh);
	  _WthetaX.setZero(_nh,_nh);
	  _WthetaY.setZero(_nh,_nh);
	  _PHIX.setZero(_nstep,_nstep);
	  _PHIY.setZero(_nstep,_nstep);
	  _PHIZ.setZero(_nstep,_nstep);
	  _Q_goal.setZero(_Nt,_Nt);
	  _q_goal.setZero(_Nt,1);
	  _Q_goal1.setZero(_Nt,_Nt);
	  _q_goal1.setZero(_Nt,1);	
	  
	  _WX = _Rx*0.5 * A_unit + _alphax*0.5 * _pvu_2 + _beltax*0.5 * _ppu_2;	  
	  _WY = _Ry/2 * A_unit + _alphay/2 * _pvu_2 + _beltay/2 * _ppu_2;
	  _WZ = _Rz/2 * A_unit + _alphaz/2 * _pvu_2 + _beltaz/2 * _ppu_2;  
	  _WthetaX = _Rthetax/2 * A_unit + _alphathetax/2 * _pvu_2 + _beltathetax/2 * _ppu_2;
	  _WthetaY = _Rthetay/2 * A_unit + _alphathetay/2 * _pvu_2 + _beltathetay/2 * _ppu_2;
	  _PHIX  = _gamax/2 * C_unit;
	  _PHIY  = _gamay/2 * C_unit;
	  _PHIZ  = _gamaz/2 * C_unit;
		  
	  _Q_goal.block< Dynamic, Dynamic>(0, 0,_nh, _nh) = _WX;
	  _Q_goal.block< Dynamic, Dynamic>(_nh, _nh,_nh, _nh) = _WY;
	  _Q_goal.block< Dynamic, Dynamic>(2*_nh, 2*_nh,_nh, _nh) = _WZ;
	  _Q_goal.block< Dynamic, Dynamic>(3*_nh, 3*_nh,_nh, _nh) = _WthetaX;
	  _Q_goal.block< Dynamic, Dynamic>(4*_nh, 4*_nh,_nh, _nh) = _WthetaY;
	  _Q_goal.block< Dynamic, Dynamic>(5*_nh, 5*_nh,_nstep,_nstep) = _PHIX;
	  _Q_goal.block< Dynamic, Dynamic>(5*_nh+_nstep, 5*_nh+_nstep,_nstep, _nstep) = _PHIY;
	  _Q_goal.block< Dynamic, Dynamic>(5*_nh+2*_nstep, 5*_nh+2*_nstep,_nstep, _nstep) = _PHIZ;	  
	
	  _Q_goal1 = 2 * _Q_goal;	
	
	
	  // constraints
	  _Sjx.setZero(_nh,_Nt);
	  _Sjy.setZero(_nh,_Nt);
	  _Sjz.setZero(_nh,_Nt);
	  _Sjthetax.setZero(_nh,_Nt);
	  _Sjthetay.setZero(_nh,_Nt);
	  _Sjx.block< Dynamic, Dynamic>(0, 0,_nh, _nh) = A_unit;
	  _Sjy.block< Dynamic, Dynamic>(0, _nh,_nh, _nh) = A_unit;
	  _Sjz.block< Dynamic, Dynamic>(0, 2*_nh,_nh, _nh) = A_unit;	
	  _Sjthetax.block< Dynamic, Dynamic>(0, 3*_nh,_nh, _nh) = A_unit;
	  _Sjthetay.block< Dynamic, Dynamic>(0, 4*_nh,_nh, _nh) = A_unit;
	  
	  // ZMP boundary preparation
	  _H_q_upx.setZero(_nh,_Nt);
	  _F_zmp_upx.setZero(_nh,1);
	  _H_q_lowx.setZero(_nh,_Nt);
	  _F_zmp_lowx.setZero(_nh,1);
	  _H_q_upy.setZero(_nh,_Nt);
	  _F_zmp_upy.setZero(_nh,1);
	  _H_q_lowy.setZero(_nh,_Nt);
	  _F_zmp_lowy.setZero(_nh,1);

	  _phi_i_x_up.setZero(_Nt,_Nt);
	  _p_i_x_t_up.setZero(_Nt,_nh);
	  _del_i_x_up.setZero(1,_nh);
	  _phi_i_x_low.setZero(_Nt,_Nt);
	  _p_i_x_t_low.setZero(_Nt,_nh);
	  _del_i_x_low.setZero(1,_nh);
	  _phi_i_y_up.setZero(_Nt,_Nt);
	  _p_i_y_t_up.setZero(_Nt,_nh);
	  _del_i_y_up.setZero(1,_nh);
	  _phi_i_y_low.setZero(_Nt,_Nt);
	  _p_i_y_t_low.setZero(_Nt,_nh);
	  _del_i_y_low.setZero(1,_nh);	  
	  
	  // angle boundary preparation
	  _q_upx.setZero(_nh,_Nt);
	  _qq_upx.setZero(_nh,1);
	  _q_lowx.setZero(_nh,_Nt);
	  _qq_lowx.setZero(_nh,1);
	  _q_upy.setZero(_nh,_Nt);
	  _qq_upy.setZero(_nh,1);
	  _q_lowy.setZero(_nh,_Nt);
	  _qq_lowy.setZero(_nh,1);
	  
	  _qq1_upx.setZero(_nh,1);
	  _qq1_lowx.setZero(_nh,1);
	  _qq1_upy.setZero(_nh,1);
	  _qq1_lowy.setZero(_nh,1);	  

	  // torque bondary preparation
	  _t_upx.setZero(_nh,_Nt);
	  _tt_upx.setZero(_nh,1);
	  _t_lowx.setZero(_nh,_Nt);
	  _tt_lowx.setZero(_nh,1);
	  _t_upy.setZero(_nh,_Nt);
	  _tt_upy.setZero(_nh,1);
	  _t_lowy.setZero(_nh,_Nt);
	  _tt_lowy.setZero(_nh,1);
	  
	  _tt1_upx.setZero(_nh,1);
	  _tt1_lowx.setZero(_nh,1);
	  _tt1_upy.setZero(_nh,1);
	  _tt1_lowy.setZero(_nh,1);
	  
	  // CoM height boundary preparation
	  _H_h_upz.setZero(_nh,_Nt);
	  _F_h_upz.setZero(_nh,1);
	  _H_h_lowz.setZero(_nh,_Nt);
	  _F_h_lowz.setZero(_nh,1);
	  _delta_footz_up.setZero(_nh,1);
	  _delta_footz_low.setZero(_nh,1);

	  // CoM height acceleration boundary preparation
	  _H_hacc_lowz.setZero(_nh,_Nt);
	  _F_hacc_lowz.setZero(_nh,1);
	  _delta_footzacc_up.setZero(_nh,1);	  

	  
	  //swing foot velocity constraints
	  _Footvx_max.setZero(1,_Nt);
	  _Footvx_min.setZero(1,_Nt);
	  _Footvy_max.setZero(1,_Nt);
	  _Footvy_min.setZero(1,_Nt);
	  _footubxv.setZero(1,1);
	  _footlbxv.setZero(1,1);
	  _footubyv.setZero(1,1);
	  _footlbyv.setZero(1,1);
	  
	  // foot vertical location-equality constraints
	  _H_q_footz.setZero(1, _Nt);
	  _F_footz.setZero(1, 1);
	  
	  // foot location constraints
	  _Sfoot.setZero(1,2);
	  _Sfoot(0,0) = -1;
	  _Sfoot(0,1) = 1;
	  
	  _S1.setZero(1,_nh);
	  _S1(0,0) = 1;	  
	  
	// offline calulated the ZMP constraints coefficient
	vector <Eigen::MatrixXd> ZMPx_constraints_offfline(_nh);
	vector <Eigen::MatrixXd> ZMPy_constraints_offfline(_nh);
	
	vector <Eigen::MatrixXd> ZMPx_constraints_half(_nh);
	vector <Eigen::MatrixXd> ZMPy_constraints_half(_nh);
	
	
	
	for(int jxx=1; jxx<=_nh; jxx++)
	{
	  _Si.setZero(1,_nh);
	  _Si(0,jxx-1) = 1;
	  // ZMP constraints
	  // x-ZMP upper boundary	      
		 
         ZMPx_constraints_offfline[jxx-1] = (_Si * _ppu * _Sjx).transpose() * _Si * _pau * _Sjz - (_Si * _pau * _Sjx).transpose() * _Si * _ppu * _Sjz;
	 ZMPx_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _pau * _Sjz;
	  
	  // y-ZMP upper boundary
	  Eigen::MatrixXd _phi_i_y_up1 = (_Si * _ppu * _Sjy).transpose() * _Si * _pau * _Sjz - (_Si * _pau * _Sjy).transpose() * _Si * _ppu * _Sjz - (_Si * _VV_i* _Sfy).transpose() * _Si * _pau * _Sjz;	      
	  _phi_i_y_up = _mass * (_phi_i_y_up1 + _phi_i_y_up1.transpose())/2;	
	  
         ZMPy_constraints_offfline[jxx-1] = (_Si * _ppu * _Sjy).transpose() * _Si * _pau * _Sjz - (_Si * _pau * _Sjy).transpose() * _Si * _ppu * _Sjz;
	 ZMPy_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _pau * _Sjz;
	      
	}

 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
// 	================ iterative calulcation: predicitve control_tracking with time-varying height+angular momentum
////////////////////////////======================//////////////////////////////////////	
// predictive model control_tracking with time_varying height	
// _nsum -_nh
  	for (int i = 1; i <_nsum -_nh; i++) {	
	  
	  /// run once
	  ///////////////////////////////////////////////////////
	  ////////////////////////////////////// time_clock0////////////////////////
	  t_start = clock();
	  
	  _bjxx = Indexfind(i*_dt,_tx,xyz1)+1;  //coincidence with matlab 
	  
	  // com_center_ref = ZMP_center_ref = v_i*f + V_i*L_ref
	  //solve the following steps: 1.5s may couve 2 or three  steps, so  one/two following steps
	  
	  _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
	  
	  _bjx1 = Indexfind(_t_f(0),_tx,xyz1)+1;
	  _bjx2 = Indexfind(_t_f(_nh-1),_tx,xyz1)+1;
	  
	  _mx = _bjx2 - _bjx1 +1;
	  
	  _bjx.setLinSpaced(_mx, _bjx1, _bjx2);
	  _tnx.setZero(_mx);

	  for (int j=1;j<_mx; j++)
	  {
	    _tnx(j-1) = Indexfind(_tx(_bjx(j)-1),_t_f,xyz2);    
	  }
	  
	  _v_i.setZero(_nh,1);
	  _VV_i.setZero(_nh, _nstep);
	  // be careful that the position is from 0;;;;;         
	  if (fabs(_tnx(0) - _nT) <=0.00001)
	  {
            _n_vis =2;
	    for (int jjj = 1; jjj <= _mx; jjj++)
	    {
	      if (jjj == 1)
	      {
		xxx = _tnx(0);
		_VV_i.block< Dynamic,1>(0, 0, xxx, 1).setOnes();
	      }
	      else
	      {	
		xxx1 = _nh-_tnx(0);
		_VV_i.block< Dynamic,1>(xxx+1, 1, xxx1, 1).setOnes();		
	      }
	    }	    
	  }
	  else
	  {	
	    xxx = _tnx(0);
	    _v_i.block< Dynamic,1>(0, 0, xxx, 1).setOnes();

	    if (abs(_mx - 2) <=0.00001)
	    {
	      _n_vis = 1;
	      _VV_i.block< Dynamic,1>(xxx, 0, _nh -xxx, 1).setOnes();
	    }
	    else
	    {
	      _n_vis = 2;
	      xxx2 = _nh -_tnx(1);
	      _VV_i.block< Dynamic,1>(xxx, 0, _nT, 1).setOnes();
	      _VV_i.block< Dynamic,1>(_tnx(1), 1, xxx2, 1).setOnes();	      	      
	    }
	  }
  	  
	  
	  
	  if (_n_vis ==1)
	  {
	    _Lx_ref(0) = _footx_ref(_bjx2-1);
	    _Ly_ref(0) = _footy_ref(_bjx2-1);
	    _Lz_ref(0) = _footz_ref(_bjx2-1);
	    _Lx_ref(1) = 0;
	    _Ly_ref(1) = 0;
	    _Lz_ref(1) = 0;	    
	  }
	  else
	  {
	    _Lx_ref(0) = _footx_ref(_bjx2-2);
	    _Ly_ref(0) = _footy_ref(_bjx2-2);
	    _Lz_ref(0) = _footz_ref(_bjx2-2);
	    _Lx_ref(1) = _footx_ref(_bjx2-1);
	    _Ly_ref(1) = _footy_ref(_bjx2-1);
	    _Lz_ref(1) = _footz_ref(_bjx2-1);	    
	  }
 
  
	  _flag(i-1,0)= _n_vis;

////////////////////////////////////////////////////////////////////////////////////	  
//============================================================//	  
	  // hot start
	  /*_V_ini.setZero(_Nt,1);*/	 
	  if (i==1)
	  {
	    _V_ini(5*_nh,0) = _footx_ref(1);
	    _V_ini(5*_nh+1,0) = _footy_ref(1);	    
	  }
	  else
	  {
	    _V_ini.topRows(5*_nh) = _V_optimal.block< Dynamic, 1>(0, i-2, 5*_nh, 1);
	    if (_n_vis > _flag(i-2))
	    { 
	      _V_ini(_Nt -1-1,0) = _V_optimal(5*_nh+6-1, i-2);
	      _V_ini(_Nt -3-1,0) = _V_optimal(5*_nh+6-2-1, i-2);
	      _V_ini(_Nt -5-1,0) = _V_optimal(5*_nh+6-4-1, i-2); 
	    }
	    else
	    {
	      if (_n_vis < _flag(i-2))
	      { 
		_V_ini(_Nt-1,0) = _V_optimal(5*_nh+6-1, i-2);
		_V_ini(_Nt -1-1,0) = _V_optimal(5*_nh+6-2-1, i-2);
		_V_ini(_Nt -2-1,0) = _V_optimal(5*_nh+6-4-1, i-2); 
	      }	   
	      else
	      {
		if (_n_vis ==1)
		{ 
		  _V_ini(_Nt-1,0) = _V_optimal(5*_nh+6-1, i-2);
		  _V_ini(_Nt -1-1,0) = _V_optimal(5*_nh+6-2-1, i-2);
		  _V_ini(_Nt -2-1,0) = _V_optimal(5*_nh+6-4-1, i-2); 
		}
		else
		{
		  _V_ini.bottomRows(6) = _V_optimal.block< Dynamic, 1>(5*_nh+6-6, i-2, 6, 1);
		}
	      }
	    }
	  }
 
	  // current foot location
	  _fx = _footx_real.row(_bjxx-1);
	  _fy = _footy_real.row(_bjxx-1);	  
	 // com_center_ref
	  _comx_center_ref = _v_i*_fx + _VV_i*_Lx_ref;
	  _comy_center_ref = _v_i*_fy + _VV_i*_Ly_ref;
 	  _comz_center_ref = _Zsc.block< Dynamic,1>(i, 0,_nh, 1) + _Hcom1;


	  B_unit.setIdentity(_n_vis,_n_vis);

	  // foot location constraints: be careful that the step number is change: so should be intialized in each whole loop
	  _H_q_footx_up.setZero(_nstep,_Nt);
	  _F_foot_upx.setZero(_nstep,1);
	  _H_q_footx_low.setZero(_nstep,_Nt);
	  _F_foot_lowx.setZero(_nstep,1);
	  _H_q_footy_up.setZero(_nstep,_Nt);
	  _F_foot_upy.setZero(_nstep,1);
	  _H_q_footy_low.setZero(_nstep,_Nt);
	  _F_foot_lowy.setZero(_nstep,1);
	  

	  // boundary initialzation
	  _Sfx.setZero(_nstep,_Nt);
	  _Sfy.setZero(_nstep,_Nt);
	  _Sfz.setZero(_nstep,_Nt);
	  _Sfx.block< Dynamic, Dynamic>(0, 5*_nh,_n_vis,_n_vis) = B_unit;
	  _Sfy.block< Dynamic, Dynamic>(0, 5*_nh+_nstep,_n_vis,_n_vis) = B_unit;	
	  _Sfz.block< Dynamic, Dynamic>(0, 5*_nh+2*_nstep,_n_vis,_n_vis) = B_unit;	 
	  

	  //SQP MOdels	 
	    _q_goal.block< Dynamic, 1>(0, 0,_nh, 1) = _alphax * _pvu.transpose() * _pvs * _xk.col(i-1) + _beltax * _ppu.transpose() * _pps * _xk.col(i-1) - _beltax * _ppu.transpose() * _comx_center_ref;
	    _q_goal.block< Dynamic, 1>(_nh, 0,_nh, 1) = _alphay * _pvu.transpose() * _pvs * _yk.col(i-1) + _beltay * _ppu.transpose() * _pps * _yk.col(i-1) - _beltay * _ppu.transpose() * _comy_center_ref;
	    _q_goal.block< Dynamic, 1>(2*_nh, 0,_nh, 1) = _alphaz * _pvu.transpose() * _pvs * _zk.col(i-1) + _beltaz * _ppu.transpose() * _pps * _zk.col(i-1) - _beltaz * _ppu.transpose() * _comz_center_ref;
	    _q_goal.block< Dynamic, 1>(3*_nh, 0,_nh, 1) = _alphathetax * _pvu.transpose() * _pvs * _thetaxk.col(i-1) + _beltathetax * _ppu.transpose() * _pps * _thetaxk.col(i-1) - _beltathetax * _ppu.transpose() * _thetax_center_ref;
	    _q_goal.block< Dynamic, 1>(4*_nh, 0,_nh, 1) = _alphathetay * _pvu.transpose() * _pvs * _thetayk.col(i-1) + _beltathetay * _ppu.transpose() * _pps * _thetayk.col(i-1) - _beltathetay * _ppu.transpose() * _thetay_center_ref;
	    _q_goal.block< Dynamic, 1>(5*_nh, 0,_nstep, 1) = -_gamax * _Lx_ref;
	    _q_goal.block< Dynamic, 1>(5*_nh+_nstep, 0,_nstep, 1) = -_gamay * _Ly_ref;
	    _q_goal.block< Dynamic, 1>(5*_nh+2*_nstep, 0,_nstep, 1) = -_gamaz * _Lz_ref;
	    	    	 
	 	    

           ///// the following block only run once in each loop   
	    for(int jxx=1; jxx<=_nh; jxx++)
	    {
	      _Si.setZero(1,_nh);
	      _Si(0,jxx-1) = 1;
	      // ZMP constraints
	      // x-ZMP upper boundary

	      Eigen::MatrixXd _p_i_x_t_up1 = ((_Si * _pps * _xk.col(i-1)).transpose() *_Si*_pau*_Sjz + (_Si*_pas*_zk.col(i-1)).transpose()*_Si*_ppu*_Sjx + _g*_Si*_ppu*_Sjx - ((_Si * _pps * _zk.col(i-1)).transpose() *_Si*_pau*_Sjx + _Si * _pas * _xk.col(i-1)* _Si* _ppu* _Sjz) + _Zsc.row(i+jxx-1)*_Si*_pau*_Sjx - ((_Si * _pas * _zk.col(i-1)).transpose() *_Si*_VV_i*_Sfx + (_Si * _v_i * _fx).transpose() *_Si*_pau*_Sjz) - _g*_Si*_VV_i*_Sfx - _zmpx_ub.row(i+jxx-1)*_Si*_pau*_Sjz).transpose();                                         
	      _p_i_x_t_up.col(jxx-1) = _mass * _p_i_x_t_up1 - (_j_ini * _Si*_pau * _Sjthetay).transpose();
	      
	      Eigen::MatrixXd _del_i_x_up1 = (_Si * _pps * _xk.col(i-1)).transpose() *_Si*_pas*_zk.col(i-1) + _g*_Si * _pps * _xk.col(i-1) - (_Si * _pas * _xk.col(i-1)).transpose() *_Si*_pps*_zk.col(i-1) + (_Si * _pas * _xk.col(i-1)).transpose() *_Zsc.row(i+jxx-1) - (_Si * _v_i * _fx).transpose() *_Si*_pas*_zk.col(i-1) - _g *_Si * _v_i * _fx - _zmpx_ub.row(i+jxx-1)*_Si*_pas*_zk.col(i-1) - _g *_zmpx_ub.row(i+jxx-1);
	      _del_i_x_up.col(jxx-1) = _mass * _del_i_x_up1 - _j_ini * _Si*_pas * _thetayk.col(i-1);


	      // x-ZMP low boundary
// 	      _phi_i_x_low = _phi_i_x_up;
	      _p_i_x_t_low.col(jxx-1) = (_p_i_x_t_up.col(jxx-1).transpose() + _mass * _zmpx_ub.row(i+jxx-1)*_Si*_pau*_Sjz - _mass * _zmpx_lb.row(i+jxx-1)*_Si*_pau*_Sjz).transpose();	      
	      _del_i_x_low.col(jxx-1) = _del_i_x_up.col(jxx-1) +_mass*_zmpx_ub.row(i+jxx-1)*_Si*_pas*_zk.col(i-1)+  _mass * _g*_zmpx_ub.row(i+jxx-1) - _mass*_zmpx_lb.row(i+jxx-1)*_Si*_pas*_zk.col(i-1)-_mass * _g*_zmpx_lb.row(i+jxx-1);
      	      
	      // y-ZMP upper boundary
   
	      Eigen::MatrixXd _p_i_y_t_up1 = ((_Si * _pps * _yk.col(i-1)).transpose() *_Si*_pau*_Sjz + (_Si*_pas*_zk.col(i-1)).transpose()*_Si*_ppu*_Sjy + _g*_Si*_ppu*_Sjy - ((_Si * _pps * _zk.col(i-1)).transpose() *_Si*_pau*_Sjy + _Si * _pas * _yk.col(i-1)* _Si* _ppu* _Sjz) + _Zsc.row(i+jxx-1)*_Si*_pau*_Sjy - ((_Si * _pas * _zk.col(i-1)).transpose() *_Si*_VV_i*_Sfy + (_Si * _v_i * _fy).transpose() *_Si*_pau*_Sjz) - _g*_Si*_VV_i*_Sfy - _zmpy_ub.row(i+jxx-1)*_Si*_pau*_Sjz).transpose();                                         
	      _p_i_y_t_up.col(jxx-1) = _mass * _p_i_y_t_up1 + (_j_ini * _Si*_pau * _Sjthetax).transpose();
	      
	      Eigen::MatrixXd _del_i_y_up1 = (_Si * _pps * _yk.col(i-1)).transpose() *_Si*_pas*_zk.col(i-1) + _g*_Si * _pps * _yk.col(i-1) - (_Si * _pas * _yk.col(i-1)).transpose() *_Si*_pps*_zk.col(i-1) + (_Si * _pas * _yk.col(i-1)).transpose() *_Zsc.row(i+jxx-1) - (_Si * _v_i * _fy).transpose() *_Si*_pas*_zk.col(i-1) - _g *_Si * _v_i * _fy - _zmpy_ub.row(i+jxx-1)*_Si*_pas*_zk.col(i-1) - _g *_zmpy_ub.row(i+jxx-1);
	      _del_i_y_up.col(jxx-1) = _mass * _del_i_y_up1 + _j_ini * _Si*_pas * _thetaxk.col(i-1);	      
	     
	      // y-ZMP low boundary
 	     _phi_i_y_low = _phi_i_y_up;  
	      _p_i_y_t_low.col(jxx-1) = (_p_i_y_t_up.col(jxx-1).transpose() + _mass * _zmpy_ub.row(i+jxx-1)*_Si*_pau*_Sjz - _mass * _zmpy_lb.row(i+jxx-1)*_Si*_pau*_Sjz).transpose();	      
	      _del_i_y_low.col(jxx-1) = _del_i_y_up.col(jxx-1) +_mass*_zmpy_ub.row(i+jxx-1)*_Si*_pas*_zk.col(i-1)+  _mass * _g*_zmpy_ub.row(i+jxx-1) - _mass*_zmpy_lb.row(i+jxx-1)*_Si*_pas*_zk.col(i-1)-_mass * _g*_zmpy_lb.row(i+jxx-1);	      	      	     

	      //angle range constraints
	      _q_upx.row(jxx-1) = _Si* _ppu* _Sjthetax;
	      _q_lowx.row(jxx-1) = -_q_upx.row(jxx-1);	         

	      _qq1_upx.row(jxx-1) = _Si* _pps* _thetaxk.col(i-1)-_thetax_max;
	      
	      _q_upy.row(jxx-1) = _Si* _ppu* _Sjthetay;
	      _q_lowy.row(jxx-1) = -_q_upy.row(jxx-1);	
	      
	      _qq1_upy.row(jxx-1) = _Si* _pps* _thetayk.col(i-1)-_thetay_max;
   

	      //torque range constraints
	      _t_upx.row(jxx-1) = _Si* _pau* _Sjthetax;
	      _t_lowx.row(jxx-1) = -_t_upx.row(jxx-1);
	      
	      _tt1_upx.row(jxx-1) = _Si* _pas* _thetaxk.col(i-1)-_torquex_max;
	      
	      _t_upy.row(jxx-1) = _Si* _pau* _Sjthetay;
	      _t_lowy.row(jxx-1) = -_t_upy.row(jxx-1);	 
	      
	      _tt1_upy.row(jxx-1) = _Si* _pas* _thetayk.col(i-1)-_torquey_max;
	      
	      
	      // body height constraints
	      _H_h_upz.row(jxx-1) = _Si* _ppu* _Sjz;
	      _H_h_lowz.row(jxx-1) = -_Si* _ppu* _Sjz;   	
	      _delta_footz_up.row(jxx-1) = _Si*_pps*_zk.col(i-1) - _comz_center_ref.row(jxx-1) - _z_max.row(i+jxx-1);
	      
	      // body height acceleration constraints	      
	      _H_hacc_lowz.row(jxx-1) = -_Si* _pau* _Sjz;   
	      _delta_footzacc_up.row(jxx-1) = _Si*_pas*_zk.col(i-1) + _ggg;
	    }
	    	   	    
  
///       only one-time caluation	  
	  Eigen::MatrixXd ZMPx_constraints_half2 = (_VV_i* _Sfx).transpose();
	  Eigen::MatrixXd ZMPy_constraints_half2 = (_VV_i* _Sfy).transpose();
	  
	  vector <Eigen::MatrixXd> _phi_i_x_up_est(_nh);
	  vector <Eigen::MatrixXd> _phi_i_y_up_est(_nh);
	  for(int jxx=1; jxx<=_nh; jxx++)
	  {
	    // ZMP constraints
	    // x-ZMP upper boundary
	    Eigen::MatrixXd  _phi_i_x_up1 = ZMPx_constraints_offfline[jxx-1] + ZMPx_constraints_half2 * ZMPx_constraints_half[jxx-1];	      
	    _phi_i_x_up_est[jxx-1] = _mass * (_phi_i_x_up1 + _phi_i_x_up1.transpose())/2;
/*	    _phi_i_x_up_est[jxx-1] = _mass * (ZMPx_constraints_offfline[jxx-1] + ZMPx_constraints_half2 * ZMPx_constraints_half[jxx-1]);
	*/    
	    // y-ZMP upper boundary
	    Eigen::MatrixXd  _phi_i_y_up1 = ZMPy_constraints_offfline[jxx-1] + ZMPy_constraints_half2 * ZMPy_constraints_half[jxx-1];
	    _phi_i_y_up_est[jxx-1] = _mass * (_phi_i_y_up1 + _phi_i_y_up1.transpose())/2;
/*	    _phi_i_y_up_est[jxx-1] = _mass * (ZMPy_constraints_offfline[jxx-1] + ZMPy_constraints_half2 * ZMPy_constraints_half[jxx-1]);
	*/    
	  }

	  
          // constraints: only once 
	  _Footvx_max = _Sfx.row(0);
	  _Footvx_min = -_Sfx.row(0);
	  _Footvy_max = _Sfy.row(0);
	  _Footvy_min = -_Sfy.row(0);	  
		  
	  ///////////// equality equation	    
	  //equality constraints
	  _H_q_footz = _Sfz.row(0);
// 	  _F_footz = _Sfz.row(0)*_V_ini - _Lz_ref.row(0);	

	 	  
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
	  // SEQUENCE QUADARTIC PROGRAMMING: lOOP_until the maximal loops reaches	
	  // SEQUENCE QUADARTIC PROGRAMMING: lOOP_until the maximal loops reaches	
	  t_start1 = clock();
  
	  for (int xxxx=1; xxxx <= _loop; xxxx++)
	  {	     
	    _q_goal1 = _Q_goal1 * _V_ini + _q_goal;	  
	 	    
///////////// inequality equation   
            t_start2 = clock();
	    
	    /// time consuming process
	    
	    for(int jxx=1; jxx<=_nh; jxx++)
	    {
	      // ZMP constraints
	      _phi_i_x_up = _phi_i_x_up_est[jxx-1];
	      _H_q_upx.row(jxx-1) = (2*_phi_i_x_up*_V_ini + _p_i_x_t_up.col(jxx-1)).transpose(); 
	      _F_zmp_upx.row(jxx-1) = -((_V_ini.transpose() * _phi_i_x_up + _p_i_x_t_up.col(jxx-1).transpose()) * _V_ini + _del_i_x_up.col(jxx-1)); 

	      // x-ZMP low boundary
	      _phi_i_x_low = _phi_i_x_up;	      	      
	      _H_q_lowx.row(jxx-1) = (- _p_i_x_t_low.col(jxx-1) + _p_i_x_t_up.col(jxx-1)).transpose() - _H_q_upx.row(jxx-1);  
	      _F_zmp_lowx.row(jxx-1) = (_p_i_x_t_low.col(jxx-1)-_p_i_x_t_up.col(jxx-1)).transpose() * _V_ini + _del_i_x_low.col(jxx-1) -  _del_i_x_up.col(jxx-1)-_F_zmp_upx.row(jxx-1); 
   	      
	      
	      // y-ZMP upper boundary	      
	      _phi_i_y_up = _phi_i_y_up_est[jxx-1];	      
	      _H_q_upy.row(jxx-1) = (2*_phi_i_y_up*_V_ini + _p_i_y_t_up.col(jxx-1)).transpose(); 
	      _F_zmp_upy.row(jxx-1) = -((_V_ini.transpose() * _phi_i_y_up + _p_i_y_t_up.col(jxx-1).transpose()) * _V_ini + _del_i_y_up.col(jxx-1)); 
	      
	      // y-ZMP low boundary
	      _phi_i_y_low = _phi_i_y_up;  	      	      
	      _H_q_lowy.row(jxx-1) = (- _p_i_y_t_low.col(jxx-1) + _p_i_y_t_up.col(jxx-1)).transpose() - _H_q_upy.row(jxx-1); 
	      _F_zmp_lowy.row(jxx-1) = (_p_i_y_t_low.col(jxx-1)-_p_i_y_t_up.col(jxx-1)).transpose() * _V_ini + _del_i_y_low.col(jxx-1) - _del_i_y_up.col(jxx-1)-_F_zmp_upy.row(jxx-1);      	      
	   
	      
	      //angle range constraints
	      _qq_upx.row(jxx-1) = -(_q_upx.row(jxx-1)* _V_ini + _qq1_upx.row(jxx-1));
	      _qq_lowx.row(jxx-1) = _thetax_max - _thetax_min - _qq_upx.row(jxx-1);	      

	      	 	      
	      _qq_upy.row(jxx-1) = -(_q_upy.row(jxx-1)* _V_ini + _qq1_upy.row(jxx-1));
	      _qq_lowy.row(jxx-1) = _thetay_max - _thetay_min - _qq_upy.row(jxx-1);	    

	      //torque range constraints	      
	      _tt_upx.row(jxx-1) = -(_t_upx.row(jxx-1)* _V_ini +  _tt1_upx.row(jxx-1));
	      _tt_lowx.row(jxx-1) = _torquex_max - _torquex_min - _tt_upx.row(jxx-1);	
	      
	      _tt_upy.row(jxx-1) = -(_t_upy.row(jxx-1)* _V_ini +  _tt1_upy.row(jxx-1));
	      _tt_lowy.row(jxx-1) = _torquey_max - _torquey_min - _tt_upy.row(jxx-1);		      
	      
	      // body height constraints	      
	      _F_h_upz.row(jxx-1) = -(_H_h_upz.row(jxx-1)*_V_ini + _delta_footz_up.row(jxx-1));
	      _F_h_lowz.row(jxx-1) = _z_max.row(i+jxx-1) - _z_min.row(i+jxx-1)-_F_h_upz.row(jxx-1);	      
	      
	      // body height acceleration constraints	      
	      _F_hacc_lowz.row(jxx-1) = (-_H_hacc_lowz.row(jxx-1)*_V_ini + _delta_footzacc_up.row(jxx-1));	      	      
	    }


	    
	    
	    
            t_start3 = clock();
	    
	    // foot location constraints
	    if (_n_vis == 1)  //one next steo
	    {
	      _H_q_footx_up.row(0) = _Sfx.row(0);
	      _F_foot_upx.row(0) = -(_H_q_footx_up.row(0) * _V_ini -_fx - _footx_max); 
	      _H_q_footx_low.row(0) = -_Sfx.row(0);
	      _F_foot_lowx.row(0) = (_H_q_footx_up.row(0) * _V_ini -_fx - _footx_min);
	      
	      
	      
	      // footy location constraints
	      if (_bjxx % 2 == 0) //odd
	      {
		_H_q_footy_up.row(0) = _Sfy.row(0);
		_F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini -_fy + _footy_min); 
		_H_q_footy_low.row(0) = -_Sfy.row(0);
		_F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini -_fy + _footy_max);		
	      }
	      else
	      {
		_H_q_footy_up.row(0) = _Sfy.row(0);
		_F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini -_fy - _footy_max); 
		_H_q_footy_low.row(0) = -_Sfy.row(0);
		_F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini -_fy - _footy_min);			
	      }	 
	      
	      
	    }
	    else   //two next steps
	    {
	      _H_q_footx_up.row(0) = _Sfx.row(0);
	      _F_foot_upx.row(0) = -(_H_q_footx_up.row(0) * _V_ini -_fx - _footx_max); 
	      _H_q_footx_low.row(0) = -_Sfx.row(0);
	      _F_foot_lowx.row(0) = (_H_q_footx_up.row(0) * _V_ini -_fx - _footx_min);
	      
	      // footy location constraints
	      if (_bjxx % 2 == 0) //odd
	      {
		_H_q_footy_up.row(0) = _Sfy.row(0);
		_F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini -_fy + _footy_min); 
		_H_q_footy_low.row(0) = -_Sfy.row(0);
		_F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini -_fy + _footy_max);		
	      }
	      else
	      {
		_H_q_footy_up.row(0) = _Sfy.row(0);
		_F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini -_fy - _footy_max); 
		_H_q_footy_low.row(0) = -_Sfy.row(0);
		_F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini -_fy - _footy_min);			
	      }
	      
	      // the next two steps 
	      _H_q_footx_up.row(1) = _Sfoot* _Sfx;
	      _F_foot_upx.row(1) = -(_H_q_footx_up.row(1) * _V_ini - _footx_max); 	      
	      _H_q_footx_low.row(1) = -_Sfoot* _Sfx;
	      _F_foot_lowx.row(1) = (_H_q_footx_up.row(1) * _V_ini - _footx_min);
	      
	      // footy location constraints
	      if (_bjxx % 2 == 0) //odd
	      {
		_H_q_footy_up.row(1) = _Sfoot* _Sfy;
		_F_foot_upy.row(1) = -(_H_q_footy_up.row(1) * _V_ini - _footy_max); 
		_H_q_footy_low.row(1) = -_H_q_footy_up.row(1);
		_F_foot_lowy.row(1) = (_H_q_footy_up.row(1) * _V_ini - _footy_min);				
	      }
	      else
	      {
		_H_q_footy_up.row(1) = _Sfoot* _Sfy;
		_F_foot_upy.row(1) = -(_H_q_footy_up.row(1) * _V_ini + _footy_min); 
		_H_q_footy_low.row(1) = -_H_q_footy_up.row(1);
		_F_foot_lowy.row(1) = (_H_q_footy_up.row(1) * _V_ini + _footy_max);		
	      }	      	     	      
	    }


	    
	    //swing foot veloctiy boundary
	    if (i ==1)
	    {
	      _footubxv = -(_Sfx.row(0) * _V_ini - _footx_max - _footx_real_next.row(i+_nT-2));
	      _footlbxv = (_Sfx.row(0) * _V_ini - _footx_min - _footx_real_next.row(i+_nT-2));
	      _footubyv = -(_Sfy.row(0) * _V_ini - _footy_max - _footy_real_next.row(i+_nT-2));
	      _footlbyv = (_Sfy.row(0) * _V_ini - _footy_min - _footy_real_next.row(i+_nT-2));	      
	    }
	    else
	    {
	      if (fabs(i*_dt - _tx(_bjxx-1))<=0.01)
	      {
		if ((_bjxx % 2) == 1)
		{
		  _footubxv = -(_Sfx.row(0) * _V_ini - _footx_max - _footx_real_next.row(i+_nT-2));
		  _footlbxv = (_Sfx.row(0) * _V_ini - _footx_min - _footx_real_next.row(i+_nT-2));
		  _footubyv = -(_Sfy.row(0) * _V_ini - _footy_max - _footy_real_next.row(i+_nT-2));
		  _footlbyv = (_Sfy.row(0) * _V_ini - _footy_min - _footy_real_next.row(i+_nT-2));		  
		}
		else
		{
		  _footubxv = -(_Sfx.row(0) * _V_ini - _footx_max - _footx_real_next.row(i+_nT-2));
		  _footlbxv = (_Sfx.row(0) * _V_ini - _footx_min - _footx_real_next.row(i+_nT-2));
		  _footubyv = -(_Sfy.row(0) * _V_ini + _footy_min - _footy_real_next.row(i+_nT-2));
		  _footlbyv = (_Sfy.row(0) * _V_ini + _footy_max - _footy_real_next.row(i+_nT-2));		
		}		
	      }
	      else
	      {
		_footubxv = -(_Sfx.row(0) * _V_ini - _footx_vmax*_dt - _footx_real_next.row(i+_nT-2));
		_footlbxv = (_Sfx.row(0) * _V_ini - _footx_vmin*_dt - _footx_real_next.row(i+_nT-2));
		_footubyv = -(_Sfy.row(0) * _V_ini - _footy_vmax*_dt - _footy_real_next.row(i+_nT-2));
		_footlbyv = (_Sfy.row(0) * _V_ini - _footy_vmin*_dt - _footy_real_next.row(i+_nT-2));		
	      }
	    }


	  ///////////// equality equation	    
	  //equality constraints
//  	 _H_q_footz = _Sfz.row(0);
	  _F_footz = _Sfz.row(0)*_V_ini - _Lz_ref.row(0);	    
	        
	    
//////////////////////////////===========////////////////////////////////////////////////////	    
// 	    // quadratic program GetSolution
	    t_start4 = clock();
	    int nVars = _Nt;
	    int nEqCon = 1;
	    int nIneqCon = 15*_nh + 4*_nstep +4;
	    resizeQP(nVars, nEqCon, nIneqCon);	    

	    _G = _Q_goal1;
	    _g0 = _q_goal1;
	    _X = _V_ini;

	    
	    _CI.block< Dynamic, Dynamic>(0,0, _Nt,_nh) = _H_q_upx.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,_nh, _Nt,_nh) = _H_q_lowx.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,2*_nh, _Nt,_nh) = _H_q_upy.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,3*_nh, _Nt,_nh) = _H_q_lowy.transpose() * (-1);
	    
	    _CI.block< Dynamic, Dynamic>(0,4*_nh, _Nt,_nh) = _H_h_upz.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,5*_nh, _Nt,_nh) = _H_h_lowz.transpose() * (-1);
	    
	    _CI.block< Dynamic, Dynamic>(0,6*_nh, _Nt,_nh) = _H_hacc_lowz.transpose() * (-1);
	    
	    _CI.block< Dynamic, Dynamic>(0,7*_nh, _Nt,_nstep) = _H_q_footx_up.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,7*_nh+_nstep, _Nt,_nstep) = _H_q_footx_low.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,7*_nh+2*_nstep, _Nt,_nstep) = _H_q_footy_up.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,7*_nh+3*_nstep, _Nt,_nstep) = _H_q_footy_low.transpose() * (-1);	    

	    _CI.block< Dynamic, Dynamic>(0,7*_nh+4*_nstep, _Nt,_nh) = _q_upx.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,8*_nh+4*_nstep, _Nt,_nh) = _q_lowx.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,9*_nh+4*_nstep, _Nt,_nh) = _q_upy.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,10*_nh+4*_nstep,_Nt,_nh) = _q_lowy.transpose() * (-1);
	    
	    _CI.block< Dynamic, Dynamic>(0,11*_nh+4*_nstep, _Nt,_nh) = _t_upx.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,12*_nh+4*_nstep, _Nt,_nh) = _t_lowx.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,13*_nh+4*_nstep, _Nt,_nh) = _t_upy.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,14*_nh+4*_nstep, _Nt,_nh) = _t_lowy.transpose() * (-1);
	    
	    _CI.block< Dynamic, Dynamic>(0,15*_nh+4*_nstep, _Nt,1) = _Footvx_max.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,15*_nh+4*_nstep+1, _Nt,1) = _Footvx_min.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,15*_nh+4*_nstep+2, _Nt,1) = _Footvy_max.transpose() * (-1);
	    _CI.block< Dynamic, Dynamic>(0,15*_nh+4*_nstep+3, _Nt,1) = _Footvy_min.transpose() * (-1);
 
	    

	    
	    _ci0.block< Dynamic, Dynamic>(0, 0,_nh,1) = _F_zmp_upx;
	    _ci0.block< Dynamic, Dynamic>(_nh, 0,_nh,1) = _F_zmp_lowx;
	    _ci0.block< Dynamic, Dynamic>(2*_nh, 0,_nh,1) = _F_zmp_upy;
	    _ci0.block< Dynamic, Dynamic>(3*_nh, 0,_nh,1) = _F_zmp_lowy;
	    
	    _ci0.block< Dynamic, Dynamic>(4*_nh, 0,_nh,1) = _F_h_upz;
	    _ci0.block< Dynamic, Dynamic>(5*_nh, 0,_nh,1) = _F_h_lowz;
	    
	    _ci0.block< Dynamic, Dynamic>(6*_nh, 0,_nh,1) = _F_hacc_lowz;
	    
	    _ci0.block< Dynamic, Dynamic>(7*_nh, 0,_nstep,1) = _F_foot_upx;
	    _ci0.block< Dynamic, Dynamic>(7*_nh+_nstep, 0,_nstep,1) = _F_foot_lowx;
	    _ci0.block< Dynamic, Dynamic>(7*_nh+2*_nstep, 0,_nstep,1) = _F_foot_upy;
	    _ci0.block< Dynamic, Dynamic>(7*_nh+3*_nstep, 0,_nstep,1) = _F_foot_lowy;	    

	    _ci0.block< Dynamic, Dynamic>(7*_nh+4*_nstep, 0,_nh,1) = _qq_upx;
	    _ci0.block< Dynamic, Dynamic>(8*_nh+4*_nstep, 0,_nh,1) = _qq_lowx;
	    _ci0.block< Dynamic, Dynamic>(9*_nh+4*_nstep, 0,_nh,1) = _qq_upy;
	    _ci0.block< Dynamic, Dynamic>(10*_nh+4*_nstep, 0,_nh,1) = _qq_lowy;
	    
	    _ci0.block< Dynamic, Dynamic>(11*_nh+4*_nstep, 0,_nh,1) = _tt_upx;
	    _ci0.block< Dynamic, Dynamic>(12*_nh+4*_nstep, 0,_nh,1) = _tt_lowx;
	    _ci0.block< Dynamic, Dynamic>(13*_nh+4*_nstep, 0,_nh,1) = _tt_upy;
	    _ci0.block< Dynamic, Dynamic>(14*_nh+4*_nstep, 0,_nh,1) = _tt_lowy;
	    
	    _ci0.block< Dynamic, Dynamic>(15*_nh+4*_nstep, 0,1,1) = _footubxv;
	    _ci0.block< Dynamic, Dynamic>(15*_nh+4*_nstep+1, 0,1,1) = _footlbxv;
	    _ci0.block< Dynamic, Dynamic>(15*_nh+4*_nstep+2, 0,1,1) = _footubyv;
	    _ci0.block< Dynamic, Dynamic>(15*_nh+4*_nstep+3, 0,1,1) = _footlbyv;
        
// 	    
	    _CE = _H_q_footz.transpose();
	    _ce0 = _F_footz;
// 	    
	    Solve();
	    
	    t_finish1 = clock();
	    _tcpu_prepara(0,i-1) = (double)(t_finish1 - t_start2)/CLOCKS_PER_SEC ;
   
	    _tcpu_prepara2(0,i-1) = (double)(t_finish1 - t_start3)/CLOCKS_PER_SEC ;
	    _tcpu_qp(0,i-1) = (double)(t_finish1 - t_start4)/CLOCKS_PER_SEC ;	    
	  }

	  
	  
	  t_finish = clock();	  
/////////////////////////////////////////////////////////
/////////===================================================%%%%%	  
	 // results postprocessed:	  
	  if (_n_vis == 1)
	  {
	    _V_optimal.block< Dynamic, 1>(0, i-1, 5*_nh, 1)  = _V_ini.topRows(5*_nh);
	    _V_optimal.block< Dynamic, 1>(5*_nh, i-1, 2, 1)  = _V_inix.setConstant(2,1,_V_ini(5*_nh,0));
	    _V_optimal.block< Dynamic, 1>(5*_nh+2, i-1, 2, 1)  = _V_inix.setConstant(2,1,_V_ini(5*_nh+1,0));
	    _V_optimal.block< Dynamic, 1>(5*_nh+4, i-1, 2, 1)  = _V_inix.setConstant(2,1,_V_ini(5*_nh+2,0));
	    
	  }
	  else
	  {
	    _V_optimal.col(i-1) = _V_ini;
	  }
	  
	  //next step location
	  _x_vacc_k.col(i-1) = _V_ini.row(0);
	  _footx_real.row(_bjxx) = _V_ini.row(5*_nh);
	  _footx_real_next.row(i+_nT -1) = _V_ini.row(5*_nh);
	  _xk.col(i) = _a * _xk.col(i-1)  + _b* _x_vacc_k.col(i-1);
	  _comx(0,i)=_xk(0,i); _comvx(0,i) = _xk(1,i); _comax(0,i)=_xk(2,i); 	  
	  
	  _y_vacc_k.col(i-1) = _V_ini.row(0+_nh);
	  _footy_real.row(_bjxx) = _V_ini.row(5*_nh + _nstep);
	  _footy_real_next.row(i+_nT -1) = _V_ini.row(5*_nh + _nstep);
	  _yk.col(i) = _a * _yk.col(i-1)  + _b* _y_vacc_k.col(i-1);
	  _comy(0,i)=_yk(0,i); _comvy(0,i) = _yk(1,i); _comay(0,i)=_yk(2,i); 
	  
// 	  if (i==40)
// 	  {
// 	    _comvx(0,i) = _comvx(0,i) + 0.55;
// 	    _comvy(0,i) = _comvy(0,i) + 0.42;
// 	  }
// 	  _yk(1,i) = _comvy(0,i);
// 	  _xk(1,i) = _comvx(0,i);
	  
	  _z_vacc_k.col(i-1) = _V_ini.row(0+2*_nh);
	  _footz_real.row(_bjxx) = _V_ini.row(5*_nh + 2*_nstep);
	  _footz_real_next.row(i+_nT -1) = _V_ini.row(5*_nh + 2*_nstep);
	  
	  _zk.col(i) = _a * _zk.col(i-1)  + _b* _z_vacc_k.col(i-1);
	  _comz(0,i)=_zk(0,i); _comvz(0,i) = _zk(1,i); _comaz(0,i)=_zk(2,i); 

	  _thetax_vacc_k.col(i-1) = _V_ini.row(0+3*_nh);
	  _thetaxk.col(i) = _a * _thetaxk.col(i-1)  + _b* _thetax_vacc_k.col(i-1);
	  _thetax(0,i) = _thetaxk(0,i); _thetavx(0,i) = _thetaxk(1,i); _thetaax(0,i)=_thetaxk(2,i); 	


	  _thetay_vacc_k.col(i-1) = _V_ini.row(0+4*_nh);
	  _thetayk.col(i) = _a * _thetayk.col(i-1)  + _b* _thetay_vacc_k.col(i-1);
	  _thetay(0,i) = _thetayk(0,i); _thetavy(0,i) = _thetayk(1,i); _thetaay(0,i)=_thetayk(2,i); 	  
	  
	  _torquex_real.col(i) = _j_ini * _thetaax.col(i);
	  _torquey_real.col(i) = _j_ini * _thetaay.col(i);
	  
	  _zmpx_real(0,i) = _comx(0,i) - (_comz(0,i) - _Zsc(i,0))/(_comaz(0,i)+_g)*_comax(0,i) - _j_ini * _thetavy(0,i)/(_mass * (_g + _comaz(0,i)));
	  _zmpy_real(0,i) = _comy(0,i) - (_comz(0,i) - _Zsc(i,0))/(_comaz(0,i)+_g)*_comay(0,i) + _j_ini * _thetavx(0,i)/(_mass * (_g + _comaz(0,i)));
	  
// 	  t_finish = clock();
	  _tcpu(0,i-1) = (double)(t_finish - t_start)/CLOCKS_PER_SEC ;
	  _tcpu_iterative(0,i-1) = (double)(t_finish - t_start1)/CLOCKS_PER_SEC ;
	  
          _footxyz_real.row(0) = _footx_real.transpose();
	  _footxyz_real.row(1) = _footy_real.transpose();	  
	  _footxyz_real.row(2) = _footz_real.transpose();

//           cout << "_bjx1:" <<endl << _bjx1 <<endl;
// 	  cout << "_bjxx:" <<endl << _bjxx <<endl;
// 	  cout << "i:"     <<endl << i     <<endl;
	  
	  Foot_trajectory_solve(i);
	  

	}
	
	
	
/*	cout << " whole run time: " <<endl<<_tcpu<<endl;
	cout << " 2 loop SQP time: " <<endl<<_tcpu_iterative<<endl;	
	cout << " optimization solver time: " <<endl<<_tcpu_prepara<<endl;
	cout << " optimization solver time (no ZMP constraints): " <<endl<<_tcpu_prepara2<<endl;
	cout << " optimization solver time (QP): " <<endl<<_tcpu_qp<<endl;*/	
	
	File_wl(_tcpu,_comx, _comy, _comz, _thetax, _thetay,_zmpx_real, _zmpy_real,_torquex_real, _torquey_real, _footx_real, _footy_real, _footz_real,_footx_real_next, _footy_real_next,_footz_real_next, _Rfootx, _Rfooty, _Rfootz, _Lfootx, _Lfooty, _Lfootz);

	
	std::cout << "_Rfootx: " <<endl<< _Rfootx<< std::endl;	
	std::cout << "_Rfooty: " <<endl<< _Rfooty<< std::endl;	
	std::cout << "_Lfootx: " <<endl<< _Lfootx<< std::endl;	
	std::cout << "_Lfooty: " <<endl<< _Lfooty<< std::endl;		
	std::cout << "_comx: " <<endl<< _comx<< std::endl;	
	std::cout << "_comy: " <<endl<< _comy<< std::endl;
	std::cout << "_comz: " <<endl<< _comz<< std::endl;
	
}




int MPCClass::Indexfind(double goalvari, Eigen::VectorXd goalarray, int xyz)
{
        int j=0;
	if (xyz<0.05)
	{
	  while (goalvari >= goalarray(j))
	  {
	    j++;
	  }
	  
	  j = j-1;	  
	}
	else
	{
	  while ( fabs(goalvari - goalarray(j)) >0.0001 )
	  {
	    j++;
	  }
	  	  
	}	  
	
	return j;
  
}

int MPCClass::is_sigular(int num)
{
	if ( num & 1)
	{	  
	  return 1; //sigular 
	}
	else
	{
	  return 0;	// odd  	 
	}	   
}


Eigen::MatrixXd  MPCClass::Matrix_ps(Eigen::MatrixXd a, int nh,Eigen::MatrixXd cxps)
{
  Eigen::MatrixXd matrixps;
  matrixps.setZero(nh,3);
  
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


Eigen::MatrixXd MPCClass::Matrix_pu(Eigen::MatrixXd a, Eigen::MatrixXd b, int nh, Eigen::MatrixXd cxpu)
{
  Eigen::MatrixXd matrixpu;
  
  matrixpu.setZero(nh,nh);
  
  Eigen::MatrixXd A;
  Eigen::MatrixXd Temp;
  
  
  for (int i = 1; i < nh+1; i++) {
    for (int j = 1; j < i+1; j++)
    { 
      A.setIdentity(a.rows(),a.cols());      
      if (j==i)
      {
	Temp = cxpu * A * b;
	matrixpu(i-1,j-1) = Temp(0,0);
      }
      else
      {	
	for (int k = 1; k < i-j+1; k++)
	{
	  A = A*a;
	}
	Temp = cxpu * A * b;
	matrixpu(i-1,j-1) = Temp(0,0);
      }          
    }       
  }
    
  return matrixpu;  
}


void MPCClass::File_wl(Eigen::RowVectorXd cputime,Eigen::RowVectorXd comx, Eigen::RowVectorXd comy, Eigen::RowVectorXd comz, Eigen::RowVectorXd thetax, Eigen::RowVectorXd thetay,Eigen::RowVectorXd zmpx, Eigen::RowVectorXd zmpy,Eigen::RowVectorXd torquex, Eigen::RowVectorXd torquey, Eigen::VectorXd foot_realx, Eigen::VectorXd foot_realy, Eigen::VectorXd foot_realz,Eigen::VectorXd foot_realx_next, Eigen::VectorXd foot_realy_next,Eigen::VectorXd foot_realz_real,Eigen::RowVectorXd Rfootx,Eigen::RowVectorXd Rfooty,Eigen::RowVectorXd Rfootz,Eigen::RowVectorXd Lfootx,Eigen::RowVectorXd Lfooty,Eigen::RowVectorXd Lfootz)
{
        
        Eigen::MatrixXd CoMMM_ZMP_foot;
	CoMMM_ZMP_foot.setZero(18,comx.cols());
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(0, 0,1,comx.cols()) = comx;
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(1, 0,1,comx.cols()) = comy;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(2, 0,1,comx.cols()) = comz;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(3, 0,1,comx.cols()) = zmpx;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(4, 0,1,comx.cols()) = zmpy;
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(5, 0,1,comx.cols()) = thetax;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(6, 0,1,comx.cols()) = thetay;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(7, 0,1,comx.cols()) = torquex;
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(8, 0,1,comx.cols()) = torquey;
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(9, 0,1,comx.cols()) = foot_realx_next.transpose();	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(10, 0,1,comx.cols()) = foot_realy_next.transpose();	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(11, 0,1,comx.cols()) = foot_realz_real.transpose();
	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(12, 0,1,comx.cols()) = Lfootx;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(13, 0,1,comx.cols()) = Lfooty;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(14, 0,1,comx.cols()) = Lfootz;
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(15, 0,1,comx.cols()) = Rfootx;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(16, 0,1,comx.cols()) = Rfooty;	
	CoMMM_ZMP_foot.block< Dynamic, Dynamic>(17, 0,1,comx.cols()) = Rfootz;	
	
	
	
  
	std::string fileName = "C++_NMPC2018_3robut3_runtime.txt" ;
	std::ofstream outfile( fileName.c_str() ) ; // file name and the operation type. 
       
        for(int i=0; i<cputime.rows(); i++){
           for(int j=0; j<cputime.cols(); j++){
                 outfile << (double) cputime(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();
	
        for(int i=0; i<cputime.rows(); i++){
           for(int j=0; j<cputime.cols(); j++){
                 outfile << (double) cputime(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();	


	std::string fileName1 = "C++_NMPC2018_3robut3_optimal_trajectory.txt" ;
	std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.        
	
        for(int i=0; i<CoMMM_ZMP_foot.rows(); i++){
           for(int j=0; j<CoMMM_ZMP_foot.cols(); j++){
                 outfile1 << (double) CoMMM_ZMP_foot(i,j) << " " ; 
           }
           outfile1 << std::endl;       // a   newline
        }
        outfile1.close();	
	
	
	
}



void MPCClass::FootStepInputs(int footstepsnumber, double stepwidth, double steplength, double stepheight)
{	
        _footstepsnumber = footstepsnumber;
	_steplength.setConstant(_footstepsnumber,steplength);
	_steplength(0) = 0;
	
	_stepwidth.setConstant(_footstepsnumber,stepwidth);
	_stepwidth(0) = _stepwidth(0)/2;
	
	_stepheight.setConstant(_footstepsnumber,stepheight);
//         
// 	cout << "stepwidth"<<stepwidth<<endl;
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

		
// 	for (int i = 0; i < _loop; i++) {
// 		updateMatrix();
// 		solveQP();
// 		_sqpX += _X;
// 	}
}





Eigen::MatrixXd MPCClass::Foot_trajectory_solve(int j_index)
{
  double  Footz_ref = 0.2;
  _footxyz_real(1,0) = -_stepwidth(0);
  
//   foot trajectory generation:
  if (_bjx1 >= 2)
  {
//     cout << "_bjx1 >= 2"<<endl;
    if (_bjx1 % 2 == 0)           //odd:left support
    {
//       cout << "left support"<<endl;
      _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab
      {
// 	cout << "dsp"<<endl;
	_Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
	_Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
	_Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
      }
      else
      {
	
// 	cout << "ssp"<<endl;
	//initial state and final state and the middle state
	double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
	Eigen::Vector3d t_plan;
	t_plan(0) = t_des - _dt;
	t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
	t_plan(2) = _ts(_bjx1-1) + 0.0001;
	
	if (abs(t_des - _ts(_bjx1-1)) <= (_dt + 0.0005))
	{
	  _Rfootx(j_index) = _footxyz_real(0,_bjxx); 
	  _Rfooty(j_index) = _footxyz_real(1,_bjxx);
	  _Rfootz(j_index) = _footxyz_real(2,_bjxx); 	  
	}
	else
	{
	  Eigen::MatrixXd AAA;
	  AAA.setZero(7,7);
	  Eigen::RowVectorXd aaaa;
	  aaaa.setZero(7);
	  
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
	  
          
	 	  
	  Eigen::RowVectorXd t_a_plan;
	  t_a_plan.setZero(7);
	  t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
	  t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
	  

	  Eigen::RowVectorXd t_a_planv;
	  t_a_planv.setZero(7);
	  t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
	  t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
	  
	  
	  Eigen::RowVectorXd t_a_plana;
	  t_a_plana.setZero(7);
	  t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
	  t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
	  
// 	  cout <<"AAA="<<endl<<AAA<<endl;
// 	  cout <<"AAA_inverse="<<endl<<AAA.inverse()<<endl;	  
// 	  cout <<"t_des="<<endl<<t_des<<endl;
// 	  cout <<"t_plan="<<endl<<t_plan<<endl;
	  
	  ////////////////////////////////////////////////////////////////////////////
	  Eigen::VectorXd Rfootx_plan;
	  Rfootx_plan.setZero(7);	
	  Rfootx_plan(0) = _Rfootvx(j_index-1);     Rfootx_plan(1) = _Rfootax(j_index-1); Rfootx_plan(2) = _Rfootx(j_index-1); Rfootx_plan(3) = _Lfootx(j_index);
	  Rfootx_plan(4) = _footxyz_real(0,_bjxx);  Rfootx_plan(5) = 0;                   Rfootx_plan(6) = 0;
	  
	  
	  Eigen::VectorXd Rfootx_co;
	  Rfootx_co.setZero(7);
	  Rfootx_co = AAA.inverse() * Rfootx_plan;
	  
	  _Rfootx(j_index) = t_a_plan * Rfootx_co;
	  _Rfootvx(j_index) = t_a_planv * Rfootx_co;
	  _Rfootax(j_index) = t_a_plana * Rfootx_co;
	  
	  /////////////////////////////////////////////////////////////////////////////
	  if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
	  {
	    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
	  }
	  
	  Eigen::VectorXd Rfooty_plan;
	  Rfooty_plan.setZero(7);	
	  Rfooty_plan(0) = _Rfootvy(j_index-1);     Rfooty_plan(1) = _Rfootay(j_index-1); Rfooty_plan(2) = _Rfooty(j_index-1); Rfooty_plan(3) = _ry_left_right;
	  Rfooty_plan(4) = _footxyz_real(1,_bjxx);  Rfooty_plan(5) = 0;                   Rfooty_plan(6) = 0;	    
	  
	  Eigen::VectorXd Rfooty_co;
	  Rfooty_co.setZero(7);
	  Rfooty_co = AAA.inverse() * Rfooty_plan;
	  
	  _Rfooty(j_index) = t_a_plan * Rfooty_co;
	  _Rfootvy(j_index) = t_a_planv * Rfooty_co;
	  _Rfootay(j_index) = t_a_plana * Rfooty_co;	
	  
	  
	  //////////////////////////////////////////////////////////
	  Eigen::VectorXd Rfootz_plan;
	  Rfootz_plan.setZero(7);	
	  Rfootz_plan(0) = _Rfootvz(j_index-1);     Rfootz_plan(1) = _Rfootaz(j_index-1); Rfootz_plan(2) = _Rfootz(j_index-1); Rfootz_plan(3) = _Lfootz(j_index)+Footz_ref;
	  Rfootz_plan(4) = _footxyz_real(2,_bjxx);  Rfootz_plan(5) = 0;                   Rfootz_plan(6) = 0;	
	  
	  Eigen::VectorXd Rfootz_co;
	  Rfootz_co.setZero(7);
	  Rfootz_co = AAA.inverse() * Rfootz_plan;
	  
	  _Rfootz(j_index) = t_a_plan * Rfootz_co;
	  _Rfootvz(j_index) = t_a_planv * Rfootz_co;
	  _Rfootaz(j_index) = t_a_plana * Rfootz_co;	
	  
// 	  cout<<"t_a_plan:"<<endl<<t_a_plan<<endl;
// 	  cout<<"t_a_planv:"<<endl<<t_a_planv<<endl;
// 	  cout<<"t_a_plana:"<<endl<<t_a_plana<<endl;	  
// 
// 	  cout<<"Rfootx_plan:"<<endl<<Rfootx_plan<<endl;
// 	  cout<<"Rfooty_plan:"<<endl<<Rfooty_plan<<endl;
// 	  cout<<"Rfootz_plan:"<<endl<<Rfootz_plan<<endl;
	  
	}
      }
 
      
    }
    
    
    else                       //right support
    {
//       cout << "right support"<<endl;
      _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab
      {
// 	cout << "dsp"<<endl;
	_Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
	_Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
	_Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
      }
      else
      {
// 	cout << "ssp"<<endl;
	//initial state and final state and the middle state
	double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
	Eigen::Vector3d t_plan;
	t_plan(0) = t_des - _dt;
	t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
	t_plan(2) = _ts(_bjx1-1) + 0.0001;
	
	if (abs(t_des - _ts(_bjx1-1)) <= (_dt + 0.0005))
	{
	  
	  _Lfootx(j_index) = _footxyz_real(0,_bjxx); 
	  _Lfooty(j_index) = _footxyz_real(1,_bjxx);
	  _Lfootz(j_index) = _footxyz_real(2,_bjxx); 	  
	}
	else
	{
	  Eigen::MatrixXd AAA;
	  AAA.setZero(7,7);
	  Eigen::RowVectorXd aaaa;
	  aaaa.setZero(7);
	  
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
	  
          
	 	  
	  Eigen::RowVectorXd t_a_plan;
	  t_a_plan.setZero(7);
	  t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
	  t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
	  

	  Eigen::RowVectorXd t_a_planv;
	  t_a_planv.setZero(7);
	  t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
	  t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
	  
	  
	  Eigen::RowVectorXd t_a_plana;
	  t_a_plana.setZero(7);
	  t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
	  t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
	  
// 	  cout <<"AAA="<<endl<<AAA<<endl;
// 	  cout <<"AAA_inverse="<<endl<<AAA.inverse()<<endl;	  
// 	  cout <<"t_des="<<endl<<t_des<<endl;
// 	  cout <<"t_plan="<<endl<<t_plan<<endl;
	  
	  ////////////////////////////////////////////////////////////////////////////
	  Eigen::VectorXd Lfootx_plan;
	  Lfootx_plan.setZero(7);	
	  Lfootx_plan(0) = _Lfootvx(j_index-1);     Lfootx_plan(1) = _Lfootax(j_index-1); Lfootx_plan(2) = _Lfootx(j_index-1); Lfootx_plan(3) = _Rfootx(j_index);
	  Lfootx_plan(4) = _footxyz_real(0,_bjxx);  Lfootx_plan(5) = 0;                   Lfootx_plan(6) = 0;	  
	  
	  
	  Eigen::VectorXd Lfootx_co;
	  Lfootx_co.setZero(7);
	  Lfootx_co = AAA.inverse() * Lfootx_plan;
	  
	  _Lfootx(j_index) = t_a_plan * Lfootx_co;
	  _Lfootvx(j_index) = t_a_planv * Lfootx_co;
	  _Lfootax(j_index) = t_a_plana * Lfootx_co;
	  
	  /////////////////////////////////////////////////////////////////////////////
	  if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
	  {
	    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
	  }
	  
	  Eigen::VectorXd Lfooty_plan;
	  Lfooty_plan.setZero(7);	
	  Lfooty_plan(0) = _Lfootvy(j_index-1);     Lfooty_plan(1) = _Lfootay(j_index-1); Lfooty_plan(2) = _Lfooty(j_index-1); Lfooty_plan(3) = _ry_left_right;
	  Lfooty_plan(4) = _footxyz_real(1,_bjxx);  Lfooty_plan(5) = 0;                   Lfooty_plan(6) = 0;		  
	  
	  
	  Eigen::VectorXd Lfooty_co;
	  Lfooty_co.setZero(7);
	  Lfooty_co = AAA.inverse() * Lfooty_plan;
	  
	  _Lfooty(j_index) = t_a_plan * Lfooty_co;
	  _Lfootvy(j_index) = t_a_planv * Lfooty_co;
	  _Lfootay(j_index) = t_a_plana * Lfooty_co;	
	  
	  
	  //////////////////////////////////////////////////////////
	  Eigen::VectorXd Lfootz_plan;
	  Lfootz_plan.setZero(7);		
	  Lfootz_plan(0) = _Lfootvz(j_index-1);     Lfootz_plan(1) = _Lfootaz(j_index-1); Lfootz_plan(2) = _Lfootz(j_index-1); Lfootz_plan(3) = _Rfootz(j_index)+Footz_ref;
	  Lfootz_plan(4) = _footxyz_real(2,_bjxx);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0;		  
	  
	  
	  Eigen::VectorXd Lfootz_co;
	  Lfootz_co.setZero(7);
	  Lfootz_co = AAA.inverse() * Lfootz_plan;
	  
	  _Lfootz(j_index) = t_a_plan * Lfootz_co;
	  _Lfootvz(j_index) = t_a_planv * Lfootz_co;
	  _Lfootaz(j_index) = t_a_plana * Lfootz_co;
	  
	  
// 	  cout<<"t_a_plan:"<<endl<<t_a_plan<<endl;
// 	  cout<<"t_a_planv:"<<endl<<t_a_planv<<endl;
// 	  cout<<"t_a_plana:"<<endl<<t_a_plana<<endl;
// 	  
// 	  cout<<"Lfootx_plan"<<endl<<Lfootx_plan<<endl;
// 	  cout<<"Lfooty_plan"<<endl<<Lfooty_plan<<endl;
// 	  cout<<"Lfootz_plan"<<endl<<Lfootz_plan<<endl;
	  
	  
	}
      }

    }
      
  }
  else
  {
    _Rfooty(j_index) = -_stepwidth(0);
    _Lfooty(j_index) = _stepwidth(0);
  }
    

  
}



void MPCClass::updateMatrix()
{
	// update your _G, _g0 ..... matices
	// _G, _g0, _CE, _ce0, _CI, _ci0, _X
}


Vector3d MPCClass::GetSolution_CoM_position(int walktime)
{
        _CoM_position_optimal.row(0) = _comx;
	_CoM_position_optimal.row(1) = _comy;
	_CoM_position_optimal.row(2) = _comz;
        
 	return _CoM_position_optimal.col(walktime);
}

Vector2d MPCClass::GetSolution_CoM_inclination(int walktime)
{	
	_torso_angle_optimal.row(0) = _thetax;
	_torso_angle_optimal.row(1) = _thetay;
        
 	return _torso_angle_optimal.col(walktime);
}



Vector3d MPCClass::GetSolution_Foot_positionR(int walktime)
{
	
        _R_foot_optition_optimal.row(0) = _Rfootx;
	_R_foot_optition_optimal.row(1) = _Rfooty;
	_R_foot_optition_optimal.row(2) = _Rfootz;
	
         return _R_foot_optition_optimal.col(walktime);
}

Vector3d MPCClass::GetSolution_Foot_positionL(int walktime)
{
        _L_foot_optition_optimal.row(0) = _Lfootx;
	_L_foot_optition_optimal.row(1) = _Lfooty;
	_L_foot_optition_optimal.row(2) = _Lfootz;

	
         return _L_foot_optition_optimal.col(walktime);
}

Vector3d MPCClass::GetSolution_foot_location(int walktime)
{
        _foot_location_optimal.row(0) = _footx_real;
	_foot_location_optimal.row(1) = _footy_real;
	_foot_location_optimal.row(2) = _footz_real;

	
         return _foot_location_optimal.col(walktime);
}


