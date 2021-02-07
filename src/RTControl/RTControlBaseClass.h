/*****************************************************************************
RTControlBaseClass.h

Description:	Header file of RTControlBaseClass

*****************************************************************************/
#ifndef RTCONTROL_BASE_CLASS_H
#define RTCONTROL_BASE_CLASS_H
#include <iostream>

#include "WBS/WholeBodySensingClass.h"
#include "Stabilizer/StabilizerClass.h"
#include "IK/IKClass.h"

#include "utils/utils.h"

#ifdef USE_XBOT_LOGGER
#include "XBotInterface/Logger.hpp"
#endif

#include <sys/time.h>

#include "RobotPara/RobotParaClass.h"


class RTControlBaseClass
{
public:
	RTControlBaseClass();
	~RTControlBaseClass();

	virtual void Run(const double &dT, std::vector<double> &pos, const std::vector<float> &OffsetAng);
	virtual void Run();
	virtual void KeyBoardControl(char cmd);

	WholeBodySensingClass _WBS;

	inline const RobotStateClass& getRobotState() {return _WBS.getRobotState();};

	virtual inline const RobotParaClass& RobotPara() const {return _WBS.RobotPara();};

	void UpdateJointTorqueFB(const std::vector<double> &tall) {_WBS.UpdateJointTorqueFB(tall);};
	virtual void UpdateWBS(const double & dT, const std::vector<double> &qall, const Eigen::Vector3d &EulerAng, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor = std::vector<double>(12, 0.0));

	virtual void UpdateWBS(const std::vector<double> &qall, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor = std::vector<double>(12, 0.0));

	virtual void savedata();
	void SortFromCOMAN(std::vector<double> &from);

	bool IsInit;

	inline void AddObstacle(const Eigen::Vector3d& obs_center_pos, const Eigen::Vector3d& obs_center_ori) {
		_obs_center_pos = obs_center_pos;
		_obs_center_ori = obs_center_ori;
	};

	void SortFromXBotToJointName(const Eigen::VectorXd &from, std::vector<double> &to);
	void SortFromXBotToJointName(const Eigen::VectorXd &from, Eigen::VectorXd &to);

	template <class T>
	void SortFromJointNameToXBot(const T &from, std::unordered_map<std::string, double> &to);

	void JointRefToXBot(Eigen::VectorXd &to);
	void JointRefToXBot_LowerBody(Eigen::VectorXd &to);

	void HomingInit(const std::vector<double> &homing_angles);

	inline const bool& IsFixedWalk() {return IsFixed;};

protected:
	bool IsStartWalk;
	virtual void StopWalking(){};
	virtual void StartWalking(){};
	
	Eigen::Vector3d _obs_center_pos, _obs_center_ori;

	std::vector<double> srdf_homing_angles;
	double dt;

	Eigen::Vector3d LEFT_HAND_HOME_POS, RIGHT_HAND_HOME_POS;
	Eigen::MatrixXd LEFT_HAND_HOME_ORI, RIGHT_HAND_HOME_ORI;
	Eigen::Vector3d LEFT_FOOT_HOME_POS, RIGHT_FOOT_HOME_POS;
	Eigen::MatrixXd LEFT_FOOT_HOME_ORI, RIGHT_FOOT_HOME_ORI;

	bool IsLoggerSaved;

#ifdef USE_MRDPLOT_LOGGER
	BatchLogger logger;
	void addToLog(Logger &logger);
#endif

#ifdef USE_XBOT_LOGGER
	XBot::MatLogger::Ptr xbot_logger;
	void initLogger(int buffer_size = -1, int interleave = 1);
	void addToLog();
	int logger_len = 100000;
#endif

	Eigen::VectorXd q_tmp;

	Eigen::Vector3d _obs_size, _obs_center_pos_w, _glft, _grft;
	std::vector<double> RelexedDOF;

	Eigen::Vector3d com_ref, lft_ref, rft_ref, zmp_ref;
	std::vector<double> Fz_ref, deltaFtZ;
	Eigen::Vector3d deltaHip;
	double TurnYaw;
	double LftYaw, RftYaw;
	virtual void UpdateWbsRef();

	virtual void InternalLoop();

	//virtual void ImpStabilizer();
	StabilizerClass Sta;
	IKClass IK;

	std::vector<double> traj_ref_old;

	double realtime;

	unsigned int dtime, walkdtime;
	double zc, zh;
	double Ksway;
	double xcop, ycop;
	double HipO_comL, HipO_comR;
	double walkstarttime;

	double steptime, steplength, stepwidth, stepheight, turnangle;
	bool IsStart, IsFixed;
	bool StaEnableX, StaEnableY, StaEnableZ;
	bool HandStaEnableX, HandStaEnableY, HandStaEnableZ;

	int KeyBoardSwitch, JointNUM;

	Eigen::Vector3d xstate, ystate;
	Eigen::Vector3d LeftFootPos, RightFootPos, PelvisPos;
	Eigen::Matrix3d LeftFootO, RightFootO, HipO_Left, HipO_Right, HipO_Left_comp, HipO_Right_comp, HipO_Turn;

	Eigen::Vector3d LeftHandPos, RightHandPos;
	Eigen::Matrix3d LeftHandO, RightHandO;

	Eigen::Vector3d vFullLeg;
	Eigen::Vector3d LeftHandPosGlobal, RightHandPosGlobal;
	Eigen::Matrix3d LeftHandOGlobal, RightHandOGlobal;
	Eigen::Matrix3d WaistO;


	virtual void Init(const double &dT);

	virtual void MoveToInitialPosition();
	virtual void RTControlKeyBoardControl(char cmd);


	Eigen::Vector3d HipPos;
	void SolveIK();

	struct timeval tvIn, tvOut;
	int inMoment, outMoment;

	double initHipx, initHipy;
	bool Is_xCOPinCenter, Is_yCOPinCenter;
	int dcounter, dbuffertime;

	virtual void COMTrajGen();
	Eigen::VectorXd zmpx_window_old, zmpy_window_old;
	Eigen::VectorXd zmpx_window, zmpy_window, zmpx_window_new, zmpy_window_new;
	bool IsInitZMPold;

	double n_RBDL_floating_jnt;
	Eigen::VectorXd q_rbdl;

	Eigen::VectorXd e_i;
	Eigen::VectorXd q_joint_ctrl;
	Eigen::VectorXd q_home;

	virtual void StandingReactStepping() {};
	virtual void WalkingReactStepping() {};
	virtual void EnableStandingReact() {};
	virtual void EnableWalkingReact() {};

	Eigen::Vector3d walkingVel;

	virtual void InternalLoggerInit() {};
	virtual void InternalLoggerLoop() {};

	bool EnableFtPos, EnableFtOri;
	Eigen::Vector3d deltaFtPos_l, deltaFtPos_r;
	Eigen::Matrix3d deltaFtOri_left, deltaFtOri_right;
	
	
	
	/////for new StabilizerClass
	Eigen::Vector3d body_thetax;
	void Admittance_controller();
	
	Eigen::Vector3d det_hip_posotion, det_hip_pose;
	Eigen::Vector6d det_foot_rpy_lr;
	Eigen::Vector2d det_footz_lr;
	
	
	
	Eigen::Vector3d ZMPxy_realx, thetaxyx,comxyzx,Lfootxyzx,Rfootxyzx;
	Eigen::Vector3d M_L, M_R, F_L,F_R;
	
        int j_count,bjx1;
	double tx, td;
	
	Eigen::Matrix3d ankle_Ori_left, ankle_Ori_right;
	Eigen::Vector3d det_ank_foot;
	
	
	Eigen::Vector3d LeftFootPosx,RightFootPosx;	
	
	
	
	
	
	
	

};

#endif
