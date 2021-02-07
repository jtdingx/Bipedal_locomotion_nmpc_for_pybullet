/*****************************************************************************
RTControlBaseClass.cpp

Description:    source file of RTControlBaseClass
*****************************************************************************/
#include "RTControl/RTControlBaseClass.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#ifdef USE_OPENSOT
#include "IK/IKClass_OpenSoT.h"
#endif

RTControlBaseClass::RTControlBaseClass()
	: zc(RobotParaClass::Z_C())
#ifdef USE_OPENSOT
	, zh(RobotParaClass::WAIST_HEIGHT() - RobotParaClass::HOMING_LIFT_HEIGHT())
#else
	, zh(RobotParaClass::FULL_LEG() - RobotParaClass::HOMING_LIFT_HEIGHT())
#endif
	, xstate(0.0, 0.0, 0.0)
	, ystate(0.0, 0.0, 0.0)
	, xcop(0.0)
	, ycop(0.0)
	, LeftFootPos(RobotParaClass::HIP_TO_ANKLE_X_OFFSET(), RobotParaClass::HALF_HIP_WIDTH(), 0)
	, RightFootPos(RobotParaClass::HIP_TO_ANKLE_X_OFFSET(), -RobotParaClass::HALF_HIP_WIDTH(), 0)
	, PelvisPos(0, 0, RobotParaClass::FULL_LEG())
	, IsStart(false)
	, IsInit(false)
	, HipO_comL(0.0)
	, HipO_comR(0.0)
	, dtime(0)
	, walkdtime(0)
	, walkstarttime(0.0)
	, StaEnableX(false)
	, StaEnableY(false)
	, StaEnableZ(false)
	, KeyBoardSwitch(1)
	, steptime(RobotPara().Tstep)
	, steplength(0.0)
	, stepwidth(0.0)
	, stepheight(0.0)
	, turnangle(0.0)
	, JointNUM(RobotParaClass::JOINT_NUM())
	, IsFixed(RobotPara().IsFixedWalk)
	, traj_ref_old(RobotParaClass::JOINT_NUM(), 0.0)
	, _obs_size(0.0, 0.0, 0.0), _obs_center_pos_w(0.0, 0.0, 0.0), _glft(0.0, 0.0, 0.0), _grft(0.0, 0.0, 0.0)
	, RelexedDOF(12, 0.0)
	, initHipx(0.0), initHipy(0.0)
	, Is_xCOPinCenter(false), Is_yCOPinCenter(false)
	, dcounter(0), dbuffertime(0)
	, IsInitZMPold(false)
	, realtime(0.0)
	, Fz_ref(2, 0)
	, deltaFtZ(2, 0)
	, TurnYaw(0.0)
	, HipPos(0, 0, 0)
	, dt(RobotParaClass::dT())
	, IsLoggerSaved(false)
	, vFullLeg(0, 0, RobotParaClass::FULL_LEG())
	, srdf_homing_angles(RobotParaClass::JOINT_NUM(), 0.0)
	, walkingVel(0.0, 0.0, 0.0)
	, EnableFtPos(false)
	, EnableFtOri(false)
	, IsStartWalk(false)
{
        DPRINTF("============== RTcontrol base class start ==========\n\n\n\n");
	deltaFtOri_left = deltaFtOri_right = Eigen::Matrix3d::Identity();
	deltaHip = deltaFtPos_l = deltaFtPos_r = Eigen::Vector3d::Zero();
	com_ref = lft_ref = rft_ref = zmp_ref = Eigen::Vector3d::Zero();


	LeftHandPosGlobal = vFullLeg ;
	RightHandPosGlobal = vFullLeg ;

	LeftFootPos = RobotPara().LEFT_FOOT_HOME_POS;
	RightFootPos = RobotPara().RIGHT_FOOT_HOME_POS;
	LeftFootO = RobotPara().LEFT_FOOT_HOME_ORI;
	RightFootO = RobotPara().RIGHT_FOOT_HOME_ORI;

	LEFT_FOOT_HOME_POS  = RobotPara().LEFT_FOOT_HOME_POS ;
	RIGHT_FOOT_HOME_POS = RobotPara().RIGHT_FOOT_HOME_POS;
	LEFT_FOOT_HOME_ORI  = RobotPara().LEFT_FOOT_HOME_ORI ;
	RIGHT_FOOT_HOME_ORI = RobotPara().RIGHT_FOOT_HOME_ORI;
	LEFT_HAND_HOME_POS  = LEFT_HAND_HOME_POS;
	RIGHT_HAND_HOME_POS = LEFT_HAND_HOME_POS;
// 	LEFT_HAND_HOME_ORI  = RobotPara().LEFT_HAND_HOME_ORI ;
// 	RIGHT_HAND_HOME_ORI = RobotPara().RIGHT_HAND_HOME_ORI;

	q_home = Eigen::VectorXd::Zero(JointNUM);
	std::vector<double> homing_angles(RobotPara().HOMING_POS());
	std::transform(homing_angles.begin(), homing_angles.end(), homing_angles.begin(), std::bind1st(std::multiplies<double>(), M_PI / 180.0));
	HomingInit(homing_angles);

	// set initial elbow angles to avoid singularity
	const std::vector<double>& OffsetAng = RobotPara().HOMING_POS();
	const RobotStateClass& irobot = _WBS.getRobotState();
	for (int i = 0; i < RobotParaClass::JOINT_NUM(); i++) {
		irobot.SendToRobot->q[i] = DEGTORAD(OffsetAng[i]);
	}

	Ksway = RobotPara().Ksway;
	HipO_Left << Eigen::Matrix3d::Identity(3, 3);
	HipO_Right << Eigen::Matrix3d::Identity(3, 3);
	HipO_Left_comp << Eigen::Matrix3d::Identity(3, 3);
	HipO_Right_comp << Eigen::Matrix3d::Identity(3, 3);
	HipO_Turn << Eigen::Matrix3d::Identity(3, 3);
	WaistO << Eigen::Matrix3d::Identity(3, 3);


#ifdef USE_XBOT_LOGGER
	xbot_logger = XBot::MatLogger::getLogger("/tmp/COMAN_RTControl_log");
	initLogger(logger_len);
#endif

	if (RBDL_API_VERSION == irobot._model->rbdl250) {
		n_RBDL_floating_jnt = 2; // for rbdl 2.5.0
	}
	else if (RBDL_API_VERSION == irobot._model->rbdl240) {
		n_RBDL_floating_jnt = 6; // for rbdl 2.4.0
	}
	q_rbdl = Eigen::VectorXd::Zero(n_RBDL_floating_jnt + JointNUM);
	e_i = Eigen::VectorXd::Zero(JointNUM);
	q_joint_ctrl = Eigen::VectorXd::Zero(JointNUM);


	_WBS.EnableGravityCompensation(true);

	q_tmp = Eigen::VectorXd::Zero(23);
	DPRINTF("============== RTControlBaseClass is ready... ==========\n\n\n\n");
	
        body_thetax.setZero();
	det_hip_posotion.setZero(); det_hip_pose.setZero();
	det_foot_rpy_lr.setZero();
	det_footz_lr.setZero();
	
	ZMPxy_realx.setZero();
	thetaxyx.setZero();
	comxyzx.setZero();
	Lfootxyzx.setZero();
	Rfootxyzx.setZero();
	M_L.setZero();
	M_R.setZero(); 
	F_L.setZero();
	F_R.setZero();
	
	F_L(2) = F_R(2)= 9.8/2*RobotPara().totalmass;
	
        j_count =0;
	bjx1 = 0;
	tx = 0;
	td=0;

        ankle_Ori_left.setZero();
	ankle_Ori_right.setZero();
	det_ank_foot.setZero();
        
	LeftFootPosx.setZero();
	LeftFootPosx(1) = RobotParaClass::HALF_HIP_WIDTH();
	RightFootPosx.setZero();
	RightFootPosx(1) = -RobotParaClass::HALF_HIP_WIDTH();	
	
	
}

RTControlBaseClass::~RTControlBaseClass()
{
	savedata();

	DPRINTF("========= RTControlBaseClass destroyed. =============\n");
}

void RTControlBaseClass::Init(const double &dT)
{
	IsInit = true;
	DPRINTF("Gait parameters initialization finished.\n");
}

void RTControlBaseClass::HomingInit(const std::vector<double> &homing_angles)
{
	const RobotStateClass& irobot = _WBS.getRobotState();

	irobot._model->Update(homing_angles);
	IOClass::vout(homing_angles, "homing_angles");
	COUT(irobot._model->q_all_floating.transpose());
// 	RBDL::Math::SpatialTransform LHandFrame = irobot._model->lwrist;
// 	RBDL::Math::SpatialTransform RHandFrame = irobot._model->rwrist;
	RBDL::Math::SpatialTransform LFootFrame = irobot._model->lsole;
	RBDL::Math::SpatialTransform RFootFrame = irobot._model->rsole;

	LEFT_FOOT_HOME_POS = LFootFrame.r;
	RIGHT_FOOT_HOME_POS = RFootFrame.r;
	LEFT_FOOT_HOME_ORI = LFootFrame.E;
	RIGHT_FOOT_HOME_ORI = RFootFrame.E;

// 	LEFT_HAND_HOME_POS = LHandFrame.r;
// 	RIGHT_HAND_HOME_POS = RHandFrame.r;
// 	LEFT_HAND_HOME_ORI = LHandFrame.E;
// 	RIGHT_HAND_HOME_ORI = RHandFrame.E;

	for (int i = 0; i < JointNUM; i++) {
		irobot.SendToRobot->q_ref[i] = irobot.SendToRobot->q[i] = homing_angles[i];
	}
	srdf_homing_angles = homing_angles;
}

void RTControlBaseClass::MoveToInitialPosition()
{
	const RobotStateClass& irobot = _WBS.getRobotState();

	double start_time = RobotPara().setParaForSimOrReal(0.5, 3.0);

	if (realtime == 0.0) {
		DPRINTF("\n======== Start moving to initial postion ===============\n");
	}

	if (realtime <= start_time) {
		double scale = 0.5 * (1 - std::cos(M_PI * realtime / start_time));

		// for analytic IK, PelvisPos[2] is the leg's first joint height, for OpenSOT, PelvisPos[2] is the actual waist height
		PelvisPos[2] = RobotParaClass::FULL_LEG() - RobotParaClass::HOMING_LIFT_HEIGHT() * scale;


		Eigen::Vector3d vWorldToWaist(0.0, 0.0, RobotParaClass::WAIST_HEIGHT());
		if (RobotPara().name == "walkman") {
			vWorldToWaist[0] = -0.051; // for new walkman upper body only, because the urdf of the waist was changed
		}
		Eigen::Vector3d LftHomingPosGlobal = vWorldToWaist + LEFT_FOOT_HOME_POS; // actual foot position in global frame
		Eigen::Vector3d RftHomingPosGlobal = vWorldToWaist + RIGHT_FOOT_HOME_POS;
		Eigen::Vector3d LFootHomingDistance = Eigen::Vector3d(0.0, RobotParaClass::HALF_FOOT_DIS(), 0.0) - LftHomingPosGlobal; // difference with ref foot position
		Eigen::Vector3d RFootHomingDistance = Eigen::Vector3d(0.0, -RobotParaClass::HALF_FOOT_DIS(), 0.0) - RftHomingPosGlobal;

		LeftFootPos = LftHomingPosGlobal + scale * LFootHomingDistance;
		RightFootPos = RftHomingPosGlobal + scale * RFootHomingDistance;
		// LeftFootO = ZMPwalk.SwingFoot->SwingFootRGen(0.0, start_time, realtime, LEFT_FOOT_HOME_ORI, Eigen::Matrix3d::Identity());
		// RightFootO = ZMPwalk.SwingFoot->SwingFootRGen(0.0, start_time, realtime, RIGHT_FOOT_HOME_ORI, Eigen::Matrix3d::Identity());

		Eigen::Vector3d com_ref_eigen = PelvisPos;
		Eigen::Vector3d lft_ref_eigen = LeftFootPos;
		Eigen::Vector3d rft_ref_eigen = RightFootPos;
		Eigen::Vector3d zmp_ref_eigen(0.0, 0.0, 0.0);
		Eigen::Vector3d fz_ref_eigen(0.5, 0.5, 0);

		_WBS.UpdateRef(0.0, com_ref_eigen, lft_ref_eigen, rft_ref_eigen, zmp_ref_eigen, fz_ref_eigen);

		for (int i = 15; i < srdf_homing_angles.size(); i++) {
			irobot.SendToRobot->q_ref[i] = irobot.SendToRobot->q[i] = srdf_homing_angles[i] + (DEGTORAD(RobotPara().HOMING_POS()[i]) - srdf_homing_angles[i]) * scale;
		}

		if (realtime == start_time) {
			DPRINTF("Finish moving to initial postion.\n");
			DPRINTF("CoM is at %.4f\t%.4f\t%.4f m.\n\n", irobot._model->com_ft[0], irobot._model->com_ft[1], irobot._model->com_ft[2]);
			DPRINTF("\n================================================\n");
			DPRINTF("Normal Keyboard Control is running, Press\n");
			DPRINTF("v:     Switch to AutoWalk Keyboard.\n");
			DPRINTF("x/y/z: Enable stabilizer in x/y/z, respectively.\n");
			DPRINTF("d:     Disable stabilizer in x, y and z.\n");
			DPRINTF("c:     Start/Restart Walking.\n");
			DPRINTF("s:     Stop Walking.\n");
			DPRINTF("F:     Enable/Disable Fixed Walking.\n");
			DPRINTF("B:     Enable/Disable Stop wiht parallel feet.\n");
			DPRINTF("e:     Enable standing reactive stepping.\n");
			DPRINTF("================================================\n");
			// KeyBoardControl('x');
			// KeyBoardControl('y');
		}
	}
	else {
		if (!IsInit) {
			Init(dt);
		}
	}
}

void RTControlBaseClass::UpdateWBS(const double & dT, const std::vector<double> &qall, const Eigen::Vector3d &EulerAng, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor)
{
	//************************ implementation of WBS class *******************************************************
	Eigen::Matrix3d Rpelvis_abs = Rz(EulerAng[2]) * Ry(EulerAng[1]) * Rx(EulerAng[0]);
	_WBS.UpdateIMU(dT, Rpelvis_abs, Acc, AngleRate, EulerAng);
	_WBS.UpdateRobotState(qall, dT, FTSensor);
	_WBS.UpdateHandFT(HandFTSensor);
	//***************************************************************
}

void RTControlBaseClass::UpdateWBS(const std::vector<double> &qall, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor)
{
	//************* implementation of WBS class ********************
	_WBS.UpdateIMU(RobotParaClass::dT(), Rpelvis_abs, Acc, AngleRate);
	_WBS.UpdateRobotState(qall, RobotParaClass::dT(), FTSensor);
	//**************************************************************

}

void RTControlBaseClass::Run(const double &dT, std::vector<double> &pos, const std::vector<float> &OffsetAng)
{
	this->Run();
}

void RTControlBaseClass::Run()
{
	const RobotStateClass& irobot = _WBS.getRobotState();
	realtime = dtime * dt;
	MoveToInitialPosition();

	Eigen::Vector3d ft_com(0, RobotParaClass::HALF_FOOT_DIS(), 0), ft_dcom(0, 0, 0), state0(0, RobotParaClass::HALF_FOOT_DIS(), 0);
	std::vector<double> applyDeltaPosX(3, 0), applyDeltaPosY(3, 0);

// 	  COUT("IsStartWalk", IsStartWalk);
	if (IsInit && IsStartWalk) {
	  
// 	  COUT("aaaaaaaaaaa");
		if (walkdtime == 0) {
			walkstarttime = dt * dtime;
		}
		double UpdateTcycle = 1 * dt;
		double time = realtime - walkstarttime;
		if (IsFixed) {
			
		}
		else {
		  
// 	  COUT("bbbbbbbbbbb");
			WalkingReactStepping();
		}

		COMTrajGen();
		walkdtime++;
	}


	if (IsInit) {
		//ImpStabilizer();
		//StandingReactStepping();
	}

	SolveIK();

#ifdef USE_XBOT_LOGGER
	addToLog();
#endif
	dtime++;
	
}

void RTControlBaseClass::InternalLoop()
{

}

////// keyboard control for push
static int jnt_idx = 12;
void RTControlBaseClass::KeyBoardControl(char cmd)
{
		StartWalking();  
	switch (cmd)
	{
	case 'c':
	{
		StartWalking();

	}
	break;

	case 's':
	{
		DPRINTF("Stop Walking! mm( _ _ )mm......o\n");
		steplength = 0;
		stepwidth = 0;
		StopWalking();
// 		DPRINTF("Reset Step Length to %.3f m!\n", steplength);
// 		DPRINTF("Reset Step Width to %.3f m!\n", stepwidth);

	}
	break;

	case 'e':
	{
		EnableStandingReact();
	}
	break;

	case 'E':
	{
		EnableWalkingReact();
	}
	break;

	case 'v':
	{
		KeyBoardSwitch *= -1;
		if (KeyBoardSwitch == 1) {
			DPRINTF("\n================================================\n");
			DPRINTF("Switched to Normal RTControl Control.\n");
			DPRINTF("v:     Switch to AutoWalk Keyboard.\n");
			DPRINTF("x/y/z: Enable stabilizer in x/y/z, respectively.\n");
			DPRINTF("d:     Disable stabilizer in x, y and z.\n");
			DPRINTF("c:     Start/Restart Walking.\n");
			DPRINTF("s:     Stop Walking.\n");
			DPRINTF("F:     Enable/Disable Fixed Walking.\n");
			DPRINTF("B:     Enable/Disable Stop wiht parallel feet.\n");
			DPRINTF("e:     Enable standing reactive stepping.\n");
			DPRINTF("================================================\n");
		}
		else if (KeyBoardSwitch == -1) {
			DPRINTF("\n================================================\n");
			DPRINTF("Switched to RTControl Keyboard Control.\n");
			DPRINTF("v:     Switch to Normal Keyboard.\n");
			DPRINTF("o/p:   Inc./Dec. Step Length.\n");
			DPRINTF("k/l:   Inc./Dec. Step Width.\n");
			DPRINTF("n/m:   Turn Left/Right.\n");
			DPRINTF("O/P:   Inc./Dec. Walking Velocity.\n");
			DPRINTF("U/I:   Inc./Dec. Step Time.\n");
			DPRINTF("================================================\n");
		}
		else {
			DPRINTF("Error! No Such Keyboard Control.\n");
		}
	}
	break;

	default:
		break;
	}

	if (KeyBoardSwitch == -1) {
		RTControlKeyBoardControl(cmd);
	}
	else {
		double interval = 0.1;
		switch (cmd)
		{

		case 'x':
		{
			StaEnableX = true;
			DPRINTF("Enable Stabilizer X!\n");
		}
		break;

		case 'y':
		{
			StaEnableY = true;
			DPRINTF("Enable Stabilizer Y!\n");
		}
		break;

		case 'z':
		{
			StaEnableZ = true;
			DPRINTF("Enable Stabilizer Z!\n");
		}
		break;

		case 'd':
		{
			StaEnableX = false;
			StaEnableY = false;
			StaEnableZ = false;
			DPRINTF("Disable Stabilizer X Y Z!\n");
		}
		break;

		case 'i':
		{
			RobotPara().kd_debug += interval;
			DPRINTF("Increase kd_debug to %.2f\n", RobotPara().kd_debug);
		}
		break;

		case 'o':
		{
			RobotPara().kd_debug -= interval;
			DPRINTF("Decrease kd_debug to %.2f\n", RobotPara().kd_debug);
		}
		break;

		default:
			break;
		}
	}
}

void RTControlBaseClass::RTControlKeyBoardControl(char cmd)
{
	double delta_sl = 0.002;
	double delta_sw = 0.002;
	double delta_ta = 2.5;
	double maxVel = 0.2;
	double minVel = -maxVel;

	switch (cmd)
	{
	case 'z':
	{
		EnableFtPos = true;
		DPRINTF("Enable Foot Position Control!\n");
	}
	break;

	case 'Z':
	{
		EnableFtOri = true;
		DPRINTF("Enable Foot Orientation Control!\n");
	}
	break;

	case 'd':
	{
		EnableFtPos = false;
		EnableFtOri = false;
		DPRINTF("Disable Landing Foot Control!\n");
	}
	break;

	case 'k':
	{
		stepwidth += delta_sw;
		DPRINTF("Increase Step Width to %.3f m!\n", stepwidth);
	}
	break;

	case 'l':
	{
		stepwidth -= delta_sw;
		DPRINTF("Decrease Step Width to %.3f m!\n", stepwidth);
	}
	break;

	case 'n':
	{
		turnangle += delta_ta;
		DPRINTF("Change Heading Angle to %.3f degree!\n", turnangle);
	}
	break;

	case 'm':
	{
		turnangle -= delta_ta;
		DPRINTF("Change Heading Angle to %.3f degree!\n", turnangle);
	}
	break;

	case 'g':
	{
		initHipx += 0.001;
		DPRINTF("Increase deltaHip X to %.3f m!\n", initHipx);
	}
	break;

	case 'h':
	{
		initHipx -= 0.001;
		DPRINTF("Decrease deltaHip X to %.3f m!\n", initHipx);
	}
	break;

	case 'G':
	{
		initHipy += 0.001;
		DPRINTF("Increase deltaHip Y to %.3f m!\n", initHipy);
	}
	break;

	case 'H':
	{
		initHipy -= 0.001;
		DPRINTF("Decrease deltaHip Y to %.3f m!\n", initHipy);
	}
	break;

	case 'u':
	{
		Ksway += 0.01;
		DPRINTF("Increase Ksway to %.3f\n", Ksway);
	}
	break;

	case 'i':
	{
		Ksway -= 0.01;
		DPRINTF("Decrease Ksway to %.3f\n", Ksway);
	}
	break;

	case 'N':
	{
		stepheight += 0.0005;
		clamp(stepheight, 0.0, RobotParaClass::HOMING_LIFT_HEIGHT());
		// clamp(stepheight, -RobotParaClass::HOMING_LIFT_HEIGHT(), RobotParaClass::HOMING_LIFT_HEIGHT());
		DPRINTF("Change Step Height to %.4f m!\n", stepheight);
	}
	break;

	case 'M':
	{
		stepheight -= 0.0005;
		clamp(stepheight, 0.0, RobotParaClass::HOMING_LIFT_HEIGHT());
		// clamp(stepheight, -RobotParaClass::HOMING_LIFT_HEIGHT(), RobotParaClass::HOMING_LIFT_HEIGHT());
		DPRINTF("Change Step Height to %.4f m!\n", stepheight);
	}
	break;

	default:
		break;
	}
}

void RTControlBaseClass::COMTrajGen()
{
  PelvisPos[1] *= Ksway;
	// ZMPwalk.FootTraj(LeftFootPos, RightFootPos, LeftFootO, RightFootO, HipO_Turn);

	// PelvisPos << hip_pos_eigen[0], hip_pos_eigen[1], zh + deltaCOMz;

}

void RTControlBaseClass::savedata()
{
	if (!IsLoggerSaved) {
#ifdef USE_XBOT_LOGGER
		DPRINTF("=========        Logging Data .....      =============\n");
		xbot_logger->flush();
#endif
		IsLoggerSaved = true;

		DPRINTF("=========     Finish Logging Data .....  =============\n");
	}
}

void RTControlBaseClass::UpdateWbsRef()
{
	Eigen::Matrix3d TurnYawO = Rz(TurnYaw);
	for (unsigned int i = 0; i < 3; i++) {
		lft_ref[i] = LeftFootPos[i];
		rft_ref[i] = RightFootPos[i];
	}
	com_ref[0] = PelvisPos[0];
	com_ref[1] = PelvisPos[1];
	com_ref[2] = PelvisPos[2];

	const RobotStateClass& irobot = _WBS.getRobotState();
	irobot.SendToRobot->gLftRef = LeftFootPos;
	irobot.SendToRobot->gRftRef = RightFootPos;
	irobot.SendToRobot->gComRef = PelvisPos;
	irobot.SendToRobot->gZmpRef[0] = zmp_ref[0];
	irobot.SendToRobot->gZmpRef[1] = zmp_ref[1];

	Eigen::Vector3d com_ref_eigen(0, 0, 0), lft_ref_eigen(0, 0, 0), rft_ref_eigen(0, 0, 0), zmp_ref_eigen(0, 0, 0), fz_ref_eigen(0, 0, 0);
	for (unsigned int i = 0; i < 2; i++) {
		com_ref_eigen[i] = com_ref[i];
		lft_ref_eigen[i] = lft_ref[i];
		rft_ref_eigen[i] = rft_ref[i];
		zmp_ref_eigen[i] = zmp_ref[i];
		fz_ref_eigen[i] = Fz_ref[i];
	}
	com_ref_eigen = TurnYawO.transpose() * com_ref_eigen;
	lft_ref_eigen = TurnYawO.transpose() * lft_ref_eigen;
	rft_ref_eigen = TurnYawO.transpose() * rft_ref_eigen;
	zmp_ref_eigen = TurnYawO.transpose() * zmp_ref_eigen;
	for (unsigned int i = 2; i < 3; i++) {
		com_ref_eigen[i] = com_ref[i];
		lft_ref_eigen[i] = lft_ref[i];
		rft_ref_eigen[i] = rft_ref[i];
		zmp_ref_eigen[i] = zmp_ref[i];
	}

	double delaytime = RobotPara().setParaForSimOrReal(0.0, 0.02);

	_WBS.UpdateRef(delaytime, com_ref_eigen, lft_ref_eigen, rft_ref_eigen, zmp_ref_eigen, fz_ref_eigen);

}

// void RTControlBaseClass::ImpStabilizer()
// {
// 	const RobotStateClass& irobot = _WBS.getRobotState();
// 	Eigen::Matrix3d TurnYawO = Rz(TurnYaw);
// 	UpdateWbsRef();
// 	//Sta.setFilterPara(5, 3);
// 	Sta.zc(zc);
// 	double VerticalScale = RobotPara().setParaForSimOrReal(0.9538, 1.005);
// 	Sta.VerticalScale(VerticalScale);
// 	//******* for walking **************
// 	Sta.Kd(RobotPara().Kx,	RobotPara().Ky,	RobotPara().Kz);
// 	Sta.Bd(RobotPara().Bx,	RobotPara().By,	RobotPara().Bz);
// 	Sta.Enable(StaEnableX, StaEnableY, StaEnableZ);
// 	deltaHip = Sta.StabilizerCart(irobot);
// 
// 	deltaHip = TurnYawO * deltaHip;
// }

void RTControlBaseClass::Admittance_controller()
{
//    const RobotStateClass& irobot = _WBS.getRobotState();
//   UpdateWbsRef();
//  det_hip_posotion = Sta.COMdampingCtrl(bjx1,Lfootxyzx,Rfootxyzx,zmp_ref,irobot);
//   det_hip_pose = Sta.COMangleCtrl(bjx1,thetaxyx,comxyzx,Lfootxyzx,Rfootxyzx,irobot);
//   det_foot_rpy_lr = Sta.FootdampiingCtrol_LR(bjx1, j_count, tx, td, M_L, M_R,irobot,IsStartWalk);
//   det_footz_lr = Sta.ForcediffCtrol_LR(bjx1, F_L,F_R,irobot,IsStartWalk);
//   
//   
//   HipO_Turn = Rz(body_thetax[2]+det_hip_pose[2])*Ry(body_thetax[1]+det_hip_pose[1])*Rx(body_thetax[0]+det_hip_pose[0]);
//   
//   deltaHip = det_hip_posotion;
//   
//   
//   /// deltaFtOri_left, deltaFtOri_right are the matrix rotation of the local framework
//   deltaFtOri_left = Rz(det_hip_pose(2))*Ry(det_hip_pose(1))*Rx(det_hip_pose(0));
//   
//   deltaFtOri_right = Rz(det_hip_pose(5))*Ry(det_hip_pose(4))*Rx(det_hip_pose(3));
// //   
// //   ankle_Ori_left = Rz(det_hip_pose(2))*Ry(det_hip_pose(1))*Rx(det_hip_pose(0));
// //   ankle_Ori_right = Rz(det_hip_pose(5))*Ry(det_hip_pose(4))*Rx(det_hip_pose(3));
//   
}



void RTControlBaseClass::SolveIK()
{

	const RobotStateClass& irobot = _WBS.getRobotState();

	// double dT = irobot.dt;


// 	std::vector<double> qL(6, 0);
// 	std::vector<double> qR(6, 0);
// 	Eigen::Vector6d qL, qR;

	Eigen::Matrix3d TurnYawO = Rz(TurnYaw);
	Eigen::Vector3d modCopHip(initHipx, initHipy, 0);
	modCopHip = TurnYawO * modCopHip;

	HipPos.setZero();
	HipPos[0] += modCopHip[0];
	HipPos[1] += modCopHip[1];

	HipPos += PelvisPos;
	HipPos += deltaHip;
	

	double yaw = 0.0;
	Eigen::Matrix3d PelvisYaw = Rz(yaw); // Pelvis Yaw orientation

	Eigen::Matrix3d HipO_Left_Ref = HipO_Left_comp;
	Eigen::Matrix3d HipO_Right_Ref = HipO_Right_comp;

	HipO_Left = HipO_Turn * PelvisYaw * HipO_Left_Ref;
	HipO_Right = HipO_Turn * PelvisYaw * HipO_Right_Ref;
	WaistO = HipO_Turn * PelvisYaw;

	Eigen::Vector3d LeftAnklePos = LeftFootPosx;
	LeftAnklePos += deltaFtPos_l;
	LeftAnklePos[2] += deltaFtZ[0] + RobotParaClass::ANKLE_HEIGHT(); // probably has potential risk that into considering the orientation of the foot
	Eigen::Vector3d RightAnklePos = RightFootPosx;
	RightAnklePos += deltaFtPos_r;
	RightAnklePos[2] += deltaFtZ[1] + RobotParaClass::ANKLE_HEIGHT();


	irobot.SendToRobot->HipPos = HipPos;
	irobot.SendToRobot->LeftAnklePos = LeftAnklePos;
	irobot.SendToRobot->RightAnklePos = RightAnklePos;
	irobot.SendToRobot->HipO_Left = HipO_Left;
	irobot.SendToRobot->HipO_Right = HipO_Right;
	irobot.SendToRobot->LeftFootO = LeftFootO;
	irobot.SendToRobot->RightFootO = RightFootO;

	Eigen::Vector3d FinalLeftFootPos = LeftFootPos;
	FinalLeftFootPos[2] += deltaFtPos_l[2];
	Eigen::Vector3d FinalRightFootPos = RightFootPos;
	FinalRightFootPos[2] += deltaFtPos_r[2];

	Eigen::Matrix3d FinalLeftFootOri = LeftFootO * deltaFtOri_left;
	Eigen::Matrix3d FinalRightFootOri = RightFootO * deltaFtOri_right;

	if(RobotPara().name == "coman"){
	  std::vector<double> qL(6, 0);
	  std::vector<double> qR(6, 0);
	  IK.LegInverseKinematics(HipPos, HipO_Left, LeftAnklePos, FinalLeftFootOri, irobot.hipwidth, qL);
	  IK.LegInverseKinematics(HipPos, HipO_Right, RightAnklePos, FinalRightFootOri, -irobot.hipwidth , qR);
	  irobot.SendToRobot->q[RIGHT_HIP_PITCH ] = qR[0]; //right hip pitch
	  irobot.SendToRobot->q[RIGHT_HIP_ROLL  ] = qR[1]; //right hip roll
	  irobot.SendToRobot->q[RIGHT_HIP_YAW   ] = qR[2]; //right hip yaw
	  irobot.SendToRobot->q[RIGHT_KNEE_PITCH] = qR[3]; //right knee
	  irobot.SendToRobot->q[RIGHT_FOOT_ROLL ] = qR[4]; //right ankle pitch
	  irobot.SendToRobot->q[RIGHT_FOOT_PITCH] = qR[5]; //right ankle roll
	  irobot.SendToRobot->q[LEFT_HIP_PITCH  ] = qL[0]; //left hip pitch
	  irobot.SendToRobot->q[LEFT_HIP_ROLL   ] = qL[1]; //left hip roll
	  irobot.SendToRobot->q[LEFT_HIP_YAW    ] = qL[2]; //left hip yaw
	  irobot.SendToRobot->q[LEFT_KNEE_PITCH ] = qL[3]; //left knee
	  irobot.SendToRobot->q[LEFT_FOOT_ROLL  ] = qL[4]; //left ankle pitch
	  irobot.SendToRobot->q[LEFT_FOOT_PITCH ] = qL[5]; //left ankle roll
	}
	else if(RobotPara().name == "bigman"){
	  Eigen::Vector6d qL, qR;
	  IK.BigmanLegIK(HipPos, HipO_Left, LeftAnklePos, FinalLeftFootOri, "L", qL);
	  IK.BigmanLegIK(HipPos, HipO_Right, RightAnklePos, FinalRightFootOri, "R", qR);
	  irobot.SendToRobot->q[RIGHT_HIP_PITCH ] = qR[2];
	  irobot.SendToRobot->q[RIGHT_HIP_ROLL  ] = qR[0];
	  irobot.SendToRobot->q[RIGHT_HIP_YAW   ] = qR[1];
	  irobot.SendToRobot->q[RIGHT_KNEE_PITCH] = qR[3];
	  irobot.SendToRobot->q[RIGHT_FOOT_ROLL ] = qR[5];
	  irobot.SendToRobot->q[RIGHT_FOOT_PITCH] = qR[4];
	  irobot.SendToRobot->q[LEFT_HIP_PITCH  ] = qL[2];
	  irobot.SendToRobot->q[LEFT_HIP_ROLL   ] = qL[0];
	  irobot.SendToRobot->q[LEFT_HIP_YAW    ] = qL[1];
	  irobot.SendToRobot->q[LEFT_KNEE_PITCH ] = qL[3];
	  irobot.SendToRobot->q[LEFT_FOOT_ROLL  ] = qL[5];
	  irobot.SendToRobot->q[LEFT_FOOT_PITCH ] = qL[4];
	}
	else{
	  DPRINTF("Errorrrrrrrr for IK\n");
	}


	double ki = 0.0;
// 	for (int i = 3; i < 15; i++) {
	for (int i = 0; i < JointNUM; i++) {
		e_i[i] += irobot.q_all[i] - irobot.SendToRobot->q[i];
		irobot.SendToRobot->q_ref[i] = irobot.SendToRobot->q[i] + _WBS.q_off[i] + (ki * e_i[i]) * dt;
// 		DPRINTF("%.4f,\t", RADTODEG(irobot.SendToRobot->q_ref[i]));
	}
// 	DPRINTF("\n");

}

void RTControlBaseClass::SortFromCOMAN(std::vector<double> &from)
{
	std::vector<double> tmp(from);

	assert(from.size() == 31);

	from[WAIST_YAW			] = tmp[ 0];
	from[WAIST_PITCH		] = tmp[ 1];
	from[WAIST_ROLL			] = tmp[ 2];
	from[RIGHT_HIP_PITCH	] = tmp[ 3];
	from[RIGHT_HIP_ROLL		] = tmp[ 5];
	from[RIGHT_HIP_YAW		] = tmp[ 6];
	from[RIGHT_KNEE_PITCH	] = tmp[ 7];
	from[RIGHT_FOOT_ROLL	] = tmp[ 9];
	from[RIGHT_FOOT_PITCH	] = tmp[ 8];
	from[LEFT_HIP_PITCH	] = tmp[ 4];
	from[LEFT_HIP_ROLL	] = tmp[10];
	from[LEFT_HIP_YAW	] = tmp[11];
	from[LEFT_KNEE_PITCH	] = tmp[12];
	from[LEFT_FOOT_ROLL	] = tmp[14];
	from[LEFT_FOOT_PITCH	] = tmp[13];
	from[NECK_PITCH		] = tmp[23];
	from[HEAD_PITCH		] = tmp[24];	
// 	from[RIGHT_SHOULDER_PITCH] = tmp[15];
// 	from[RIGHT_SHOULDER_ROLL] = tmp[16];
// 	from[RIGHT_SHOULDER_YAW	] = tmp[17];
// 	from[RIGHT_ELBOW_PITCH	] = tmp[18];
// 	from[RIGHT_FOREARM_YAW	] = tmp[25];
// 	from[RIGHT_WRIST_PITCH	] = tmp[26];
// 	from[RIGHT_WRIST_ROLL	] = tmp[27];
// 	from[LEFT_SHOULDER_PITCH] = tmp[19];
// 	from[LEFT_SHOULDER_ROLL	] = tmp[20];
// 	from[LEFT_SHOULDER_YAW	] = tmp[21];
// 	from[LEFT_ELBOW_PITCH	] = tmp[22];
// 	from[LEFT_FOREARM_YAW	] = tmp[28];
// 	from[LEFT_WRIST_PITCH	] = tmp[29];
// 	from[LEFT_WRIST_ROLL	] = tmp[30];
// 	from[NECK_PITCH			] = tmp[23];
// 	from[HEAD_PITCH			] = tmp[24];

}

void RTControlBaseClass::SortFromXBotToJointName(const Eigen::VectorXd &from, std::vector<double> &to)
{
	assert(from.size() == JointNUM);
	assert(to.size() == JointNUM);

	const RobotStateClass& irobot = _WBS.getRobotState();

	q_rbdl.segment(n_RBDL_floating_jnt, from.size()) = from;
	irobot._model->vRBDLToJointName(q_rbdl, to);
}

void RTControlBaseClass::SortFromXBotToJointName(const Eigen::VectorXd &from, Eigen::VectorXd &to)
{
	assert(from.size() == JointNUM);
	assert(to.size() == JointNUM);

	const RobotStateClass& irobot = _WBS.getRobotState();

	q_rbdl.segment(n_RBDL_floating_jnt, JointNUM) = from;
	irobot._model->vRBDLToJointName(q_rbdl, to);
}

template <class T>
void RTControlBaseClass::SortFromJointNameToXBot(const T &from, std::unordered_map<std::string, double> &to)
{
	assert(from.size() == JointNUM);
	assert(to.size() == JointNUM);

	for ( const auto& jnt : to ) {
		jnt.second = from[RobotPara().getJointName(jnt.first)];
	}
}

void RTControlBaseClass::JointRefToXBot(Eigen::VectorXd &to)
{
	assert(to.size() == JointNUM);
	const RobotStateClass& irobot = _WBS.getRobotState();

	irobot._model->vJointNameToRBDL(irobot.SendToRobot->q_ref, q_tmp);
	to = q_tmp.segment(6, to.size());
}

void RTControlBaseClass::JointRefToXBot_LowerBody(Eigen::VectorXd &to)
{
	assert(to.size() == JointNUM);
	const RobotStateClass& irobot = _WBS.getRobotState();

	irobot._model->vJointNameToRBDL(irobot.SendToRobot->q_ref, q_tmp);
//         to.segment(0, 12) = q_tmp.segment(6, 12); //only update the lower body // rbdl 2.4.0, not tested in rbdl 2.5.0, but should be the same
	to.segment(0, 12) = q_tmp.segment(6, 12);
}


#ifdef USE_XBOT_LOGGER
void RTControlBaseClass::initLogger(int buffer_size, int interleave)
{
	xbot_logger->createScalarVariable("time", interleave, buffer_size);
	xbot_logger->createScalarVariable("WhichFoot", interleave, buffer_size);
	xbot_logger->createScalarVariable("WhichFootRef", interleave, buffer_size);
	xbot_logger->createScalarVariable("Fzl_ref", interleave, buffer_size);
	xbot_logger->createScalarVariable("Fzr_ref", interleave, buffer_size);
	xbot_logger->createScalarVariable("FallPredictor", interleave, buffer_size);
	xbot_logger->createScalarVariable("E_p", interleave, buffer_size);
	xbot_logger->createScalarVariable("E_k", interleave, buffer_size);
	xbot_logger->createScalarVariable("irobot_scale", interleave, buffer_size);

	xbot_logger->createVectorVariable("ComRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("ZmpRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gComRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gLftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gRftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gZmpRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gcom", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gdcom", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gddcom", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glft", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grft", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("cop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gcop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glcop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grcop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gCapturePoint", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_Euler", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_LinearVel_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_LinearAcc_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_AngularVel_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_AngularAcc_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_foot_left", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_foot_right", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_fl_filter", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_fr_filter", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("q_msr", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("q_des", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("q_ref", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("tau_msr", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("L_com", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LhdRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RhdRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaHip", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaFtZ", 2, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaFtPos_l", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaFtPos_r", 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("gcom_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gdcom_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gddcom_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glft_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grft_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("cop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gcop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glcop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grcop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("lankle", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("rankle", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("Lft", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("Rft", 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("des_lft_vel", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_rft_vel", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_lft_acc", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_rft_acc", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_lhd_vel", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_rhd_vel", 6, interleave, buffer_size);

	xbot_logger->createMatrixVariable("IMU_abs", 3, 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("PelvisPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LeftFootPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RightFootPos", 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("HipPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LeftAnklePos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RightAnklePos", 3, interleave, buffer_size);

	xbot_logger->createMatrixVariable("HipO_Left", 3, 3, interleave, buffer_size);
	xbot_logger->createMatrixVariable("HipO_Right", 3, 3, interleave, buffer_size);
	xbot_logger->createMatrixVariable("LeftFootO", 3, 3, interleave, buffer_size);
	xbot_logger->createMatrixVariable("RightFootO", 3, 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("stabilizer_torque", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("applyDeltaPosX", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("applyDeltaPosY", 3, interleave, buffer_size);

	xbot_logger->createScalarVariable("TurnYaw", interleave, buffer_size);
	xbot_logger->createScalarVariable("LftYaw", interleave, buffer_size);
	xbot_logger->createScalarVariable("RftYaw", interleave, buffer_size);
	
	
	xbot_logger->createVectorVariable("det_hip_posotion", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("det_hip_pose", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("det_foot_rpy_lr", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("det_footz_lr", 2, interleave, buffer_size);	
	
	

	Sta.initLogger(xbot_logger, logger_len);

	InternalLoggerInit();
}

void RTControlBaseClass::addToLog()
{
	const RobotStateClass& irobot = _WBS.getRobotState();

	xbot_logger->add("time", realtime);
	xbot_logger->add("WhichFoot", (irobot.WhichFoot));
	xbot_logger->add("WhichFootRef", (irobot.WhichFootRef));
	xbot_logger->add("Fzl_ref", (irobot.FzRef[0]));
	xbot_logger->add("Fzr_ref", (irobot.FzRef[1]));
	xbot_logger->add("FallPredictor", (irobot.fall_state));
	xbot_logger->add("E_p", irobot._model->Ep);
	xbot_logger->add("E_k", irobot._model->Ek);
	xbot_logger->add("irobot_scale", (irobot.scale));

	xbot_logger->add("ComRef", irobot.ComRef);
	xbot_logger->add("LftRef", irobot.LftRef);
	xbot_logger->add("RftRef", irobot.RftRef);
	xbot_logger->add("ZmpRef", irobot.ZmpRef);
	xbot_logger->add("gComRef", com_ref);
	xbot_logger->add("gLftRef", lft_ref);
	xbot_logger->add("gRftRef", rft_ref);
	xbot_logger->add("gZmpRef", zmp_ref);
	xbot_logger->add("gcom", irobot.gcom);
	xbot_logger->add("gdcom", irobot.gdcom);
	xbot_logger->add("gddcom", irobot.gddcom);
	xbot_logger->add("glft", irobot.glft);
	xbot_logger->add("grft", irobot.grft);
	xbot_logger->add("cop", irobot.cop);
	xbot_logger->add("gcop", irobot.gcop);
	xbot_logger->add("glcop", irobot.glcop);
	xbot_logger->add("grcop", irobot.grcop);
	xbot_logger->add("gCapturePoint", irobot.gCapturePoint);
	xbot_logger->add("IMU_Euler", irobot.IMU_Euler);
	xbot_logger->add("IMU_LinearVel_raw", irobot.IMU_LinearVel_raw);
	xbot_logger->add("IMU_LinearAcc_raw", irobot.IMU_LinearAcc_raw);
	xbot_logger->add("IMU_AngularVel_raw", irobot.IMU_AngularVel_raw);
	xbot_logger->add("IMU_AngularAcc_raw", irobot.IMU_AngularAcc_raw);
	xbot_logger->add("FT_foot_left", irobot.FT_foot_left);
	xbot_logger->add("FT_foot_right", irobot.FT_foot_right);
	xbot_logger->add("FT_fl_filter", irobot.FT_fl_filter);
	xbot_logger->add("FT_fr_filter", irobot.FT_fr_filter);
	xbot_logger->add("q_msr", irobot.q_all_mesure);
	xbot_logger->add("q_des", irobot.SendToRobot->q);
	xbot_logger->add("q_ref", irobot.SendToRobot->q_ref);
	xbot_logger->add("tau_msr", irobot.tau_all);
	xbot_logger->add("L_com", irobot._model->angular_momentum);
	xbot_logger->add("LhdRef", irobot.LhdRef);
	xbot_logger->add("RhdRef", irobot.RhdRef);
	xbot_logger->add("deltaHip", deltaHip);
	xbot_logger->add("deltaFtZ", deltaFtZ);
	xbot_logger->add("deltaFtPos_l", deltaFtPos_l);
	xbot_logger->add("deltaFtPos_r", deltaFtPos_r);

	xbot_logger->add("gcom_raw", irobot.gcom_raw);
	xbot_logger->add("gdcom_raw", irobot.gdcom_raw);
	xbot_logger->add("gddcom_raw", irobot.gddcom_raw);
	xbot_logger->add("glft_raw", irobot.glft_raw);
	xbot_logger->add("grft_raw", irobot.grft_raw);
	xbot_logger->add("cop_raw", irobot.cop_raw);
	xbot_logger->add("gcop_raw", irobot.gcop_raw);
	xbot_logger->add("glcop_raw", irobot.glcop_raw);
	xbot_logger->add("grcop_raw", irobot.grcop_raw);
	xbot_logger->add("lankle", irobot.lankle);
	xbot_logger->add("rankle", irobot.rankle);
	xbot_logger->add("Lft", irobot.Lft);
	xbot_logger->add("Rft", irobot.Rft);

	xbot_logger->add("des_lft_vel", irobot.SendToRobot->des_lft_vel);
	xbot_logger->add("des_rft_vel", irobot.SendToRobot->des_rft_vel);
	xbot_logger->add("des_lft_acc", irobot.SendToRobot->des_lft_acc);
	xbot_logger->add("des_rft_acc", irobot.SendToRobot->des_rft_acc);
	xbot_logger->add("des_lhd_vel", irobot.SendToRobot->des_lhd_vel);
	xbot_logger->add("des_rhd_vel", irobot.SendToRobot->des_rhd_vel);

	xbot_logger->add("IMU_abs", irobot.IMU_abs);

	xbot_logger->add("PelvisPos", irobot.SendToRobot->HipPos);
	xbot_logger->add("LeftFootPos", irobot.SendToRobot->LeftAnklePos);
	xbot_logger->add("RightFootPos", irobot.SendToRobot->RightAnklePos);

	xbot_logger->add("HipPos", irobot.SendToRobot->HipPos);
	xbot_logger->add("LeftAnklePos", irobot.SendToRobot->LeftAnklePos);
	xbot_logger->add("RightAnklePos", irobot.SendToRobot->RightAnklePos);

	xbot_logger->add("HipO_Left", irobot.SendToRobot->HipO_Left);
	xbot_logger->add("HipO_Right", irobot.SendToRobot->HipO_Right);
	xbot_logger->add("LeftFootO", irobot.SendToRobot->LeftFootO);
	xbot_logger->add("RightFootO", irobot.SendToRobot->RightFootO);

	xbot_logger->add("stabilizer_torque", irobot.SendToRobot->stabilizer_torque);
	xbot_logger->add("applyDeltaPosX", irobot.SendToRobot->applyDeltaPosX);
	xbot_logger->add("applyDeltaPosY", irobot.SendToRobot->applyDeltaPosY);

	xbot_logger->add("TurnYaw", TurnYaw);
	xbot_logger->add("LftYaw", LftYaw);
	xbot_logger->add("RftYaw", RftYaw);

	xbot_logger->add("det_hip_posotion", det_hip_posotion);
	xbot_logger->add("det_hip_pose", det_hip_pose);
	xbot_logger->add("det_foot_rpy_lr", det_foot_rpy_lr);
	xbot_logger->add("det_footz_lr", det_footz_lr);		
	
	Sta.addToLog(xbot_logger);

	InternalLoggerLoop();
}
#endif
