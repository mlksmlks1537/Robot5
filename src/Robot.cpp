#define _USE_MATH_DEFINES
#include "Robot.h"
#include"MCSL_Fun.h"
#include"MCSL.h"
#include"kinematics.h"
#include"EcmDriver.h"
#include<cmath>

#define nAxisNum 8
#define CARD_INDEX 0
#define nGroupIndex 0
double nInterpolateTime = 1;

std::ostream& operator<<(std::ostream &os, const JointPositions &pos) {
    return os << "JointPositions(j1: " << pos.j1
       << ", j2: " << pos.j2
       << ", j3: " << pos.j3
       << ", j4: " << pos.j4
       << ", j5: " << pos.j5
       << ", j6: " << pos.j6
       << ", j7: " << pos.j7
       << ", j8: " << pos.j8
       << ")";
}
int controlSystem::InitSystem(double ratio[], double Pitch[], int pusle[], double HLimit[], double LLimit[], int dirReverse[], int wAxisMap[],int wAxisMirror[])
{
    SYS_MAC_PARAM      stMacParam;
    SYS_ENCODER_CONFIG stENCConfig;
    SYS_CARD_CONFIG    stCardConfig;
    int m_nAxisNum = nAxisNum;
    int nRtn;
    int g_nGroupIndex;
    MCS_SetSysMaxSpeed(1500);//1000mm/s
    nRtn = MCS_CloseSystem();
    //////////////////////////////////////////////////////////////////////////////////////////
    //各轴最大安全速度 = (wRPM / 60) ×(dfPitch / dfGearRatio)
    for (WORD wChannel = 0; wChannel < (m_nAxisNum); wChannel++)
    {
        stMacParam.wPosToEncoderDir = dirReverse[wChannel];
        stMacParam.dwPPR = pusle[wChannel];//10000;//8388608;//10000;//131072;
        stMacParam.wRPM = 3000;
        stMacParam.dfPitch = Pitch[wChannel];//40.0;
        stMacParam.dfGearRatio = ratio[wChannel];//5.0;
        stMacParam.dfHighLimit = HLimit[wChannel];
        stMacParam.dfLowLimit = LLimit[wChannel];
        stMacParam.dfHighLimitOffset = 0;
        stMacParam.dfLowLimitOffset = 0;
        stMacParam.wPulseMode = DDA_FMT_PD;
        stMacParam.wPulseWidth = 100;
        stMacParam.wCommandMode = OCM_PULSE;
        stMacParam.wOverTravelUpSensorMode = 2;//  not checking
        stMacParam.wOverTravelDownSensorMode = 2;
        //  set encoder configures
        stENCConfig.wType = ENC_TYPE_AB;
        stENCConfig.wAInverse = _NO_;
        stENCConfig.wBInverse = _NO_;
        stENCConfig.wCInverse = _NO_;
        stENCConfig.wABSwap = _NO_;
        stENCConfig.wInputRate = 4;//  set encoder input rate : x4
        nRtn=MCS_SetMacParam(&stMacParam, wChannel, 0);      //  mechanism parameters are the same for all axes
        //MCS_SetEncoderConfig(&stENCConfig, wChannel, CARD_INDEX);//  encoder configures are the same for all axes
    }
    //  set group parameters
    MCS_CloseAllGroups();
    g_nGroupIndex = MCS_CreateGroup(wAxisMap[0], wAxisMap[1], wAxisMap[2],
        wAxisMap[3], wAxisMap[4], wAxisMap[5], wAxisMap[6], wAxisMap[7], CARD_INDEX);
    nRtn = MCS_SetMirrorAxis(wAxisMirror[0], wAxisMirror[1], wAxisMirror[2], wAxisMirror[3], wAxisMirror[4], wAxisMirror[5], wAxisMirror[6], wAxisMirror[7], g_nGroupIndex);
    stCardConfig.wCardType = 4;
    ECM_SetPdoConfEnable(1);
    nRtn = ECM_NewPdoConfTbl(7);
    nRtn = ECM_SetPdoAsDrive(0, 1);
    nRtn = ECM_SetPdoAsDrive(1, 1);
    nRtn = ECM_SetPdoAsDrive(2, 1);
    nRtn = ECM_SetPdoAsDrive(3, 1);
    nRtn = ECM_SetPdoAsDrive(4, 1);
    nRtn = ECM_SetPdoAsDrive(5, 1);
    nRtn = ECM_SetPdoAsDrive(6, 1);
    nRtn = MCS_InitSystemEx(nInterpolateTime, &stCardConfig, 1);
    if (nRtn != 0)
    {
        return nRtn;
    }
    MCS_SetOverTravelCheck(1, 1, 1, 1, 1, 1, 1, 1, g_nGroupIndex);
    ECM_EncEnableLatch(1);
    MCS_SetAccTime(300, g_nGroupIndex);
    MCS_SetDecTime(300, g_nGroupIndex);

    MCS_SetAccType('T', g_nGroupIndex);
    MCS_SetDecType('T', g_nGroupIndex);
    MCS_SetPtPAccType('T', 'T', 'T', 'T', 'T', 'T', 'T', 'T', g_nGroupIndex);
    MCS_SetPtPDecType('T', 'T', 'T', 'T', 'T', 'T', 'T', 'T', g_nGroupIndex);

    MCS_SetAbsolute(g_nGroupIndex);   // 设为绝对坐标模式
    for (int i = 0; i < (m_nAxisNum); i++)
    {
        MCS_SetServoOn(i); // 使能伺服，电机上电锁住
        MCS_DefinePos(i, 0, g_nGroupIndex); // 将当前位置定义为 0
    }
    return 0;
}

void controlSystem::CloseSystem()
{
    int i;
    for (i = 0; i < (nAxisNum); i++)
    {
        MCS_SetServoOff(i);
    }
    MCS_CloseSystem();
}

int controlSystem::GetAbsEncValue(int* absshift23, int channel)
{
    int* pEncHandle = nullptr;
    int ret = ECM_GetObjAddr(0, channel, 1, &pEncHandle);
    if (ret != 1 || pEncHandle == nullptr) {
        return -1;   // 读取失败
    }
    *absshift23 = *pEncHandle;
    return 0;
}

int controlSystem::SetAbsPos(int Encvalue0[], double ratio[], double Pitch[], int pusle[], int wAxisMap[])
{
    int Encvalue[8] = { 0 };   // 全部初始化为0
	double pos[8] = { 0.0 };   // 全部初始化为0.0
    for (int i = 0; i < 8; i++) {
        if (wAxisMap[i] == -1) continue;   // 跳过未使用的轴

        if (GetAbsEncValue(&Encvalue[i], i) != 0) {
            return -1;
        }

        // 计算当前物理位置 (单位：mm)
        pos[i] = (double)(Encvalue[i] - Encvalue0[i]) / pusle[i] * Pitch[i] / ratio[i];
    }
	for (int i = 0; i < 8; i++) {
        if (wAxisMap[i] == -1) continue;   // 跳过未使用的轴
        MCS_DefinePos(i, pos[wAxisMap[i]], 0);   // 将当前位置定义为 pos[i]
    }
    return 0;
}

int controlSystem::DefinePos(int Axis, double pos)
{
    return MCS_DefinePos(Axis, pos, nGroupIndex);
}

int controlSystem::GetCurJPos(JointPositions* pos)
{
	int Status = 0;
    Status = MCS_GetCurPos(&pos->j1, &pos->j2, &pos->j3, &pos->j4, &pos->j5, &pos->j6, &pos->j7, &pos->j8, nGroupIndex);
    pos->j4 = pos->j4 * 180 / M_PI;
    pos->j5 = pos->j5 * 180 / M_PI;
    pos->j6 = pos->j6 * 180 / M_PI;
    return Status;
}

double controlSystem::SetPtPSpeed(double dRatio)
{
    return MCS_SetPtPSpeed(dRatio, nGroupIndex);
}

int controlSystem::MovePTP(JointPositions* pos)
{
    if(pos==nullptr)
        return -1;
    pos->j4 = pos->j4 * M_PI / 180;
    pos->j5 = pos->j5 * M_PI / 180;
    pos->j6 = pos->j6 * M_PI / 180;
    return MCS_PtP(pos->j1, pos->j2, pos->j3, pos->j4, pos->j5, pos->j6, 0, 0, nGroupIndex);
}

void controlSystem::SetServoON(int axis, bool Enable)
{
    if (Enable)
    {
        MCS_SetServoOn(axis, nGroupIndex);
    }
    else
    {
        MCS_SetServoOff(axis, nGroupIndex);
    }
}

void controlSystem::AbortMotion()
{
    MCS_AbortMotion(nGroupIndex);
}

int controlSystem::GetMotionStatus()
{
    return MCS_GetMotionStatus(nGroupIndex);
}

int controlSystem::GoHome(double dfSpeedRatio)
{
    // 1. 检查运动状态
    if (MCS_GetMotionStatus(nGroupIndex) != GMS_STOP) {
        return -1;
    }
    // 2. 检查错误
    if (MCS_GetErrorCode(nGroupIndex) != 0) {
        return -2;
    }
    // 3. 确保是绝对坐标模式
    MCS_SetAbsolute(nGroupIndex);
    // 4. 设置速度
    if (dfSpeedRatio < 1) dfSpeedRatio = 1;
    if (dfSpeedRatio > 100) dfSpeedRatio = 100;
    MCS_SetPtPSpeed(dfSpeedRatio, nGroupIndex);
    // 5. 移动到 0
    int ret = MCS_PtP(0, 0, 0, 0, 0, 0, 0, 0, nGroupIndex, AXIS_ALL);
    if (ret < 0) return ret;
    // 6. 等待完成
    while (MCS_GetMotionStatus(nGroupIndex) != GMS_STOP) {
        Sleep(10);
    }
    return 0;
}

int controlSystem::JogPulse(int Axis, int Pulse)
{
    return MCS_JogPulse(Pulse, Axis, nGroupIndex);
}

int controlSystem::JogPtpSpace(int Axis, double Space, double speedRatio)
{
    return MCS_JogSpace(Space, speedRatio, Axis, nGroupIndex);
}

int controlSystem::SetAccTime(double dfAccTime)
{
    const double MIN_ACC_TIME = 20.0;   // 最小20ms
    const double MAX_ACC_TIME = 5000.0; // 最大5000ms

    if (dfAccTime < MIN_ACC_TIME) dfAccTime = MIN_ACC_TIME;
    if (dfAccTime > MAX_ACC_TIME) dfAccTime = MAX_ACC_TIME;
    return MCS_SetAccTime(dfAccTime, nGroupIndex);
}

int controlSystem::SetDecTime(double dfDecTime)
{
    const double MIN_DEC_TIME = 20.0;
    const double MAX_DEC_TIME = 5000.0;

    if (dfDecTime < MIN_DEC_TIME) dfDecTime = MIN_DEC_TIME;
    if (dfDecTime > MAX_DEC_TIME) dfDecTime = MAX_DEC_TIME;
    return MCS_SetDecTime(dfDecTime, nGroupIndex);
}

double controlSystem::SetFeedSpeed(double Speed)
{
    return MCS_SetFeedSpeed(Speed, nGroupIndex);
}

double controlSystem::SetInterpolateTime(double msec)
{
    if (msec < 1 || msec >= 50)return-1;
    nInterpolateTime = msec;
    return 0.0;
}

int controlSystem::MoveLine(JointPositions* pos)
{
    if (pos == nullptr)
        return -1;
    pos->j4 = pos->j4 * M_PI / 180;
    pos->j5 = pos->j5 * M_PI / 180;
    pos->j6 = pos->j6 * M_PI / 180;
    return MCS_Line(pos->j1, pos->j2, pos->j3, pos->j4, pos->j5, pos->j6, 0, 0, nGroupIndex, 255);
}

int controlSystem::MoveArc(JointPositions* mid_pos, JointPositions* targer_pos_rot)
{
    if (mid_pos == nullptr|| targer_pos_rot == nullptr)
        return -1;
    targer_pos_rot->j4 = targer_pos_rot->j4 * M_PI / 180;
    targer_pos_rot->j5 = targer_pos_rot->j5 * M_PI / 180;
    targer_pos_rot->j6 = targer_pos_rot->j6 * M_PI / 180;
    return MCS_ArcXYZ_Aux(mid_pos->j1, mid_pos->j2, mid_pos->j3,
        targer_pos_rot->j1, targer_pos_rot->j2, targer_pos_rot->j3,
        targer_pos_rot->j4, targer_pos_rot->j5, targer_pos_rot->j6,
        0, 0, nGroupIndex);
}

int controlSystem::GetECatErrorCode()
{
    return MCS_GetErrorCode(nGroupIndex);
}

int controlSystem::ClearError()
{
    return MCS_ClearError(nGroupIndex);
}