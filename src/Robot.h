#pragma once
#include<iostream>

typedef struct {
	double j1; //X
	double j2; //Y
	double j3; //Z
	double j4; // RXdeg
	double j5; // RYdeg
	double j6; // RZdeg
	double j7;
	double j8;
} JointPositions;

std::ostream& operator<<(std::ostream &os, const JointPositions &pos);
class controlSystem {
    controlSystem() = delete;
public:
	// 初始化系统：减速比、导程、脉冲数、正限位、负限位、电机正反转、轴映射
	static int InitSystem(double ratio[], double Pitch[], int pusle[], double HLimit[], double LLimit[], int dirReverse[], int wAxisMap[], int wAxisMirror[]);
	// 关闭系统
    static void CloseSystem();
	// 获取绝对编码器值
    static int GetAbsEncValue(int* absshift23, int channel);
	//设置当前绝对位置
    static int SetAbsPos(int Encvalue0[], double ratio[], double Pitch[], int pusle[], int wAxisMap[]);
	// 设置轴为关节坐标值
	static int DefinePos(int Axis, double pos);
	// 获取关节坐标
	static int  GetCurJPos(JointPositions* pos);
	// 设置点位速度比例
	static double  SetPtPSpeed(double dRatio);
	// ptp点位运动到绝对位置
	static int  MovePTP(JointPositions* pos);
	// 上、下使能伺服
	static void SetServoON(int axis, bool Enable);
	// 停止运行
	static void AbortMotion();
	/*  GetMotionStatus() 返回值
	0 GMS_RUNNING 处于运动状态，尚有运动命令未执行完成
	1 GMS_STOP 处于停止状态，已无库存运动命令
	2 GMS_HOLD 处于暂停状态(因使用者呼叫MCS_HoldMotion)
	3 GMS_DELAYING 处于延迟状态(因使用者呼叫MCS_DelayMotion)
	4 GMS_BLOCKHOLD
	5 GMS_MPGING
	其他失败*/
	static int GetMotionStatus();
	//回零点
	static int GoHome(double dfSpeedRatio = 10.0);
	// Jog 点位运行脉冲数
	static int  JogPulse(int Axis, int Pulse);
	// Jog 点位运行距离 /rad
	static int  JogPtpSpace(int Axis, double Space, double speedRatio);
	// 设置加减速时间 
	static int SetAccTime(double dfAccTime);
	static int SetDecTime(double dfDecTime);
	// 设置插补速度mm/sec
	static double  SetFeedSpeed(double Speed);
	// 设置插值时间ms
	static double  SetInterpolateTime(double msec);
	// 法兰盘坐标运动到绝对位置
	static int  MoveLine(JointPositions* pos);
    // 法兰盘坐标运动空间圆轨迹
	static int  MoveArc(JointPositions* mid_pos, JointPositions* targer_pos_rot);
	// 获取总线错误代码
	static int  GetECatErrorCode();
	// 清除报警
	static int ClearError();

	// Jog运动直线距离
	int  JogLineSpace(int Axis, double Space, double Speed);

	// 获取EIO输入值
	void GetEIOIntPutValue(int* Value);
	// 设置EIO输出值
	void SetEIOOutPutValue(int Value);
	// 获取GPIO输入值
	void GetGPIOIntPutValue(int* Value);
	// 设置GPIO输出值
	void SetGPIOOutPutValue(int Value);
	// 获取GPIO输出值
	void GetGPIOOutPutValue(int* Value);

	void GetCompPos(int32_t* X0, int32_t* X1, int* num, int32_t* A0, int32_t* A1);
	void SetCompEnable(int status, int Speca);

	// 获取缓冲区运动指令数量
	int GetCommandCount(int* pnCmdCount);
	// 清空缓冲区运动指令数量
	int ResetCommandIndex();
	// 设置平滑
	int  SetBlending(bool  sw);
};
