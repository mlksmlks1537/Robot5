#define _USE_MATH_DEFINES
#include<fstream>
#include<iomanip>
#include<iostream>
#include<cmath>
#include<Windows.h>
#include"Robot.h"
#include"MoveCommander.h"
#include<vector>

void printPos(JointPositions* pos) {
	if (pos == nullptr) {
		std::cout << "Position is null." << std::endl;
		return;
	}
	std::cout << "Current Joint Positions: "
			  << "j1: " << pos->j1 << ", "
			  << "j2: " << pos->j2 << ", "
			  << "j3: " << pos->j3 << ", "
			  << "j4: " << pos->j4 << ", "
			  << "j5: " << pos->j5 << ", "
			  << "j6: " << pos->j6 << ", "
			  << "j7: " << pos->j7 << ", "
			  << "j8: " << pos->j8
			  << std::endl;
}
void printEnc()
{
	int absshift23[8] = { 0 };
	std::cout << std::endl << "enc:";
	for (int i = 0; i < 8; i++)
	{
		controlSystem::GetAbsEncValue(&absshift23[i], i);
		std::cout << absshift23[i] << ",";
	}
	std::cout << "end" << std::endl;

}
void rotateCur(double deg){
	JointPositions pos;
	controlSystem::GetCurJPos(&pos);
	
	auto pos1 = RinnRobotCommander::MoveCommander::
		Eigen::Vector3d(pos.j4, pos.j5, pos.j6), 
		Eigen::Vector3d(pos.j1, pos.j2, pos.j3), deg/2);
	auto pos2 = RinnRobotCommander::MoveCommander::
		rotateRy(Eigen::Vector3d(pos.j4, pos.j5, pos.j6), 
		Eigen::Vector3d(pos.j1, pos.j2, pos.j3), deg);
	auto pos1_struct = RinnRobotCommander::MoveCommander::newPos(pos1, Eigen::Vector3d(pos.j4, pos.j5 + deg/2, pos.j6));
	auto pos2_struct = RinnRobotCommander::MoveCommander::newPos(pos2, Eigen::Vector3d(pos.j4, pos.j5 + deg, pos.j6));
	controlSystem::MoveArc(&pos1_struct, &pos2_struct);
}
void rotateCur2(double dry,double drz){
	JointPositions pos;
	controlSystem::GetCurJPos(&pos);
	
	auto pos1 = RinnRobotCommander::MoveCommander::
		rotateRyRz(Eigen::Vector3d(pos.j4, pos.j5, pos.j6), 
		Eigen::Vector3d(pos.j1, pos.j2, pos.j3), dry/2, drz/2);
	auto pos2 = RinnRobotCommander::MoveCommander::
		rotateRyRz(Eigen::Vector3d(pos.j4, pos.j5, pos.j6), 
		Eigen::Vector3d(pos.j1, pos.j2, pos.j3), dry, drz);
	auto pos1_struct = RinnRobotCommander::MoveCommander::newPos(pos1, Eigen::Vector3d(pos.j4, pos.j5 + dry/2, pos.j6 + drz/2));
	auto pos2_struct = RinnRobotCommander::MoveCommander::newPos(pos2, Eigen::Vector3d(pos.j4, pos.j5 + dry, pos.j6 + drz));
	controlSystem::MoveArc(&pos1_struct, &pos2_struct);
}
bool SavePositionsToTxt(const std::vector<JointPositions>& positions, const std::string& filename, int precision = 6)
{
	if (positions.empty()) {
		return false;
	}

	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		return false;
	}

	outFile << std::fixed << std::setprecision(precision);
	outFile << "start" << "\n";
	outFile << positions.size() << "\n";
	for (const auto& pos : positions) {
		outFile << pos.j1 << " " << pos.j2 << " " << pos.j3 << "\n";
	}

	outFile.close();
	return true;
}
int testmotion()
{
	int nRtn;
	std::vector<JointPositions> positions;
	JointPositions* pos = new JointPositions();
	JointPositions* mpos = new JointPositions();
	JointPositions* mpos1 = new JointPositions();

	controlSystem::GetCurJPos(pos);
	printPos(pos);
	mpos->j1 = 50, mpos->j2 = 0, mpos->j3 = 0;
	mpos->j4 = 0, mpos->j5 = 0, mpos->j6 = 0;
	//mpos1->j1 = 50, mpos1->j2 = 0, mpos1->j3 = 0;
	//mpos1->j4 = 90, mpos1->j5 = 0, mpos1->j6 = 0;
	std::cout << "start motion test..." << std::endl;
	//motion function

	//controlSystem::SetPtPSpeed(1);
	//controlSystem::MovePTP(mpos);

	controlSystem::SetFeedSpeed(2);
	//controlSystem::MoveLine(mpos);
	//rotateCur(20);
	//controlSystem::MoveArc(mpos, mpos1);

	//record positions
	while (true)
	{

		Sleep(10);
		controlSystem::GetCurJPos(pos);
		positions.push_back(*pos);
		int Status = controlSystem::GetMotionStatus();
		if (Status == 1) {
			break;
		}
	}
	std::cout << "end motion test..." << std::endl;
	controlSystem::GetCurJPos(pos);
	printPos(pos);
	SavePositionsToTxt(positions, "C:\\Users\\Administrator\\Desktop\\Robot5\\test\\positions.txt");
	delete mpos1;
	delete mpos;
	delete pos;
	return 0;
}
int main()
{	//参数设置
	//int   Pusle[8] = { 131072, 131072, 131072, 131072, 131072, 131072, 131072, 131072 };//龙门
	//                  //Y1,Y2,X,Z,RZ,RY;
	//double Pitch[8] = { 10,10,10,124.54,2 * M_PI,2 * M_PI,2 * M_PI,20 };
	//double Ratio[8] = { 1.0, 1.0, 1.0, 10.0, 50, 81, 25, 1 };
	//double HLimit[8] = { 200,200,200,200,200,200,200,20 };
	//double LLimit[8] = { -200,-200,-200,-200,-200,-200,-200,20 };
	//int dirReverse[8] = { 0,0,0,0,0,0,0,0 };
	//                   // X,Y,Z,RX,RY,RZ
	//int wAxisMap[8] = { 2, 0, 3, 6, 5, 4, -1, -1 };
	////零点编码器值
	//int Encvalue0[8] = { 8105640,-9610163,-2333917,2130859,-8213689,-1682896,0,0 };
	////将物理通道镜像逻辑轴，-1表示不使用镜像
	//int wAxisMirror[8] = { -1,1,-1,-1,-1,-1 ,-1,-1 };


	int   Pusle[8] = { 131072, 131072, 131072, 131072, 131072, 131072, 131072, 131072 };
	// 在InitSystem前 接线排序 X、Y、Z、A、B  131072*4/20
	double Pitch[8] = { 10,			16.66,    124.54,	2 * M_PI,		2 * M_PI,2 * M_PI,20 ,2 * M_PI };
	double Ratio[8] = { 1,			 1,			20,     50,			30,   1,     1,      1 };
	double HLimit[8] = { 400,  200,   10,  M_PI / 2,   M_PI / 4,    2 * M_PI + 0.5 ,  2.5 * M_PI , 20 };
	double LLimit[8] = { -400,  -200,  -400, -M_PI / 2,   -M_PI / 4,    -2 * M_PI - 0.5 , -2.5 * M_PI ,-20 };
	int dirReverse[8] = { 0,0,0,0,0,0,0,0 };
	// X,Y,Z,RX,RY,RZ
	int wAxisMap[8] = { 0,1,2,3,4,-1,-1, -1 };
	//零点编码器值
	int Encvalue0[8] = { 132282,-1184821,-2421091,7450751,70946,0,0,0 };
	//int Encvalue0[8] = { 132258,1027941,-2421002,6904615,16339,0,0,0 };
	//将物理通道镜像逻辑轴，-1表示不使用镜像
	int wAxisMirror[8] = { -1,-1,-1,-1,-1,-1 ,-1,-1 };
	//测试
	controlSystem::InitSystem(Ratio, Pitch, Pusle, HLimit, LLimit, dirReverse, wAxisMap, wAxisMirror);
	printEnc();
	controlSystem::SetAbsPos(Encvalue0, Ratio, Pitch, Pusle, wAxisMap);
	controlSystem::GoHome(1);


	testmotion();

	printEnc();
	controlSystem::CloseSystem();
	return 0;
}