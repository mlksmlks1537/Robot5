#pragma once

#include"Robot.h"
#include<Eigen/Dense>
#include<iostream>
namespace RinnRobotCommander{
    class MoveCommander{
    private:
        static Eigen::Matrix3d eulerZYXToRotation(double rxDeg, double ryDeg, double rzDeg) {
            const double deg2rad = std::acos(-1.0) / 180.0;
            const double rx = rxDeg * deg2rad;
            const double ry = ryDeg * deg2rad;
            const double rz = rzDeg * deg2rad;

            auto Rz = [](double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()); };
            auto Ry = [](double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()); };

            return (Rz(rz) * Ry(ry)).toRotationMatrix();
        }
        static const Eigen::Vector3d moveDir;
        static Eigen::Vector3d getFlangeLoc(Eigen::Vector3d dir, Eigen::Vector3d p,Eigen::Vector3d seenDir = moveDir){
            Eigen::Matrix3d mat = eulerZYXToRotation(dir.x(), dir.y(), dir.z());
            return p + mat * seenDir;
        }
        static Eigen::Vector3d deFlangeLoc(Eigen::Vector3d dir, Eigen::Vector3d p,Eigen::Vector3d seenDir = moveDir){
            Eigen::Matrix3d mat = eulerZYXToRotation(dir.x(), dir.y(), dir.z());
            return p + mat * -seenDir;
        }
        double tool_Pos[3];
        double tool_Speed[6];
        double wRPM;
        double convexSpeed(double d,size_t idx){
            return d/tool_Speed[idx]*100;
        }
        double toRad(double deg){
            return deg*M_PI/180;
        }
        void SetSpeed(double dx,double dy,double dz,double drx,double dry,double drz){
            //SetPtPSpeedEx����
            //controlSystem::SetPtPSpeedEx(convexSpeed(dx,0),convexSpeed(dy,1),convexSpeed(dz,2),
            //    convexSpeed(drx,3),convexSpeed(dry,4),convexSpeed(drz,5));
        }
    public:
        MoveCommander(double ratio[], double Pitch[], int pusle[], double HLimit[], double LLimit[], int dirReverse[], int wAxisMap[],double toolPos[3] = nullptr){
            //int nRtn = controlSystem::InitSystem(ratio, Pitch, pusle, HLimit, LLimit, dirReverse, wAxisMap);
            wRPM = 50;
            if(toolPos){
                tool_Pos[0] = toolPos[0];
                tool_Pos[1] = toolPos[1];
                tool_Pos[2] = toolPos[2];
            }
            for(int i = 0; i < 6; i++){
                tool_Speed[i] = wRPM*Pitch[i+1]/ratio[i+1];
            }
        }
        
        static Eigen::Vector3d rotateRy(Eigen::Vector3d dir,Eigen::Vector3d p, double deg){
            Eigen::Vector3d newDir = dir;
            newDir.y() += deg;
            auto targetLoc = deFlangeLoc(dir,p);
            return getFlangeLoc(newDir,targetLoc);
        }
        static Eigen::Vector3d rotateRyRz(Eigen::Vector3d dir,Eigen::Vector3d p, double ryDeg,double rzDeg){
            Eigen::Vector3d newDir = dir;
            newDir.y() += ryDeg;
            newDir.z() += rzDeg;
            auto targetLoc = deFlangeLoc(dir,p);
            return getFlangeLoc(newDir,targetLoc);
        }
        static JointPositions newPos(Eigen::Vector3d pos,Eigen::Vector3d dir){
            JointPositions *ret = new JointPositions();
            ret->j1 = pos.x(), ret->j2 = pos.y(), ret->j3 = pos.z();
            ret->j4 = dir.z(), ret->j5 = dir.y(), ret->j6 = dir.x();
            return *ret;
        }
        ~MoveCommander(){
            controlSystem::CloseSystem();
        }
    };
    const Eigen::Vector3d MoveCommander::moveDir(0,0,-25);
}