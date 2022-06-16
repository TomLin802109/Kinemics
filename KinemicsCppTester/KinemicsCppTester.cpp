// KinemicsCppTester.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。
//
#include <windows.h>
#include <iostream>
#include "Kinematics.h"
#include "Matrix.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
using namespace std;

int main()
{
#pragma region Eigen Example
    /*double angle_pi = 90.0 / 180.0 * M_PI;
    Eigen::AngleAxisd rotx(angle_pi, Eigen::Vector3d(1, 0, 0));
    Eigen::AngleAxisd roty(angle_pi, Eigen::Vector3d(0, 1, 0));
    Eigen::AngleAxisd rotz(angle_pi, Eigen::Vector3d(0, 0, 1));
    auto rot_xyz = rotx * roty * rotz;
    cout << rot_xyz.matrix() << endl;
    cout << rot_xyz.matrix().inverse() << endl;
    cout << Eigen::Matrix3d::Identity() << endl;
    cout << (rot_xyz * Eigen::Vector3d(0, 0, 1)).matrix() << endl;*/
    Eigen::Matrix4d m;
    m << 1, 2, 3, 4, 
        5, 6, 7, 8, 
        9, 10, 11, 12, 
        13, 14, 15, 16;
    cout << "m10 = " << m(1, 0) << endl;
    cout << "m01 = " << m(0, 1) << endl;
    cout << "m30 = " << m(3, 0) << endl;
    cout << "m03 = " << m(0, 3) << endl;
    cout << m * Eigen::Vector4d(1, 2, 3, 1) << endl;

#pragma endregion

    //return 0;
    //float j1 = 0, j2 = 0, j3 = 0, j4 = 0, j5 = -90, j6 = 0;
    float j1 = 45, j2 = -45, j3 = -50, j4 = 0, j5 = -45, j6 = 30;
    //float j1 = 21.8f, j2 = -52.2f, j3 = 2.5f, j4 = 0, j5 = 0, j6 = 0;
    //YASKAWA
    vector<DH_param> dhTable = {
        DH_param{0,0,0,0},
        DH_param{-90,155,0,-90},
        DH_param{180,614,0,0},
        DH_param{-90,200,-640,0},
        DH_param{90,0,0,0},
        DH_param{-90,0,0,0},
        DH_param{180,0,100,0}
    };
    //KAWASAKI
    /*vector<DH_param> dhTable = {
        DH_param(0,0,0,90),
        DH_param(-90,150,0,-90),
        DH_param(180,770,0,90),
        DH_param(90,0,805,0),
        DH_param(-90,0,0,0),
        DH_param(90,0,0,0),
        DH_param(0,0,95,90)
    };*/
    /*vector<DH_param> dhTable{
        DH_param{0,0,0,0},
        DH_param{-90,-30,0,0},
        DH_param{0,340,0,0},
        DH_param{-90,-40,338,0},
        DH_param{90,0,0,0},
        DH_param{-90,0,0,0},
    };*/
    vector<JointLimit> limts = {
        JointLimit{180,-180}
    };
    RobotSpec spec(dhTable, limts, EulerAngle::XYZ);
    RobotArmKinematics _k(spec, WorldCoordinate{ 0,0,0,0,0,0 });

    LARGE_INTEGER cpuFreq;
    LARGE_INTEGER startTime;
    LARGE_INTEGER endTime;
    double runTime = 0.0;
    QueryPerformanceFrequency(&cpuFreq);

    QueryPerformanceCounter(&startTime);
    auto w = _k.Forward(JointCoordinate{ j1,j2,j3,j4,j5,j6 });
    QueryPerformanceCounter(&endTime);
    runTime = (((endTime.QuadPart - startTime.QuadPart) * 1000.0f) / cpuFreq.QuadPart);
    cout << "Cost Time: " << runTime << endl;
    cout << "Forward World: " << w.x << "," << w.y << "," << w.z << "," << w.a << "," << w.b << "," << w.c << endl;

    QueryPerformanceCounter(&startTime);
    //auto j = _k.Inverse(WorldCoordinate{ 795,0,714,180,0,0 });
    auto j = _k.Inverse(w);
    QueryPerformanceCounter(&endTime);
    runTime = (((endTime.QuadPart - startTime.QuadPart) * 1000.0f) / cpuFreq.QuadPart);
    cout << "Inverse Joint: " << j.j1 << "," << j.j2 << "," << j.j3 << "," << j.j4 << "," << j.j5 << "," << j.j6 << endl;
#ifdef  _DEBUG
    cout << "Cost Time: " << runTime << endl;
#endif
}
