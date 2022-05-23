// KinemicsCppTester.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。
//
#include <windows.h>
#include <iostream>
#include "Kinemics.h"
#include "Matrix.h"

using namespace std;

int main()
{
    float j1 = 0, j2 = 0, j3 = 0, j4 = 0, j5 = -90, j6 = 0;
    //float j1 = 0, j2 = -45, j3 = -45, j4 = 45, j5 = 45, j6 = 45;    
    /*DH_param params[7]{
        DH_param{0,0,0,0},
        DH_param{-90,155,0,-90},
        DH_param{180,614,0,0},
        DH_param{-90,200,-640,0},
        DH_param{90,0,0,0},
        DH_param{-90,0,0,0},
        DH_param{180,0,100,0}
    };*/
    DH_param params[7]{
        DH_param{0,0,0,0},
        DH_param{-90,-30,0,0},
        DH_param{0,340,0,0},
        DH_param{-90,-40,338,0},
        DH_param{90,0,0,0},
        DH_param{-90,0,0,0},
    };
    DH_table dh{ sizeof(params) / sizeof(*params), params };
    Kinemics _k(dh, WorldCoordinate{ 0,0,0,0,0,0 });

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
    cout << "world: " << w.x << "," << w.y << "," << w.z << "," << w.a << "," << w.b << "," << w.c << endl;

    QueryPerformanceCounter(&startTime);
    auto j = _k.Inverse(WorldCoordinate{ 381.3f,151.8f,19.5f });
    QueryPerformanceCounter(&endTime);
    runTime = (((endTime.QuadPart - startTime.QuadPart) * 1000.0f) / cpuFreq.QuadPart);
    cout << "joint: " << j.j1 << "," << j.j2 << "," << j.j3 << "," << j.j4 << "," << j.j5 << "," << j.j6 << endl;
#ifdef  _DEBUG
    cout << "Cost Time: " << runTime << endl;
#endif
}
