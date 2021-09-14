#include <motionControl.h>
#define PI 3.1415926
using namespace std;
using namespace Eigen;

MotionControl::MotionControl(float tP, float tFGP, Matrix<float, 4, 2> tFSP)
{
    initFlag = false;
    timePeriod = tP;
    timeForGaitPeriod = tFGP;
    timeForStancePhase = tFSP;
    timePresent = 0.0;
    timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
    targetCoMVelocity << 0.0, 0.0, 0.0;
    L1 = 132.0;
    L2 = 138.0;
    L3 = 0.0;
    width = 132.0;
    length = 172.0;  
    shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH
}



void MotionControl::creepingGait(float X_tar, float Y_tar, float Yaw_tar)
{
    float L1_creep = 132.0;
    float L2_creep = 112.0;
    float L3_creep = 20.0;
    initPos << 112.0, 132.0, -20.0, 112.0, -132.0, -20.0, -112.0, 132.0, -20.0, -112.0, -132.0, -20.0;  //unknown z
    
}