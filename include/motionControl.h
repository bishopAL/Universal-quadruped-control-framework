#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include "thread.h"
#include "motor.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class MotionControl
{
    public:
        float timeForGaitPeriod;  // The time of the whole period
        float timePeriod;  // The time of one period
        float timePresent;
        Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
        Vector<float, 3> targetCoMVelocity;  // X, Y , alpha c in world cordinate
        Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        Vector<bool, 4> stanceFlag;  // True, False: LF, RF, LH, RH
        Vector<float, 4> timePresentForSwing;
        float L1, L2, L3;  // The length of L
        float width, length;
        Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
        Matrix<float, 4, 3> stancePhaseStartPos;
        Matrix<float, 4, 3> stancePhaseEndPos;
        Matrix<float, 4, 3> legPresentPos;  // present X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Matrix<float, 4, 3> legCmdPos;  // command X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Matrix<float, 4, 3> jointPresentPos;  // present motor 0-11
        float jointCmdPos[12];  // command motor 0-11
        void setInitPos(Matrix<float, 4, 3> initPosition);
        void setCoMVel(Vector<float, 3> tCV);
        void nextStep();
        void inverseKinematics();
        MotionControl(float tP, float tFGP, Matrix<float, 4, 2> tFSP);

};