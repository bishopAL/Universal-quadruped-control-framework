#include <thread.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <Eigen/Core>
#include <motionControl.h>
#include "motor.h"

using namespace std;


int main(int argc, char ** argv)
{
	Matrix<float, 4, 2> timeForStancePhase;
    float timePeriod = 0.01;
    float timeForGaitPeriod = 0.49;
	timeForStancePhase<<0, 0.24, 0.25, 0.49, 0.25, 0.49, 0, 0.24;
	MotionControl mc(timePeriod, timeForGaitPeriod, timeForStancePhase); // time
	Matrix<float, 4, 3> initPos;
	initPos<< mc.L1, mc.L2, mc.L3, mc.L1, mc.L2, mc.L3, mc.L1, mc.L2, mc.L3, mc.L1, mc.L2, mc.L3;

	Vector<float, 3> tCV;
	tCV<<10.0, 0.0, 0.0;

	mc.setInitPos(initPos);
    cout<<"initPos vel"<<endl;
	mc.setCoMVel(tCV);
    for(int times=0; times<100; times++)
    {
        mc.nextStep();
        cout<<"Time present: "<<mc.timePresent<<"; leg stance flag: "<<mc.stanceFlag.transpose()
        <<"; CoM position: "<<mc.targetCoMPosition.transpose()<<endl;
    }
    // Matrix<float, 2, 2> ta;
    // ta<<1,2,3,4;
	// cout<<ta*mc.shoulderPos.row(0).transpose()<<endl;
    //thread_init();
    return 0;
}
