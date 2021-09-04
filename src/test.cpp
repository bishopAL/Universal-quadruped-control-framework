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
const int num = 12;
int ID[num] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
struct timeval startTime1, endTime1;

int main(int argc, char ** argv)
{
    // Motion control init start
	Matrix<float, 4, 2> timeForStancePhase;
    float timePeriod = 0.01;
    float timeForGaitPeriod = 0.49;
	timeForStancePhase<<0, 0.24, 0.25, 0.49, 0.25, 0.49, 0, 0.24;
	MotionControl mc(timePeriod, timeForGaitPeriod, timeForStancePhase); // time
	Matrix<float, 4, 3> initPos; 
	initPos<< 3.0, 0.0, -233.83, 3.0, 0.0, -233.83, -10.0, 0.0, -233.83, -10.0, 0.0, -233.83;

	Vector<float, 3> tCV;
	tCV<<0.0, 0.0, 0.0;

	mc.setInitPos(initPos);
    cout<<(mc.timeForGaitPeriod - (mc.timeForStancePhase(0,1) - mc.timeForStancePhase(0,0)))/2<<endl;
    cout<<"initPos vel"<<endl;
	mc.setCoMVel(tCV);

    // Port init start
    set_port_baudrate_ID("/dev/ttyUSB0", 3000000, ID, num);
    dxl_init();
    set_operation_mode(3); //3 position control; 0 current control
    torque_enable();
    mc.inverseKinematics();
    set_position(mc.jointCmdPos);
    usleep(3e6);
    for(int times=0; times<1000; times++)
    {
        gettimeofday(&startTime1,NULL);
        // tCV(0) = times * 0.1;
        // mc.setCoMVel(tCV);
        mc.nextStep();
        cout<<"Time present: "<<mc.timePresent - mc.timePeriod<<"; leg stance flag: "<<mc.stanceFlag.transpose()<<endl;
        // <<"; CoM position: "<<mc.targetCoMPosition.transpose()<<endl;
        // cout<<"shoulder pos: "<<endl;
        // cout<<mc.shoulderPos<<endl;
        // cout<<"End pos: "<<endl;
        // cout<<mc.stancePhaseEndPos<<endl;
        // cout<<"foot cmd: "<<endl;
        cout<<mc.legCmdPos.row(0)<<" "<<mc.legCmdPos.row(1)<<" "<<mc.legCmdPos.row(2)<<" "<<mc.legCmdPos.row(3)<<" "<<endl;
        mc.inverseKinematics();
        set_position(mc.jointCmdPos);
        // for(int nums=0; nums<12; nums++) cout<<mc.jointCmdPos[nums]<<", ";
        // cout<<"."<<endl;
        gettimeofday(&endTime1,NULL);
        double timeUse1 = 1000000*(endTime1.tv_sec - startTime1.tv_sec) + endTime1.tv_usec - startTime1.tv_usec;
        usleep(timePeriod*1e6 - (double)(timeUse1) - 10); // /* 1e4 / 1e6 = 0.01s */
    }
    
    //cout<<timeUse1<<endl;
    
    return 0;
}
