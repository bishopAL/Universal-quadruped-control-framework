#include <motionControl.h>
#define PI 3.1415926
using namespace std;
using namespace Eigen;

MotionControl::MotionControl(float tP, float tFGP, Matrix<float, 4, 2> tFSP)
{
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

void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    stancePhaseStartPos = initPosition;
    stancePhaseEndPos = initPosition;
    legPresentPos = initPosition;
    legCmdPos = initPosition;
    targetCoMPosition << 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0;
}

void MotionControl::setCoMVel(Vector<float, 3> tCV)
{
    targetCoMVelocity = tCV;
}

void MotionControl::nextStep()
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {
        if(timePresent > timeForStancePhase.row(legNum)(0)-timePeriod/2 && timePresent < timeForStancePhase.row(legNum)(1)+timePeriod/2 )
        {     // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
            if(abs(timePresent - timeForStancePhase.row(legNum)(0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                for(uint8_t pos=0; pos<3; pos++)
                targetCoMPosition(legNum, pos) = 0.0;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(targetCoMPosition(legNum,2)), -sin(targetCoMPosition(legNum,2)), targetCoMPosition(legNum,0),
                   sin(targetCoMPosition(legNum,2)), cos(targetCoMPosition(legNum,2)), targetCoMPosition(legNum,1),
                   0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<shoulderPos.row(legNum)(0), shoulderPos.row(legNum)(1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

            if(abs(timePresent - timeForStancePhase.row(legNum)(0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(timePresent - timeForStancePhase.row(legNum)(1)) < 1e-4)  // if on the end pos
                stancePhaseEndPos(legNum) = legCmdPos(legNum);

            legCmdPos(legNum, 0) = stancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            legCmdPos(legNum, 1) = stancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
            stanceFlag(legNum) = true;
        }
        else
        {
            Matrix<float, 1, 3> swingPhaseVelocity = (stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / 
                                        (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)) - timePeriod);
            
            for(uint8_t pos=0; pos<3; pos++)
            legCmdPos(legNum, pos) = legCmdPos(legNum, pos) - swingPhaseVelocity(pos) * timePeriod;
            
            if( ( timePresentForSwing(legNum) - (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)))/2 ) > 1e-4)
            legCmdPos(legNum, 2) -= 3.0;
            if( ( timePresentForSwing(legNum) - (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)))/2 ) < -1e-4 && timePresentForSwing(legNum) > 1e-4)
            legCmdPos(legNum, 2) += 3.0;
            stanceFlag(legNum) = false;
        }
    }

    timePresent += timePeriod;
    for(uint8_t leg=0; leg<4; leg++)
    {
        for(uint8_t pos=0; pos<3; pos++)
        {
            targetCoMPosition(leg, pos) += targetCoMVelocity(pos) * timePeriod;
        }
        if(stanceFlag(leg) == 0) timePresentForSwing(leg) += timePeriod;
        else timePresentForSwing(leg) = 0;
    }

    if (abs(timePresent - timeForGaitPeriod - timePeriod) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        timePresent = 0.0;
    }

}

void MotionControl::inverseKinematics()
{
    float theta[4][3] = {0};
    float jo_ang[4][3] = {0};
    float a1[4] = {0};
    float b1[4] = {0};
    float c1[4] = {0};
    float a2[4] = {0};
    float b2[4] = {0};
    float c2[4] = {0};
    static int times = 0;

    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdPosLast[joints] = jointCmdPos[joints];
        }
    }

    for(int leg_num = 0; leg_num < 4; leg_num++)
    {
        a1[leg_num] = - legCmdPos(leg_num,2);
        b1[leg_num] = - legCmdPos(leg_num,1);
        c1[leg_num] = 0;
        theta[leg_num][0] = asin((c1[leg_num])/sqrt((a1[leg_num])*(a1[leg_num]) + (b1[leg_num])*(b1[leg_num]))) - atan(b1[leg_num]/a1[leg_num]);
        jo_ang[leg_num][0] = theta[leg_num][0];
        theta[leg_num][2] = acos(((- legCmdPos(leg_num,2) * cos(theta[leg_num][0]) + legCmdPos(leg_num,1) * sin(theta[leg_num][0]))*
                        (- legCmdPos(leg_num,2) * cos(theta[leg_num][0]) + legCmdPos(leg_num,1) * sin(theta[leg_num][0])) + 
                        legCmdPos(leg_num,0)*legCmdPos(leg_num,0) - L1*L1 - L2*L2)/(2*L1*L2));
        //jo_ang[leg_num][2] = theta[leg_num][2] - PI / 3;
        jo_ang[leg_num][2] = theta[leg_num][2];
        a2[leg_num] = -(- legCmdPos(leg_num,2)) * cos(theta[leg_num][0]) - (legCmdPos(leg_num,1)) * sin(theta[leg_num][0]);
        b2[leg_num] = legCmdPos(leg_num,0);
        c2[leg_num] = L2 * sin(theta[leg_num][2]);
        theta[leg_num][1] = asin((-c2[leg_num])/sqrt((a2[leg_num])*(a2[leg_num]) + (b2[leg_num])*(b2[leg_num]))) - atan(b2[leg_num]/a2[leg_num]);
        //jo_ang[leg_num][1] = -(theta[leg_num][1] + PI / 6);
        jo_ang[leg_num][1] = -theta[leg_num][1];
    }

    jointCmdPos[0] =  -1.3315 - jo_ang[0][0] + jo_ang[0][1];
    jointCmdPos[1] =  1.4788 + jo_ang[0][0] + jo_ang[0][1];
    jointCmdPos[2] = 0.7854 - jo_ang[0][2];
    jointCmdPos[3] = 0.5292 - jo_ang[1][0] - jo_ang[1][1];
    jointCmdPos[4] = 0.7670 + jo_ang[1][0] - jo_ang[1][1];
    jointCmdPos[5] = -0.7854 + jo_ang[1][2];
    jointCmdPos[6] = 1.7058 + jo_ang[3][0] - jo_ang[3][1];
    jointCmdPos[7] = -0.3421 - jo_ang[3][0] - jo_ang[3][1];
    jointCmdPos[8] = -0.7854 + jo_ang[3][2];
    jointCmdPos[9] = 0.2770 + jo_ang[2][0] - jo_ang[2][1];
    jointCmdPos[10] = -0.2954 - jo_ang[2][0] - jo_ang[2][1];
    jointCmdPos[11] = 0.7854 - jo_ang[2][2];

    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = (jointCmdPos[joints] - jointCmdPosLast[joints]) / timePeriod;
        }
    }
    else
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = 0;
        }
    }
}

void MotionControl::updateState()
{
    
}


void MotionControl::setInitial()
{
    // float jo_pos[12] = {0.0};
    // float ap,bt,gm;
    // float T_cal = 0.00;

    // for(int item = 0; item < 200; item++)
    // {
    //     ap = 0.0;
    //     bt = (PI/6) / T_cycle * T_cal;
    //     gm = (PI/3) / T_cycle * T_cal;
    //     T_cal = T_cal + timePeriod;
    // }
    
    // jointCmdPos[0] =  -1.3315 - ap + bt;
    // jointCmdPos[1] =  1.4788 + ap + bt;
    // jointCmdPos[2] = 0.7854 - gm;
    // jointCmdPos[3] = 0.5292 - ap - bt;
    // jointCmdPos[4] = 0.7670 + ap - bt;
    // jointCmdPos[5] = -0.7854 + gm;
    // jointCmdPos[6] = 1.7058 + ap - bt;
    // jointCmdPos[7] = -0.3421 - ap - bt;
    // jointCmdPos[8] = -0.7854 + gm;
    // jointCmdPos[9] = 0.2770 + ap - bt;
    // jointCmdPos[10] = -0.2954 - ap - bt;
    // jointCmdPos[11] = 0.7854 - gm;
}


void MotionControl::creepingGait()
{
    
}
