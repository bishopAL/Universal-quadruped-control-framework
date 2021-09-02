#include <motionControl.h>

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
    L1 = 100;
    L2 = 100;
    L3 = 0;
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