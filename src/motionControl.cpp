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

    jointCmdPos[0] = motorInitPos[0] - jo_ang[0][0] + jo_ang[0][1];
    jointCmdPos[1] = motorInitPos[1] + jo_ang[0][0] + jo_ang[0][1];
    jointCmdPos[2] = motorInitPos[2] - jo_ang[0][2];
    jointCmdPos[3] = motorInitPos[3] - jo_ang[1][0] - jo_ang[1][1];
    jointCmdPos[4] = motorInitPos[4] + jo_ang[1][0] - jo_ang[1][1];
    jointCmdPos[5] = motorInitPos[5] + jo_ang[1][2];
    jointCmdPos[6] = motorInitPos[6] + jo_ang[3][0] - jo_ang[3][1];
    jointCmdPos[7] = motorInitPos[7] - jo_ang[3][0] - jo_ang[3][1];
    jointCmdPos[8] = motorInitPos[8] + jo_ang[3][2];
    jointCmdPos[9] = motorInitPos[9] + jo_ang[2][0] - jo_ang[2][1];
    jointCmdPos[10] = motorInitPos[10] - jo_ang[2][0] - jo_ang[2][1];
    jointCmdPos[11] = motorInitPos[11] - jo_ang[2][2];

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
    times++;
}

void MotionControl::forwardKinematics()
{
    float joint_pre_pos[4][3];
    joint_pre_pos[0][0] = (jointPresentPos[1] - motorInitPos[1] - jointPresentPos[0] + motorInitPos[0])/2;
    joint_pre_pos[0][1] = (jointPresentPos[1] - motorInitPos[1] + jointPresentPos[0] - motorInitPos[0])/2;
    joint_pre_pos[0][2] = - (jointPresentPos[2] - motorInitPos[2]);
    joint_pre_pos[1][0] = (jointPresentPos[4] - motorInitPos[4] - jointPresentPos[3] + motorInitPos[3])/2;
    joint_pre_pos[1][1] = -(jointPresentPos[4] - motorInitPos[4] + jointPresentPos[3] - motorInitPos[3])/2;
    joint_pre_pos[1][2] = jointPresentPos[5] - motorInitPos[5];
    joint_pre_pos[2][0] = (jointPresentPos[9] - motorInitPos[9] - jointPresentPos[10] + motorInitPos[10])/2;
    joint_pre_pos[2][1] = (-jointPresentPos[9] + motorInitPos[9] - jointPresentPos[10] + motorInitPos[10])/2;
    joint_pre_pos[2][2] = - (jointPresentPos[11] - motorInitPos[11]);
    joint_pre_pos[3][0] = (jointPresentPos[6] - motorInitPos[6] - jointPresentPos[7] + motorInitPos[7])/2;
    joint_pre_pos[3][1] = (-jointPresentPos[6] + motorInitPos[6] - jointPresentPos[7] + motorInitPos[7])/2;
    joint_pre_pos[3][2] = jointPresentPos[8] - motorInitPos[8];


    for(int leg_nums = 0; leg_nums < 4; leg_nums++)
    {
        legPresentPos(leg_nums,0) = L2 * cos(joint_pre_pos[leg_nums][0]) * cos(joint_pre_pos[leg_nums][1] + joint_pre_pos[leg_nums][2]) + L1 * cos(joint_pre_pos[leg_nums][0]) * cos(joint_pre_pos[leg_nums][1]);
        legPresentPos(leg_nums,1) = L2 * sin(joint_pre_pos[leg_nums][0]) * cos(joint_pre_pos[leg_nums][1] + joint_pre_pos[leg_nums][2]) + L1 * sin(joint_pre_pos[leg_nums][0]) * cos(joint_pre_pos[leg_nums][1]);
        legPresentPos(leg_nums,2) = L2 * sin(joint_pre_pos[leg_nums][1] + joint_pre_pos[leg_nums][2]) + L1 * sin(joint_pre_pos[leg_nums][1]);
    }

    for(int leg_num1 = 0; leg_num1 < 4; leg_num1++)
    {
        leg2CoMPrePos(leg_num1,0) = shoulderPos(leg_num1,1) + legPresentPos(leg_num1,2);
        leg2CoMPrePos(leg_num1,0) = shoulderPos(leg_num1,1) + legPresentPos(leg_num1,2);
        leg2CoMPrePos(leg_num1,0) = shoulderPos(leg_num1,1) + legPresentPos(leg_num1,2);
    }


}

void MotionControl::jacobians()
{
    jacobian(0 ,0) = -L3 * sin(jointPresentPos[0]) * cos(jointPresentPos[1] + jointPresentPos[2]) - L1 * sin(jointPresentPos[0]) * cos(jointPresentPos[1]);
    jacobian(0 ,1) =  L3 * cos(jointPresentPos[0]) * cos(jointPresentPos[1] + jointPresentPos[2]) + L1 * cos(jointPresentPos[0]) * cos(jointPresentPos[1]);
    jacobian(0 ,2) = 0;
    jacobian(0 ,3) = -L3 * cos(jointPresentPos[0]) * sin(jointPresentPos[1] + jointPresentPos[2]) - L1 * cos(jointPresentPos[0]) * sin(jointPresentPos[1]);
    jacobian(0 ,4) = -L3 * sin(jointPresentPos[0]) * sin(jointPresentPos[1] + jointPresentPos[2]) - L1 * sin(jointPresentPos[0]) * sin(jointPresentPos[1]);
    jacobian(0 ,5) = L3 * cos(jointPresentPos[1] + jointPresentPos[2]) + L1 * cos(jointPresentPos[1]);
    jacobian(0 ,6) = -L3 * cos(jointPresentPos[0]) * sin(jointPresentPos[1] + jointPresentPos[2]);
    jacobian(0 ,7) = -L3 * sin(jointPresentPos[0]) * sin(jointPresentPos[1] + jointPresentPos[2]);
    jacobian(0 ,8) = L3 * cos(jointPresentPos[1] + jointPresentPos[2]);

    jacobian(1 ,0)= -L3 * sin(jointPresentPos[3]) * cos(jointPresentPos[4] + jointPresentPos[5]) - L1 * sin(jointPresentPos[3]) * cos(jointPresentPos[4]);
    jacobian(1 ,1) =  L3 * cos(jointPresentPos[3]) * cos(jointPresentPos[4] + jointPresentPos[5]) + L1 * cos(jointPresentPos[3]) * cos(jointPresentPos[4]);
    jacobian(1 ,2) = 0;
    jacobian(1 ,3) = -L3 * cos(jointPresentPos[3]) * sin(jointPresentPos[4] + jointPresentPos[5]) - L1 * cos(jointPresentPos[3]) * sin(jointPresentPos[4]);
    jacobian(1 ,4) = -L3 * sin(jointPresentPos[3]) * sin(jointPresentPos[4] + jointPresentPos[5]) - L1 * sin(jointPresentPos[3]) * sin(jointPresentPos[4]);
    jacobian(1 ,5) = L3 * cos(jointPresentPos[4] + jointPresentPos[5]) + L1 * cos(jointPresentPos[4]);
    jacobian(1 ,6) = -L3 * cos(jointPresentPos[3]) * sin(jointPresentPos[4] + jointPresentPos[5]);
    jacobian(1 ,7) = -L3 * sin(jointPresentPos[3]) * sin(jointPresentPos[4] + jointPresentPos[5]);
    jacobian(1 ,8) = L3 * cos(jointPresentPos[4] + jointPresentPos[5]);

    jacobian(2 ,0)= -L3 * sin(jointPresentPos[6]) * cos(jointPresentPos[7] + jointPresentPos[8]) - L1 * sin(jointPresentPos[6]) * cos(jointPresentPos[7]);
    jacobian(2 ,1) =  L3 * cos(jointPresentPos[6]) * cos(jointPresentPos[7] + jointPresentPos[8]) + L1 * cos(jointPresentPos[6]) * cos(jointPresentPos[7]);
    jacobian(2 ,2) = 0;
    jacobian(2 ,3) = -L3 * cos(jointPresentPos[6]) * sin(jointPresentPos[7] + jointPresentPos[8]) - L1 * cos(jointPresentPos[6]) * sin(jointPresentPos[7]);
    jacobian(2 ,4) = -L3 * sin(jointPresentPos[6]) * sin(jointPresentPos[7] + jointPresentPos[8]) - L1 * sin(jointPresentPos[6]) * sin(jointPresentPos[7]);
    jacobian(2 ,5) = L3 * cos(jointPresentPos[7] + jointPresentPos[8]) + L1 * cos(jointPresentPos[7]);
    jacobian(2 ,6) = -L3 * cos(jointPresentPos[6]) * sin(jointPresentPos[7] + jointPresentPos[8]);
    jacobian(2 ,7) = -L3 * sin(jointPresentPos[6]) * sin(jointPresentPos[7] + jointPresentPos[8]);
    jacobian(2 ,8) = L3 * cos(jointPresentPos[7] + jointPresentPos[8]);

    jacobian(3, 0) = -L3 * sin(jointPresentPos[9]) * cos(jointPresentPos[10] + jointPresentPos[11]) - L1 * sin(jointPresentPos[9]) * cos(jointPresentPos[10]);
    jacobian(3, 1) =  L3 * cos(jointPresentPos[9]) * cos(jointPresentPos[10] + jointPresentPos[11]) + L1 * cos(jointPresentPos[9]) * cos(jointPresentPos[10]);
    jacobian(3, 2) = 0;
    jacobian(3, 3) = -L3 * cos(jointPresentPos[9]) * sin(jointPresentPos[10] + jointPresentPos[11]) - L1 * cos(jointPresentPos[9]) * sin(jointPresentPos[10]);
    jacobian(3, 4) = -L3 * sin(jointPresentPos[9]) * sin(jointPresentPos[10] + jointPresentPos[11]) - L1 * sin(jointPresentPos[9]) * sin(jointPresentPos[10]);
    jacobian(3, 5) = L3 * cos(jointPresentPos[10] + jointPresentPos[11]) + L1 * cos(jointPresentPos[10]);
    jacobian(3, 6) = -L3 * cos(jointPresentPos[9]) * sin(jointPresentPos[10] + jointPresentPos[11]);
    jacobian(3, 7) = -L3 * sin(jointPresentPos[9]) * sin(jointPresentPos[10] + jointPresentPos[11]);
    jacobian(3, 8) = L3 * cos(jointPresentPos[10] + jointPresentPos[11]);
}

void MotionControl::updateState()
{
    
}

void MotionControl::vmc()
{
    if (stanceFlag[0] == 0)
    {
        float xf= leg2CoMPrePos(0, 0);
        float yf= leg2CoMPrePos(0, 1);
        float zf= leg2CoMPrePos(0, 2);
        float xh= leg2CoMPrePos(3, 0);
        float yh= leg2CoMPrePos(3, 1);
        float zh= leg2CoMPrePos(3, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0, 
             0, 0, 1, 0, 0, 1, 
             0, -zf, yf, 0, -zh, yh, 
             zf, 0, -xf, zh, 0, -xh, 
             -yh, -xf, 0, yh, xh, 0;
        float kx = 0.02;
        float ky = 0.001;
        float kw = 0.02;
        float Fx = kx * (presentCoMVelocity[0] - targetCoMVelocity[0]);
        float Fy = ky * (presentCoMVelocity[1] - targetCoMVelocity[1]);
        float tao_z = kw * (presentCoMVelocity[2] - targetCoMVelocity[2]);
        B << Fx, Fy, 9.8, 0, 0, tao_z;
        double u=2.14;
        double k=1;
        a << -1, 0, 0, 1, 0, 0, 
            0, -1, 0, 0, 1, 0, 
            1, 1, -sqrt(2)*u/k, 0, 0, 0, 
            0, 0, 0, 1, 1, -sqrt(2)*u/k;
        b << Fx, Fy, 0, 0;
    }
    else
    {
        float xf= leg2CoMPrePos(1, 0);
        float yf= leg2CoMPrePos(1, 1);
        float zf= leg2CoMPrePos(1, 2);
        float xh= leg2CoMPrePos(2, 0);
        float yh= leg2CoMPrePos(2, 1);
        float zh= leg2CoMPrePos(2, 2);
        A << 1, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 1, 0, 
             0, 0, 1, 0, 0, 1, 
             0, -zf, yf, 0, -zh, yh, 
             zf, 0, -xf, zh, 0, -xh, 
             -yh, -xf, 0, yh, xh, 0;
        float kx = 0.02;
        float ky = 0.001;
        float kw = 0.02;
        float Fx = kx * (presentCoMVelocity[0] - targetCoMVelocity[0]);
        float Fy = ky * (presentCoMVelocity[1] - targetCoMVelocity[1]);
        float tao_z = kw * (presentCoMVelocity[2] - targetCoMVelocity[2]);
        B << Fx, Fy, 9.8, 0, 0, tao_z;
        double u=2.14;
        double k=1;
        a << -1, 0, 0, 1, 0, 0, 
            0, -1, 0, 0, 1, 0, 
            1, 1, -sqrt(2)*u/k, 0, 0, 0, 
            0, 0, 0, 1, 1, -sqrt(2)*u/k;
        b << Fx, Fy, 0, 0;
    }
    int m = 4;
    MatrixXf n(6+m, 6+m);
    n.block(0, 0, 6, 6) = A.transpose()*A;
    n.block(6, 0, m, 6) = a;
    n.block(0, 6, 6, m) = -a.transpose(); 
    n.block(6, 6, m, m) = MatrixXf::Zero(m,m);
    VectorXf nt(6+m, 1);
    nt.head(6) = A.transpose()*B;
    nt.tail(m) = b; 
    VectorXf x(6+m, 1);
    x = n.colPivHouseholderQr().solve(nt);
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
