#include <motionControl.h>
#define PI 3.1415926
#define BAUD 9600 // baudrate of imu
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
    motorInitPos[0] = 0.3145;
    motorInitPos[1] = 0.2378;
    motorInitPos[2] = 0.7854;
    motorInitPos[3] = 0.6888;
    motorInitPos[4] = 0.9050;
    motorInitPos[5] = -0.7854;
    motorInitPos[6] = 0.9234;
    motorInitPos[7] = -0.5906;
    motorInitPos[8] = -0.7854;
    motorInitPos[9] = 0.4663;
    motorInitPos[10] = -0.2424;
    motorInitPos[11] = 0.7854;

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

    joint_cmd_pos[0] = jo_ang[0][0];
    joint_cmd_pos[1] = jo_ang[0][1];
    joint_cmd_pos[2] = jo_ang[0][2];
    joint_cmd_pos[3] = jo_ang[1][0];
    joint_cmd_pos[4] = jo_ang[1][1];
    joint_cmd_pos[5] = jo_ang[1][2];
    joint_cmd_pos[6] = jo_ang[2][0];
    joint_cmd_pos[7] = jo_ang[2][1];
    joint_cmd_pos[8] = jo_ang[2][2];
    joint_cmd_pos[9] = jo_ang[3][0];
    joint_cmd_pos[10] = jo_ang[3][1];
    joint_cmd_pos[11] = jo_ang[3][2];

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
    float joint_pres_pos[4][3];
    joint_pres_pos[0][0] = (jointPresentPos[1] - motorInitPos[1] - jointPresentPos[0] + motorInitPos[0])/2;
    joint_pres_pos[0][1] = (jointPresentPos[1] - motorInitPos[1] + jointPresentPos[0] - motorInitPos[0])/2;
    joint_pres_pos[0][2] = - (jointPresentPos[2] - motorInitPos[2]);
    joint_pres_pos[1][0] = (jointPresentPos[4] - motorInitPos[4] - jointPresentPos[3] + motorInitPos[3])/2;
    joint_pres_pos[1][1] = -(jointPresentPos[4] - motorInitPos[4] + jointPresentPos[3] - motorInitPos[3])/2;
    joint_pres_pos[1][2] = jointPresentPos[5] - motorInitPos[5];
    joint_pres_pos[2][0] = (jointPresentPos[9] - motorInitPos[9] - jointPresentPos[10] + motorInitPos[10])/2;
    joint_pres_pos[2][1] = (-jointPresentPos[9] + motorInitPos[9] - jointPresentPos[10] + motorInitPos[10])/2;
    joint_pres_pos[2][2] = - (jointPresentPos[11] - motorInitPos[11]);
    joint_pres_pos[3][0] = (jointPresentPos[6] - motorInitPos[6] - jointPresentPos[7] + motorInitPos[7])/2;
    joint_pres_pos[3][1] = (-jointPresentPos[6] + motorInitPos[6] - jointPresentPos[7] + motorInitPos[7])/2;
    joint_pres_pos[3][2] = jointPresentPos[8] - motorInitPos[8];

    joint_pre_pos[0] = joint_pres_pos[0][0];
    joint_pre_pos[1] = joint_pres_pos[0][1];
    joint_pre_pos[2] = joint_pres_pos[0][2];
    joint_pre_pos[3] = joint_pres_pos[1][0];
    joint_pre_pos[4] = joint_pres_pos[1][1];
    joint_pre_pos[5] = joint_pres_pos[1][2];
    joint_pre_pos[6] = joint_pres_pos[2][0];
    joint_pre_pos[7] = joint_pres_pos[2][1];
    joint_pre_pos[8] = joint_pres_pos[2][2];
    joint_pre_pos[9] = joint_pres_pos[3][0];
    joint_pre_pos[10] = joint_pres_pos[3][1];
    joint_pre_pos[11] = joint_pres_pos[3][2];

    for(int leg_nums = 0; leg_nums < 4; leg_nums++)
    {
        legPresentPos(leg_nums,0) = L2 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]) + L1 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]);
        legPresentPos(leg_nums,1) = L2 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]) + L1 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]);
        legPresentPos(leg_nums,2) = L2 * sin(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]) + L1 * sin(joint_pres_pos[leg_nums][1]);
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
    jacobian(0 ,0) = -L3 * sin(joint_pre_pos[0]) * cos(joint_pre_pos[1] + joint_pre_pos[2]) - L1 * sin(joint_pre_pos[0]) * cos(joint_pre_pos[1]);
    jacobian(0 ,1) =  L3 * cos(joint_pre_pos[0]) * cos(joint_pre_pos[1] + joint_pre_pos[2]) + L1 * cos(joint_pre_pos[0]) * cos(joint_pre_pos[1]);
    jacobian(0 ,2) = 0;
    jacobian(0 ,3) = -L3 * cos(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]) - L1 * cos(joint_pre_pos[0]) * sin(joint_pre_pos[1]);
    jacobian(0 ,4) = -L3 * sin(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]) - L1 * sin(joint_pre_pos[0]) * sin(joint_pre_pos[1]);
    jacobian(0 ,5) = L3 * cos(joint_pre_pos[1] + joint_pre_pos[2]) + L1 * cos(joint_pre_pos[1]);
    jacobian(0 ,6) = -L3 * cos(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]);
    jacobian(0 ,7) = -L3 * sin(joint_pre_pos[0]) * sin(joint_pre_pos[1] + joint_pre_pos[2]);
    jacobian(0 ,8) = L3 * cos(joint_pre_pos[1] + joint_pre_pos[2]);

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
    Vector<float, 6> jacobian_Force;
    jacobian_Force = x.head(6);
    // Matrix<float, 3 ,3>
    // if (stanceFlag[0] == 0)
    // {
    //     jacobian_torque.head(6) = 
    // }
    
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


// creeping gait generated by X,Y,yaw from via points(position planning, independent part)
void MotionControl::creepingGait(float X_tar, float Y_tar, float Yaw_tar) 
{
    // // variables
    // float k1 = 0.25;
    // float k2 = 0.50;
    // float k3 = 0.25;
    
    // // record the cycle times
    // int count_forward = 1;
    // int num_forward = 3;
    
    // float L_diag;  // half of body diagonal size
    // float beta_diag, alpha_diag; // structural angle of the body
    // float v_body_x, v_body_y;    // the velocity of CoM
    // float v_leg[4][2];  // the velocity of 4 legs
    // float endPosition[4][2];  // the final feet position after one gait cycle
    
    // timeForGaitPeriod = 0.5;
    // timeOneSwingPeriod = 0.25;

    // Matrix<float, 4, 3> initPosS2L;  // init position from Shoulder to Leg
    // initPosS2L << 112.0, 132.0, -20.0, 112.0, -132.0, -20.0, -112.0, 132.0, -20.0, -112.0, -132.0, -20.0;  //unknown z
    // Matrix<float, 4, 2> initPosC2L;  // init position from CoM to Leg
    // initPosC2L << 198.0, 198.0, 198.0, -198.0, -198.0, 198.0, -198.0, -198.0;
    // Matrix<float, 4, 2> initPosC2S;  // init position from CoM to Shoulder
    // initPosC2S << 86.0, 66.0, 86.0, -66.0, -86.0, 66.0, -86.0, -66.0;

    // float p_w2c[4][3];  // The relative position from world to CoM
    // float p_c2s[4][3];  // The relative position from CoM to Shoulder
    // float p_w2f[4][3];  // The relative position from world to leg

    // float L_onestep = 50.0;  // The distance of one step
    // float H_onestep = 20.0;  // The height of one step
    // float v_x;
    // float Yaw_rad;      // The rad of yaw angle
    
    // int status = 1;   // for switching state

    
    // // some structural calculation
    // Yaw_rad = Yaw_tar * PI / 180.0;
    // L_diag = sqrt(initPosC2L(0,0)*initPosC2L(0,0) + initPosC2L(0,1)*initPosC2L(0,1));
    // beta_diag = atan(initPosC2L(0,0)/initPosC2L(0,1));
    // alpha_diag = PI / 2 - beta_diag;

    // endPosition[0][0] = X_tar + L_diag*sin(beta_diag - Yaw_rad);
    // endPosition[0][1] = Y_tar + L_diag*cos(beta_diag - Yaw_rad);
    // endPosition[1][0] = X_tar - L_diag*cos(alpha_diag - Yaw_rad);
    // endPosition[1][1] = Y_tar + L_diag*sin(alpha_diag - Yaw_rad);
    // endPosition[2][0] = X_tar + L_diag*cos(alpha_diag - Yaw_rad);
    // endPosition[2][1] = Y_tar - L_diag*sin(alpha_diag - Yaw_rad);
    // endPosition[3][0] = X_tar - L_diag*sin(beta_diag - Yaw_rad);
    // endPosition[3][1] = Y_tar - L_diag*cos(beta_diag - Yaw_rad);

    // v_body_x = X_tar / timeForGaitPeriod;
    // v_body_y = Y_tar / timeForGaitPeriod;
    // for(int leg_num1 = 0; leg_num1 < 4; leg_num1++)
    // {
    //     v_leg[leg_num1][0] = (endPosition[leg_num1][0] - initPosC2L(leg_num1,0)) / timeOneSwingPeriod;
    //     v_leg[leg_num1][1] = (endPosition[leg_num1][1] - initPosC2L(leg_num1,1)) / timeOneSwingPeriod;
    // }

    // // core creeping gait codes
    // // always changing variables
    // yawCreep = Yaw_rad / timeForGaitPeriod * timePresent;
    // for(int leg_num2 = 0; leg_num2 < 4; leg_num2++)
    // {
    //     // CoM planning(Uniform speed)
    //     p_w2c[leg_num2][0] = v_body_x * timePresent;
    //     p_w2c[leg_num2][1] = v_body_y * timePresent;
    //     p_w2c[leg_num2][2] = L3_creep;
    //     p_c2s[leg_num2][0] = initPosC2S(leg_num2,0);
    //     p_c2s[leg_num2][1] = initPosC2S(leg_num2,1);
    //     p_c2s[leg_num2][2] = 0.0;
    // }

    // // leg trajectory planning
    // if(timePresent < (0.5 * timeForGaitPeriod))
    // {
    //     //LF
    //     p_w2f[0][0] = initPosC2L(0,0) + v_leg[0][0] * timePresent;
    //     p_w2f[0][1] = initPosC2L(0,1) + v_leg[0][1] * timePresent;
    //     if(timePresent <= k1 * timeOneSwingPeriod)
    //     {
    //         p_w2f[0][2] = (H_onestep / (k1 * timeOneSwingPeriod)) * timePresent;
    //     }
    //     else if(timePresent <= (k1+k2) * timeOneSwingPeriod)
    //     {
    //         p_w2f[0][2] = H_onestep;
    //     }
    //     else
    //     {
    //         p_w2f[0][2] = (-H_onestep / (k1 * timeOneSwingPeriod)) * (timePresent-(k1+k2)*timeOneSwingPeriod) + H_onestep;
    //     }
    //     //RF
    //     p_w2f[1][0] = initPosC2L(1,0);
    //     p_w2f[1][1] = initPosC2L(1,1);
    //     p_w2f[1][2] = 0;
    //     //LH
    //     p_w2f[2][0] = initPosC2L(2,0);
    //     p_w2f[2][1] = initPosC2L(2,1);
    //     p_w2f[2][2] = 0;
    //     //RH
    //     p_w2f[3][0] = initPosC2L(3,0) + v_leg[3][0] * timePresent;
    //     p_w2f[3][1] = initPosC2L(3,1) + v_leg[3][1] * timePresent;
    //     if(timePresent <= k1 * timeOneSwingPeriod)
    //     {
    //         p_w2f[3][2] = (H_onestep / (k1 * timeOneSwingPeriod)) * timePresent;
    //     }
    //     else if(timePresent <= (k1+k2) * timeOneSwingPeriod)
    //     {
    //         p_w2f[3][2] = H_onestep;
    //     }
    //     else
    //     {
    //         p_w2f[3][2] = (-H_onestep / (k1 * timeOneSwingPeriod)) * (timePresent-(k1+k2)*timeOneSwingPeriod) + H_onestep;
    //     }

    //     // calculation from Shoulder to leg
    //     for(int leg_num3 = 0; leg_num3 < 4; leg_num3++)
    //     {
    //         legCmdVia(leg_num3,0) = (p_w2f[leg_num3][0] - p_w2c[leg_num3][0]) * cos(yawCreep) + (p_w2f[leg_num3][1] - p_w2c[leg_num3][1]) * sin(yawCreep) - p_c2s[leg_num3][0];
    //         legCmdVia(leg_num3,1) = (p_w2f[leg_num3][1] - p_w2c[leg_num3][1]) * cos(yawCreep) + (p_w2c[leg_num3][0] - p_w2f[leg_num3][0]) * sin(yawCreep) - p_c2s[leg_num3][1];
    //         legCmdVia(leg_num3,2) = p_w2f[leg_num3][2] - p_w2c[leg_num3][2] - p_c2s[leg_num3][2];
    //     }	
    // }
    // else
    // {            
    //     // leg trajectory planning
    //     //LF
    //     p_w2f[0][0] = endPosition[0][0];
    //     p_w2f[0][1] = endPosition[0][1];
    //     p_w2f[0][2] = 0.0;
    //     //RF
    //     p_w2f[1][0] = initPosC2L(1,0) + v_leg[1][0] * (timePresent - 0.5 * timeForGaitPeriod);
    //     p_w2f[1][1] = initPosC2L(1,1) + v_leg[1][1] * (timePresent - 0.5 * timeForGaitPeriod);
    //     if(timePresent <= (0.5 * timeForGaitPeriod + k1 * timeOneSwingPeriod))
    //     {
    //         p_w2f[1][2] = (H_onestep / (k1 * timeOneSwingPeriod)) * (timePresent - 0.5 * timeForGaitPeriod);
    //     }
    //     else if(timePresent <= (0.5 * timeForGaitPeriod + (k1+k2) * timeOneSwingPeriod))
    //     {
    //         p_w2f[1][2] = H_onestep;
    //     }
    //     else
    //     {
    //         p_w2f[1][2] = (-H_onestep / (k1 * timeOneSwingPeriod)) * ((timePresent - 0.5 * timeForGaitPeriod)-(k1+k2)*timeOneSwingPeriod) + H_onestep;
    //     }
    //     //LH
    //     p_w2f[2][0] = initPosC2L(2,0) + v_leg[2][0] * (timePresent - 0.5 * timeForGaitPeriod);
    //     p_w2f[2][1] = initPosC2L(2,1) + v_leg[2][1] * (timePresent - 0.5 * timeForGaitPeriod);
    //     p_w2f[2][2] = 0;
    //     if(timePresent <= (0.5 * timeForGaitPeriod + k1 * timeOneSwingPeriod))
    //     {
    //         p_w2f[2][2] = (H_onestep / (k1 * timeOneSwingPeriod)) * (timePresent - 0.5 * timeForGaitPeriod);
    //     }
    //     else if(timePresent <= (0.5 * timeForGaitPeriod + (k1+k2) * timeOneSwingPeriod))
    //     {
    //         p_w2f[2][2] = H_onestep;
    //     }
    //     else
    //     {
    //         p_w2f[2][2] = (-H_onestep / (k1 * timeOneSwingPeriod)) * ((timePresent - 0.5 * timeForGaitPeriod)-(k1+k2)*timeOneSwingPeriod) + H_onestep;
    //     }
    //     //RH
    //     p_w2f[3][0] = endPosition[3][0];
    //     p_w2f[3][1] = endPosition[3][1];
    //     p_w2f[3][2] = 0.0;
        

    //     // calculation from Shoulder to leg
    //     for(int leg_num3 = 0; leg_num3 < 4; leg_num3++)
    //     {
    //         legCmdVia(leg_num3,0) = (p_w2f[leg_num3][0] - p_w2c[leg_num3][0]) * cos(yawCreep) + (p_w2f[leg_num3][1] - p_w2c[leg_num3][1]) * sin(yawCreep) - p_c2s[leg_num3][0];
    //         legCmdVia(leg_num3,1) = (p_w2f[leg_num3][1] - p_w2c[leg_num3][1]) * cos(yawCreep) + (p_w2c[leg_num3][0] - p_w2f[leg_num3][0]) * sin(yawCreep) - p_c2s[leg_num3][1];
    //         legCmdVia(leg_num3,2) = p_w2f[leg_num3][2] - p_w2c[leg_num3][2] - p_c2s[leg_num3][2];
    //     }
    // }

    // timePresent += timePeriod;

    // if (timePresent >= timeForGaitPeriod)
    // {
    //     timePresent = 0;
    //     if(count_forward < num_forward)
    //     {
    //         count_forward = count_forward + 1;	
    //     }
    //     else
    //     {
    //         count_forward = 1;
    //     // delay_ms(2000);
    //     }
    // }
}

void MotionControl::pid()
{
    Vector<float, 12> temp_jointCmdPos, temp_jointCmdVel;
    for(uint8_t joints=0; joints<12; joints++)
    {
        temp_jointCmdPos(joints) = jointCmdPos[joints];
        temp_jointCmdVel(joints) = jointCmdVel[joints];
    }
    // cout<<"jointPresentPos: "<<mc.jointPresentPos.transpose()<<endl;
    // cout<<"yawVelocity: "<<mc.yawVelocity<<endl;
    //cout<<"jointPresentVel: "<<mc.jointPresentVel.transpose()<<endl;
    // cout<<"jointCmdPos: "<<temp_jointCmdPos.transpose()<<endl;
    //cout<<"jointCmdVel: "<<temp_jointCmdVel.transpose()<<endl;
    Vector<float, 12> temp_motorCmdTorque;
    temp_motorCmdTorque = 5 * (temp_jointCmdPos - jointPresentPos) + 0.2 * (temp_jointCmdVel - jointPresentVel);
    // float motorCmdTorque[12];
    for(uint8_t joints=0; joints<12; joints++)
    {
        motorCmdTorque[joints] = temp_motorCmdTorque(joints);
    }
}

void MotionControl::standing2creeping()
{

}

void MotionControl::creeping2standing()
{

}

void MotionControl::imu()
{
    // static int ret;
    // static int fd;

    // char r_buf[1024];
    // bzero(r_buf,1024);

    // fd = uart_open(fd,"/dev/ttyUSB0");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn */ 
    // if(fd == -1)
    // {
    //     fprintf(stderr,"uart_open error\n");
    //     exit(EXIT_FAILURE);
    // }

    // if(uart_set(fd,BAUD,8,'N',1) == -1)
    // {
    //     fprintf(stderr,"uart set failed!\n");
    //     exit(EXIT_FAILURE);
    // }

	// FILE *fp;
	// fp = fopen("Record.txt","w");
    // while(1)
    // {
    //     ret = recv_data(fd,r_buf,44);
    //     if(ret == -1)
    //     {
    //         fprintf(stderr,"uart read failed!\n");
    //         exit(EXIT_FAILURE);
    //     }
	// 	for (int i=0;i<ret;i++) 
    //     {
    //         fprintf(fp,"%2X ",r_buf[i]);
    //         yawVelocity = ParseData(r_buf[i]);
    //     }
    //     usleep(1000);
    // }

    // ret = uart_close(fd);
    // if(ret == -1)
    // {
    //     fprintf(stderr,"uart_close error\n");
    //     exit(EXIT_FAILURE);
    // }

    // exit(EXIT_SUCCESS);
}



