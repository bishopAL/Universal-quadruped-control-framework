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
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <fstream>

using namespace std;
using namespace Eigen;

class MotionControl
{
    public:
        float timeForGaitPeriod;  // The time of the whole period
        float timePeriod;  // The time of one period
        float timePresent;  
        float timeOneSwingPeriod;  // The swing time for diag legs
        Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
        Vector<float, 3> targetCoMVelocity;  // X, Y , alpha c in world cordinate
        Vector<float, 3> presentCoMVelocity;  // X, Y , alpha c in world cordinate
        Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        float yawVelocity;   // yaw velocity from imu
        Vector<bool, 4> stanceFlag;  // True, False: LF, RF, LH, RH
        Vector<float, 4> timePresentForSwing;
        float L1, L2, L3;  // The length of L
        float width, length;
        Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
        Matrix<float, 4, 3> stancePhaseStartPos;
        Matrix<float, 4, 3> stancePhaseEndPos;
        Matrix<float, 4, 3> legPresentPos;  // present X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
        Matrix<float, 4, 3> legCmdPos;  // command X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
        Matrix<float, 4, 3> leg2CoMPrePos;  // present X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Matrix<float, 4, 3> leg2CoMCmdPos;  // command X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Vector<float, 12> joint_pre_pos;  // present joint angle 0-11
        Vector<float, 12> joint_cmd_pos;  // command joint angle 0-11
        Vector<float, 12> jointPresentPos;  // present motor 0-11
        Vector<float, 12> jointPresentVel;  // present motor 0-11
        Matrix<float, 4, 9>jacobian; 
        Matrix<float, 6, 6>A;     //VMC
        Vector<float, 6>B;        //VMC
        Matrix<float, 4, 6>a;     //VMC
        Vector<float, 4>b;        //VMC
        float jointCmdPos[12];  // command motor 0-11
        float jointCmdPosLast[12];
        float jointCmdVel[12];
        float motorTorque[1];
        float motorInitPos[12];   // init motor angle of quadruped state
        float pid_motortorque[12];
        float jacobian_motortorque[12]; //motor torque in VMC
        float motorCmdTorque[12];
        bool initFlag;

        // the parameters of creeping gait
        float motIniPoCreep[12];    // init motor angle of creeping gait
        float H_onestep;  // The height of one step
        float Yaw_rad;      // The rad of yaw angle
        float k1,k2,k3;
        float L_diag;  // half of body diagonal size
        float beta_diag, alpha_diag; // structural angle of the body
        float v_body_x, v_body_y;    // the velocity of CoM
        float v_leg[4][2];  // the velocity of 4 legs
        float endPosition[4][2];  // the final feet position after one gait cycle
        Matrix<float, 4, 3> initPosS2L;  // init position from Shoulder to Leg
        Matrix<float, 4, 2> initPosC2L;  // init position from CoM to Leg
        Matrix<float, 4, 2> initPosC2S;  // init position from CoM to Shoulder
        float p_w2c[4][3];  // The relative position from world to CoM
        float p_c2s[4][3];  // The relative position from CoM to Shoulder
        float p_w2f[4][3];  // The relative position from world to leg
        float yawCreep;  // The command yaw of creeping gait
        float L1_creep, L2_creep, L3_creep;  // The length of creeping legs
        Matrix<float, 4, 3> legCmdVia;  // The intermediate variables for legCmdPos calculation of creeping gait

        void setInitPos(Matrix<float, 4, 3> initPosition);
        void setCoMVel(Vector<float, 3> tCV);
        void nextStep();
        void inverseKinematics();   // standing state
        void setInitial();
        void updateState();
        void creepingGait(float X_tar, float Y_tar, float Yaw_tar);   // the creeping gait from gecko_inspired
        void creepingIK();          // The inverse kinematics of creeping gait
        void standing2creeping();   // transfer from standing to creeping gait
        void creeping2standing();   // transfer from creeping to standing gait
        void forwardKinematics();
        void jacobians();
        void vmc();
        void pid();
        MotionControl(float tP, float tFGP, Matrix<float, 4, 2> tFSP);
};