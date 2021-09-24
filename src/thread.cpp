#include <iostream>
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
#include <math.h>
#include <CppLinuxSerial/SerialPort.hpp>
#include <motionControl.h>
#include <Eigen/Core>
#include <js.h>
#include <imu.h>

#define PI 3.1415926
#define _JOYSTICK 1
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
#define THREAD3_ENABLE 1
#define THREAD4_ENABLE 1


using namespace std;

struct timeval startTime, endTime;
float loopRate1 = 10; //receive velocity data 10 Hz
float loopRate2 = 300; // send velocity & IMU data 10 Hz
float loopRate3 = 100; // update robot state 100 Hz
float loopRate4 = 100; // motion control 100 Hz

const int num = 12;
int ID[num] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

vector<float> present_position;
vector<float> present_velocity;
vector<int> present_torque;
float target_com_velocity[3];
float present_com_velocity[3];

float timePeriod = 0.01;
float timeForGaitPeriod = 0.49;
Matrix<float, 4, 2> timeForStancePhase;
MotionControl mc(timePeriod, timeForGaitPeriod, timeForStancePhase); // time

void *thread1_func(void *data) // receive velocity data
{
    struct timeval startTime1, endTime1;

    #ifdef _JOYSTICK
    int xbox_fd ;
    xbox_map_t map;
    int len, type;
    int axis_value, button_value;
    int number_of_axis, number_of_buttons ;
    float vel = 0;
    float theta = 0;
    memset(&map, 0, sizeof(xbox_map_t));
    xbox_fd = xbox_open("/dev/input/js0");
    map.lt = -32767;
    map.rt = -32767;
    while(1)
    {
        gettimeofday(&startTime1,NULL);
        len = xbox_map_read(xbox_fd, &map);
        if (len < 0)
        {
            usleep(10*1000);
            continue;
        }
        if (map.lx==0) theta = 0;
        else theta = float(atan2(-map.ly, map.lx) - 3.1416/2) / 5.0;
        vel = float(map.rt - map.lt) / 200.0;
        Vector<float, 3> tCV;
        tCV<<vel, 0.0, theta;
        mc.setCoMVel(tCV);
        cout<<tCV.transpose()<<", "<<-map.ly<<", "<<map.lx<<", "<<map.lt<<", "<<map.rt<<endl;
        gettimeofday(&endTime1,NULL);
        double timeUse = 1000000*(endTime1.tv_sec - startTime1.tv_sec) + endTime1.tv_usec - startTime1.tv_usec;
        fflush(stdout);
        // cout<<"thread1: "<<timeUse<<endl;
        // usleep(1/loopRate1*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    }
    #endif
    // SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600);
	// // Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	// serialPort.SetTimeout(100); // Block when reading until any data is received
	// serialPort.Open();
    // string endFlag = "E";
}

void *thread2_func(void *data) // send velocity & IMU data
{
    struct timeval startTime2, endTime2;
    float temp_imu;

    fd = uart_open(fd,"/dev/ttyUSB0");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn */ 
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    while(1)
    {
        gettimeofday(&startTime2,NULL);
   
        /* imu start */
        char r_buf[1024];
        bzero(r_buf,1024);

        if(uart_set(fd,BAUD,8,'N',1) == -1)
        {
            fprintf(stderr,"uart set failed!\n");
            exit(EXIT_FAILURE);
        }

        // FILE *fp;
        // fp = fopen("Record.txt","w");
        while(1)
        {
            ret = recv_data(fd,r_buf,44);
            if(ret == -1)
            {
                fprintf(stderr,"uart read failed!\n");
                break;
            }
            for (int i=0;i<ret;i++) 
            {
                //fprintf(fp,"%2X ",r_buf[i]);
                float temp;
                temp = ParseData(r_buf[i]);
                if(temp<10000) 
                {
                mc.z_pre_vel = temp;
                cout << "z_pre_vel: " << mc.z_pre_vel << endl;
                }
            }
            // usleep(500);
        }

        ret = uart_close(fd);
        if(ret == -1)
        {
            fprintf(stderr,"uart_close error\n");
            exit(EXIT_FAILURE);
        }

        //exit(EXIT_SUCCESS);
        /* imu end */
        

        gettimeofday(&endTime2,NULL);
        double timeUse = 1000000*(endTime2.tv_sec - startTime2.tv_sec) + endTime2.tv_usec - startTime2.tv_usec;
        cout<<"thread2: "<<timeUse<<endl;
        usleep(1/loopRate2*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    }
}

void *thread3_func(void *data) // update robot state
{// Port init start
    set_port_baudrate_ID("/dev/ttyAMA0", 3000000, ID, num);
    dxl_init();
    set_operation_mode(0); //3 position control; 0 current control
    torque_enable();
    while(mc.initFlag==false);
    struct timeval startTime3, endTime3;
    while(1)
    {
        gettimeofday(&startTime3,NULL);
        /*
        YOUR CODE HERE
        */
        // set_position(target_position);
        // get_position(present_position);
		// get_velocity(present_velocity);
		// get_torque(present_torque);
        get_position(present_position);
        get_velocity(present_velocity);
        for(int joints=0; joints<12; joints++)
        {
            mc.jointPresentPos(joints) = present_position[joints];
            mc.jointPresentVel(joints) = present_velocity[joints];
        }

        mc.jacobians();
        mc.pid();
        mc.vmc();
        float k = 0.0;
        for(uint8_t joints=0; joints<12; joints++)
        {
            mc.motorCmdTorque[joints] = mc.pid_motortorque[joints] + k * mc.jacobian_motortorque[joints];
        }
        // set_torque(mc.motorCmdTorque);

        // cout << "present zitai :" << mc.presentCoMVelocity(0) <<" "<< mc.presentCoMVelocity(1) << " " << mc.presentCoMVelocity(2) << endl;
        // cout << "target zitai :"<<mc.targetCoMVelocity(0) <<" "<< mc.targetCoMVelocity(1) << " " << mc.targetCoMVelocity(2) << endl;
        // cout<<"jointPresentPos: "<<mc.jointPresentPos.transpose()<<endl;
        // cout<<"jointPresentVel: "<<mc.jointPresentVel.transpose()<<endl;
        // cout <<"vmc: "<< mc.jacobian_motortorque[0]<< " "<< mc.jacobian_motortorque[1] << " "<< mc.jacobian_motortorque[2]<< " "<< mc.jacobian_motortorque[3]<< " "<< mc.jacobian_motortorque[4]<< " "<< mc.jacobian_motortorque[5]<< " "<< mc.jacobian_motortorque[6]<< " "<< mc.jacobian_motortorque[7]<< " "<< mc.jacobian_motortorque[8]<< " "<< mc.jacobian_motortorque[9]<< " "<< mc.jacobian_motortorque[10]<< " "<< mc.jacobian_motortorque[11]<< endl;
        // cout <<"pid: "<<mc.pid_motortorque[0]<<" "<<mc.pid_motortorque[1]<< " "<< mc.pid_motortorque[2]<< " "<< mc.pid_motortorque[3]<< " "<< mc.pid_motortorque[4]<< " "<< mc.pid_motortorque[5]<< " "<< mc.pid_motortorque[6]<< " "<< mc.pid_motortorque[7]<< " "<< mc.pid_motortorque[8]<< " "<< mc.pid_motortorque[9]<< " "<< mc.pid_motortorque[10]<< " "<< mc.pid_motortorque[11]<< endl;

        //set_position(mc.jointCmdPos);
        //cout << "jointcmdpos: " << mc.jointCmdPos[1] << endl;
        //cout << "legcmdpos: " << mc.legCmdPos.row(0) << endl;

        gettimeofday(&endTime3,NULL);
        double timeUse = 1000000*(endTime3.tv_sec - startTime3.tv_sec) + endTime3.tv_usec - startTime3.tv_usec;  // us

        ofstream f("data.txt", ios::app);
	    f<<mc.presentCoMVelocity(0)<<" "<<mc.presentCoMVelocity(1)<<" "<<mc.presentCoMVelocity(2)<<endl;
        f.close();
    
        //cout<<"thread3: "<<temp_motorCmdTorque.transpose()<<endl;
        usleep(1/loopRate3*1e6 - (double)(timeUse) - 10); // Time for one period: 1/loopRate3*1e6 (us)
    }
}

void *thread4_func(void *data) // motion control, update goal position
{
    struct timeval startTime4, endTime4;
    Matrix<float, 4, 3> initPos; 
	initPos<< 3.0, 0.0, -225.83, 3.0, 0.0, -225.83, -20.0, 0.0, -243.83, -20.0, 0.0, -243.83;
	Vector<float, 3> tCV;
	tCV<< 0.0, 0.0, 0.0;
	mc.setInitPos(initPos);
	mc.setCoMVel(tCV);
    mc.forwardKinematics();
    mc.inverseKinematics();
    mc.initFlag = true;
    usleep(3e6);
    while(1)
    {
        gettimeofday(&startTime4,NULL);
        /*
        YOUR CODE HERE
        */
        mc.nextStep();
        mc.inverseKinematics();
        gettimeofday(&endTime4,NULL);
        double timeUse = 1000000*(endTime4.tv_sec - startTime4.tv_sec) + endTime4.tv_usec - startTime4.tv_usec;
        // cout<<"thread4: "<<timeUse<<endl;
        usleep(1/loopRate4*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    }

    // struct timeval startTime4, endTime4;
    // Matrix<float, 4, 3> initPos; 
	// initPos<< -132.0, 112.0, -46.0, 132.0, 112.0, -46.0, -132.0, -112.0, -46.0, 132.0, -112.0, -46.0;
	// Vector<float, 3> tCV;
	// tCV<< 0.0, 0.0, 0.0;
	// mc.setInitPos(initPos);
	// //mc.setCoMVel(tCV);
    // mc.creepingIK();
    // mc.initFlag = true;
    // usleep(3e6);
    // while(1)
    // {
    //     gettimeofday(&startTime4,NULL);
    //     mc.creepingGait(30,0,0);
    //     mc.creepingIK();
    //     gettimeofday(&endTime4,NULL);
    //     double timeUse = 1000000*(endTime4.tv_sec - startTime4.tv_sec) + endTime4.tv_usec - startTime4.tv_usec;
    //     // cout<<"thread4: "<<timeUse<<endl;
    //     usleep(1/loopRate4*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    // }
}

void thread_init()
{
    struct sched_param param1, param2, param3, param4;
    pthread_attr_t attr1, attr2, attr3, attr4;
    pthread_t thread1 ,thread2 ,thread3, thread4;
    int ret;

    timeForStancePhase << 0, 0.24, 0.25, 0.49, 0.25, 0.49, 0, 0.24;
    mc.timeForStancePhase = timeForStancePhase;
    
    /* 1.Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlockall failed: %m\n");
        exit(-2);
    }
    /* 2. Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr1);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr2);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr3);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
    ret = pthread_attr_init(&attr4);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
 
    /* 3. Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr1, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr2, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr3, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
    ret = pthread_attr_setstacksize(&attr4, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
 
    /*4. Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr1, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr2, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr3, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    ret = pthread_attr_setschedpolicy(&attr4, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    
    param1.sched_priority = 20;
    param2.sched_priority = 1;
    param3.sched_priority = 20;
    param4.sched_priority = 20;

    ret = pthread_attr_setschedparam(&attr1, &param1);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr2, &param2);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr3, &param3);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    ret = pthread_attr_setschedparam(&attr4, &param4);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }

    /*5. Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr1, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr2, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr3, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
    ret = pthread_attr_setinheritsched(&attr4, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
            goto out;
    }
 
    /*6. Create a pthread with specified attributes */
    #ifdef THREAD1_ENABLE
    ret = pthread_create(&thread1, &attr1, thread1_func, NULL);
    if (ret) {
            printf("create pthread1 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD2_ENABLE
    ret = pthread_create(&thread2, &attr2, thread2_func, NULL);
    if (ret) {
            printf("create pthread2 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD3_ENABLE
    ret = pthread_create(&thread3, &attr3, thread3_func, NULL);
    if (ret) {
            printf("create pthread3 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD4_ENABLE
    ret = pthread_create(&thread4, &attr4, thread4_func, NULL);
    if (ret) {
            printf("create pthread4 failed\n");
            goto out;
    }
    #endif

    #ifdef THREAD1_ENABLE
    ret = pthread_join(thread1, NULL);
    if (ret)
        printf("join pthread1 failed: %m\n");
    #endif

    #ifdef THREAD2_ENABLE
    ret = pthread_join(thread2, NULL);
    if (ret)
        printf("join pthread2 failed: %m\n");
    #endif

    #ifdef THREAD3_ENABLE
    ret = pthread_join(thread3, NULL);
    if (ret)
        printf("join pthread3 failed: %m\n");
    #endif

    #ifdef THREAD4_ENABLE
    ret = pthread_join(thread4, NULL);
    if (ret)
        printf("join pthread4 failed: %m\n");
    #endif
 
    /*7. Join the thread and wait until it is done */
    
out:
    ret;

}

vector<string> split(const string& str, const string& delim) {  
	vector<string> res;  
	if("" == str) return res;  
	char * strs = new char[str.length() + 1] ; 
	strcpy(strs, str.c_str());   
 
	char * d = new char[delim.length() + 1];  
	strcpy(d, delim.c_str());  
 
	char *p = strtok(strs, d);  
	while(p) {  
		string s = p;   
		res.push_back(s); 
		p = strtok(NULL, d);  
	}  
 
	return res;  
}