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
#include <iostream>

using namespace std;

struct timeval startTime, endTime;
float loopRate1 = 10; //receive velocity data 10 Hz
float loopRate2 = 10; // send velocity & IMU data 10 Hz
float loopRate3 = 100; // update robot state 200 Hz
float loopRate4 = 100; // motion control 200 Hz

vector<float> present_position;
vector<float> present_velocity;
vector<int> present_torque;
float target_position[12] = {0.702565, 0.446389, -0.493943, -0.909653, -0.949536, 0.518487, 
-1.2548, 0.702565, 0.36202, -1.21031, -0.779264, 0.406506};

void *thread1_func(void *data) // receive velocity data
{
    while(1)
    {
        gettimeofday(&startTime,NULL);
        /*
        YOUR CODE HERE
        */
        gettimeofday(&endTime,NULL);
        double timeUse = 1000000*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        usleep(1/loopRate1*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    }
}

void *thread2_func(void *data) // send velocity & IMU data
{
    while(1)
    {
        gettimeofday(&startTime,NULL);
        /*
        YOUR CODE HERE
        */
        gettimeofday(&endTime,NULL);
        double timeUse = 1000000*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        usleep(1/loopRate2*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    }
}


void *thread3_func(void *data) // update robot state
{
    while(1)
    {
        gettimeofday(&startTime,NULL);
        /*
        YOUR CODE HERE
        */
        
        set_position(target_position);
        get_position(present_position);
		get_velocity(present_velocity);
		get_torque(present_torque);
        gettimeofday(&endTime,NULL);
        double timeUse = 1000000*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;  // us
        cout<<"Time used: "<<timeUse<<", "<<present_position[0]<<", "<<present_velocity[0]<<", "<<present_torque[0]<<endl;
        usleep(1/loopRate3*1e6 - (double)(timeUse) - 10); // Time for one period: 1/loopRate3*1e6 (us)
    }
}

void *thread4_func(void *data) // motion control, update goal position
{
    while(1)
    {
        gettimeofday(&startTime,NULL);
        /*
        YOUR CODE HERE
        */
        
        gettimeofday(&endTime,NULL);
        double timeUse = 1000000*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        usleep(1/loopRate4*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    }
}


void thread_init()
{
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread1 ,thread2 ,thread3, thread4;
    int ret;
    /* 1.Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlockall failed: %m\n");
        exit(-2);
    }
    /* 2. Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret) {
        printf("init pthread attributes failed\n");
        goto out;
    }
 
    /* 3. Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }
 
    /*4. Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    param.sched_priority = 20;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
            printf("pthread setschedparam failed\n");
            goto out;
    }
    /*5. Use scheduling parameters of attr */
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (ret) {
                printf("pthread setinheritsched failed\n");
                goto out;
        }
 
    /*6. Create a pthread with specified attributes */
        ret = pthread_create(&thread1, &attr, thread1_func, NULL);
        if (ret) {
                printf("create pthread1 failed\n");
                goto out;
        }

        ret = pthread_create(&thread2, &attr, thread2_func, NULL);
        if (ret) {
                printf("create pthread2 failed\n");
                goto out;
        }

        ret = pthread_create(&thread3, &attr, thread3_func, NULL);
        if (ret) {
                printf("create pthread3 failed\n");
                goto out;
        }

        ret = pthread_create(&thread4, &attr, thread4_func, NULL);
        if (ret) {
                printf("create pthread4 failed\n");
                goto out;
        }


 
    /*7. Join the thread and wait until it is done */
        ret = pthread_join(thread1, NULL);
        if (ret)
            printf("join pthread1 failed: %m\n");

        ret = pthread_join(thread2, NULL);
        if (ret)
            printf("join pthread2 failed: %m\n");

        ret = pthread_join(thread3, NULL);
        if (ret)
            printf("join pthread4 failed: %m\n");

        ret = pthread_join(thread4, NULL);
        if (ret)
            printf("join pthread4 failed: %m\n");
out:
    ret;

}