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
#include "motor.h"

using namespace std;

const int num = 12;
int ID[num] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
int main(int argc, char ** argv)
{
	set_port_baudrate_ID("/dev/ttyUSB0", 3000000, ID, num);
	dxl_init();
	set_P_I_D(400,10,10); //P,I,D
	set_operation_mode(3); //3 position control; 0 current control
	torque_enable();
    thread_init();
    return 0;
}
