#ifndef _THREAD_H_
#define _THREAD_H_ 
#include "motor.h"

void *thread1_func(void *data);
void *thread2_func(void *data);
void *thread3_func(void *data);
void *thread4_func(void *data);
void thread_init();
vector<string> split(const string& str, const string& delim);


#endif

