/*
 * imu_sensor_struct.c
 *
 * Created: 1/21/2009 12:31:08 PM
 *  Author: Haichen Shen
 */ 

#include "twi_cdc_timer.h"

//initialize struct values
void init_sensor_struct(sensor *sen)
{
	sen->appID = 4;
	sen->notificationID = 4;
	sen->taskID = 0;
	sen->persistent = 0; //continue after numSamples are done
	sen->sampleRate = 0; //rate of sampling
	sen->numSamples = 0; //num of samples to store
	sen->rateCntr = 0;
	sen->state = IDLE;
}

void reset_sensor_struct(sensor *sen)
{
	sen->rateCntr = 0;
}