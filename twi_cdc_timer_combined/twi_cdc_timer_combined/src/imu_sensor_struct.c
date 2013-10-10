/*
 * imu_sensor_struct.c
 *
 * Created: 1/21/2009 12:31:08 PM
 *  Author: Haichen Shen
 */ 

#include "twi_cdc_timer.h"

//initialize struct values
void init_sensor_struct(imu_struct *imu)
{
	imu->appID = 0;
	imu->notificationID = 0;
	imu->taskID = 0;
	imu->persistent = 0; //continue after numSamples are done
	imu->sampleRate = 0; //rate of sampling
	imu->numSamples = 0; //num of samples to store
	imu->rateCntr = 0;
	imu->state = IDLE;
}

void reset_sensor_struct(imu_struct *imu)
{
	imu->rateCntr = 0;
}