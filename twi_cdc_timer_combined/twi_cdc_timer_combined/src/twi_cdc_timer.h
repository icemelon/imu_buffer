/******************************************************************************
 * FILE NAME : twi_cdc_timer.h
 * DESCRIPTION : header file for the sensor phone interface buffering system
 * AUTHORS : Eric Yuan
 * DATE : 8/14/2013 7:53:27 PM
 ****************************************************************************/

#ifndef TWI_CDC_TIMER_H_
#define TWI_CDC_TIMER_H_

#include <asf.h>
#include <stdio.h>

//twi addresses for working with the imu
#define TWI_MASTER       TWIC
#define TWI_SPEED        50000
#define TWI_MASTER_ADDR  0x50
#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_ACCEL_XOUT_H       0x3B   // R

#define BUFFER_CAPACITY			1024
#define BUFFER_CAPACITY_MASK	0x3FF

//#define DEBUG

//imu struct that holds data values
typedef struct imu_data_struct
{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} imu_data;

typedef enum enum_working_state {
	IDLE = 0,
	BUFFER_ONCE,
	BUFFER_PERSIST,
} working_state;

//imu general struct to store cell info
typedef struct imu_sensor_struct
{
	char appID;
	char notificationID;
	char taskID;
	char persistent; //continue after numSamples are done
	uint16_t sampleRate; //rate of sampling
	uint16_t numSamples; //num of samples before CPU is woken up
	int16_t rateCntr;
	working_state state;
} imu_struct;

typedef struct imu_data_buffer {
	imu_data *data;
	int16_t capacity;
	int16_t write_pt;
	int16_t read_pt;
} imu_data_buffer;

/***********************************************************************************
FUNCTION DECLARATIONS
***********************************************************************************/
void init_sensor_struct(imu_struct *imu);
void reset_sensor_struct(imu_struct *imu);

void init_buffer(imu_data_buffer *buffer);
void reset_buffer(imu_data_buffer *buffer);
int16_t buffer_size(imu_data_buffer *buffer);

void twi_init();
void mpu6050_write_reg();
void mpu6050_read(imu_data *data);
void cancel();

#endif /* TWI_CDC_TIMER_H_ */