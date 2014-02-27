/******************************************************************************
 * FILE NAME : twi_cdc_timer.h
 * DESCRIPTION : header file for the sensor phone interface buffering system
 * AUTHORS : Eric Yuan, Ryan Tsoi
 * DATE : 8/14/2013 7:53:27 PM
 ****************************************************************************/

#ifndef TWI_CDC_TIMER_H_
#define TWI_CDC_TIMER_H_

#include <asf.h>
#include <stdio.h>
#include <adc.h>

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

typedef enum enum_working_state
{
	IDLE = 0,
	BUFFER_ONCE,
	BUFFER_PERSIST,
} working_state;

//imu general struct to store cell info
typedef struct sensor_struct
{
	char appID;
	char notificationID;
	char taskID;
	char persistent; //continue after numSamples are done
	uint16_t sampleRate; //rate of sampling
	uint16_t numSamples; //num of samples before CPU is woken up
	int16_t rateCntr;
	working_state state;
} sensor;

typedef struct data_buffer_struct
{
	void *data;
	int16_t capacity;
	int16_t write_pt;
	int16_t read_pt;
	int16_t min;
	int16_t max;
} data_buffer;

/////////////////////////////////////////////
///////////////  IMU STRUCTS  ///////////////
/////////////////////////////////////////////

//imu struct that holds data values for orientation
typedef struct gyro_data_struct
{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} gyro_data;

//imu struct that holds data values for temperature
typedef struct temp_data_struct
{
	int16_t temp;
} temp_data;

//imu struct that holds data values for acceleration
typedef struct accel_data_struct
{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} accel_data;

/////////////////////////////////////////////
///////////////  MIC STRUCTS  ///////////////
/////////////////////////////////////////////

//mic struct that holds data values for volume
typedef struct vol_data_struct
{
	int16_t vol_sample;
} vol_data;

/*
typedef struct mic_data_buffer_struct
{
	mic_data *data;
	int16_t capacity;
	int16_t write_pt;
	int16_t read_pt;
} mic_data_buffer;
*/
/***********************************************************************************
FUNCTION DECLARATIONS
***********************************************************************************/
void init_sensor_struct(sensor *imu);
void reset_sensor_struct(sensor *imu);

void init_gyro_buffer(data_buffer *buffer);
void init_temp_buffer(data_buffer *buffer);
void init_accel_buffer(data_buffer *buffer);
void init_vol_buffer(data_buffer *buffer);
void reset_buffer(data_buffer *buffer);
int16_t buffer_size(data_buffer *buffer);

void twi_init(void);
void imu_write_reg(void);
void imu_read(data_buffer *buffer, int sensor_num);
void mic_read(vol_data *data);
void set_sensor_idle(sensor *sen);
void send_back_data(data_buffer *buffer, int sensor_num);

void sendCommand(uint8_t in_appID, uint8_t in_noteID);
void cancelCommand(uint8_t in_appID, uint8_t in_noteID);
void bufferCommand(uint8_t in_appID, uint8_t in_noteID, uint8_t in_taskID);

#endif /* TWI_CDC_TIMER_H_ */