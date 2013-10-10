/**************************************************************************
* FILE NAME :main.c
* DESCRIPTION : Runs a data buffering interface for IMU and other sensors to interface 
				with a phone through the USB CDC service
* AUTHORS : Eric Yuan
* DATE : August 14, 2013
****************************************************************************/
#include "twi_cdc_timer.h"

static imu_struct imu;
static imu_data_buffer imu_buffer;

// idx: line number, starting from 0
inline void print_line(int16_t idx, char *str)
{
#ifdef DEBUG
	int16_t y = idx * 10;
	gfx_mono_draw_filled_rect(0, y, 128, 10, GFX_PIXEL_CLR);
	gfx_mono_draw_string(str, 0, y, &sysfont);
#endif
}

//initalize twi functionality
inline void twi_init(void)
{
	twi_master_options_t opt = {
		.speed = TWI_SPEED,
		.chip  = TWI_MASTER_ADDR
	};
	twi_master_setup(&TWI_MASTER, &opt);
}

//takes the imu out of sleep so data can be collected
void mpu6050_write_reg(void)
{
	uint8_t test_pattern[] = {0};
	twi_package_t packet_write = {
		.addr         = MPU6050_PWR_MGMT_1,      // TWI slave memory address data
		.addr_length  =  sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = MPU6050_I2C_ADDRESS,      // TWI slave bus address
		.buffer       = (void *)test_pattern, // transfer data source buffer
		.length       = sizeof(test_pattern)  // transfer data size (bytes)
	};
	while (twi_master_write(&TWI_MASTER, &packet_write) != TWI_SUCCESS);
}

//read data from imu
void mpu6050_read(imu_data *data)
{
	uint8_t i2c_buffer[14];
	//read the data
	twi_package_t packet_read = {
		.addr         = MPU6050_ACCEL_XOUT_H,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = MPU6050_I2C_ADDRESS,      // TWI slave bus address
		.buffer       = i2c_buffer,        // transfer data destination buffer
		.length       = sizeof(i2c_buffer)                    // transfer data size (bytes)
	};
	// Perform a multi-byte read access then check the result.
	if(twi_master_read(&TWI_MASTER, &packet_read) == TWI_SUCCESS){
		data->accel_x = (((int16_t)i2c_buffer[0]) << 8) | i2c_buffer[1];
		data->accel_y = (((int16_t)i2c_buffer[2]) << 8) | i2c_buffer[3];
		data->accel_z = (((int16_t)i2c_buffer[4]) << 8) | i2c_buffer[5];
		//data->temp = i2c_buffer[6] << 8;
		//data->temp |= i2c_buffer[7];
		//data->temp = (data->temp+12142)/340;
		//data->gyro_x = i2c_buffer[4] << 8;
		//data->gyro_x |= i2c_buffer[5];
		//data->gyro_y = i2c_buffer[2] << 8;
		//data->gyro_y |= i2c_buffer[3];
		//data->gyro_z = i2c_buffer[0] << 8;
		//data->gyro_z |= i2c_buffer[1];
		//debug output on lcd
		
		char out_str[50];
		
		snprintf(out_str, 50, "a:%d, %d, %d\r",  data->accel_x,  data->accel_y,  data->accel_z);
		/*snprintf(out_str, 50, "%d %d %d %d %d %d", i2c_buffer[0], i2c_buffer[1], i2c_buffer[2], i2c_buffer[3], i2c_buffer[4],
			i2c_buffer[5], i2c_buffer[6]);*/
		print_line(1, out_str);
	}
}

void set_sensor_idle()
{
	imu.state = IDLE;
	print_line(0, "idle");
}
	
//helper method to cancel a task running task
void cancel()
{
	set_sensor_idle();
	init_sensor_struct(&imu);
}

void send_back_data()
{
	int16_t curr_write_pt = imu_buffer.write_pt;
	int16_t count = curr_write_pt - imu_buffer.read_pt;
	if (count < 0)
		count += imu_buffer.capacity;

	if (count < imu.numSamples)
		count = 0;

	char debug[30];
	snprintf(debug, 20, "send data %d.", count);
	print_line(2, debug);

	char out_str[128];
	int str_size = snprintf(out_str, 128, "%c,%c,%u\n", imu.appID, imu.notificationID, count);
	
	udi_cdc_write_buf(out_str, str_size * sizeof(char));
	int j;
	for(j = 0; j < count; j++)
	{
		imu_data *data = imu_buffer.data + imu_buffer.read_pt;
		str_size = snprintf(out_str, 128, "%d,%d,%d\n", data->accel_x, data->accel_y, data->accel_z);
		udi_cdc_write_buf(out_str, str_size * sizeof(char));
		imu_buffer.read_pt = (imu_buffer.read_pt + 1) & BUFFER_CAPACITY_MASK;
	}

	snprintf(debug, 30, "send data %d end.", count);
	print_line(2, debug);
}

//interrupt handler function
static void my_callback(void) //the timing is a little off here, need to fix it
{
	//imu.rateCntr++; //this is wrong but it kinda works anyway, the right logic doesn't produce the right results so something is wrong
	if (imu.state != IDLE)
	{
		imu.rateCntr ++;
		if (imu.rateCntr == imu.sampleRate)
		{
			imu.rateCntr = 0;

			mpu6050_read(imu_buffer.data + imu_buffer.write_pt);
			imu_buffer.write_pt = (imu_buffer.write_pt + 1) & BUFFER_CAPACITY_MASK;
			
			if (imu.state == BUFFER_ONCE && buffer_size(&imu_buffer) >= imu.numSamples) {
				imu.state = IDLE;
				set_sensor_idle();
			}
		}
	}
}

int main (void)
{
	//initialize all the things
	init_sensor_struct(&imu);
	init_buffer(&imu_buffer);

	irq_initialize_vectors();
	sleepmgr_init();
	board_init();
	pmic_init();
	sysclk_init();
	gfx_mono_init();
	twi_init();
#ifdef DEBUG
	// turn on the LCD
	ioport_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
#endif
	//start usb cdc service
	udc_start();
	udc_attach();
	mpu6050_write_reg();
	
	//initialize timer
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, my_callback);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 48000); //set interrupt to fire every 1 ms 48000
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	cpu_irq_enable();
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);
	
	print_line(0, "Hello"); //make sure program is actually running
	print_line(1, "World");
	//uint32_t hz = sysclk_get_main_hz();
	//char debug[30];
	//snprintf(debug, 30, "main: %d", hz);
	//print_line(2, debug);

	while (1)
	{
		if (udi_cdc_is_rx_ready())
		{
			//uint8_t in_appID = 1;
			//uint8_t in_noteID = 1;
			print_line(2, "start");
			char debug[20];
			
			uint8_t in_appID = udi_cdc_getc();
			snprintf(debug, 20, "appID %d", in_appID);
			print_line(2, debug);
			
			uint8_t in_noteID = udi_cdc_getc();
			snprintf(debug, 20, "notID %d", in_noteID);
			print_line(2, debug);
			
			uint8_t in_taskID = udi_cdc_getc();
			snprintf(debug, 20, "taskID %d", in_taskID);
			print_line(2, debug);
			
			//task related logic
			if (in_taskID == '0') //send data back to phone
			{
				print_line(2, "send data");
				//check if the input matches an imu task
				if (imu.appID == in_appID && imu.notificationID == in_noteID)
				{
					send_back_data();
				}
				// else if(mic->appID == in_appID && mic->notificationID == in_noteID)
				// {
				// 	udi_cdc_putc(in_appID);
				// 	udi_cdc_putc(in_noteID);
				// 	udi_cdc_putc(0);
				// 	udi_cdc_putc(sizeof(mic->buffer->data));
				// 	udi_cdc_write_buf(mic->buffer->data, sizeof(mic->buffer->data));
				// }
			}
			else if(in_taskID == '1') //cancel a task
			{
				print_line(2, "cancel");
				if(imu.appID == in_appID && imu.notificationID == in_noteID)
					cancel();
			}			
			else if(in_taskID == '2') //buffer task
			{
				print_line(2, "buffer");
				char sensor = udi_cdc_getc();
				if(sensor == '0')
				{
					if (imu.state != IDLE)
					{
						print_line(2, "not idle! ignore");
						for (int i = 0; i < 11; i++)
							udi_cdc_getc();
						continue;
					}

					reset_buffer(&imu_buffer);

					imu.appID = in_appID;
					imu.notificationID = in_noteID;
					imu.taskID = in_taskID;
					imu.rateCntr = 0;

					imu.persistent = udi_cdc_getc();

					char sampleRate1 = udi_cdc_getc();
					char sampleRate2 = udi_cdc_getc();
					char sampleRate3 = udi_cdc_getc();
					char sampleRate4 = udi_cdc_getc();
					char sampleRate5 = udi_cdc_getc();
					char sampleRate6 = udi_cdc_getc();
					imu.sampleRate = (sampleRate1 - 48) * 100000 + (sampleRate2 - 48) * 10000 + (sampleRate3 - 48) * 1000 + (sampleRate4 - 48) * 100 + (sampleRate5 - 48) * 10 + (sampleRate6 - 48);

					char numSamples1 = udi_cdc_getc();
					char numSamples2 = udi_cdc_getc();
					char numSamples3 = udi_cdc_getc();
					char numSamples4 = udi_cdc_getc();
					imu.numSamples = (numSamples1 - 48) * 1000 + (numSamples2 - 48) * 100 + (numSamples3 - 48) * 10 + (numSamples4 - 48); 

					if (imu.persistent == '0')
					{
						imu.state = BUFFER_ONCE;
						print_line(0, "buffer once");
					}
					else
					{
						imu.state = BUFFER_PERSIST;
						print_line(0, "buffer persist");
					}
				}
			}
			else
			{
				print_line(0, "error command");
			}
		}
	}
}
