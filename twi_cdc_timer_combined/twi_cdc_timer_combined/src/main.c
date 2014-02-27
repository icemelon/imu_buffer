/**************************************************************************
* FILE NAME :main.c
* DESCRIPTION : Runs a data buffering interface for IMU and other sensors to interface 
				with a phone through the USB CDC service
* AUTHORS : Eric Yuan, Ryan Tsoi
* DATE : August 14, 2013
****************************************************************************/
#include "twi_cdc_timer.h"
#include "led.h"
#include "GPSWrapper.h"

#define INPUT_PIN              ADCCH_POS_PIN0

static const int num_sensors = 4;
static sensor gyro_sensor;
static sensor temp_sensor;
static sensor accel_sensor;
static sensor vol_sensor;
static sensor* sensors[4];
static data_buffer gyro_buffer;
static data_buffer temp_buffer;
static data_buffer accel_buffer;
static data_buffer vol_buffer;
static data_buffer* data_buffers[4];

//static gps GPS(PC2, PC3);

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
void imu_write_reg(void)
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

//read data from imu, sensor 0 = gyro, sensor 1 = temp, sensor 2 = accel
void imu_read(data_buffer *buffer, int sensor_num)
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
		if (sensor_num == 0)
		{
			gyro_data *gyro = ((gyro_data *) buffer->data) + buffer->write_pt;
			gyro->gyro_x  = (((int16_t)i2c_buffer[0]) << 8)  | i2c_buffer[1]; //0 and 1
			gyro->gyro_y  = (((int16_t)i2c_buffer[2]) << 8)  | i2c_buffer[3]; //2 and 3
			gyro->gyro_z  = (((int16_t)i2c_buffer[4]) << 8)  | i2c_buffer[5]; //4 and 5
			
			char out_str[128];
			int str_size = snprintf(out_str, 128, "GYRO: %d %d %d XXXXX \n", gyro->gyro_x, gyro->gyro_y, gyro->gyro_z);
			udi_cdc_write_buf(out_str, str_size);
		}
		else if (sensor_num == 1)
		{
			temp_data *temp = ((temp_data *) buffer->data) + buffer->write_pt;
			temp->temp    = (((int16_t)i2c_buffer[6]) << 8)  | i2c_buffer[7]; //6 and 7
			temp->temp    = (temp->temp+12142)/340;
			
			char out_str[128];
			int str_size = snprintf(out_str, 128, "TEMP: %d XXXXX \n", temp->temp);
			udi_cdc_write_buf(out_str, str_size);
		}
		else if (sensor_num == 2)
		{
			accel_data *accel = ((accel_data *) buffer->data) + buffer->write_pt;
			accel->accel_x = (((int16_t)i2c_buffer[8]) << 8)  | i2c_buffer[9]; //8 and 9
			accel->accel_y = (((int16_t)i2c_buffer[10]) << 8) | i2c_buffer[11]; //10 and 11
			accel->accel_z = (((int16_t)i2c_buffer[12]) << 8) | i2c_buffer[13]; //12 and 13
			
			char out_str[128];
			int str_size = snprintf(out_str, 128, "ACCEL: %d %d %d XXXXX \n", accel->accel_x, accel->accel_y, accel->accel_z);
			udi_cdc_write_buf(out_str, str_size);
		}
	}
}

/**
 * \brief Callback function for ADC interrupts
 *
 * \param adc Pointer to ADC module.
 * \param ch_mask ADC channel mask.
 * \param result Conversion result from ADC channel.
 */
//static
/*void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	int32_t audio;

	//if(result > 520)
		audio = result;

	char out_str[128];
	int str_size = snprintf(out_str, 128, "audio: %d XXXXX", audio);
	udi_cdc_write_buf(out_str, str_size);

	// Start next conversion.
	adc_start_conversion(adc, ch_mask);
}*/

void mic_read(vol_data *data)
{
	char out_str[128];
	int str_size = snprintf(out_str, 128, "USING MICROPHONE!!!");
	udi_cdc_write_buf("USING MICROPHONE!", str_size * sizeof(char));
	//data->sample = (int16_t)
	//
	//ADCB_CH0_CTRL |= 0x80;
	//if (result_flag) {
		
	//}
	while (1) {
		
	//	ADCA.CH0.CTRL |= ADC_CH_START_bm; // start conversion on channel 0
	//	while(!ADCA.CH0.INTFLAGS);
	//	int result = ADCA.CH0RES;
	//	int str_size = snprintf(out_str, 128, "%d   ", result);
	//	udi_cdc_write_buf(out_str, str_size * sizeof(char));
	}
}

void set_sensor_idle(sensor *sen)
{
	sen->state = IDLE;
	print_line(0, "idle");
}

void send_back_data(data_buffer *buffer, int sensor_num)
{
	int16_t curr_write_pt = buffer->write_pt;
	int16_t count = curr_write_pt - buffer->read_pt;
	if (count < 0)
		count += buffer->capacity;

	if (count < sensors[sensor_num]->numSamples)
		count = 0;

	char debug[30];
	snprintf(debug, 20, "send data %d.", count);
	print_line(2, debug);

	char out_str[128];
	int str_size = snprintf(out_str, 128, "%c,%c,%u X \n", sensors[sensor_num]->appID, sensors[sensor_num]->notificationID, count);
	
	udi_cdc_write_buf(out_str, str_size * sizeof(char));
	int j;
	for(j = 0; j < count; j++)
	{
		if (sensor_num == 0)
		{
			gyro_data *data = buffer->data + buffer->read_pt;
			str_size = snprintf(out_str, 128, "%d,%d,%d X \n", data->gyro_x, data->gyro_y, data->gyro_z);
			buffer->read_pt = (buffer->read_pt + sizeof(gyro_data)) & BUFFER_CAPACITY_MASK;
		}
		else if (sensor_num == 1)
		{
			temp_data *data = buffer->data + buffer->read_pt;
			str_size = snprintf(out_str, 128, "%d X \n", data->temp);
			buffer->read_pt = (buffer->read_pt + sizeof(gyro_data)) & BUFFER_CAPACITY_MASK;
		}
		else if (sensor_num == 2)
		{
			accel_data *data = buffer->data + buffer->read_pt;
			str_size = snprintf(out_str, 128, "%d,%d,%d X \n", data->accel_x, data->accel_y, data->accel_z);
			buffer->read_pt = (buffer->read_pt + sizeof(gyro_data)) & BUFFER_CAPACITY_MASK;
		}
		else if (sensor_num == 3)
		{
			vol_data *data = buffer->data + buffer->read_pt;
			str_size = snprintf(out_str, 128, "%d X \n", data->vol_sample);
			buffer->read_pt = (buffer->read_pt + 1) & BUFFER_CAPACITY_MASK;
		}
		udi_cdc_write_buf(out_str, str_size * sizeof(char));
		//buffer->read_pt = (buffer->read_pt + 1) & BUFFER_CAPACITY_MASK;
	}

	snprintf(debug, 30, "send data %d end.", count);
	print_line(2, debug);
}

//interrupt handler function
static void my_callback(void) //the timing is a little off here, need to fix it
{
	//imu.rateCntr++; //this is wrong but it kinda works anyway, the right logic doesn't produce the right results so something is wrong
	for (int i = 0; i < num_sensors; i++)
	{
		if (sensors[i]->state != IDLE)
		{
			sensors[i]->rateCntr++;
			
			if (sensors[i]->rateCntr == sensors[i]->sampleRate)
			{
				sensors[i]->rateCntr = 0;
				
				imu_read(data_buffers[i], i);
				data_buffers[i]->write_pt = (data_buffers[i]->write_pt + 1) & BUFFER_CAPACITY_MASK;
				
				if (sensors[i]->state == BUFFER_ONCE && buffer_size(data_buffers[i]) >= sensors[i]->numSamples)
				{
					set_sensor_idle(sensors[i]);
				}
			}
		}
	}
}

void sendCommand(uint8_t in_appID, uint8_t in_noteID)
{
	for (int i = 0; i < num_sensors; i++)
	{
		print_line(2, "send data");
		//check if the input matches an imu task
		if (sensors[i]->appID == in_appID && sensors[i]->notificationID == in_noteID)
		{
			send_back_data(data_buffers[i], i);
		}
	}
}

void cancelCommand(uint8_t in_appID, uint8_t in_noteID)
{
	print_line(2, "cancel");
	for (int i = 0; i < num_sensors; i++)
	{
		if (sensors[i]->appID == in_appID && sensors[i]->notificationID == in_noteID)
		{
			set_sensor_idle(sensors[i]);
			init_sensor_struct(&sensors[i]);	
		}
	}
}

void bufferCommand(uint8_t in_appID, uint8_t in_noteID, uint8_t in_taskID)
{
	print_line(2, "buffer");
	
	// Buffer the rest of the input
	int input_buffer_size = 22;
	char input_buffer[input_buffer_size];          // Buffer Index  |  Content
	for (int i = 0; i < input_buffer_size; i++)    // 0             |  sensor
	{                                              // 1             |  if persistent
		input_buffer[i] = udi_cdc_getc();          // 2-7           |  sample rate
	}                                              // 8-11          |  number of samples
	                                               // 12-16         |  min range
												   // 17-21         |  max range

	int sensor_num = input_buffer[0] - 48;
	
	if (sensors[sensor_num]->state != IDLE)
	{
		print_line(2, "not idle! ignore");
		return;
	}
	
	int16_t sensor_min = (input_buffer[13] - 48) * 1000 + (input_buffer[14] - 48) * 100
	+ (input_buffer[15] - 48) * 10 + (input_buffer[16] - 48);
	if (input_buffer[12] == 45)
		sensor_min *= -1;

	int16_t sensor_max = (input_buffer[18] - 48) * 1000 + (input_buffer[19] - 48) * 100
	+ (input_buffer[20] - 48) * 10 + (input_buffer[21] - 48);
	if (input_buffer[17] == 45)
		sensor_max *= -1;		
	
	reset_buffer(&data_buffers[sensor_num]);
	data_buffers[sensor_num]->min = sensor_min;
	data_buffers[sensor_num]->max = sensor_max;

	sensors[sensor_num]->appID = in_appID;
	sensors[sensor_num]->notificationID = in_noteID;
	sensors[sensor_num]->taskID = in_taskID;
	sensors[sensor_num]->rateCntr = 0;

	sensors[sensor_num]->persistent = input_buffer[1];

	sensors[sensor_num]->sampleRate = (input_buffer[2] - 48) * 100000 + (input_buffer[3] - 48) * 10000
								   + (input_buffer[4] - 48) * 1000 + (input_buffer[5] - 48) * 100
								   + (input_buffer[6] - 48) * 10   + (input_buffer[7] - 48);

	sensors[sensor_num]->numSamples = (input_buffer[8] - 48) * 1000 + (input_buffer[9] - 48)
								   * 100 + (input_buffer[10] - 48) * 10 + (input_buffer[11] - 48);

	if (sensors[sensor_num]->persistent == '0')
	{
		sensors[sensor_num]->state = BUFFER_ONCE;
		print_line(0, "buffer once");
	}
	else
	{
		sensors[sensor_num]->state = BUFFER_PERSIST;
		print_line(0, "buffer persist");
	}
}

int main (void)
{
	//initialize all the things
	init_sensor_struct(&gyro_sensor);
	init_sensor_struct(&temp_sensor);
	init_sensor_struct(&accel_sensor);
	init_sensor_struct(&vol_sensor);
	sensors[0] = &gyro_sensor;
	sensors[1] = &temp_sensor;
	sensors[2] = &accel_sensor;
	sensors[3] = &vol_sensor;
	init_gyro_buffer(&gyro_buffer);
	init_temp_buffer(&temp_buffer);
	init_accel_buffer(&accel_buffer);
	init_vol_buffer(&vol_buffer);
	data_buffers[0] = &gyro_buffer;
	data_buffers[1] = &temp_buffer;
	data_buffers[2] = &accel_buffer;
	data_buffers[3] = &vol_buffer;

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
	imu_write_reg();
	
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

	/*

	//ADC initialization
	struct adc_config         adc_conf;
	struct adc_channel_config adcch_conf;
	
	// Initialize configuration structures.
	adc_read_configuration(&ADCB, &adc_conf);
	adcch_read_configuration(&ADCB, ADC_CH0, &adcch_conf);
	
	/* Configure the ADC module:
	 * - unsigned, 12-bit results
	 * - VCC voltage reference
	 * - 200 kHz maximum clock rate
	 * - manual conversion triggering
	 * - temperature sensor enabled
	 * - callback function
	 */
	
	/*
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12,
			ADC_REF_VCC);
	adc_set_clock_rate(&adc_conf, 200UL);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 0, 0);
	// adc_enable_internal_input(&adc_conf, ADC_INT_TEMPSENSE);
	adc_write_configuration(&ADCB, &adc_conf);
	adc_set_callback(&ADCB, &adc_handler);

	/* Configure ADC channel 0:
	 * - single-ended measurement from temperature sensor
	 * - interrupt flag set on completed conversion
	 * - interrupts disabled
	 */
	
	/*
	adcch_set_input(&adcch_conf, INPUT_PIN, ADCCH_NEG_NONE, 1);
	adcch_set_interrupt_mode(&adcch_conf, ADCCH_MODE_COMPLETE);
	adcch_enable_interrupt(&adcch_conf);

	adcch_write_configuration(&ADCB, ADC_CH0, &adcch_conf);

	// Enable the ADC and start the first conversion.
	adc_enable(&ADCB);
	adc_start_conversion(&ADCB, ADC_CH0);
	
	*/

	while (1)
	{
		if (udi_cdc_is_rx_ready())
		{
			if (udi_cdc_getc()!= 's')
				continue; // Wrong key
			if (udi_cdc_getc()!= 't')
				continue; // Wrong key
			if (udi_cdc_getc()!= 'a')
				continue; // Wrong key
			if (udi_cdc_getc()!= 'r')
				continue; // Wrong key
			if (udi_cdc_getc()!= 't')
				continue; // Wrong key
			
			udi_cdc_putc('\n');
			udi_cdc_write_buf("Starting!!!\n", sizeof("Starting!!!\n"));
			break;
		}
	}

	while (1)
	{
		if (udi_cdc_is_rx_ready())
		{	
			print_line(2, "start");
			char debug[20];
			
			uint8_t in_appID = udi_cdc_getc();
			snprintf(debug, 20, "appID %d", in_appID);
			print_line(2, debug);
			
			uint8_t in_noteID = udi_cdc_getc();
			snprintf(debug, 20, "noteID %d", in_noteID);
			print_line(2, debug);
			
			uint8_t in_taskID = udi_cdc_getc();
			snprintf(debug, 20, "taskID %d", in_taskID);
			print_line(2, debug);
			
			// Task related logic
			if (in_taskID == '0') // Send data back to phone
			{
				sendCommand(in_appID, in_noteID);

			}
			else if(in_taskID == '1') // Cancel task
			{
				cancelCommand(in_appID, in_noteID);
			}
			else if(in_taskID == '2') // Buffer task   Ex. 002000010000010         -0000+0000-1234+1234-9999+9999
			{
				bufferCommand(in_appID, in_noteID, in_taskID);
			}
			else
			{
				print_line(0, "error command");
			}
		}
	}
}