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

static sensor imu;
static sensor mic;
static imu_data_buffer imu_buffer;
static mic_data_buffer mic_buffer;

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

//read data from imu
void imu_read(imu_data *data)
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
		data->gyro_x  = (((int16_t)i2c_buffer[0]) << 8)  | i2c_buffer[1]; //0 and 1
		data->gyro_y  = (((int16_t)i2c_buffer[2]) << 8)  | i2c_buffer[3]; //2 and 3
		data->gyro_z  = (((int16_t)i2c_buffer[4]) << 8)  | i2c_buffer[5]; //4 and 5
		data->temp    = (((int16_t)i2c_buffer[6]) << 8)  | i2c_buffer[7]; //6 and 7
		data->temp    = (data->temp+12142)/340;
		data->accel_x = (((int16_t)i2c_buffer[8]) << 8)  | i2c_buffer[9]; //8 and 9
		data->accel_y = (((int16_t)i2c_buffer[10]) << 8) | i2c_buffer[11]; //10 and 11
		data->accel_z = (((int16_t)i2c_buffer[12]) << 8) | i2c_buffer[13]; //12 and 13
		
		char out_str[128];
		// int str_size = snprintf(out_str, 128, "%d,%d,%d X \n", data->accel_x, data->accel_y, data->accel_z);
		int str_size = snprintf(out_str, 128, "%d XXXXX \n", data->temp);
		udi_cdc_write_buf(out_str, str_size);
		
		if (data->accel_x > 6000 || data->accel_x < -6000)
		{
			LED_On(LED1);
		}
		else
		{
			LED_Off(LED1);
		}
	}
}

void mic_read(mic_data *data)
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
		ADCA.CH0.CTRL |= ADC_CH_START_bm; // start conversion on channel 0
		while(!ADCA.CH0.INTFLAGS);
		int result = ADCA.CH0RES;
		int str_size = snprintf(out_str, 128, "%d   ", result);
		udi_cdc_write_buf(out_str, str_size * sizeof(char));
	}
}

void set_sensor_idle(void)
{
	imu.state = IDLE;
	print_line(0, "idle");
}

void send_back_data(void)
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
	int str_size = snprintf(out_str, 128, "%c,%c,%u X \n", imu.appID, imu.notificationID, count);
	
	udi_cdc_write_buf(out_str, str_size * sizeof(char));
	int j;
	for(j = 0; j < count; j++)
	{
		imu_data *data = imu_buffer.data + imu_buffer.read_pt;
		str_size = snprintf(out_str, 128, "%d,%d,%d X \n", data->accel_x, data->accel_y, data->accel_z);
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
		imu.rateCntr++;
		
		if (imu.rateCntr + 10 >= imu.sampleRate) {
			LED_On(LED0);
		}
		else
		{
			LED_Off(LED0);
		}
		
		if (imu.rateCntr == imu.sampleRate)
		{
			imu.rateCntr = 0;

			imu_read(imu_buffer.data + imu_buffer.write_pt);
			imu_buffer.write_pt = (imu_buffer.write_pt + 1) & BUFFER_CAPACITY_MASK;
			
			if (imu.state == BUFFER_ONCE && buffer_size(&imu_buffer) >= imu.numSamples) {
				set_sensor_idle();
			}
		}
	}
}

void sendCommand(uint8_t in_appID, uint8_t in_noteID)
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

void cancelCommand(uint8_t in_appID, uint8_t in_noteID)
{
	print_line(2, "cancel");
	if(imu.appID == in_appID && imu.notificationID == in_noteID)
	{
		set_sensor_idle();
		init_sensor_struct(&imu);	
	}
}

void bufferCommand(uint8_t in_appID, uint8_t in_noteID, uint8_t in_taskID)
{
	print_line(2, "buffer");
	
	char out_str[128];
	int str_size = snprintf(out_str, 128, "USING MICROPHONE!!!");
	udi_cdc_write_buf(out_str, str_size * sizeof(char));
	
	// Buffer the rest of the input
	int input_buffer_size = 12;
	char input_buffer[input_buffer_size];          // Buffer Index  |  Content
	for (int i = 0; i < input_buffer_size; i++)    // 0             |  sensor
	{                                              // 1             |  if persistent
		input_buffer[i] = udi_cdc_getc();          // 2-7           |  sample rate
	}                                              // 8-11          |  number of samples
	
	sensor sen;
	if(input_buffer[0] == '0')
	{
		sen = imu;
	}
	else if(input_buffer[0] == '1')
	{
		sen = mic;
	}
		
		
		
	if (sen.state != IDLE)
	{
		print_line(2, "not idle! ignore");
		return;
	}
	
	if(input_buffer[0] == '0')
	{
		reset_buffer(&imu_buffer);
	}
	else if(input_buffer[0] == '1')
	{
		reset_buffer(&mic_buffer);
	}

	sen.appID = in_appID;
	sen.notificationID = in_noteID;
	sen.taskID = in_taskID;
	sen.rateCntr = 0;

	sen.persistent = input_buffer[1];

	sen.sampleRate = (input_buffer[2] - 48) * 100000 + (input_buffer[3] - 48)
	* 10000 + (input_buffer[4] - 48) * 1000 + (input_buffer[5] - 48)
	* 100 + (input_buffer[6] - 48) * 10 + (input_buffer[7] - 48);

	sen.numSamples = (input_buffer[8] - 48) * 1000 + (input_buffer[9] - 48)
	* 100 + (input_buffer[10] - 48) * 10 + (input_buffer[11] - 48);

	if (sen.persistent == '0')
	{
		sen.state = BUFFER_ONCE;
		print_line(0, "buffer once");
	}
	else
	{
		sen.state = BUFFER_PERSIST;
		print_line(0, "buffer persist");
	}
}

int main (void)
{
	PORTA.DIR = 0;	 // configure PORTA as input
	ADCA.CTRLA |= 0x1;	 // enable adc
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;	 // 12 bit conversion
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | 0x02;	 // internal 1V bandgap reference
	ADCA.PRESCALER = ADC_PRESCALER_DIV8_gc;	 // peripheral clk/8 (2MHz/16=250kHz)
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;	 // single ended
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;	 // PORTA:2
	
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
			else if(in_taskID == '2') // Buffer task
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
