/*
	Copyright 2021 Marcos Chaparro	mchaparro@powerdesigns.ca
	Copyright 2021 Maximiliano Cordoba	mcordoba@powerdesigns.ca

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "conf_general.h"

#include "hw.h"
#include "no2_display_serial.h"
#include "app.h"
#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "commands.h"
#include "mc_interface.h"
#include "utils.h"
#include <math.h>
#include <string.h>
#include "comm_can.h"
#include "datatypes.h"

// Threads
static THD_WORKING_AREA(display_process_thread_wa, 1024);
static THD_FUNCTION(display_process_thread, arg);

volatile No2_t no2_data[2];
int new = 1;
int old = 0;

static void serial_send_packet(unsigned char *data, unsigned int len);
static void serial_display_byte_process(unsigned char byte);
static void serial_display_check_rx(void);

int No2_Service(uint8_t *No2_Message);

int calculate_checksum(unsigned char *frame_buf, uint8_t length);

uint8_t lowByte(uint16_t word);
uint8_t highByte(uint16_t word);

#define NO2_SERIAL_BUFFER_SIZE 60
#define NO2_MSG_SIZE 20

typedef struct
{
	unsigned int rd_ptr;
	unsigned int wr_ptr;
	unsigned char data[NO2_SERIAL_BUFFER_SIZE];
} no2_serial_buffer_t;

static volatile bool display_thread_is_running = false;
static volatile bool display_uart_is_running = false;

/* UART driver configuration structure */
static SerialConfig uart_cfg = {
	9600, // baud rate
	0,
	USART_CR2_LINEN,
	0};

static no2_serial_buffer_t serial_buffer;

static void (*set_assist_level)(float);
static void (*set_head_light)(bool);

void no2_display_serial_start(void (*assist_level_cb)(float), void (*head_light_cb)(bool))
{
	set_assist_level = assist_level_cb;
	set_head_light = head_light_cb;

	commands_printf("Starting No 2 Display!");

	memset(&no2_data, 0, sizeof(no2_data));

	if (!display_thread_is_running)
	{
		chThdCreateStatic(display_process_thread_wa, sizeof(display_process_thread_wa),
						  NORMALPRIO, display_process_thread, NULL);
		display_thread_is_running = true;
	}
	serial_buffer.rd_ptr = 0;
	serial_buffer.wr_ptr = 0;

	sdStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);

	display_uart_is_running = true;
}

bool no2_display_serial_is_active()
{
	return ((chVTGetSystemTimeX() - no2_data[old].last_msg) / (float)CH_CFG_ST_FREQUENCY) < 1.0; // no msg since 1 sec
}

void no2_display_serial_set_wheel_rpm(float rpm)
{
	no2_data[new].Tx.Wheeltime_ms = 60 / rpm * 1000;
}

int No2_Service(uint8_t *No2_Message)
{
	static uint8_t TxBuffer[14] = {0x2, 0x0E, 0x1, 0x0, 0x80, 0x0, 0x0, 0x2C, 0x0, 0xF9, 0x0, 0x0, 0xFF, 0xA};

	// check alignment
	int idx = 0;
	while (!(No2_Message[idx] == 0x01 && No2_Message[idx + 1] == 0x14 && No2_Message[idx + 2] == 0x01))
	{
		idx++;
		if (idx > NO2_MSG_SIZE - 3)
			break; // Magic number not found
	}
	if (idx > 0)
		return idx; // discard bytes

	if (No2_Message[19] == calculate_checksum(No2_Message, NO2_MSG_SIZE))
	{
		// swap data
		old = !old;
		new = !new;

		// to do bit indexing
		no2_data[new].Rx.AssistLevel = No2_Message[4];
		no2_data[new].Rx.NumberOfPasMagnets = No2_Message[18] & 0x0F;
		no2_data[new].Rx.CUR_Limit_A = No2_Message[13];
		no2_data[new].Rx.Voltage_min_x10 = (No2_Message[14] << 8) + No2_Message[15];
		no2_data[new].Rx.WheelSizeInch_x10 = (No2_Message[7] << 8) + No2_Message[8];
		no2_data[new].Rx.Throttle_mode = No2_Message[3];
		no2_data[new].Rx.Start_delay_PAS = No2_Message[9];
		no2_data[new].Rx.BoostPower = No2_Message[10];
		no2_data[new].Rx.ZeroStart = (No2_Message[5] >> 6) & 0x01;
		no2_data[new].Rx.Headlight = (No2_Message[5] >> 5) & 0x01;
		no2_data[new].Rx.PushAssist = (No2_Message[5] >> 1) & 0x01;
		no2_data[new].Rx.CruiseControl = (No2_Message[18] >> 6) & 0x01;
		no2_data[new].Rx.SPEEDMAX_Limit = No2_Message[12];
		no2_data[new].Rx.GearRatio = No2_Message[6];

		no2_data[new].last_msg = chVTGetSystemTimeX();

		TxBuffer[3] = no2_data[old].Tx.Error;
		TxBuffer[4] = no2_data[old].Tx.BrakeActive << 5; // 0b00100000;
		TxBuffer[6] = highByte(no2_data[old].Tx.Current_x10);
		TxBuffer[7] = lowByte(no2_data[old].Tx.Current_x10);
		TxBuffer[8] = highByte(no2_data[old].Tx.Wheeltime_ms);
		TxBuffer[9] = lowByte(no2_data[old].Tx.Wheeltime_ms);

		TxBuffer[13] = calculate_checksum(TxBuffer, 14);
		serial_send_packet(TxBuffer, sizeof(TxBuffer));
	}
	else
	{
		commands_printf("process no2 CHECKSUM error!");
	}

	return NO2_MSG_SIZE;
}

uint8_t lowByte(uint16_t word)
{
	return word & 0xFF;
}

uint8_t highByte(uint16_t word)
{
	return word >> 8;
}

int calculate_checksum(unsigned char *frame_buf, uint8_t length)
{
	unsigned char xor = 0;
	unsigned char *p;
	unsigned char tmp;
	for (p = frame_buf; p < frame_buf + (length - 1); p++)
	{
		tmp = *p;
		// printf("%d, %x\r\n ", p,tmp);
		xor = xor ^ tmp;
	}
	return (xor);
}

static void serial_send_packet(unsigned char *data, unsigned int len)
{
	if (display_uart_is_running)
	{
		sdWrite(&HW_UART_DEV, data, len);
	}
}

static void serial_display_byte_process(unsigned char byte)
{
	// append new byte to the buffer.
	serial_buffer.data[serial_buffer.wr_ptr] = byte;
	serial_buffer.wr_ptr++;

	// process with at least NO2_MSG_SIZE bytes available to read
	while ((serial_buffer.wr_ptr - serial_buffer.rd_ptr) >= NO2_MSG_SIZE)
	{

		// commands_printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		// 				serial_buffer.data[serial_buffer.rd_ptr],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 1],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 2],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 3],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 4],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 5],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 6],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 7],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 8],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 9],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 10],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 11],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 12],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 13],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 14],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 15],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 16],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 17],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 18],
		// 				serial_buffer.data[serial_buffer.rd_ptr + 19]);

		serial_buffer.rd_ptr += No2_Service(&serial_buffer.data[serial_buffer.rd_ptr]);
	}

	if (serial_buffer.rd_ptr > 0)
	{
		memmove(serial_buffer.data, serial_buffer.data + serial_buffer.rd_ptr, NO2_SERIAL_BUFFER_SIZE - serial_buffer.rd_ptr);
		serial_buffer.wr_ptr -= serial_buffer.rd_ptr;
		serial_buffer.rd_ptr = 0;
	}
	if (serial_buffer.wr_ptr == (NO2_SERIAL_BUFFER_SIZE - 1))
	{
		// shift buffer to the left discarding the oldest byte
		memmove(serial_buffer.data, serial_buffer.data + 1, NO2_SERIAL_BUFFER_SIZE - 1);
		serial_buffer.wr_ptr -= 1;
	}
}

static void serial_display_check_rx(void)
{
	bool rx = true;
	while (rx)
	{
		rx = false;

		if (display_uart_is_running)
		{
			msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
			if (res != MSG_TIMEOUT)
			{
				serial_display_byte_process(res);
				rx = true;
			}
		}
	}
}

static propagate_changes()
{
	if (no2_data[new].Rx.AssistLevel != no2_data[old].Rx.AssistLevel)
		set_assist_level((no2_data[new].Rx.AssistLevel - 1) / 14.0);
	// commands_printf("process no2 Assist lvl: %d ", no2_data[new].Rx.AssistLevel);

	if (no2_data[new].Rx.Headlight != no2_data[old].Rx.Headlight)
		set_head_light(no2_data[new].Rx.Headlight);
}

static THD_FUNCTION(display_process_thread, arg)
{
	(void)arg;
	chRegSetThreadName("no2 serial display");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&HW_UART_DEV.event, &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

	// Wait for motor config initialization
	chThdSleepMilliseconds(500);

	// Set default power level
	set_assist_level(0);

	commands_printf("Starting No 2 Display Thread!");

	for (;;)
	{
		chEvtWaitAnyTimeout(ALL_EVENTS, ST2MS(100));
		serial_display_check_rx();
		propagate_changes();
	}
}
