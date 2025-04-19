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
#include "app.h"
#include "ch.h"
#include "hal.h"

#include "packet.h"
#include "commands.h"
#include "mc_interface.h"
#include "utils.h"
#include "terminal.h"
#include "datatypes.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "no2_display_serial.h"

struct rx_param
{
#define NO2_RX_MAGIC_NUM 0x011401
	uint32_t magic_num : 24;	 // 0-2
	uint8_t throttle_mode;		 // 3 P10
	uint8_t assist_level;		 // 4
	uint8_t unk0 : 1;			 // 5
	uint8_t push_assist : 1;	 //
	uint8_t comm_error : 1;		 //
	uint8_t unk1 : 2;			 //
	uint8_t headlight : 1;		 //
	uint8_t zero_start : 1;		 // P09
	uint8_t unk2 : 1;			 //
	uint8_t motor_ratio;		 // 6   P07
	uint16_t wheel_size_dIn;	 // 7,8 bs P06
	uint8_t boost_power;		 // 9 P11
	uint8_t start_delay_pas;	 // 10 P12
	uint8_t unk3;				 // 11
	uint8_t speed_limit_kmh;	 // 12 P08
	uint8_t current_limit_A;	 // 13 P14
	uint16_t voltage_min_dV;	 // 14,15 bs P15
	uint16_t unk4;				 // 16,17
	uint8_t num_pas_magnets : 4; // 18 P13
	uint8_t unk5 : 2;			 //
	uint8_t cruise_control : 1;	 // P17
	uint8_t unk6 : 1;			 //
	uint8_t crc;				 //
} __attribute__((packed));
#define NO2_RX_MSG_SIZE sizeof(struct rx_param)

struct tx_param
{
#define NO2_TX_MAGIC_NUM 0x010e02
	uint32_t magic_num : 24;   // 0-2
	uint8_t e07 : 1;		   // 3
	uint8_t ukn0 : 1;		   //
	uint8_t push_assist : 1;   //
	uint8_t e06 : 1;		   //
	uint8_t e09 : 1;		   //
	uint8_t ukn1 : 1;		   //
	uint8_t e07_2 : 1;		   //
	uint8_t push_assist_2 : 1; //
	uint8_t ukn2 : 4;		   // 4
	uint8_t e11 : 1;		   //
	uint8_t brake : 1;		   //
	uint8_t ukn3 : 2;		   //
	uint8_t ukn4;			   // 5
	uint16_t current_dA;	   // 6,7 bs
	uint16_t wheel_period_ms;  // 8,9 bs
	uint16_t ukn5;			   // 10,11
	uint8_t ukn6;			   // 12
	uint8_t crc;			   // 13
} __attribute__((packed));
#define NO2_TX_MSG_SIZE sizeof(struct tx_param)

typedef struct
{
	struct rx_param rx;
	struct tx_param tx;
	systime_t last_msg;

} no2_message_t;

// Threads
static THD_WORKING_AREA(display_process_thread_wa, 1024);
static THD_FUNCTION(display_process_thread, arg);

volatile no2_message_t no2_data[2];
int new = 1;
int old = 0;

static void serial_send_packet(unsigned char *data, unsigned int len);
static void serial_display_byte_process(unsigned char byte);
static void serial_display_check_rx(void);

int No2_Service(uint8_t *No2_Message);

int calculate_checksum(unsigned char *frame_buf, uint8_t length);

uint8_t lowByte(uint16_t word);
uint8_t highByte(uint16_t word);

#define NO2_SERIAL_BUFFER_SIZE (3 * NO2_RX_MSG_SIZE)

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

static void debug_en(int argc, const char **argv);
static void mod_tx(int argc, const char **argv);

void no2_display_serial_start(void (*assist_level_cb)(float), void (*head_light_cb)(bool))
{
	set_assist_level = assist_level_cb;
	set_head_light = head_light_cb;

	commands_printf("Starting No 2 Display!");

	memset((void *)no2_data, 0, sizeof(no2_data));

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

	terminal_register_command_callback(
		"no2_en_dbg",
		"Enable debug.",
		"[bool]",
		debug_en);

	terminal_register_command_callback(
		"no2_mod_byte",
		"mod tx byte.",
		"[byte][bit]",
		mod_tx);
}

bool no2_display_serial_is_active(void)
{
	return ((chVTGetSystemTimeX() - no2_data[old].last_msg) / (float)CH_CFG_ST_FREQUENCY) < 1.0; // no msg since 1 sec
}

void no2_display_serial_set_wheel_rpm(float rpm)
{
	no2_data[new].tx.wheel_period_ms = __bswap16(60 / rpm * 1000);
}

volatile int debug_enabled = 0;
volatile uint8_t tx_pos = 20;
volatile uint8_t tx_byte = 0;

int No2_Service(uint8_t *msg)
{
	// static uint8_t TxBuffer[14] = {0x2, 0x0E, 0x1, 0x0, 0x80, 0x0, 0x0, 0x2C, 0x0, 0xF9, 0x0, 0x0, 0xFF, 0xA};

	struct rx_param *rx_msg = (struct rx_param *)msg;

	// check alignment
	unsigned int idx = 0;
	while (rx_msg->magic_num != NO2_RX_MAGIC_NUM)
	{
		idx++;
		if (idx > NO2_RX_MSG_SIZE - 3)
			break; // Magic number not found
		rx_msg = (struct rx_param *)(msg + idx);
	}
	if (idx > 0)
		return idx; // discard bytes

	if (rx_msg->crc == calculate_checksum((uint8_t *)rx_msg, NO2_RX_MSG_SIZE))
	{
		// swap data
		old = !old;
		new = !new;

		no2_data[new].rx = *rx_msg;
		no2_data[new].last_msg = chVTGetSystemTimeX();

		// send response
		no2_data[old].tx.magic_num = NO2_TX_MAGIC_NUM;

		if (tx_pos < 20)
		{
			struct tx_param tmp = no2_data[old].tx;
			((uint8_t *)&tmp)[tx_pos] = tx_byte;
			no2_data[old].tx.crc = calculate_checksum((uint8_t *)&tmp, sizeof(tmp));
			serial_send_packet((uint8_t *)&tmp, sizeof(tmp));
			return NO2_RX_MSG_SIZE;
		}

		no2_data[old].tx.crc = calculate_checksum((uint8_t *)&(no2_data[old].tx), sizeof(no2_data[old].tx));
		serial_send_packet((uint8_t *)&(no2_data[old].tx), sizeof(no2_data[old].tx));
	}
	else
	{
		commands_printf("process no2 CHECKSUM error!");
	}

	return NO2_RX_MSG_SIZE;
}

int calculate_checksum(unsigned char *frame_buf, uint8_t length)
{
	unsigned char xor = 0;
	unsigned char *p;
	unsigned char tmp;
	for (p = frame_buf; p < frame_buf + (length - 1); p++)
	{
		tmp = *p;
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

void print_rx_param(const struct rx_param *param)
{
	commands_printf("RX Parameter Struct:");
	commands_printf("---------------------");
	commands_printf("Magic Number: 0x%06X", param->magic_num);
	commands_printf("Throttle Mode: %u", param->throttle_mode);
	commands_printf("Assist Level: %u", param->assist_level);
	commands_printf("Unknown 0 (unk0): %u", param->unk0);
	commands_printf("Push Assist: %u", param->push_assist);
	commands_printf("Unknown 1 (unk1): 0b%03u", param->unk1);
	commands_printf("Headlight: %u", param->headlight);
	commands_printf("Zero Start: %u", param->zero_start);
	commands_printf("Unknown 2 (unk2): %u", param->unk2);
	commands_printf("Motor Ratio: %u", param->motor_ratio);
	commands_printf("Wheel Size (dIn): %u", __bswap16(param->wheel_size_dIn));
	commands_printf("Boost Power: %u", param->boost_power);
	commands_printf("Start Delay PAS: %u", param->start_delay_pas);
	commands_printf("Unknown 3 (unk3): %u", param->unk3);
	commands_printf("Speed Limit (km/h): %u", param->speed_limit_kmh);
	commands_printf("Current Limit (A): %u", param->current_limit_A);
	commands_printf("Voltage Min (dV): %u", __bswap16(param->voltage_min_dV));
	commands_printf("Unknown 4 (unk4): %u", param->unk4);
	commands_printf("Number of PAS Magnets: %u", param->num_pas_magnets);
	commands_printf("Unknown 5 (unk5): 0b%02u", param->unk5);
	commands_printf("Cruise Control: %u", param->cruise_control);
	commands_printf("Unknown 6 (unk6): %u", param->unk6);
	commands_printf("CRC: 0x%02X", param->crc);
	commands_printf("---------------------");
}

static void serial_display_byte_process(unsigned char byte)
{
	// append new byte to the buffer.
	serial_buffer.data[serial_buffer.wr_ptr] = byte;
	serial_buffer.wr_ptr++;

	uint8_t last_buf[NO2_RX_MSG_SIZE];

	// process with at least NO2_RX_MSG_SIZE bytes available to read
	while ((serial_buffer.wr_ptr - serial_buffer.rd_ptr) >= NO2_RX_MSG_SIZE)
	{
		if (debug_enabled)
		{
			int n = memcmp(last_buf, &serial_buffer.data[serial_buffer.rd_ptr], NO2_RX_MSG_SIZE);
			if (n)
			{
				commands_printf("Difference at byte %d", n);
				print_rx_param((const struct rx_param *)&serial_buffer.data[serial_buffer.rd_ptr]);
				memcpy(last_buf, &serial_buffer.data[serial_buffer.rd_ptr], NO2_RX_MSG_SIZE);
			}
		}

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

static void propagate_changes(void)
{
	if (no2_data[new].rx.assist_level != no2_data[old].rx.assist_level)
		set_assist_level((no2_data[new].rx.assist_level) / 15.0);
	// commands_printf("process no2 Assist lvl: %d ", no2_data[new].Rx.AssistLevel);

	if (no2_data[new].rx.headlight != no2_data[old].rx.headlight)
		set_head_light(no2_data[new].rx.headlight);
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

static void debug_en(int argc, const char **argv)
{
	if (argc == 2)
	{
		sscanf(argv[1], "%d", &debug_enabled);
	}
	else
	{
		commands_printf("This command requires two arguments.");
	}
}

static void mod_tx(int argc, const char **argv)
{
	if (argc == 3)
	{
		sscanf(argv[1], "%hhu", &tx_pos);
		sscanf(argv[2], "%hhu", &tx_byte);
	}
	else
	{
		commands_printf("This command requires two arguments.");
	}
}
