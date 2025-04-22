/*
	Copyright 2025 Andreas Daasch

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

#include "hal.h"

#include "commands.h"
#include "terminal.h"

#include <string.h>
#include <stdio.h>

#include "no2_display_serial.h"

// private types

struct rx_param
{
#define NO2_RX_MAGIC_NUM 0x011401
	uint32_t magic_num : 24;	 // 0-2
	uint8_t throttle_mode;		 // 3 P10 (0-2)
	uint8_t assist_level;		 // 4
	uint8_t unk0 : 1;			 // 5
	uint8_t push_assist : 1;	 //
	uint8_t comm_error : 1;		 //
	uint8_t unk1 : 2;			 //
	uint8_t headlight : 1;		 //
	uint8_t zero_start : 1;		 // P09
	uint8_t unk2 : 1;			 //
	uint8_t motor_ratio;		 // 6 P07
	uint16_t wheel_size_dIn;	 // 7,8 byteswap P06
	uint8_t pas_sensitivity;	 // 9 P11 (1-24)
	uint8_t pas_attack;			 // 10 P12 (0-5)
	uint8_t unk3;				 // 11
	uint8_t speed_limit_kmh;	 // 12 P08 (0-100)
	uint8_t current_limit_A;	 // 13 P14 (1-20)
	uint16_t voltage_min_dV;	 // 14,15 byteswap P15
	uint16_t unk4;				 // 16,17
	uint8_t num_pas_magnets : 4; // 18 P13 (5,8,12)
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
	uint16_t current_dA;	   // 6,7 byteswap
	uint16_t wheel_period_ms;  // 8,9 byteswap
	uint16_t ukn5;			   // 10,11
	uint8_t ukn6;			   // 12
	uint8_t crc;			   // 13
} __attribute__((packed));
#define NO2_TX_MSG_SIZE sizeof(struct tx_param)

typedef struct
{
	struct rx_param rx[2];
	struct tx_param tx;
	systime_t last_msg;

} no2_message_t;

#define NO2_SERIAL_BUFFER_SIZE (3 * NO2_RX_MSG_SIZE)

typedef struct
{
	unsigned int rd_ptr;
	unsigned int wr_ptr;
	uint8_t data[NO2_SERIAL_BUFFER_SIZE];
} no2_serial_buffer_t;

// Threads
static THD_WORKING_AREA(display_process_thread_wa, 1024);
static THD_FUNCTION(display_process_thread, arg);

// private variables

static volatile no2_message_t no2_data;
static volatile int current_rx_idx = 1;
static volatile int last_rx_idx = 0;

static volatile bool display_thread_is_running = false;
static volatile bool display_uart_is_running = false;
static no2_serial_buffer_t serial_buffer;

// call backs
static void (*set_assist_level)(float);
static void (*set_head_light)(bool);
static void (*request_push_assist)(void);
static void (*set_pas_params)(uint8_t, uint8_t);

// debug

static volatile int debug_enabled = 0;
static volatile uint8_t debug_tx_pos = 20;
static volatile uint8_t debug_tx_byte = 0;

// function definitions

static void debug_en(int argc, const char **argv)
{
	if (argc == 2)
	{
		sscanf(argv[1], "%d", &debug_enabled);
	}
	else
	{
		commands_printf("This command requires one argument.");
	}
}

static void mod_tx(int argc, const char **argv)
{
	if (argc == 3)
	{
		sscanf(argv[1], "%hhu", &debug_tx_pos);
		sscanf(argv[2], "%hhu", &debug_tx_byte);
	}
	else
	{
		commands_printf("This command requires two arguments.");
	}
}

void no2_display_serial_start(void (*assist_level_cb)(float), void (*head_light_cb)(bool), void (*push_assist_cb)(void),
							  void (*pas_params_cb)(uint8_t, uint8_t))
{
	commands_printf("Starting No 2 Display Driver!");

	set_assist_level = assist_level_cb;
	set_head_light = head_light_cb;
	request_push_assist = push_assist_cb;
	set_pas_params = pas_params_cb;

	// init messages
	memset((void *)no2_data.rx, 0, sizeof(no2_data.rx));
	no2_data.tx.magic_num = NO2_TX_MAGIC_NUM;

	if (!display_thread_is_running)
	{
		chThdCreateStatic(display_process_thread_wa, sizeof(display_process_thread_wa),
						  NORMALPRIO, display_process_thread, NULL);
		display_thread_is_running = true;
	}
	serial_buffer.rd_ptr = 0;
	serial_buffer.wr_ptr = 0;

	// config uart

	static const SerialConfig uart_cfg = {
		9600, // baud rate
		0,
		USART_CR2_LINEN,
		0};

	sdStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);

	display_uart_is_running = true;

	// debug stuff

	terminal_register_command_callback(
		"no2_en_dbg",
		"Enable debug.",
		"[bool]",
		debug_en);

	terminal_register_command_callback(
		"no2_mod_byte",
		"mod tx byte.",
		"[byte][value]",
		mod_tx);
}

bool no2_display_serial_is_active(void)
{
	return ((chVTGetSystemTimeX() - no2_data.last_msg) / (float)CH_CFG_ST_FREQUENCY) < 1.0; // no msg since 1 sec
}

void no2_display_serial_set_wheel_rpm(float rpm)
{
	no2_data.tx.wheel_period_ms = __bswap16(60 / rpm * 1000);
}

void no2_display_serial_set_current(float current)
{
	no2_data.tx.current_dA = __bswap16(current * 10);
}

void no2_display_serial_set_push_assist(bool active)
{
	no2_data.tx.push_assist = active;
}

void no2_display_serial_set_brake(bool active)
{
	no2_data.tx.brake = active;
}

void no2_display_serial_set_error(int error)
{
	switch (error)
	{
	case NO2_ERROR_E06:
		no2_data.tx.e06 = 1;
		break;
	case NO2_ERROR_E07:
		no2_data.tx.e07 = 1;
		break;
	case NO2_ERROR_E09:
		no2_data.tx.e09 = 1;
		break;
	case NO2_ERROR_E11:
		no2_data.tx.e11 = 1;
		break;
	default:
		no2_data.tx.e06 = 0;
		no2_data.tx.e07 = 0;
		no2_data.tx.e09 = 0;
		no2_data.tx.e11 = 0;
		break;
	}
}

static int calculate_checksum(uint8_t *frame_buf, uint8_t length)
{
	uint8_t xor = 0;
	for (uint8_t *p = frame_buf; p < frame_buf + (length - 1); p++)
	{
		uint8_t tmp = *p;
		xor = xor ^ tmp;
	}
	return xor;
}

static void send_packet(uint8_t *data, unsigned int len)
{
	if (display_uart_is_running)
	{
		sdWrite(&HW_UART_DEV, data, len);
	}
}

static int parse_message(uint8_t *msg)
{
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
		last_rx_idx = !last_rx_idx;
		current_rx_idx = !current_rx_idx;

		no2_data.rx[current_rx_idx] = *rx_msg;
		no2_data.last_msg = chVTGetSystemTimeX();

		// send response
		if (debug_tx_pos < 20)
		{
			struct tx_param tmp = no2_data.tx;
			((uint8_t *)&tmp)[debug_tx_pos] = debug_tx_byte;
			no2_data.tx.crc = calculate_checksum((uint8_t *)&tmp, sizeof(tmp));
			send_packet((uint8_t *)&tmp, sizeof(tmp));
			return NO2_RX_MSG_SIZE;
		}

		no2_data.tx.crc = calculate_checksum((uint8_t *)&(no2_data.tx), sizeof(no2_data.tx));
		send_packet((uint8_t *)&(no2_data.tx), sizeof(no2_data.tx));
	}
	else
	{
		commands_printf("No2 Display: CHECKSUM error in rx message!");
	}

	return NO2_RX_MSG_SIZE;
}

static void print_rx_param(const struct rx_param *param)
{
	commands_printf("RX Parameter Struct:");
	commands_printf("---------------------");
	commands_printf("Magic Number: 0x%06X", param->magic_num);
	commands_printf("Throttle Mode: %u", param->throttle_mode);
	commands_printf("Assist Level: %u", param->assist_level);
	commands_printf("Unknown 0 (unk0): %u", param->unk0);
	commands_printf("Push Assist: %u", param->push_assist);
	commands_printf("Unknown 1 (unk1): 0x%x", param->unk1);
	commands_printf("Headlight: %u", param->headlight);
	commands_printf("Zero Start: %u", param->zero_start);
	commands_printf("Unknown 2 (unk2): %u", param->unk2);
	commands_printf("Motor Ratio: %u", param->motor_ratio);
	commands_printf("Wheel Size (dIn): %u", __bswap16(param->wheel_size_dIn));
	commands_printf("Boost Power: %u", param->pas_sensitivity);
	commands_printf("Start Delay PAS: %u", param->pas_attack);
	commands_printf("Unknown 3 (unk3): %u", param->unk3);
	commands_printf("Speed Limit (km/h): %u", param->speed_limit_kmh);
	commands_printf("Current Limit (A): %u", param->current_limit_A);
	commands_printf("Voltage Min (dV): %u", __bswap16(param->voltage_min_dV));
	commands_printf("Unknown 4 (unk4): %u", param->unk4);
	commands_printf("Number of PAS Magnets: %u", param->num_pas_magnets);
	commands_printf("Unknown 5 (unk5): 0x%x", param->unk5);
	commands_printf("Cruise Control: %u", param->cruise_control);
	commands_printf("Unknown 6 (unk6): %u", param->unk6);
	commands_printf("CRC: 0x%02X", param->crc);
	commands_printf("---------------------");
}

static void write_to_buffer(uint8_t byte)
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

		serial_buffer.rd_ptr += parse_message(&serial_buffer.data[serial_buffer.rd_ptr]);
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

static void receive_byte(void)
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
				write_to_buffer(res);
				rx = true;
			}
		}
	}
}

static void propagate_changes(void)
{
	if (no2_data.rx[current_rx_idx].assist_level != no2_data.rx[last_rx_idx].assist_level)
		set_assist_level((no2_data.rx[current_rx_idx].assist_level) / 15.0);

	if (no2_data.rx[current_rx_idx].headlight != no2_data.rx[last_rx_idx].headlight)
		set_head_light(no2_data.rx[current_rx_idx].headlight);

	if (no2_data.rx[current_rx_idx].push_assist == 1)
		request_push_assist();

	if (no2_data.rx[current_rx_idx].pas_attack != no2_data.rx[last_rx_idx].pas_attack ||
		no2_data.rx[current_rx_idx].pas_sensitivity != no2_data.rx[last_rx_idx].pas_sensitivity)
		set_pas_params(no2_data.rx[current_rx_idx].pas_attack, no2_data.rx[current_rx_idx].pas_sensitivity);
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

	for (;;)
	{
		chEvtWaitAnyTimeout(ALL_EVENTS, ST2MS(100));
		receive_byte();
		propagate_changes();
	}
}
