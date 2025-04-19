/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include "no2_display_serial.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void assist_level_callback(float rel_current);
static void head_light_callback(bool on);
static void pas_conf(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// PAS
static volatile float current_assist_level = 0.0;
static volatile bool pas_enabled = false;
static volatile float pas_threshold = 0.7;
static volatile float pas_filter_loss = 0.5;

// CH_IRQ_HANDLER(HW_ENC_EXTI_ISR_VEC) {
// 	if (EXTI_GetITStatus(HW_ENC_EXTI_LINE) != RESET) {
// 		encoder_pin_isr();

// 		// Clear the EXTI line pending bit
// 		EXTI_ClearITPendingBit(HW_ENC_EXTI_LINE);
// 	}
// }

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{

	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP); // tacho

	// // Interrupt on HALL ROTARY A Pin
	// // Connect EXTI Line to pin
	// SYSCFG_EXTILineConfig(HW_HALL_ROTARY_A_EXTI_PORTSRC, HW_HALL_ROTARY_A_EXTI_PINSRC);

	// // Configure EXTI Line
	// EXTI_InitStructure.EXTI_Line = HW_HALL_ROTARY_A_EXTI_LINE;
	// EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	// EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// EXTI_Init(&EXTI_InitStructure);

	// // Enable and set EXTI Line Interrupt to the highest priority
	// nvicEnableVector(HW_HALL_ROTARY_A_EXTI_CH, 0);

	// mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
					  NORMALPRIO, my_thread, NULL);

	no2_display_serial_start(assist_level_callback, head_light_callback);

	volatile mc_configuration *conf = mc_interface_get_configuration();
	conf->l_max_erpm = 10e3; // ERPM limits 10k ~ 27km/h

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
		"set_pas_config",
		"Set pas start level and filter loss.",
		"[pas_threshold][pas_filter_loss]",
		pas_conf);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{

	// nvicDisableVector(cfg->exti_ch);

	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(pas_conf);

	stop_now = true;
	while (is_running)
	{
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf)
{
	(void)conf;
}

static void run_pas(void)
{
	static float adc_val_filt = 0;

	UTILS_LP_FAST(adc_val_filt, ADC_VOLTS(ADC_IND_TEMP_MOTOR), pas_filter_loss);
	if (adc_val_filt > pas_threshold)
	{
		if (!pas_enabled && no2_display_serial_is_active())
		{
			mc_interface_set_current_rel(current_assist_level);
			pas_enabled = true;
		}
	}
	else
	{
		if (pas_enabled)
		{
			mc_interface_set_current_rel(0.0);
			pas_enabled = false;
		}
	}
	if (!no2_display_serial_is_active())
	{
		current_assist_level = 0.0;
		mc_interface_set_current_rel(0.0);
	}
}

volatile float wheel_rpm_filtered = 0;
volatile float trip_odometer = 1.0; // avoids huge consumption numbers in the gauges

void hw_update_speed_sensor(void)
{
	static float wheel_rpm = 0;
	static uint8_t sensor_state = 0;
	static uint8_t sensor_state_old = 0;
	static float last_sensor_event_time = 0;
	float current_time = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

	sensor_state = palReadPad(HW_ICU_GPIO, HW_ICU_PIN);

	if (sensor_state == 0 && sensor_state_old == 1)
	{
		float revolution_duration = current_time - last_sensor_event_time;

		if (revolution_duration > 0.11)
		{ // ignore periods <110ms, which is about 68km/h
			last_sensor_event_time = current_time;
			wheel_rpm = 60.0 / revolution_duration;
			UTILS_LP_FAST(wheel_rpm_filtered, (float)wheel_rpm, 0.5);

			// For some reason a race condition on startup crashes the OS if this is executed too soon.
			// So don't track odometer for the first 4 seconds
			if (current_time > 4.0)
			{
				trip_odometer += mc_interface_get_configuration()->si_wheel_diameter * M_PI;
			}
		}
	}
	else
	{
		// After 3 seconds without sensor signal, set RPM as zero
		if ((current_time - last_sensor_event_time) > 3.0)
		{
			wheel_rpm_filtered = 0.0;
		}
	}
	sensor_state_old = sensor_state;

	no2_display_serial_set_wheel_rpm(wheel_rpm_filtered);
}

/* Get speed in m/s */
float hw_get_speed(void)
{
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	float speed = wheel_rpm_filtered * conf->si_wheel_diameter * M_PI / 60.0;
	return speed;
}

/* Get trip distance in meters */
float hw_get_distance(void)
{
	return trip_odometer;
}

float hw_get_distance_abs(void)
{
	return trip_odometer;
}

static THD_FUNCTION(my_thread, arg)
{
	(void)arg;

	chRegSetThreadName("App Custom");

	is_running = true;
	int cnt = 0;

	// Example of using the experiment plot
	//	chThdSleepMilliseconds(8000);
	//	commands_init_plot("Sample", "Voltage");
	//	commands_plot_add_graph("Temp Fet");
	//	commands_plot_add_graph("Input Voltage");
	//	float samp = 0.0;
	//
	//	for(;;) {
	//		commands_plot_set_graph(0);
	//		commands_send_plot_points(samp, mc_interface_temp_fet_filtered());
	//		commands_plot_set_graph(1);
	//		commands_send_plot_points(samp, GET_INPUT_VOLTAGE());
	//		samp++;
	//		chThdSleepMilliseconds(10);
	//	}

	for (;;)
	{
		// Check if it is time to stop.
		if (stop_now)
		{
			is_running = false;
			return;
		}

		hw_update_speed_sensor();

		if (cnt % 4 == 0) // 50Hz
		{
			run_pas();
			timeout_reset(); // Reset timeout if everything is OK.
		}

		// Run your logic here. A lot of functionality is available in mc_interface.h.

		chThdSleepMilliseconds(5); // 200Hz
	}
}

// Callback function for the terminal command with arguments.
static void pas_conf(int argc, const char **argv)
{
	if (argc == 3)
	{
		commands_printf("Old: Threshold: %.2f V Filter: %.2f",
						pas_threshold, pas_filter_loss);

		sscanf(argv[1], "%f", &pas_threshold);
		sscanf(argv[2], "%f", &pas_filter_loss);

		commands_printf("New: Threshold: %.2f V Filter: %.2f",
						pas_threshold, pas_filter_loss);
	}
	else
	{
		commands_printf("Current: Threshold: %.2f V Filter: %.2f",
						pas_threshold, pas_filter_loss);
		commands_printf("This command requires two arguments.\n");
	}
}

void assist_level_callback(float rel_current)
{
	current_assist_level = rel_current;
	if (pas_enabled)
		mc_interface_set_current_rel(current_assist_level);
}

void head_light_callback(bool on)
{
	volatile mc_configuration *conf = mc_interface_get_configuration();
	if (on)
		conf->l_max_erpm = 20e3; 
	else
		conf->l_max_erpm = 10e3; // ERPM limits 10k ~ 27km/h
}
