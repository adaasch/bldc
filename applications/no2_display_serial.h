/*
    Copyright 2025 Andreas Daasch

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

// --- VESC display driver for displays using the "No2" protocol like the S866

#ifndef APP_NO2_DISPLAY_SERIAL_H_
#define APP_NO2_DISPLAY_SERIAL_H_

#include "stdint.h"

enum No2Error
{
    NO2_ERROR_NONE,
    NO2_ERROR_E06,
    NO2_ERROR_E07,
    NO2_ERROR_E09,
    NO2_ERROR_E11
};

void no2_display_serial_start(void (*assist_level_cb)(float), void (*head_light_cb)(bool), void (*push_assist_cb)(void),
                              void (*pas_params_cb)(uint8_t, uint8_t));
bool no2_display_serial_is_active(void);
void no2_display_serial_set_wheel_rpm(float rpm);
void no2_display_serial_set_current(float current);
void no2_display_serial_set_push_assist(bool active);
void no2_display_serial_set_brake(bool active);
void no2_display_serial_set_error(int error);

#endif /* APP_NO2_DISPLAY_SERIAL_H_ */
