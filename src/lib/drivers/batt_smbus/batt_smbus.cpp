/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file batt_smbus.h
 *
 * This is a basic, SBS v1.1 compliant implementation of
 * an SMBUS Smart Battery. This driver is to be used as a default,
 * or as a base-class for more specific implementations such as
 * the Rotoye Batmon of TI BQ40z50/80
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 * @author Alex Mikhalev <alex@corvus-robotics.com>
 *
 */

#include <lib/drivers/batt_smbus/batt_smbus.hpp>

BATT_SMBUS_Base::BATT_SMBUS_Base(SMBus *interface) :
	_interface(interface)
{
	_interface->init();
}

BATT_SMBUS_Base::~BATT_SMBUS_Base()
{
	perf_free(_cycle);

	if (_interface != nullptr) {
		delete _interface;
	}
}

uint16_t BATT_SMBUS_Base::get_serial_number()
{
	uint16_t serial_num = 0;

	if (_interface->read_word(SBS_REG_SERIAL_NUMBER, serial_num) == PX4_OK) {
		return serial_num;
	}

	return PX4_ERROR;
}

int BATT_SMBUS_Base::get_startup_info()
{
	int ret = PX4_OK;

	// Read battery threshold params on startup.
	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	param_get(param_find("BAT_C_MULT"), &_c_mult);

	int32_t cell_count_param = 0;
	param_get(param_find("BAT_N_CELLS"), &cell_count_param);
	_cell_count = (uint8_t)cell_count_param;

	ret |= _interface->block_read(SBS_REG_MANUFACTURER_NAME, _manufacturer_name, MANUFACTURER_NAME_SIZE,
				      true);
	_manufacturer_name[sizeof(_manufacturer_name) - 1] = '\0';

	uint16_t serial_num;
	ret |= _interface->read_word(SBS_REG_SERIAL_NUMBER, serial_num);

	uint16_t remaining_cap;
	ret |= _interface->read_word(SBS_REG_REMAINING_CAPACITY, remaining_cap);

	uint16_t cycle_count;
	ret |= _interface->read_word(SBS_REG_CYCLE_COUNT, cycle_count);

	uint16_t full_cap;
	ret |= _interface->read_word(SBS_REG_FULL_CHARGE_CAPACITY, full_cap);

	uint16_t manufacture_date;
	ret |= _interface->read_word(SBS_REG_MANUFACTURE_DATE, manufacture_date);

	if (!ret) {
		_serial_number = serial_num;
		_batt_startup_capacity = (uint16_t)((float)remaining_cap * _c_mult);
		_cycle_count = cycle_count;
		_batt_capacity = (uint16_t)((float)full_cap * _c_mult);
		_manufacture_date = manufacture_date;
	}

	return ret;
}

int BATT_SMBUS_Base::populate_smbus_data(battery_status_s &data)
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	int ret = _interface->read_word(SBS_REG_VOLTAGE, result);

	// Convert millivolts to volts.
	data.voltage_v = ((float)result) * 0.001f;
	data.voltage_filtered_v = data.voltage_v;

	// Read current.
	ret |= _interface->read_word(SBS_REG_CURRENT, result);

	data.current_a = (-1.0f * ((float)(*(int16_t *)&result)) * 0.001f) * _c_mult;
	data.current_filtered_a = data.current_a;

	// Read remaining capacity.
	ret |= _interface->read_word(SBS_REG_RELATIVE_SOC, result);
	data.remaining = (float)result * 0.01f;

	// Read remaining capacity.
	ret |= _interface->read_word(SBS_REG_REMAINING_CAPACITY, result);
	data.discharged_mah = _batt_startup_capacity - result;

	// Read full capacity.
	ret |= _interface->read_word(SBS_REG_FULL_CHARGE_CAPACITY, result);
	data.capacity = result;

	// Read cycle count.
	ret |= _interface->read_word(SBS_REG_CYCLE_COUNT, result);
	data.cycle_count = result;

	// Read serial number.
	ret |= _interface->read_word(SBS_REG_SERIAL_NUMBER, result);
	data.serial_number = result;

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(SBS_REG_TEMP, result);
	data.temperature = ((float)result * 0.1f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	return ret;
}

void BATT_SMBUS_Base::run()
{
	// Get the current time.
	uint64_t now = hrt_absolute_time();

	battery_status_s new_report = {};

	new_report.id = 1;
	new_report.timestamp = now;
	new_report.connected = true;
	new_report.cell_count = _cell_count;

	// Read data from sensor.
	int ret = populate_smbus_data(new_report);

	// Only publish if no errors.
	if (!ret) {
		_battery_status_pub.publish(new_report);

		_last_report = new_report;
	}
}


int BATT_SMBUS_Base::manufacture_date()
{
	uint16_t date;
	int result = _interface->read_word(SBS_REG_MANUFACTURE_DATE, date);

	if (result != PX4_OK) {
		return result;
	}

	return date;
}

int BATT_SMBUS_Base::manufacturer_name(uint8_t *man_name, const uint8_t length)
{
	uint8_t code = SBS_REG_MANUFACTURER_NAME;
	uint8_t rx_buf[21] = {};

	// Returns 21 bytes, add 1 byte for null terminator.
	int result = _interface->block_read(code, rx_buf, length - 1, true);

	memcpy(man_name, rx_buf, sizeof(rx_buf));

	man_name[21] = '\0';

	return result;
}

