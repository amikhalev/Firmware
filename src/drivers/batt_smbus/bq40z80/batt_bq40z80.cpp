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
 * @file batt_bq40z80.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2 and BQ40Z80
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 *
 */

#include "batt_bq40z80.h"

extern "C" __EXPORT int batt_bq40z80_main(int argc, char *argv[]);

BATT_BQ40Z80::BATT_BQ40Z80(I2CSPIBusOption bus_option, const int bus, SMBus *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus,
		     interface->get_device_address()),
	_interface(interface)
{
	perf_alloc(PC_ELAPSED, "batt_bq40z80_cycle");
	_interface->init();
}

BATT_BQ40Z80::~BATT_BQ40Z80()
{
	perf_free(_cycle);

	if (_interface != nullptr) {
		delete _interface;
	}
}

void BATT_BQ40Z80::RunImpl()
{
	int ret = PX4_OK;

	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	// Read data from sensor.
	battery_status_s new_report = {};

	// TODO(hyonlim): this driver should support multiple SMBUS going forward.
	new_report.id = 1;

	// Set time of reading.
	new_report.timestamp = hrt_absolute_time();

	new_report.connected = true;

	ret |= _interface->read_word(BATT_BQ40Z80_VOLTAGE, result);

	// Convert millivolts to volts.
	new_report.voltage_v = ((float)result) / 1000.0f;
	new_report.voltage_filtered_v = new_report.voltage_v;

	ret |= get_cell_voltages();

	for (int i = 0; i < _cell_count; i++) {
		new_report.voltage_cell_v[i] = _cell_voltages[i];
	}

	// Read current.
	ret |= _interface->read_word(BATT_BQ40Z80_CURRENT, result);

	new_report.current_a = (-1.0f * ((float)(*(int16_t *)&result)) / 1000.0f) * _c_mult;
	new_report.current_filtered_a = new_report.current_a;

	// Read average current.
	ret |= _interface->read_word(BATT_BQ40Z80_AVERAGE_CURRENT, result);

	float average_current = (-1.0f * ((float)(*(int16_t *)&result)) / 1000.0f) * _c_mult;

	new_report.average_current_a = average_current;

	// Read run time to empty (minutes).
	ret |= _interface->read_word(BATT_BQ40Z80_RUN_TIME_TO_EMPTY, result);
	new_report.run_time_to_empty = result;

	// Read average time to empty (minutes).
	ret |= _interface->read_word(BATT_BQ40Z80_AVERAGE_TIME_TO_EMPTY, result);
	new_report.average_time_to_empty = result;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_BQ40Z80_REMAINING_CAPACITY, result);

	// Calculate total discharged amount in mah.
	new_report.discharged_mah = _batt_startup_capacity - (float)result * _c_mult;

	// Read Relative SOC.
	ret |= _interface->read_word(BATT_BQ40Z80_RELATIVE_SOC, result);

	// Normalize 0.0 to 1.0
	new_report.remaining = (float)result / 100.0f;

	// Read Max Error
	ret |= _interface->read_word(BATT_BQ40Z80_MAX_ERROR, result);
	new_report.max_error = result;

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(BATT_BQ40Z80_TEMP, result);
	new_report.temperature = ((float)result / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	// Only publish if no errors.
	if (ret == PX4_OK) {
		new_report.capacity = _batt_capacity;
		new_report.cycle_count = _cycle_count;
		new_report.serial_number = _serial_number;
		new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
		new_report.cell_count = _cell_count;
		new_report.state_of_health = _state_of_health;

		// Check if max lifetime voltage delta is greater than allowed.
		if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
			new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else if (new_report.remaining > _low_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

		} else if (new_report.remaining > _crit_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

		} else if (new_report.remaining > _emergency_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else {
			new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
		}

		new_report.interface_error = perf_event_count(_interface->_interface_errors);

		_battery_status_pub.publish(new_report);

		_last_report = new_report;
	}
}

void BATT_BQ40Z80::suspend()
{
	ScheduleClear();
}

void BATT_BQ40Z80::resume()
{
	ScheduleOnInterval(BATT_BQ40Z80_MEASUREMENT_INTERVAL_US);
}

int BATT_BQ40Z80::get_cell_voltages()
{
	int ret = PX4_OK;

	uint8_t DAstatus1[32 + 2] = {}; // 32 bytes of data and 2 bytes of address

	//TODO: need to consider if set voltages to 0? -1?
	if (PX4_OK != manufacturer_read(BATT_BQ40Z80_DASTATUS1, DAstatus1, sizeof(DAstatus1))) {
		return PX4_ERROR;
	}

	// Convert millivolts to volts.
	_cell_voltages[0] = ((float)((DAstatus1[1] << 8) | DAstatus1[0]) / 1000.0f);
	_cell_voltages[1] = ((float)((DAstatus1[3] << 8) | DAstatus1[2]) / 1000.0f);
	_cell_voltages[2] = ((float)((DAstatus1[5] << 8) | DAstatus1[4]) / 1000.0f);
	_cell_voltages[3] = ((float)((DAstatus1[7] << 8) | DAstatus1[6]) / 1000.0f);

	_pack_power = ((float)((DAstatus1[29] << 8) | DAstatus1[28]) / 100.0f); //TODO: decide if both needed
	_pack_average_power = ((float)((DAstatus1[31] << 8) | DAstatus1[30]) / 100.0f);

	uint8_t DAstatus3[18 + 2] = {}; // 18 bytes of data and 2 bytes of address

	//TODO: need to consider if set voltages to 0? -1?
	if (PX4_OK != manufacturer_read(BATT_BQ40Z80_DASTATUS3, DAstatus3, sizeof(DAstatus3))) {
		return PX4_ERROR;
	}

	_cell_voltages[4] = ((float)((DAstatus3[1] << 8) | DAstatus3[0]) / 1000.0f);
	_cell_voltages[5] = ((float)((DAstatus3[7] << 8) | DAstatus3[6]) / 1000.0f);
	_cell_voltages[6] = ((float)((DAstatus3[13] << 8) | DAstatus3[12]) / 1000.0f);

	//Calculate max cell delta
	_min_cell_voltage = _cell_voltages[0];
	float max_cell_voltage = _cell_voltages[0];

	for (uint8_t i = 1; (i < _cell_count && i < (sizeof(_cell_voltages) / sizeof(_cell_voltages[0]))); i++) {
		_min_cell_voltage = math::min(_min_cell_voltage, _cell_voltages[i]);
		max_cell_voltage = math::max(max_cell_voltage, _cell_voltages[i]);
	}

	// Calculate the max difference between the min and max cells with complementary filter.
	_max_cell_voltage_delta = (0.5f * (max_cell_voltage - _min_cell_voltage)) +
				  (0.5f * _last_report.max_cell_voltage_delta);

	return ret;
}

void BATT_BQ40Z80::set_undervoltage_protection(float average_current)
{
	// Disable undervoltage protection if armed. Enable if disarmed and cell voltage is above limit.
	if (average_current > BATT_CURRENT_UNDERVOLTAGE_THRESHOLD) {
		if (_cell_undervoltage_protection_status != 0) {
			// Disable undervoltage protection
			uint8_t protections_a_tmp = BATT_BQ40Z80_ENABLED_PROTECTIONS_A_CUV_DISABLED;
			uint16_t address = BATT_BQ40Z80_ENABLED_PROTECTIONS_A_ADDRESS;

			if (dataflash_write(address, &protections_a_tmp, 1) == PX4_OK) {
				_cell_undervoltage_protection_status = 0;
				PX4_WARN("Disabled CUV");

			} else {
				PX4_WARN("Failed to disable CUV");
			}
		}

	} else {
		if (_cell_undervoltage_protection_status == 0) {
			if (_min_cell_voltage > BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD) {
				// Enable undervoltage protection
				uint8_t protections_a_tmp = BATT_BQ40Z80_ENABLED_PROTECTIONS_A_DEFAULT;
				uint16_t address = BATT_BQ40Z80_ENABLED_PROTECTIONS_A_ADDRESS;

				if (dataflash_write(address, &protections_a_tmp, 1) == PX4_OK) {
					_cell_undervoltage_protection_status = 1;
					PX4_WARN("Enabled CUV");

				} else {
					PX4_WARN("Failed to enable CUV");
				}
			}
		}
	}

}

//@NOTE: Currently unused, could be helpful for debugging a parameter set though.
int BATT_BQ40Z80::dataflash_read(const uint16_t address, void *data, const unsigned length)
{
	uint8_t code = BATT_BQ40Z80_MANUFACTURER_BLOCK_ACCESS;

	int ret = _interface->block_write(code, &address, 2, true);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = _interface->block_read(code, data, length, true);

	return ret;
}

int BATT_BQ40Z80::dataflash_write(const uint16_t address, void *data, const unsigned length)
{
	uint8_t code = BATT_BQ40Z80_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};

	tx_buf[0] = address & 0xff;
	tx_buf[1] = (address >> 8) & 0xff;

	if (length > MAC_DATA_BUFFER_SIZE) {
		return PX4_ERROR;
	}

	memcpy(&tx_buf[2], data, length);

	// code (1), byte_count (1), addr(2), data(32) + pec
	int ret = _interface->block_write(code, tx_buf, length + 2, false);

	return ret;
}

int BATT_BQ40Z80::get_startup_info()
{
	int ret = PX4_OK;

	// Read battery threshold params on startup.
	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	param_get(param_find("BAT_C_MULT"), &_c_mult);

	ret |= _interface->block_read(BATT_BQ40Z80_MANUFACTURER_NAME, _manufacturer_name, BATT_BQ40Z80_MANUFACTURER_NAME_SIZE,
				      true);
	_manufacturer_name[sizeof(_manufacturer_name) - 1] = '\0';

	uint16_t serial_num;
	ret |= _interface->read_word(BATT_BQ40Z80_SERIAL_NUMBER, serial_num);

	uint16_t remaining_cap;
	ret |= _interface->read_word(BATT_BQ40Z80_REMAINING_CAPACITY, remaining_cap);

	uint16_t cycle_count;
	ret |= _interface->read_word(BATT_BQ40Z80_CYCLE_COUNT, cycle_count);

	uint16_t full_cap;
	ret |= _interface->read_word(BATT_BQ40Z80_FULL_CHARGE_CAPACITY, full_cap);

	uint16_t manufacture_date;
	ret |= _interface->read_word(BATT_BQ40Z80_MANUFACTURE_DATE, manufacture_date);

	uint16_t state_of_health;
	ret |= _interface->read_word(BATT_BQ40Z80_STATE_OF_HEALTH, state_of_health);

	if (!ret) {
		_serial_number = serial_num;
		_batt_startup_capacity = (uint16_t)((float)remaining_cap * _c_mult);
		_cycle_count = cycle_count;
		_batt_capacity = (uint16_t)((float)full_cap * _c_mult);
		_manufacture_date = manufacture_date;
		_state_of_health = state_of_health;
	}

	if (lifetime_data_flush() == PX4_OK) {
		// Flush needs time to complete, otherwise device is busy. 100ms not enough, 200ms works.
		px4_usleep(200_ms);

		if (lifetime_read_block_one() == PX4_OK) {
			if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
				PX4_WARN("Battery Damaged Will Not Fly. Lifetime max voltage difference: %4.2f",
					 (double)_lifetime_max_delta_cell_voltage);
			}
		}

	} else {
		PX4_WARN("Failed to flush lifetime data");
	}

	return ret;
}

int BATT_BQ40Z80::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_BQ40Z80_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	int ret = _interface->block_write(code, address, 2, false);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = _interface->block_read(code, data, length, true);
	memmove(data, &((uint8_t *)data)[2], length - 2); // remove the address bytes

	return ret;
}

int BATT_BQ40Z80::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_BQ40Z80_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};
	tx_buf[0] = cmd_code & 0xff;
	tx_buf[1] = (cmd_code >> 8) & 0xff;

	if (data != nullptr && length <= MAC_DATA_BUFFER_SIZE) {
		memcpy(&tx_buf[2], data, length);
	}

	int ret = _interface->block_write(code, tx_buf, length + 2, false);

	return ret;
}

int BATT_BQ40Z80::unseal()
{
	// See bq40z50 technical reference.
	uint16_t keys[2] = {0x0414, 0x3672};

	int ret = _interface->write_word(BATT_BQ40Z80_MANUFACTURER_ACCESS, keys[0]);

	ret |= _interface->write_word(BATT_BQ40Z80_MANUFACTURER_ACCESS, keys[1]);

	return ret;
}

int BATT_BQ40Z80::seal()
{
	// See bq40z50 technical reference.
	return manufacturer_write(BATT_BQ40Z80_SEAL, nullptr, 0);
}

int BATT_BQ40Z80::lifetime_data_flush()
{
	return manufacturer_write(BATT_BQ40Z80_LIFETIME_FLUSH, nullptr, 0);
}

int BATT_BQ40Z80::lifetime_read_block_one()
{
	uint8_t lifetime_block_one[32 + 2] = {}; // 32 bytes of data and 2 bytes of address

	if (PX4_OK != manufacturer_read(BATT_BQ40Z80_LIFETIME_BLOCK_ONE, lifetime_block_one, sizeof(lifetime_block_one))) {
		PX4_INFO("Failed to read lifetime block 1.");
		return PX4_ERROR;
	}

	//Get max cell voltage delta and convert from mV to V.
	_lifetime_max_delta_cell_voltage = (float)(lifetime_block_one[29] << 8 | lifetime_block_one[28]) / 1000.0f;

	PX4_INFO("Max Cell Delta: %4.2f", (double)_lifetime_max_delta_cell_voltage);

	return PX4_OK;
}

void BATT_BQ40Z80::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Smart battery driver for the BQ40Z80 fuel gauge IC.

### Examples
To write to flash to set parameters. address, number_of_bytes, byte0, ... , byteN
$ batt_bq40z80 -X write_flash 19069 2 27 0

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("batt_bq40z80", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x0B);

	PRINT_MODULE_USAGE_COMMAND_DESCR("man_info", "Prints manufacturer info.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("unseal", "Unseals the devices flash memory to enable write_flash commands.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("seal", "Seals the devices flash memory to disbale write_flash commands.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("suspend", "Suspends the driver from rescheduling the cycle.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resumes the driver from suspension.");

	PRINT_MODULE_USAGE_COMMAND_DESCR("write_flash", "Writes to flash. The device must first be unsealed with the unseal command.");
	PRINT_MODULE_USAGE_ARG("address", "The address to start writing.", true);
	PRINT_MODULE_USAGE_ARG("number of bytes", "Number of bytes to send.", true);
	PRINT_MODULE_USAGE_ARG("data[0]...data[n]", "One byte of data at a time separated by spaces.", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BATT_BQ40Z80::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	SMBus *interface = new SMBus(iterator.bus(), cli.i2c_address);
	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}
	BATT_BQ40Z80 *instance = new BATT_BQ40Z80(iterator.configuredBusOption(), iterator.bus(), interface);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	// TODO: probe here

	// unseal() here to allow an external config script to write to protected flash.
	// This is neccessary to avoid bus errors due to using standard i2c mode instead of SMbus mode.
	// The external config script should then seal() the device.
	instance->unseal();

	int ret = instance->get_startup_info();

	if (ret != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->ScheduleOnInterval(BATT_BQ40Z80_MEASUREMENT_INTERVAL_US);

	return instance;
}

void
BATT_BQ40Z80::custom_method(const BusCLIArguments &cli)
{
	switch(cli.custom1) {
		case 1: {
			PX4_INFO("The manufacturer name: %s", _manufacturer_name);
			PX4_INFO("The manufacturer date: %d", _manufacture_date);
			PX4_INFO("The serial number: %d", _serial_number);
		}
			break;
		case 2:
			unseal();
			break;
		case 3:
			seal();
			break;
		case 4:
			suspend();
			break;
		case 5:
			resume();
			break;
		case 6:
			if (cli.custom_data) {
				unsigned address = cli.custom2;
				uint8_t *tx_buf = (uint8_t*)cli.custom_data;
				unsigned length = tx_buf[0];

				if (PX4_OK != dataflash_write(address, tx_buf+1, length)) {
					PX4_ERR("Dataflash write failed: %d", address);
				}
				px4_usleep(100_ms);
			}
			break;
	}
}

extern "C" __EXPORT int batt_bq40z80_main(int argc, char *argv[])
{
	using ThisDriver = BATT_BQ40Z80;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = BATT_BQ40Z80_ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_DEVTYPE_SMBUS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "man_info")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}
	if (!strcmp(verb, "unseal")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "seal")) {
		cli.custom1 = 3;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "suspend")) {
		cli.custom1 = 4;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "resume")) {
		cli.custom1 = 5;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "write_flash")) {
		cli.custom1 = 6;
		if (argc >= 3) {
			uint16_t address = atoi(argv[1]);
			unsigned length = atoi(argv[2]);
			uint8_t tx_buf[33];
			cli.custom_data = &tx_buf;

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			tx_buf[0] = length;
			// Data needs to be fed in 1 byte (0x01) at a time.
			for (unsigned i = 0; i < length; i++) {
				if ((unsigned)argc <= 3 + i) {
					tx_buf[i+1] = atoi(argv[3 + i]);
				}
			}
			cli.custom2 = address;
			return ThisDriver::module_custom_method(cli, iterator);
		}
	}

	ThisDriver::print_usage();
	return -1;
}
