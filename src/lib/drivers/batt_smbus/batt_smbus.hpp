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
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for SMBUS SBS v1.1-compatible Smart Batteries
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 * @author Alex Mikhalev <alex@corvus-robotics.com>
 */

#pragma once

#include <ecl/geo/geo.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

using namespace time_literals;

class BATT_SMBUS_Base
{
protected:
	static constexpr hrt_abstime MEASUREMENT_INTERVAL_US =	100_ms;	///< time in microseconds, measure at 10Hz

	static constexpr uint8_t BATT_ADDR = 			0x0B;	///< Default 7 bit address I2C address. 8 bit = 0x16

	enum SBS_Register {
		SBS_REG_TEMP =                                  0x08,   ///< temperature register
		SBS_REG_VOLTAGE =                               0x09,   ///< voltage register
		SBS_REG_CURRENT =                               0x0A,   ///< current register
		SBS_REG_AVERAGE_CURRENT =                       0x0B,   ///< average current register
		SBS_REG_MAX_ERROR =                             0x0C,   ///< max error
		SBS_REG_RELATIVE_SOC =                          0x0D,   ///< Relative State Of Charge
		SBS_REG_ABSOLUTE_SOC =                          0x0E,   ///< Absolute State of charge
		SBS_REG_REMAINING_CAPACITY =                    0x0F,   ///< predicted remaining battery capacity as a percentage
		SBS_REG_FULL_CHARGE_CAPACITY =                  0x10,   ///< capacity when fully charged
		SBS_REG_RUN_TIME_TO_EMPTY =                     0x11,   ///< predicted remaining battery capacity based on the present rate of discharge in min
		SBS_REG_AVERAGE_TIME_TO_EMPTY =                 0x12,   ///< predicted remaining battery capacity based on the present rate of discharge in min
		SBS_REG_CYCLE_COUNT =                           0x17,   ///< number of cycles the battery has experienced
		SBS_REG_DESIGN_CAPACITY =                       0x18,   ///< design capacity register
		SBS_REG_DESIGN_VOLTAGE =                        0x19,   ///< design voltage register
		SBS_REG_MANUFACTURER_NAME =                     0x20,   ///< manufacturer name
		SBS_REG_MANUFACTURE_DATE =                      0x1B,   ///< manufacture date register
		SBS_REG_SERIAL_NUMBER =                         0x1C,   ///< serial number register
		SBS_REG_MANUFACTURER_ACCESS =                   0x00,
		SBS_REG_MANUFACTURER_DATA =                     0x23,
	};

	static constexpr size_t MANUFACTURER_NAME_SIZE =        21;     ///< manufacturer name data size
	static constexpr size_t MAX_CELL_COUNT = 		10;

	enum BATT_SMBUS_CustomCommand {
		CCMD_MAN_INFO = 1,
		CCMD_SUSPEND = 2,
		CCMD_RESUME = 3,
		CCMD_BATT_SMBUS_MAX = 3,
	};

public:
	BATT_SMBUS_Base(SMBus *interface);

	~BATT_SMBUS_Base();

	virtual int populate_smbus_data(battery_status_s &msg);

	virtual void run();

	/**
	 * @brief Returns the SBS serial number of the battery device.
	 * @return Returns the SBS serial number of the battery device.
	 */
	uint16_t get_serial_number();

	/**
	* @brief Read info from battery on startup.
	* @return Returns PX4_OK on success, PX4_ERROR on failure.
	*/
	virtual int get_startup_info();

	/**
	 * @brief Gets the SBS manufacture date of the battery.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacture_date();

	/**
	 * @brief Gets the SBS manufacturer name of the battery device.
	 * @param manufacturer_name Pointer to a buffer into which the manufacturer name is to be written.
	 * @param max_length The maximum number of bytes to attempt to read from the manufacturer name register,
	 *                   including the null character that is appended to the end.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_name(uint8_t *manufacturer_name, const uint8_t length);

protected:
	SMBus *_interface;

	perf_counter_t _cycle{perf_alloc(PC_ELAPSED, "batt_smbus_cycle")};

	float _cell_voltages[MAX_CELL_COUNT] = {};

	float _max_cell_voltage_delta{0};

	float _min_cell_voltage{0};

	float _pack_power{0};
	float _pack_average_power{0};

	/** @param _last_report Last published report, used for test(). */
	battery_status_s _last_report{};

	/** @param _battery_status_pub uORB battery topic. */
	uORB::Publication<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

	/** @param _cell_count Number of series cell. */
	uint8_t _cell_count{0};

	/** @param _batt_capacity Battery design capacity in mAh (0 means unknown). */
	uint16_t _batt_capacity{0};

	/** @param _batt_startup_capacity Battery remaining capacity in mAh on startup. */
	uint16_t _batt_startup_capacity{0};

	/** @param _cycle_count The number of cycles the battery has experienced. */
	uint16_t _cycle_count{0};

	/** @param _serial_number Serial number register. */
	uint16_t _serial_number{0};

	/** @param _crit_thr Critical battery threshold param. */
	float _crit_thr{0.f};

	/** @param _emergency_thr Emergency battery threshold param. */
	float _emergency_thr{0.f};

	/** @param _low_thr Low battery threshold param. */
	float _low_thr{0.f};

	/** @parama _c_mult Capacity/current multiplier param  */
	float _c_mult{1.f};

	/** @param _manufacturer_name Name of the battery manufacturer. */
	char _manufacturer_name[MANUFACTURER_NAME_SIZE + 1] {};	// Plus one for terminator

	/** @param _manufacture_date Date of the battery manufacturing. */
	uint16_t _manufacture_date{0};

	/** @param _state_of_health state of health as read on connection  */
	float _state_of_health{0.f};

	uint8_t _smart_battery_type;
	/** @param _lifetime_max_delta_cell_voltage Max lifetime delta of the battery cells */
	float _lifetime_max_delta_cell_voltage{0.f};

	/** @param _cell_undervoltage_protection_status 0 if protection disabled, 1 if enabled */
	uint8_t _cell_undervoltage_protection_status{1};

	BATT_SMBUS_Base(const BATT_SMBUS_Base &) = delete;
	BATT_SMBUS_Base operator=(const BATT_SMBUS_Base &) = delete;
};


template<typename ThisDriver>
class BATT_SMBUS : public BATT_SMBUS_Base, public I2CSPIDriver<ThisDriver>
{
public:
	static constexpr int NO_SUCH_COMMAND = 0xABCD;

	BATT_SMBUS(I2CSPIBusOption bus_option, const int bus, SMBus *interface);
	~BATT_SMBUS();

	void suspend();

	void resume();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	void custom_method(const BusCLIArguments &cli) override;
	void RunImpl();

	/**
	 * Handle a command verb.
	 *
	 * Returns NO_SUCH_COMMAND if no such command verb exists.
	 */
	static int handle_command(const char *verb, BusCLIArguments &cli, BusInstanceIterator &iterator);

	static void print_module_description();
	static void print_usage_commands();
	static void print_usage();

	static int main(int argc, char *argv[]);
};

template<typename ThisDriver>
BATT_SMBUS<ThisDriver>::BATT_SMBUS(I2CSPIBusOption bus_option, const int bus, SMBus *interface) :
	BATT_SMBUS_Base(interface),
	I2CSPIDriver<ThisDriver>(ThisDriver::MOD_NAME, px4::device_bus_to_wq(interface->get_device_id()),
				 bus_option, bus)
{
}

template<typename ThisDriver>
BATT_SMBUS<ThisDriver>::~BATT_SMBUS()
{
}

template<typename ThisDriver>
void BATT_SMBUS<ThisDriver>::suspend()
{
	this->ScheduleClear();
}

template<typename ThisDriver>
void BATT_SMBUS<ThisDriver>::resume()
{
	this->ScheduleOnInterval(ThisDriver::MEASUREMENT_INTERVAL_US);
}

template<typename ThisDriver>
void BATT_SMBUS<ThisDriver>::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case CCMD_MAN_INFO: {
			uint8_t man_name[22];
			int result = manufacturer_name(man_name, sizeof(man_name));
			PX4_INFO("The manufacturer name: %s", man_name);

			result = manufacture_date();
			PX4_INFO("The manufacturer date: %d", result);

			uint16_t serial_num = 0;
			serial_num = get_serial_number();
			PX4_INFO("The serial number: %d", serial_num);
		}
		break;

	case CCMD_SUSPEND:
		suspend();
		break;

	case CCMD_RESUME:
		resume();
		break;

	default: break;
	}
}

template<typename ThisDriver>
void BATT_SMBUS<ThisDriver>::RunImpl()
{
	run();
}

template<typename ThisDriver>
I2CSPIDriverBase *BATT_SMBUS<ThisDriver>::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	SMBus *interface = new SMBus(iterator.bus(), cli.i2c_address);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	BATT_SMBUS<ThisDriver> *instance = new ThisDriver(iterator.configuredBusOption(), iterator.bus(), interface);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	int ret = instance->get_startup_info();

	if (ret != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->resume();

	return instance;
}

template<typename ThisDriver>
int BATT_SMBUS<ThisDriver>::handle_command(const char *verb, BusCLIArguments &cli, BusInstanceIterator &iterator)
{
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
		cli.custom1 = CCMD_MAN_INFO;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "suspend")) {
		cli.custom1 = CCMD_SUSPEND;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "resume")) {
		cli.custom1 = CCMD_RESUME;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return NO_SUCH_COMMAND;
}

template<typename ThisDriver>
void BATT_SMBUS<ThisDriver>::print_module_description()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for generic SMBUS Compatible smart-batteries.

### Examples
To read manufacturer information
$ batt_smbus -X man_info

)DESCR_STR");
}

template<typename ThisDriver>
void BATT_SMBUS<ThisDriver>::print_usage_commands()
{
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(ThisDriver::BATT_ADDR);

	PRINT_MODULE_USAGE_COMMAND_DESCR("man_info", "Prints manufacturer info.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("suspend", "Suspends the driver from rescheduling the cycle.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resumes the driver from suspension.");
}

template<typename ThisDriver>
void BATT_SMBUS<ThisDriver>::print_usage()
{
	ThisDriver::print_module_description();
	PRINT_MODULE_USAGE_NAME(ThisDriver::MOD_NAME, "driver");

	ThisDriver::print_usage_commands();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

template<typename ThisDriver>
int BATT_SMBUS<ThisDriver>::main(int argc, char *argv[])
{
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = ThisDriver::BATT_ADDR;;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(ThisDriver::MOD_NAME, cli, DRV_BAT_DEVTYPE_SMBUS);

	int res = ThisDriver::handle_command(verb, cli, iterator);

	if (res == NO_SUCH_COMMAND) {
		ThisDriver::print_usage();
		return -1;
	} else {
		return res;
	}
}
