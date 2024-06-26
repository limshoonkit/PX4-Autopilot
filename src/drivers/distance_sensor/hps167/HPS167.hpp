/**
 * @file hps167.cpp
 * @author limshoonkit<lsk950329@hotmail.com>
 *
 * Driver for the Hypersen HPS167 Time-of-Flight (TOF) distance sensor
 * for the serial interface. Make sure to disable MAVLINK messages
 * (MAV_0_CONFIG PARAMETER) on the serial port you connect the sensor,i.e TELEM2.
 *
 */


#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


using namespace time_literals;

/* Configuration Constants */
static constexpr uint32_t HPS167_MEASURE_INTERVAL{20_ms};	// 50hz (20ms) sensor data rate

/* Frame start delimiter */
static constexpr unsigned char START_BYTE{0x0A};

/**
 * CMD_CONTINUOUS_RANGING
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)               |  CRC (2B)
 * | 0x0A       |  0x24     |  0x00 0x00 0x00 0x00 0x00 0x00 |  0x0F 0x72
 *
 * CMD_SINGLE_RANGING
 *
 * | Start (1B) |  CMD (1B) |  DATA FIELD (6B)               |  CRC (2B)
 * | 0x0A       |  0x22     |  0x00 0x00 0x00 0x00 0x00 0x00 |  0xAE 0x57
 *
 * Returned Data
 * | Start (1B) |  Len (1B) | Reserved (3B) | Distance (2B) | Magnitude (3B) | Ambient (1B) | Precision (2B) | CRC (2B)
 * | 0x0A       |  0x0D     | ...           | MSB LSB       | MSB LSB Exp.   | ...          | MSB LSB        | CRC data frame (Byte 2 to Byte 12)
 *
 * Example Decoding:
 * 0x0A: Start byte
 * 0x0D: Data length (13 byte data)
 * Distance = (0x06 * 256 + 0xD9) / 1000.0f = 1.753 (unit: m)
 * Magnitude = ((0xFC * 256 + 0x8C) << 0x02) / 10000.0f = 25.8608
 * Ambient ADC, Relative ambient IR intensity = 1
 * Precision, small values correspond to small measurement erroes = (0x00 * 256) + 0x01 = 1
 * 0x9B 0x94: CRC16-CCITT MSB and LSB byte
 *
 * Note: Sensor will output a 65.53m over range indication if the measurement result is over ranged or
 * receiving signal is too low.
 */
static constexpr uint8_t CMD_CONTINUOUS_RANGING[] = {0x0A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x72};
static constexpr uint8_t DISTANCE_MSB_POS{5};
static constexpr uint8_t DISTANCE_LSB_POS{6};

class HPS167 : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	HPS167(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~HPS167() override;

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:

	/**
	 * Calculates the 16 byte crc value for the data frame.
	 * @param data_frame The data frame to compute a checksum for.
	 * @param crc16_length The length of the data frame.
	 */
	uint16_t crc16_calc(const unsigned char *data_frame, uint8_t crc16_length);

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	void stop();

	PX4Rangefinder	_px4_rangefinder;

	char _port[20] {};

	int _file_descriptor{-1};

	uint8_t _linebuf[15] {};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
