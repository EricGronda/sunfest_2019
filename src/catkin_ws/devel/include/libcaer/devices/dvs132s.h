/**
 * @file dvs132s.h
 *
 * DVS132S specific configuration defines and information structures.
 */

#ifndef LIBCAER_DEVICES_DVS132S_H_
#define LIBCAER_DEVICES_DVS132S_H_

#include "../events/polarity.h"
#include "../events/special.h"

#include "usb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device type definition for iniVation DVS132S.
 */
#define CAER_DEVICE_DVS132S 7

/**
 * DVS132S chip identifier.
 * 104x132, synchronous readout.
 */
#define DVS132S_CHIP_ID 15

/**
 * Module address: device-side Multiplexer configuration.
 * The Multiplexer is responsible for mixing, timestamping and outputting
 * (via USB) the various event types generated by the device. It is also
 * responsible for timestamp generation and synchronization.
 */
#define DVS132S_CONFIG_MUX 0
/**
 * Module address: device-side DVS configuration.
 * The DVS state machine interacts with the DVS chip and gets the
 * polarity events from it. It supports various configurable delays, as
 * well as advanced filtering capabilities on the polarity events.
 */
#define DVS132S_CONFIG_DVS 1
/**
 * Module address: device-side IMU (Inertial Measurement Unit) configuration.
 * The IMU module connects to the external IMU chip and sends data on the
 * device's movement in space. It can configure various options on the external
 * chip, such as accelerometer range or gyroscope refresh rate.
 */
#define DVS132S_CONFIG_IMU 3
/**
 * Module address: device-side External Input (signal detector/generator) configuration.
 * The External Input module is used to detect external signals on the external input
 * jack and inject an event into the event stream when this happens. It can detect pulses
 * of a specific length or rising and falling edges.
 * On some systems, a signal generator module is also present, which can generate
 * PWM-like pulsed signals with configurable timing.
 */
#define DVS132S_CONFIG_EXTINPUT 4
/**
 * Module address: device-side chip bias configuration.
 * This state machine is responsible for configuring the chip's bias generator.
 */
#define DVS132S_CONFIG_BIAS 5
/**
 * Module address: device-side system information.
 * The system information module provides various details on the device, such
 * as currently installed logic revision or clock speeds.
 * All its parameters are read-only.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dvs132s_info'
 * documentation for more details on what information is available.
 */
#define DVS132S_CONFIG_SYSINFO 6
/**
 * Module address: device-side USB output configuration.
 * The USB output module forwards the data from the device and the FPGA/CPLD to
 * the USB chip, usually a Cypress FX2 or FX3.
 */
#define DVS132S_CONFIG_USB 9

/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * run the Multiplexer state machine, which is responsible for
 * mixing the various event types at the device level, timestamping
 * them and outputting them via USB or other connectors.
 */
#define DVS132S_CONFIG_MUX_RUN 0
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * run the Timestamp Generator inside the Multiplexer state machine,
 * which will provide microsecond accurate timestamps to the
 * events passing through.
 */
#define DVS132S_CONFIG_MUX_TIMESTAMP_RUN 1
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * reset the Timestamp Generator to zero. This also sends a reset
 * pulse to all connected slave devices, resetting their timestamp too.
 */
#define DVS132S_CONFIG_MUX_TIMESTAMP_RESET 2
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * power up the chip's bias generator, enabling the chip to work.
 */
#define DVS132S_CONFIG_MUX_RUN_CHIP 3
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * drop External Input events if the USB output FIFO is full, instead of having
 * them pile up at the input FIFOs.
 */
#define DVS132S_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL 4
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * drop DVS events if the USB output FIFO is full, instead of having
 * them pile up at the input FIFOs.
 */
#define DVS132S_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL 5
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * read-only parameter, information about the presence of the
 * statistics feature.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_davis_info'
 * documentation to get this information.
 */
#define DVS132S_CONFIG_MUX_HAS_STATISTICS 80
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * read-only parameter, representing the number of dropped
 * External Input events on the device due to full USB buffers.
 * This is a 64bit value, and should always be read using the
 * function: caerDeviceConfigGet64().
 */
#define DVS132S_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED 81
/**
 * Parameter address for module DVS132S_CONFIG_MUX:
 * read-only parameter, representing the number of dropped
 * DVS events on the device due to full USB buffers.
 * This is a 64bit value, and should always be read using the
 * function: caerDeviceConfigGet64().
 */
#define DVS132S_CONFIG_MUX_STATISTICS_DVS_DROPPED 83

/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * read-only parameter, contains the X axis resolution of the
 * DVS events returned by the camera.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_davis_info'
 * documentation to get proper size information that already
 * considers the rotation and orientation settings.
 */
#define DVS132S_CONFIG_DVS_SIZE_COLUMNS 0
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * read-only parameter, contains the Y axis resolution of the
 * DVS events returned by the camera.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_davis_info'
 * documentation to get proper size information that already
 * considers the rotation and orientation settings.
 */
#define DVS132S_CONFIG_DVS_SIZE_ROWS 1
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * read-only parameter, contains information on the orientation
 * of the X/Y axes, whether they should be inverted or not on
 * the host when parsing incoming events.
 * Bit 2: dvsInvertXY
 * Bit 1: reserved
 * Bit 0: reserved
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_davis_info'
 * documentation to get proper size information that already
 * considers the rotation and orientation settings.
 */
#define DVS132S_CONFIG_DVS_ORIENTATION_INFO 2
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * run the DVS state machine and read out polarity events from the chip.
 */
#define DVS132S_CONFIG_DVS_RUN 3
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * if the output FIFO for this module is full, stall the chip readout
 * and wait until it's free again, instead of just continuing
 * reading and dropping the resulting events.
 */
#define DVS132S_CONFIG_DVS_WAIT_ON_TRANSFER_STALL 4
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_UNSIGNED 5
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_UNSIGNED 6
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_SIGNED 7
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_SIGNED 8
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * 7 bits, Time unit: 1us. Max: ~125us.
 */
#define DVS132S_CONFIG_DVS_RESTART_TIME 9
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * 21 bits, Time unit: 1us. Max: ~2s.
 */
#define DVS132S_CONFIG_DVS_CAPTURE_INTERVAL 10
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_ROW_ENABLE_31_TO_0 20
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_ROW_ENABLE_63_TO_32 21
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_ROW_ENABLE_65_TO_64 22
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_COLUMN_ENABLE_31_TO_0 50
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 *
 */
#define DVS132S_CONFIG_DVS_COLUMN_ENABLE_51_TO_32 51
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * read-only parameter, information about the presence of the
 * statistics feature.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_davis_info'
 * documentation to get this information.
 */
#define DVS132S_CONFIG_DVS_HAS_STATISTICS 80
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * read-only parameter, representing the number of event
 * transactions completed successfully on the device.
 * This is a 64bit value, and should always be read using the
 * function: caerDeviceConfigGet64().
 */
#define DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_SUCCESS 81
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * read-only parameter, representing the number of dropped
 * transaction sequences on the device due to full buffers.
 * This is a 64bit value, and should always be read using the
 * function: caerDeviceConfigGet64().
 */
#define DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_SKIPPED 83
/**
 * Parameter address for module DVS132S_CONFIG_DVS:
 * read-only parameter, representing the number of erroneous
 * transaction sequences on the device due problems in the
 * address or polarities returned by the chip.
 */
#define DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_ERRORED 91

/**
 * List of supported IMU models.
 */
enum caer_dvs132s_imu_types {
	IMU_NONE_DVS132S  = 0,
	IMU_BOSCH_BMI_160 = 3,
};

/**
 * List of accelerometer scale settings for Bosch IMU.
 */
enum caer_dvs132s_imu_bosch_accel_scale {
	BOSCH_ACCEL_2G  = 0,
	BOSCH_ACCEL_4G  = 1,
	BOSCH_ACCEL_8G  = 2,
	BOSCH_ACCEL_16G = 3,
};

/**
 * List of accelerometer data rate settings for Bosch IMU.
 */
enum caer_dvs132s_imu_bosch_accel_data_rate {
	BOSCH_ACCEL_12_5HZ = 0,
	BOSCH_ACCEL_25HZ   = 1,
	BOSCH_ACCEL_50HZ   = 2,
	BOSCH_ACCEL_100HZ  = 3,
	BOSCH_ACCEL_200HZ  = 4,
	BOSCH_ACCEL_400HZ  = 5,
	BOSCH_ACCEL_800HZ  = 6,
	BOSCH_ACCEL_1600HZ = 7,
};

/**
 * List of accelerometer filter settings for Bosch IMU.
 */
enum caer_dvs132s_imu_bosch_accel_filter {
	BOSCH_ACCEL_OSR4   = 0,
	BOSCH_ACCEL_OSR2   = 1,
	BOSCH_ACCEL_NORMAL = 2,
};

/**
 * List of gyroscope scale settings for Bosch IMU.
 */
enum caer_dvs132s_imu_bosch_gyro_scale {
	BOSCH_GYRO_2000DPS = 0,
	BOSCH_GYRO_1000DPS = 1,
	BOSCH_GYRO_500DPS  = 2,
	BOSCH_GYRO_250DPS  = 3,
	BOSCH_GYRO_125DPS  = 4,
};

/**
 * List of gyroscope data rate settings for Bosch IMU.
 */
enum caer_dvs132s_imu_bosch_gyro_data_rate {
	BOSCH_GYRO_25HZ   = 0,
	BOSCH_GYRO_50HZ   = 1,
	BOSCH_GYRO_100HZ  = 2,
	BOSCH_GYRO_200HZ  = 3,
	BOSCH_GYRO_400HZ  = 4,
	BOSCH_GYRO_800HZ  = 5,
	BOSCH_GYRO_1600HZ = 6,
	BOSCH_GYRO_3200HZ = 7,
};

/**
 * List of gyroscope filter settings for Bosch IMU.
 */
enum caer_dvs132s_imu_bosch_gyro_filter {
	BOSCH_GYRO_OSR4   = 0,
	BOSCH_GYRO_OSR2   = 1,
	BOSCH_GYRO_NORMAL = 2,
};

/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * read-only parameter, contains information on the type of IMU
 * chip being used in this device:
 * 0 - no IMU present
 * 3 - Bosch BMI 160
 * This is reserved for internal use and should not be used by
 * anything other than libcaer.
 */
#define DVS132S_CONFIG_IMU_TYPE 0
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * read-only parameter, contains information on the orientation
 * of the X/Y/Z axes, whether they should be flipped or not on
 * the host when parsing incoming IMU data samples.
 * Bit 2: imuFlipX
 * Bit 1: imuFlipY
 * Bit 0: imuFlipZ
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Generated IMU events are already
 * properly flipped when returned to the user.
 */
#define DVS132S_CONFIG_IMU_ORIENTATION_INFO 1
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * enable the IMU's accelerometer. This takes the
 * IMU chip out of sleep.
 */
#define DVS132S_CONFIG_IMU_RUN_ACCELEROMETER 2
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * enable the IMU's gyroscope. This takes the
 * IMU chip out of sleep.
 */
#define DVS132S_CONFIG_IMU_RUN_GYROSCOPE 3
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * enable the IMU's temperature sensor. This takes the
 * IMU chip out of sleep.
 */
#define DVS132S_CONFIG_IMU_RUN_TEMPERATURE 4
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * 8 settings:
 * 0 - 12.5 Hz
 * 1 - 25 Hz
 * 2 - 50 Hz
 * 3 - 100 Hz
 * 4 - 200 Hz
 * 5 - 400 Hz
 * 6 - 800 Hz
 * 7 - 1600 Hz
 */
#define DVS132S_CONFIG_IMU_ACCEL_DATA_RATE 5
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * 3 settings:
 * 0 - OSR4
 * 1 - OSR2
 * 2 - Normal
 */
#define DVS132S_CONFIG_IMU_ACCEL_FILTER 6
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * 4 settings:
 * 0 - +- 2g
 * 1 - +- 4g
 * 2 - +- 8g
 * 3 - +- 16g
 */
#define DVS132S_CONFIG_IMU_ACCEL_RANGE 7
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * 8 settings:
 * 0 - 25 Hz
 * 1 - 50 Hz
 * 2 - 100 Hz
 * 3 - 200 Hz
 * 4 - 400 Hz
 * 5 - 800 Hz
 * 6 - 1600 Hz
 * 7 - 3200 Hz
 */
#define DVS132S_CONFIG_IMU_GYRO_DATA_RATE 8
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * 3 settings:
 * 0 - OSR4
 * 1 - OSR2
 * 2 - Normal
 */
#define DVS132S_CONFIG_IMU_GYRO_FILTER 9
/**
 * Parameter address for module DVS132S_CONFIG_IMU:
 * 5 settings:
 * 0 - +- 2000°/s
 * 1 - +- 1000°/s
 * 2 - +- 500°/s
 * 3 - +- 250°/s
 * 4 - +- 125°/s
 */
#define DVS132S_CONFIG_IMU_GYRO_RANGE 10

/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * enable the signal detector module. It generates events
 * when it sees certain types of signals, such as edges or
 * pulses of a defined length, on the SIGNAL pin of the
 * INPUT synchronization connector.
 * This can be useful to inject events into the event
 * stream in response to external stimuli or controls,
 * such as turning on a LED lamp.
 */
#define DVS132S_CONFIG_EXTINPUT_RUN_DETECTOR 0
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * send a special EXTERNAL_INPUT_RISING_EDGE event when a
 * rising edge is detected (transition from low voltage to high).
 */
#define DVS132S_CONFIG_EXTINPUT_DETECT_RISING_EDGES 1
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * send a special EXTERNAL_INPUT_FALLING_EDGE event when a
 * falling edge is detected (transition from high voltage to low).
 */
#define DVS132S_CONFIG_EXTINPUT_DETECT_FALLING_EDGES 2
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * send a special EXTERNAL_INPUT_PULSE event when a pulse, of
 * a specified, configurable polarity and length, is detected.
 * See DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY and
 * DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH for more details.
 */
#define DVS132S_CONFIG_EXTINPUT_DETECT_PULSES 3
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * the polarity the pulse must exhibit to be detected as such.
 * '1' means active high; a pulse will start when the signal
 * goes from low to high and will continue to be seen as the
 * same pulse as long as it stays high.
 * '0' means active low; a pulse will start when the signal
 * goes from high to low and will continue to be seen as the
 * same pulse as long as it stays low.
 */
#define DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY 4
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * the minimal length that a pulse must have to trigger the
 * sending of a special event. This is measured in cycles
 * at LogicClock frequency (see 'struct caer_davis_info' for
 * details on how to get the frequency).
 */
#define DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH 5
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * read-only parameter, information about the presence of the
 * signal generator feature.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_davis_info'
 * documentation to get this information.
 */
#define DVS132S_CONFIG_EXTINPUT_HAS_GENERATOR 10
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * enable the signal generator module. It generates a
 * PWM-like signal based on configurable parameters and
 * outputs it on the OUT JACK signal.
 */
#define DVS132S_CONFIG_EXTINPUT_RUN_GENERATOR 11
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * polarity of the PWM-like signal to be generated.
 * '1' means active high, '0' means active low.
 */
#define DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY 12
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * the interval between the start of two consecutive pulses,
 * expressed in cycles at LogicClock frequency (see
 * 'struct caer_davis_info' for details on how to get the frequency).
 * This must be bigger or equal to DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH.
 * To generate a signal with 50% duty cycle, this would
 * have to be exactly double of DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH.
 */
#define DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL 13
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * the length a pulse stays active, expressed in cycles at
 * LogicClock frequency (see 'struct caer_davis_info' for
 * details on how to get the frequency). This must be
 * smaller or equal to DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL.
 * To generate a signal with 50% duty cycle, this would
 * have to be exactly half of DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL.
 */
#define DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH 14
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * enables event injection when a rising edge occurs in the
 * generated signal; a special event EXTERNAL_GENERATOR_RISING_EDGE
 * is emitted into the event stream.
 */
#define DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE 15
/**
 * Parameter address for module DVS132S_CONFIG_EXTINPUT:
 * enables event injection when a falling edge occurs in the
 * generated signal; a special event EXTERNAL_GENERATOR_FALLING_EDGE
 * is emitted into the event stream.
 */
#define DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE 16

/**
 * Parameter address for module DVS132S_CONFIG_SYSINFO:
 * read-only parameter, the version of the logic currently
 * running on the device's FPGA/CPLD.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dvs132s_info'
 * documentation to get this information.
 */
#define DVS132S_CONFIG_SYSINFO_LOGIC_VERSION 0
/**
 * Parameter address for module DVS132S_CONFIG_SYSINFO:
 * read-only parameter, an integer used to identify the different
 * types of sensor chips used on the device.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dvs132s_info'
 * documentation to get this information.
 */
#define DVS132S_CONFIG_SYSINFO_CHIP_IDENTIFIER 1
/**
 * Parameter address for module DVS132S_CONFIG_SYSINFO:
 * read-only parameter, whether the device is currently a timestamp
 * master or slave when synchronizing multiple devices together.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer. Please see the 'struct caer_dvs132s_info'
 * documentation to get this information.
 */
#define DVS132S_CONFIG_SYSINFO_DEVICE_IS_MASTER 2
/**
 * Parameter address for module DVS132S_CONFIG_SYSINFO:
 * read-only parameter, the frequency in MHz at which the main
 * FPGA/CPLD logic is running.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer.
 */
#define DVS132S_CONFIG_SYSINFO_LOGIC_CLOCK 3
/**
 * Parameter address for module DVS132S_CONFIG_SYSINFO:
 * read-only parameter, the frequency in MHz at which the FPGA/CPLD
 * logic related to USB data transmission is running.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer.
 */
#define DVS132S_CONFIG_SYSINFO_USB_CLOCK 5
/**
 * Parameter address for module DVS132S_CONFIG_SYSINFO:
 * read-only parameter, the deviation factor for the clocks.
 * Due to how FX3 generates the clocks, which are then used by
 * FPGA/CPLD, they are not integers but have a fractional part.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer.
 */
#define DVS132S_CONFIG_SYSINFO_CLOCK_DEVIATION 6
/**
 * Parameter address for module DVS132S_CONFIG_SYSINFO:
 * read-only parameter, the patch version of the logic currently
 * running on the device's FPGA/CPLD.
 * This is reserved for internal use and should not be used by
 * anything other than libcaer.
 */
#define DVS132S_CONFIG_SYSINFO_LOGIC_PATCH 7

/**
 * Parameter address for module DVS132S_CONFIG_USB:
 * enable the USB FIFO module, which transfers the data from the
 * FPGA/CPLD to the USB chip, to be then sent to the host.
 * Turning this off will suppress any USB data communication!
 */
#define DVS132S_CONFIG_USB_RUN 0
/**
 * Parameter address for module DVS132S_CONFIG_USB:
 * the time delay after which a packet of data is committed to
 * USB, even if it is not full yet (short USB packet).
 * The value is in 125µs time-slices, corresponding to how
 * USB schedules its operations (a value of 4 for example
 * would mean waiting at most 0.5ms until sending a short
 * USB packet to the host).
 */
#define DVS132S_CONFIG_USB_EARLY_PACKET_DELAY 1

//@{
/**
 * Parameter address for module DVS132S_CONFIG_BIAS:
 * DVS132S chip biases.
 * Bias configuration values must be generated using the proper
 * functions, which are:
 * - caerBiasCoarseFine1024Generate() for coarse-fine (current) biases.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define DVS132S_CONFIG_BIAS_PRBP 0
#define DVS132S_CONFIG_BIAS_PRSFBP 1
#define DVS132S_CONFIG_BIAS_BLPUBP 2
#define DVS132S_CONFIG_BIAS_BIASBUFBP 3
#define DVS132S_CONFIG_BIAS_OFFBN 4
#define DVS132S_CONFIG_BIAS_DIFFBN 5
#define DVS132S_CONFIG_BIAS_ONBN 6
#define DVS132S_CONFIG_BIAS_CASBN 7
#define DVS132S_CONFIG_BIAS_DPBN 8
#define DVS132S_CONFIG_BIAS_BIASBUFBN 9
#define DVS132S_CONFIG_BIAS_ABUFBN 10
//@}

/**
 * DVS132S device-related information.
 */
struct caer_dvs132s_info {
	/// Unique device identifier. Also 'source' for events.
	int16_t deviceID;
	/// Device serial number.
	char deviceSerialNumber[8 + 1];
	/// Device USB bus number.
	uint8_t deviceUSBBusNumber;
	/// Device USB device address.
	uint8_t deviceUSBDeviceAddress;
	/// Device information string, for logging purposes.
	/// If not NULL, pointed-to memory is *only* valid while the corresponding
	/// device is open! After calling deviceClose() this is invalid memory!
	char *deviceString;
	/// USB firmware version.
	int16_t firmwareVersion;
	/// Logic (FPGA/CPLD) version.
	int16_t logicVersion;
	/// Chip identifier/type.
	int16_t chipID;
	/// Whether the device is a time-stamp master or slave.
	bool deviceIsMaster;
	/// Feature test: Multiplexer statistics support (event drops).
	bool muxHasStatistics;
	/// DVS X axis resolution.
	int16_t dvsSizeX;
	/// DVS Y axis resolution.
	int16_t dvsSizeY;
	/// Feature test: DVS statistics support.
	bool dvsHasStatistics;
	/// IMU chip type on device.
	enum caer_dvs132s_imu_types imuType;
	/// Feature test: External Input module supports Signal-Generation.
	bool extInputHasGenerator;
};

/**
 * Return basic information on the device, such as its ID, its
 * resolution, the logic version, and so on. See the 'struct
 * caer_dvs132s_info' documentation for more details.
 *
 * @param handle a valid device handle.
 *
 * @return a copy of the device information structure if successful,
 *         an empty structure (all zeros) on failure.
 */
struct caer_dvs132s_info caerDVS132SInfoGet(caerDeviceHandle handle);

/**
 * On-chip simplified coarse-fine bias current configuration.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
struct caer_bias_coarsefine1024 {
	/// Coarse current, from 0 to 1023, creates big variations in output current.
	uint16_t coarseValue;
	/// Fine current, from 0 to 1023, creates small variations in output current.
	uint16_t fineValue;
};

/**
 * Transform simplified coarse-fine bias structure into internal integer representation,
 * suited for sending directly to the device via caerDeviceConfigSet().
 *
 * @param coarseFine1024Bias coarse-fine bias structure.
 *
 * @return internal integer representation for device configuration.
 */
uint32_t caerBiasCoarseFine1024Generate(struct caer_bias_coarsefine1024 coarseFine1024Bias);

/**
 * Transform internal integer representation, as received by calls to
 * caerDeviceConfigGet(), into a simplified coarse-fine bias structure, for easier
 * handling and understanding of the various parameters.
 *
 * @param coarseFine1024Bias internal integer representation from device.
 *
 * @return coarse-fine bias structure.
 */
struct caer_bias_coarsefine1024 caerBiasCoarseFine1024Parse(uint32_t coarseFine1024Bias);

/**
 * Transform current value in pico-Ampere to coarse-fine bias structure.
 * Limit is 1.0 micro-Ampere.
 *
 * @param picoAmps desired current value in pico-Ampere.
 *
 * @return coarse-fine bias structure.
 */
struct caer_bias_coarsefine1024 caerBiasCoarseFine1024FromCurrent(uint32_t picoAmps);

/**
 * Transform coarse-fine bias structure into corresponding current
 * value in pico-Ampere.
 *
 * @param coarseFine1024Bias coarse-fine bias structure.
 *
 * @return corresponding current value in pico-Ampere.
 */
uint32_t caerBiasCoarseFine1024ToCurrent(struct caer_bias_coarsefine1024 coarseFine1024Bias);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_DVS132S_H_ */
