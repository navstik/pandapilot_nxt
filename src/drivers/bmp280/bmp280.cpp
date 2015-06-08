/****************************************************************************
 *   Copyright (c) 2014 NavStik Development Team. All rights reserved.
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file bmp280.cpp
 * Driver for the BMP280 barometric pressure sensor connected via I2C.
 */

#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <board_config.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_baro.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct bmp280_data {
        uint8_t OSS ;
  	int16_t DELAY_PRESSURE ;
  	int16_t DELAY_TEMP ;
  	int16_t MEASUREMENT_MODE ;
  	uint16_t dig_T1 ;
  	int16_t dig_T2 ;
  	int16_t dig_T3 ;
  	uint16_t dig_P1 ;
  	int16_t dig_P2 ;
  	int16_t dig_P3 ;
  	int16_t dig_P4  ;
  	int16_t dig_P5  ;
  	int16_t dig_P6  ;
  	int16_t dig_P7  ;
  	int16_t dig_P8  ;
	int16_t dig_P9  ;
  	float PRESSURE ;
  	float TEMP ;
};

#pragma pack(pop)

class BMP280 : public device::I2C
{
public:
	BMP280(int bus);
	~BMP280();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:
	struct bmp280_data	_data;

	struct work_s		_work;
	signed			_measure_ticks;

	unsigned		_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	struct baro_report	*_reports;

	bool			_collect_phase;
	unsigned		_measure_phase;

	long unsigned int 	_t_fine;

	/* intermediate temperature values per BMP280 datasheet */
	int32_t			_TEMP;
	int64_t			_OFF;
	int64_t			_SENS;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in kPa */

	orb_advert_t		_baro_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int			probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Send a reset command to the BMP280.
	 *
	 * This is required after any bus reset.
	 */
	int			cmd_reset();

	/**
	 * Read the BMP280 PROM
	 *
	 * @return		OK if the PROM reads successfully.
	 */
	int			read_prom();

	/**
	 * PROM CRC routine ported from BMP280 application note
	 *
	 * @param n_prom	Pointer to words read from PROM.
	 * @return		True if the CRC matches.
	 */
	bool			crc4(uint16_t *n_prom);

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { _x++; if (_x >= _lim) _x = 0; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * BMP280 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
//#define BMP280_CONVERSION_INTERVAL	10000	/* microseconds */
#define BMP280_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

#define BMP280_BUS			NAVSTIK_I2C_BUS_SENSORS
#define BMP280_ADDRESS			NAVSTIK_I2C_OBDEV_BMP280_1 /* address select pins pulled high */

#define BMP280_CTL        		0xF4 // Measurement Control Register.
#define BMP280_ADC_PRES_MSB      	0xF7 // Read Only Register. Contains Output Data.
#define BMP280_ADC_PRES_LSB      	0xF8 // Read Only Register. Contains Output Data.
#define BMP280_ADC_PRES_XLSB      	0xF9 // Read Only Register. Contains Output Data.
#define BMP280_ADC_TEMP_MSB      	0xFA // Read Only Register. Contains Output Data.
#define BMP280_ADC_TEMP_LSB      	0xFB // Read Only Register. Contains Output Data.
#define BMP280_ADC_TEMP_XLSB      	0xFC // Read Only Register. Contains Output Data.
#define BMP280_ID        		0xD0 // Read Only Register. Value is 0x55 can be used to check communication.
#define BMP280_SOFT_RESET        	0xE0 // Write Only Register. If set to 0xB6 performs same sequence as power on reset

#define BMP280_TEMP        		0x2E // Value of Control Register for temperature measurment
#define BMP280_OSS0        		0x25 // Value of Control Register for Conversion time 4.5ms
#define BMP280_OSS1        		0x29 // Value of Control Register for Conversion time 7.5ms
#define BMP280_OSS2        		0x2D // Value of Control Register for Conversion time 13.5ms
#define BMP280_OSS3        		0x31 // Value of Control Register for Conversion time 25.5ms
#define BMP280_OSS4        		0x5D // Value of Control Register for Conversion time 25.5ms

#define BMP280_PRES        		0x34 // base value for pressure measurement (page 15 of datasheet)

#define ULTRA_LOW_POWER     		0
#define LOW_POWER     			1
#define STANDARD        		2
#define HIGH_RESOLUTION      		3
#define ULTRA_HIGH_RESOLUTION  		4

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp280_main(int argc, char *argv[]);


BMP280::BMP280(int bus) :
	I2C("BMP280", BARO0_DEVICE_PATH, bus, BMP280_ADDRESS, 400000),
	_measure_ticks(0),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	_TEMP(0),
	_OFF(0),
	_SENS(0),
	_msl_pressure(101325),
	_baro_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp280_read")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp280_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "bmp280_buffer_overflows"))
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

BMP280::~BMP280()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete[] _reports;
}

int
BMP280::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct baro_report[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the baro topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_baro_topic = orb_advertise(ORB_ID(sensor_baro), &_reports[0]);

	if (_baro_topic < 0)
		debug("failed to create sensor_baro object");

	ret = OK;
out:
	return ret;
}

int
BMP280::probe()
{
	_retries = 10;

	if (OK == probe_address(BMP280_ADDRESS)) {
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
BMP280::probe_address(uint8_t address)
{
	int	ret;
	uint8_t bmp_txbuf[2];

	/* select the address we are going to try */
	set_address(address);

	/* send reset command */
	if (OK != cmd_reset())
		return -EIO;

	/* read PROM */
	if (OK != read_prom())
		return -EIO;

	_data.DELAY_TEMP = 5000;
	_data.OSS = STANDARD;
	_data.MEASUREMENT_MODE = BMP280_OSS2;
	_data.DELAY_PRESSURE = 15000;

	bmp_txbuf[0] =  BMP280_CTL;
	bmp_txbuf[1] = _data.MEASUREMENT_MODE;
	ret = transfer(bmp_txbuf, 2, nullptr, 0);
	if (OK != ret)
		perf_count(_comms_errors);
	usleep(_data.DELAY_PRESSURE);

	return OK;
}

ssize_t
BMP280::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_oldest_report != _next_report) {
				memcpy(buffer, _reports + _oldest_report, sizeof(*_reports));
				ret += sizeof(_reports[0]);
				INCREMENT(_oldest_report, _num_reports);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_measure_phase = 0;
		_oldest_report = _next_report = 0;

		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(_data.DELAY_TEMP);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(_data.DELAY_PRESSURE);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		memcpy(buffer, _reports, sizeof(*_reports));
		ret = sizeof(*_reports);

	} while (0);

	return ret;
}

int
BMP280::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_data.DELAY_PRESSURE);
					warnx("\nmeasure ticks = %d",_measure_ticks);
					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					signed ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_data.DELAY_PRESSURE))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;
					warnx("\nmeasure ticks = %d",_measure_ticks);
					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* add one to account for the sentinel in the ring */
			arg++;

			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 2) || (arg > 100))
				return -EINVAL;

			/* allocate new buffer */
			struct baro_report *buf = new struct baro_report[arg];

			if (nullptr == buf)
				return -ENOMEM;

			/* reset the measurement state machine with the new buffer, free the old */
			stop();
			delete[] _reports;
			_num_reports = arg;
			_reports = buf;
			start();

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _num_reports - 1;

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000))
			return -EINVAL;

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the superclass */
	return I2C::ioctl(filp, cmd, arg);
}

void
BMP280::start()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BMP280::cycle_trampoline, this, 1);
}

void
BMP280::stop()
{
	work_cancel(HPWORK, &_work);
}

void
BMP280::cycle_trampoline(void *arg)
{
	BMP280 *dev = (BMP280 *)arg;

	dev->cycle();
}

void
BMP280::cycle()
{

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* reset the collection state machine and try again */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_ticks > USEC2TICK(_data.DELAY_PRESSURE))) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&BMP280::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(_data.DELAY_PRESSURE));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		/* 
		 * We failed to send the I2C command to start the next
		 * reading. Hopefully this is a transient bus
		 * error. Schedule a fresh cycle call to try the
		 * command again in one tick
		 */		
		log("measure error");
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&BMP280::cycle_trampoline,
			   this, 1);
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&BMP280::cycle_trampoline,
		   this,
		   USEC2TICK(_data.DELAY_PRESSURE));
}

int
BMP280::measure()
{
	int ret;

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	//uint8_t	cmd_data = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;
	uint8_t bmp_txbuf[2];

	bmp_txbuf[0] = BMP280_CTL;
	bmp_txbuf[1] = _data.MEASUREMENT_MODE;

	/*
	 * Send the command to begin measuring.
	 */
	ret = transfer(bmp_txbuf, 2, nullptr, 0);

	if (OK != ret)
		perf_count(_comms_errors);

	return ret;
}

int
BMP280::collect()
{
  	uint8_t bmp_txbuf[1], bmp_rxbuf[6] ;
	
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	_reports[_next_report].timestamp = hrt_absolute_time();

//	if (OK != transfer(&cmd, 1, &data[0], 3)) {
//		perf_count(_comms_errors);
//		return -EIO;
//   	}

    	bmp_txbuf[0] = BMP280_ADC_PRES_MSB ;
	/* handle a measurement */
	double var1,var2,var3,var4,temperature,pressure;
	uint32_t up, ut ;

	if (OK != transfer(bmp_txbuf, 1, bmp_rxbuf, 6)) {
              perf_count(_comms_errors);
              return -EIO;
	}
	
	up = (bmp_rxbuf[0] << 12) + (bmp_rxbuf[1] << 4) + (bmp_rxbuf[2] >> 4) ;		
	ut = (bmp_rxbuf[3] << 12) + (bmp_rxbuf[4] << 4) + (bmp_rxbuf[5] >> 4) ;

	/* calculate var1*/
	var1  = (((double)ut) / 16384.0 - ((double)_data.dig_T1) / 1024.0) * ((double)_data.dig_T2);
	/* calculate var2*/
	var2  = ((((double)ut) / 131072.0 - ((double)_data.dig_T1) / 8192.0) * (((double)ut) / 131072.0 - ((double)_data.dig_T1) / 8192.0)) * ((double)_data.dig_T3);
	/* calculate t_fine*/
	_t_fine = (long unsigned int)(var1 + var2);
	/* calculate true temparature*/
	temperature  = (var1 + var2) / 5120.0;
	_data.TEMP = (float) temperature;
	_TEMP = (int32_t) _data.TEMP;

	
	var3 = ((double)_t_fine/2.0) - 64000.0;
	var4 = var3 * var3 * ((double)_data.dig_P6) / 32768.0;
	var4 = var4 + var3 * ((double)_data.dig_P5) * 2.0;
	var4 = (var4 / 4.0) + (((double)_data.dig_P4) * 65536.0);
	var3 = (((double)_data.dig_P3) * var3 * var3 / 524288.0 + ((double)_data.dig_P2) * var3) / 524288.0;
	var3 = (1.0 + var3 / 32768.0) * ((double)_data.dig_P1);
	pressure = 1048576.0 - (double)up;
	/* Avoid exception caused by division by zero */
	if ((int)var3 != 0)
		pressure = (pressure - (var4 / 4096.0)) * 6250.0 / var3;
	else
		return -EIO;
	var3 = ((double)_data.dig_P9) * pressure * pressure / 2147483648.0;
	var4 = pressure * ((double)_data.dig_P8) / 32768.0;
	pressure = pressure + (var3 + var4 + ((double)_data.dig_P7)) / 16.0;
	_data.PRESSURE = (float)pressure;


	/* pressure calculation, result in Pa */
	int32_t P = _data.PRESSURE;

	/* generate a new report */
	_reports[_next_report].temperature = (float) temperature;
	_reports[_next_report].pressure = P / 100.0f;		/* convert to millibar */

	/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

	/*
	 * PERFORMANCE HINT:
	 *
	 * The single precision calculation is 50 microseconds faster than the double
	 * precision variant. It is however not obvious if double precision is required.
	 * Pending more inspection and tests, we'll leave the double precision variant active.
	 *
	 * Measurements:
	 * 	double precision: bmp280_read: 992 events, 258641us elapsed, min 202us max 305us
	 *	single precision: bmp280_read: 963 events, 208066us elapsed, min 202us max 241us
	 */
#if 0/* USE_FLOAT */
	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0f + 273.15f;	/* temperature at base height in Kelvin */
	const float a  = -6.5f / 1000f;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	float p1 = _msl_pressure / 1000.0f;

	/* measured pressure in kPa */
	float p = P / 1000.0f;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	_reports[_next_report].altitude = (((powf((p / p1), (-(a * R) / g))) * T1) - T1) / a;
#else
	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = _msl_pressure / 1000.0;

	/* measured pressure in kPa */
	double pp = P / 1000.0;

	/*
	 * Solve:
	 *
 *     /        -(aR / g)      \
 *    | (pp / p1)          . T1 | - T1
 *     \                       /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	_reports[_next_report].altitude = (((pow((pp / p1), (-(a * R) / g))) * T1) - T1) / a;
#endif
	/* publish it */
	orb_publish(ORB_ID(sensor_baro), _baro_topic, &_reports[_next_report]);

	/* post a report to the ring - note, not locked */
	INCREMENT(_next_report, _num_reports);

	/* if we are running up against the oldest report, toss it */
	if (_next_report == _oldest_report) {
		perf_count(_buffer_overflows);
		INCREMENT(_oldest_report, _num_reports);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);
	

	/* update the measurement state machine */
	INCREMENT(_measure_phase, BMP280_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

int
BMP280::cmd_reset()
{
	unsigned	old_retrycount = _retries;
	uint8_t		bmp_txbuf[2];
	int		result;
	
	bmp_txbuf[0] = BMP280_SOFT_RESET; 
	bmp_txbuf[1] = 0xB6;

	/* bump the retry count */
	_retries = 10;
	result = transfer(bmp_txbuf, 2, nullptr, 0);
	_retries = old_retrycount;

	return result;
}

int
BMP280::read_prom()
{

	uint8_t bmp_txbuf[1], bmp_rxbuf[24] ;

	usleep(10000);
  
	bmp_txbuf[0] = 0x88 ;

	if (OK == transfer(bmp_txbuf, 1, bmp_rxbuf, 24)) {
		_data.dig_T1 = (bmp_rxbuf[1] << 8) + bmp_rxbuf[0] ;    /*dig_T1 */
   		_data.dig_T2 = (bmp_rxbuf[3] << 8) + bmp_rxbuf[2] ;    /*dig_T2 */
  		_data.dig_T3 = (bmp_rxbuf[5] << 8) + bmp_rxbuf[4] ;    /*dig_T3 */
  		_data.dig_P1 = (bmp_rxbuf[7] << 8) + bmp_rxbuf[6] ;    /*dig_P1 */
  		_data.dig_P2 = (bmp_rxbuf[9] << 8) + bmp_rxbuf[8] ;    /*dig_P2 */
  		_data.dig_P3 = (bmp_rxbuf[11] << 8) + bmp_rxbuf[10] ;    /*dig_P3 */
  		_data.dig_P4  = (bmp_rxbuf[13] << 8) + bmp_rxbuf[12] ;    /*dig_P4 */
  		_data.dig_P5  = (bmp_rxbuf[15] << 8) + bmp_rxbuf[14] ;    /*dig_P5 */
  		_data.dig_P6  = (bmp_rxbuf[17] << 8) + bmp_rxbuf[16] ;    /*dig_P6 */
  		_data.dig_P7  = (bmp_rxbuf[19] << 8) + bmp_rxbuf[18] ;    /*dig_P7 */
  		_data.dig_P8  = (bmp_rxbuf[21] << 8) + bmp_rxbuf[20] ;    /*dig_P8 */
		_data.dig_P9  = (bmp_rxbuf[23] << 8) + bmp_rxbuf[22] ;    /*dig_P9 */
	}
	return OK;
}

void
BMP280::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("report queue:   %u (%u/%u @ %p)\n",
	       _num_reports, _oldest_report, _next_report, _reports);
	printf("TEMP:           %d\n", _TEMP);
	printf("SENS:           %lld\n", _SENS);
	printf("OFF:            %lld\n", _OFF);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));
}

/**
 * Local functions in support of the shell command.
 */
namespace bmp280
{

BMP280	*g_dev;

void	start();
void	test();
void	reset();
void	info();
void	calibrate(unsigned altitude);

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new BMP280(BMP280_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(BARO0_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct baro_report report;
	ssize_t sz;
	int ret;

	int fd = open(BARO0_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmp280 start' if the driver is not running)", BARO0_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.pressure);
	warnx("altitude:    %11.4f", (double)report.altitude);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", report.timestamp);

	/* set the queue depth to 10 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 15; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("pressure:    %10.4f", (double)report.pressure);
		warnx("altitude:    %11.4f", (double)report.altitude);
		warnx("temperature: %8.4f", (double)report.temperature);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(BARO0_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(unsigned altitude)
{
	struct baro_report report;
	float	pressure;
	float	p1;

	int fd = open(BARO0_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmp280 start' if the driver is not running)", BARO0_DEVICE_PATH);

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX))
		errx(1, "failed to set poll rate");

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "sensor read failed");

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK)
		err(1, "BAROIOCSMSLPRESSURE");

	exit(0);
}

} // namespace

int
bmp280_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		bmp280::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		bmp280::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		bmp280::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		bmp280::info();

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(argv[1], "calibrate")) {
		if (argc < 2)
			errx(1, "missing altitude");

		long altitude = strtol(argv[2], nullptr, 10);

		bmp280::calibrate(altitude);
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
