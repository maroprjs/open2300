/*
 * WS2300.h
 *
 *  Created on: 23 Oct 2020
 *      Author: maro
 *
 *      WS2300 class has been derived from rw2300.c/.h  implementation of "open2300" project,
 *      focusing on Arduino type of boards
 *      -not all functions tested!!
 *
 */

#ifndef WS2300_H_
#define WS2300_H_
#include <Arduino.h>

#define BAUDRATE	(2400)
#define MAXRETRIES	(5)
#define MAXRESET_06	(4)
#define MAXWINDRETRIES	(1)

#define RESET_MIN           0x01
#define RESET_MAX           0x02
#define WRITENIB            0x42
#define SETBIT              0x12
#define UNSETBIT            0x32
#define WRITEACK            0x10
#define SETACK              0x04
#define UNSETACK            0x0C

#define METERS_PER_SECOND   1.0
#define KILOMETERS_PER_HOUR 3.6
#define MILES_PER_HOUR      2.23693629
#define KNOTS				1.94384
#define CELCIUS             0
#define FAHRENHEIT          1
#define MILLIMETERS         1
#define INCHES              25.4
#define HECTOPASCAL         1.0
#define MILLIBARS           1.0
#define INCHES_HG           33.8638864

class WS2300 {


	struct config_type //TODO: delete this
	{
		double wind_speed_conv_factor;     //from m/s to km/h or miles/hour
		int    temperature_conv;           //0=Celcius, 1=Fahrenheit
		double rain_conv_factor;           //from mm to inch
		double pressure_conv_factor;       //from hPa (=millibar) to mmHg

	};

public:
	struct timestamp
	{
		uint8_t second;
		uint8_t minute;
		uint8_t hour;
		uint8_t wday;
		uint8_t day;
		uint8_t month;
		uint8_t year;
	};
	//equals tmElements_t from TimeLib

public:
	WS2300(HardwareSerial *port, int dtr, int rts = 255);//if rts TTL side is put to ground in Arduino Environment
	virtual ~WS2300();
	void begin();
	void loop();
	void utc(struct timestamp *utc);
	void sensorConnectStatus(uint8_t *connType, uint8_t *nextReading);
	double temperature_indoor( int temperature_conv = CELCIUS);
	void temperature_indoor_minmax(int temperature_conv,
	                               double *temp_min,
	                               double *temp_max,
	                               struct timestamp *time_min,
	                               struct timestamp *time_max);
	int temperature_indoor_reset(  char minmax);
	double temperature_outdoor(int temperature_conv = CELCIUS);
	void temperature_outdoor_minmax(int temperature_conv,
	                                double *temp_min,
	                                double *temp_max,
	                                struct timestamp *time_min,
	                                struct timestamp *time_max);
	int temperature_outdoor_reset(  char minmax);
	double dewpoint(  int temperature_conv = CELCIUS);
	void dewpoint_minmax(int temperature_conv,
	                     double *dp_min,
	                     double *dp_max,
	                     struct timestamp *time_min,
	                     struct timestamp *time_max);
	int dewpoint_reset(char minmax);
	int humidity_indoor();
	int humidity_indoor_all(int *hum_min,
	                        int *hum_max,
	                        struct timestamp *time_min,
	                        struct timestamp *time_max);
	int humidity_indoor_reset( char minmax);
	int humidity_outdoor();
	int humidity_outdoor_all(int *hum_min,
	                         int *hum_max,
	                         struct timestamp *time_min,
	                         struct timestamp *time_max);
	int humidity_outdoor_reset(char minmax);
	double wind_current(double *winddir,
						double wind_speed_conv_factor = KNOTS);
	double wind_all(double wind_speed_conv_factor,
	                int *winddir_index,
	                double *winddir);
	double wind_minmax(double wind_speed_conv_factor,
	                   double *wind_min,
	                   double *wind_max,
	                   struct timestamp *time_min,
	                   struct timestamp *time_max);
	int wind_reset( char minmax);
	double windchill( int temperature_conv = CELCIUS);
	void windchill_minmax(int temperature_conv,
	                      double *wc_min,
	                      double *wc_max,
	                      struct timestamp *time_min,
	                      struct timestamp *time_max);
	int windchill_reset( char minmax);
	double rain_1h( double rain_conv_factor = MILLIMETERS);
	double rain_1h_all(double *rain_max,
	                   struct timestamp *time_max,
					   double rain_conv_factor = MILLIMETERS);
	int rain_1h_max_reset();
	int rain_1h_reset();
	double rain_24h( double rain_conv_factor = MILLIMETERS);
	double rain_24h_all(double *rain_max,
	                   struct timestamp *time_max,
	                   double rain_conv_factor = MILLIMETERS);
	int rain_24h_max_reset();
	int rain_24h_reset();
	double rain_total( double rain_conv_factor = MILLIMETERS);
	double rain_total_all(double rain_conv_factor,
	                   struct timestamp *time_since);
	int rain_total_reset();
	double rel_pressure( double pressure_conv_factor = HECTOPASCAL);
	void rel_pressure_minmax(double pressure_conv_factor,
	                         double *pres_min,
	                         double *pres_max,
	                         struct timestamp *time_min,
	                         struct timestamp *time_max);
	double abs_pressure( double pressure_conv_factor = HECTOPASCAL );
	void abs_pressure_minmax(double pressure_conv_factor,
	                         double *pres_min,
	                         double *pres_max,
	                         struct timestamp *time_min,
	                         struct timestamp *time_max);
	int pressure_reset( char minmax);
	double pressure_correction(  double pressure_conv_factor);
	void tendency_forecast( char *tendency, char *forecast);
	int read_history_info( int *interval, int *countdown,
	                 struct timestamp *time_last, int *no_records);
	int read_history_record(int record,
	                        struct config_type *config,
	                        double *temperature_indoor,
	                        double *temperature_outdoor,
	                        double *pressure,
	                        int *humidity_indoor,
	                        int *humidity_outdoor,
	                        double *raincount,
	                        double *windspeed,
	                        double *winddir_degrees,
	                        double *dewpoint,
	                        double *windchill);
	void light( int control);



private:
	int read_safe(int address, int number, unsigned char *readdata,
																		unsigned char *commanddata);
	int write_safe(int address, int number,
	               unsigned char encode_constant, unsigned char *writedata,
	               unsigned char *commanddata);
	void read_error_exit(void);
	void write_error_exit(void);
	int read_data(int address, int number, unsigned char *readdata,
																		unsigned char *commanddata);
	int write_data(int address, int number,
				   unsigned char encode_constant, unsigned char *writedata,
				   unsigned char *commanddata);
	void address_encoder(int address_in, unsigned char *address_out);
	void data_encoder(int number, unsigned char encode_constant,
	                  unsigned char *data_in, unsigned char *data_out);
	unsigned char numberof_encoder(int number);
	unsigned char command_check0123(unsigned char *command, int sequence);
	unsigned char command_check4(int number);
	unsigned char data_checksum(unsigned char *data, int number);

	void reset_06();
	int write_device(unsigned char *buffer, int size);
	int read_device( unsigned char *buffer, int size);
	void sleep_long(int seconds);
	void load_defaultConfig();

	HardwareSerial *_serialPort;
	int _dtrPin;
	int _rtsPin;
	struct config_type *config;



};

#endif /* WS2300_H_ */
