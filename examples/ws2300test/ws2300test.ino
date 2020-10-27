/*
 *
 *	ws2300test.ino
 *
 *	example code illustrating usage of WS2300 class, retrieving measurement data from
 *	WS2300 Weather Station through serial interface.
 *
 *
 *	WS2300 class has been derived from rw2300.c/.h  implementation of "open2300" project
 *
 *	-https://www.heavyweather.info/
  *	-http://www.lavrsen.dk/foswiki/bin/view/Open2300/WebHome
 *	-https://github.com/wezm/open2300
 *
 *	those functions below tested with Teensy3.5 (https://www.pjrc.com/teensy/)
 *
 * */


#include "Arduino.h"

#include "WS2300.h"



const int rts = 3;
const int dtr = 2;



WS2300 ws2300(&Serial1, dtr, rts);

double timeToRunAll = 0;
double winddir;
char tend[7];
char forecast[7];
double rain1h_max;
struct WS2300::timestamp rain1h_time;
struct WS2300::timestamp utc;
uint8_t connType;
uint8_t nextReading;

void setup()
{
	Serial.begin(9600);
	ws2300.begin();
	delay(5000);


}


void loop()
{

	timeToRunAll = millis();
	Serial.print("Temp.i: "); Serial.println(ws2300.temperature_indoor());
	Serial.print("Temp.o: "); Serial.println(ws2300.temperature_outdoor());
	Serial.print("Dewp.: "); Serial.println(ws2300.dewpoint());
	Serial.print("Hum.i: "); Serial.println(ws2300.humidity_indoor());
	Serial.print("Hum.o: "); Serial.println(ws2300.humidity_outdoor());

	double w = ws2300.wind_current(&winddir);
	Serial.print("Windspeed: ");Serial.print(w);Serial.print(" Winddir.: ");Serial.println(winddir);

	Serial.print("Windchill: ");Serial.println(ws2300.windchill( ));
	Serial.print("Rain 1h: ");Serial.println(ws2300.rain_1h());
	Serial.print("Rain 1h all: ");Serial.println(ws2300.rain_1h_all(&rain1h_max, &rain1h_time));
	Serial.print("Rain 1h max: ");Serial.println(rain1h_max);
	Serial.print("Rain 1h max time: ");Serial.print(rain1h_time.hour);Serial.print(":");Serial.print(rain1h_time.minute);Serial.print(" ");Serial.print(rain1h_time.day);Serial.print("/");Serial.print(rain1h_time.month);Serial.print("/");Serial.println(rain1h_time.year);

	Serial.print("Rain 24h: ");Serial.println(ws2300.rain_24h());
	Serial.print("Rain total: ");Serial.println(ws2300.rain_total());
	Serial.print("pressure rel: ");Serial.println(ws2300.rel_pressure());
	Serial.print("pressure abs: ");Serial.println(ws2300.abs_pressure());
	ws2300.tendency_forecast( tend, forecast);
	Serial.print("tendency: ");Serial.println(tend);
	Serial.print("forcast: ");Serial.println(forecast);
	ws2300.utc(&utc);
	Serial.print("UTC: ");Serial.print(utc.hour);Serial.print(":");Serial.print(utc.minute);Serial.print(":");Serial.print(utc.second);Serial.print(" wday:");Serial.print(utc.wday);Serial.print(" date: ");Serial.print(utc.day);Serial.print("/");Serial.print(utc.month);Serial.print("/");Serial.println(utc.year);
	ws2300.sensorConnectStatus(&connType, &nextReading);
	Serial.print("Sensor Connection Type: ");Serial.println(connType);
	Serial.print("Sensor next Reading [s]: ");Serial.println(nextReading);
	Serial.println("----------------------------------------------------------------------------");
	Serial.print("time to run above measuremnts: ");Serial.println(millis() - timeToRunAll);
	Serial.println("----------------------------------------------------------------------------");

	delay(10000);

}

