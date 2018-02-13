// The MIT License (MIT)
//
// Copyright (c) 2017 THINK BIG LABS SL
// Author: jorge.trincadoc@gmail.com (Jorge Trincado Castán)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CLIMA_STICK_V1_H
#define CLIMA_STICK_V1_H

#ifndef _DISABLE_BATTERY_SW_PROTECTION_
#define _BATTERY_SW_PROTECTION_
#endif

#include <ESP8266WiFi.h>		//GENERIC ESP8266
#include <ThingerESP8266.h>		//THINGER.IO IOT PLATFORM
#include <Wire.h>				//I2C BUS
#include <WiFiUdp.h>            // Wifi UDP for NTP

#include "Adafruit_Sensor.hpp"
#include "HMC5883L.hpp"			//MAGNETOMETER
#include "MPU6050.hpp" 			//ACELEROMETER & GYRO
#include "Adafruit_BME280.hpp"	//ENVIRONEMENT SENSOR
#include "TSL2561.h"			//LUMINOSITY SENSOR
#include "NTPClient.hpp"        //NTP CLIENT


//battery parameters and variables

#define BATTERY_VOLTAGE_GAUGE 154		 //  x = analogRead(A0)/5
#define BATTERY_PERCENT_GAUGE 142.85 	 //
#define BATTERY_MIN_VOLTAGE 3.5
#define BATTERY_SIZE 500
#define BATTERY_CARGING_INTENSITY 500
int last_battery_actualization;
float last_rb,initial_bat;


// structs for different return types
typedef struct accelerometer {int16_t ax, ay, az;} Accelerometer;
typedef struct gyroscope {int16_t gx, gy, gz;} Gyroscope;
typedef struct motion {int16_t ax, ay, az, gx, gy, gz;} Motion;
typedef struct compass {float heading, headingDegrees;} Compass;
typedef struct magnetometer {float x, y, z;} Magnetometer;
typedef struct environmental {float temperature, humidity, pressure, altitude; uint32_t lux;} Environmental;
typedef struct time{int hour, minute, second;} Time;
typedef struct rgb{int r, g, b;} RGB;

class ClimaStick : public ThingerESP8266{

public:
	// rgb led pins
    #define BUILTIN_LED 13
    static const uint8_t RED = 13;
    static const uint8_t GREEN = 12;
    static const uint8_t BLUE = 14;
    static const uint8_t R = 13;
    static const uint8_t G = 12;
    static const uint8_t B = 14;

	// battery voltage to ADC pin, 400K pullup 100K pulldown voltage divider
    static const uint8_t BAT   = 0;

	// user button to GPIO_0
    static const uint8_t BUTTON = 0;

	// GPIO_16 is connected to RESET circuitry
    static const uint8_t WUP   = 16;
    static const uint8_t WAKEUP   = 16;

	// reference pressure
    static constexpr double SEALEVELPRESSURE_HPA = 1013.25;

protected:

	// sensor instances
 	HMC5883L compass;
  	MPU6050 mpu;
  	Adafruit_BME280 bme; 
  	TSL2561 tsl = TSL2561(TSL2561_ADDR_FLOAT); 

 	// other instances
 	WiFiUDP ntpUDP;
	NTPClient timeClient;

	// current rgb led state
	int r=0, g=0, b=0;

    // self reference
    static ClimaStick* clima_;

public:

    ClimaStick(const char* username, const char* device, const char* password) :
       //tsl(TSL2561_ADDR_FLOAT),
		timeClient(ntpUDP),
		ThingerESP8266(username, device, password)
	{
        clima_ = this;
    }

	// default constructor (not using thinger)
    ClimaStick() :
       // tsl(TSL2561_ADDR_FLOAT),
        timeClient(ntpUDP),
        ThingerESP8266("", "", "")
	{
        clima_ = this;
    }

    static ClimaStick& get(){
        return *clima_;
    };

	/*
	*	SENSORS FUNCTIONS
	*/

	// initialize all sensors: I2C handshake
	bool init_sensors(){
        #ifdef _DEBUG_
                Serial.begin(115200);
                    Serial.println();
        #endif

                if(my_init_sensors()){
                    return true;
                }else{
        #ifdef _DEBUG_
                    Serial.println("I2C failsafe reset");
        #endif  //if anyting fails, system will make an I2C remair routine
                    Wire.status();
                    return my_init_sensors();
                }
	}

    bool my_init_sensors(){

        Wire.begin();

            /*initializing sensors*/
            bool mpu_begin_ok = mpu.testConnection();
    #ifdef _DEBUG_
            Serial.println(mpu_begin_ok ? "MPU6050 connection successful" : "MPU6050 connection failed");
    #endif
            if(mpu_begin_ok){
                mpu.initialize();
                mpu.setI2CMasterModeEnabled(false);
                mpu.setI2CBypassEnabled(true) ;
                mpu.setSleepEnabled(false);
                bme.begin();
            }


            bool bme_begin_ok = bme.begin();
    #ifdef _DEBUG_
            Serial.println(bme_begin_ok ? "BME280 connection successful" : "BME280 connection failed");
    #endif

            bool tsl_begin_ok = tsl.begin();
    #ifdef _DEBUG_
            Serial.println(tsl_begin_ok ? "TSL2561 connection successful" : "TSL connection failed");
    #endif
            if(tsl_begin_ok){
                tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
                tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
            }

            bool HMC_begin_ok = compass.begin();
    #ifdef _DEBUG_
            Serial.println(HMC_begin_ok ? "HMC5883L connection successful" : "HMC5883L connection failed");
    #endif
            if (HMC_begin_ok){
                compass.setRange(HMC5883L_RANGE_1_3GA);         // Set measurement range
                compass.setMeasurementMode(HMC5883L_CONTINOUS); // Set measurement mode
                compass.setDataRate(HMC5883L_DATARATE_30HZ);    // Set data rate
                compass.setSamples(HMC5883L_SAMPLES_8);         // Set number of samples averaged
                compass.setOffset(0, 0);                        // Set calibration offset. See HMC5883L_calibration.ino
            }

        return(mpu_begin_ok && bme_begin_ok && tsl_begin_ok && HMC_begin_ok);

    }


	// get motion (accelerometer and gyroscope measures)
	Motion get_motion(){
        Motion motion;
	  	mpu.getMotion6(&motion.ax, &motion.ay, &motion.az, &motion.gx, &motion.gy, &motion.gz);
		return motion;
	}

	// get gyroscope
	Gyroscope get_gyroscope(){
        Gyroscope gyro;
		mpu.getRotation(&gyro.gx, &gyro.gy, &gyro.gz);
		return gyro;
	}

	// get accelerometer and gyroscope measures
	Accelerometer get_acceleration(){
        Accelerometer accel;
		mpu.getAcceleration(&accel.ax, &accel.ay, &accel.az);
		return accel;
	}
 
	// get compass measure
	Compass get_compass(){
        Compass mCompass;
	  	Vector norm = compass.readNormalize();
        mCompass.heading = atan2(norm.YAxis, norm.XAxis); // Calculate heading

		// Set declination angle on your location and fix heading
		// You can find  your declination on: http://magnetic-declination.com/
		float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI); // Formula: (deg + (min / 60.0)) / (180 / M_PI);
        mCompass.heading += declinationAngle;

		// Correct for heading < 0deg and heading > 360deg
		if (mCompass.heading < 0)   mCompass.heading += 2 * PI;
		if (mCompass.heading > 2 * PI)  mCompass.heading -= 2 * PI;
        mCompass.headingDegrees = mCompass.heading * 180/M_PI; // Convert to degrees
	
		return mCompass;
	}
	 
	// get magnetometer measure
	Magnetometer get_magnetometer(){
        Magnetometer magnet;
		Vector norm;
		norm = compass.readNormalize();
		magnet.x = norm.XAxis;
		magnet.y = norm.YAxis;
		magnet.z = norm.ZAxis;
		return magnet;
	}

    // get magnetometer measure
    Magnetometer get_raw_magnetometer(){
        Magnetometer magnet;
        Vector raw;
        raw = compass.readRaw();
        magnet.x = raw.XAxis;
        magnet.y = raw.YAxis;
        magnet.x = raw.ZAxis;
        return magnet;
    }
	 
	// get clima
	Environmental get_clima(){
        Environmental clima;
 		clima.temperature = bme.readTemperature();
		clima.humidity = bme.readHumidity();
		clima.pressure = bme.readPressure() / 100.0F;
		clima.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        	clima.lux = tsl.getLuminosity(TSL2561_VISIBLE);

		 
		return clima;
	}

	// get temperature
	float get_temperature() {
		return bme.readTemperature();
	}

	// get humidity
	float get_humidity() {
		return bme.readHumidity();
	}

	// get pressure
	float get_pressure() {
		return bme.readPressure();
	}

	// get altitude
	float get_altitude() {
		return bme.readAltitude(SEALEVELPRESSURE_HPA);
	}

	// get visible spectrum
	uint16_t get_visible_spectrum() {
        return tsl.getLuminosity(TSL2561_VISIBLE);
	}

	// get full spectrum
	uint16_t get_full_spectrum() {
        return tsl.getLuminosity(TSL2561_FULLSPECTRUM);
    }

	// get infrared spectrum
	uint16_t get_ir_spectrum() {

	    return tsl.getLuminosity(TSL2561_INFRARED);

	 }

	// get lux (visible)
	uint16_t get_lux() {

			return tsl.getLuminosity(TSL2561_VISIBLE);

	}

	// get full luminosity
	uint32_t get_full_luminosity() {
		return tsl.getFullLuminosity();
	}

	// get current battery voltage (volts)
	float get_battery_voltage(){ // read the ADC value and calculate the real time voltage
		return (float)0.792683*((float)analogRead(BAT)/BATTERY_VOLTAGE_GAUGE)+1.03659;          //This function will linealize the voltage analog read
	}

	// get the remain battery load
	float get_battery_load(){
		float remain_battery=0;
		float volt=get_battery_voltage();

	#ifdef _BATTERY_SW_PROTECTION_
		if(volt<=BATTERY_MIN_VOLTAGE) {
			for(int t=0;t<3;t++){
				digitalWrite(13,1);
				delay(70);
				digitalWrite(13,0);
				delay(70);
			}
			 ESP.deepSleep(0);                                                                  // forced deepsleep if battery breaks the voltage range
		}
    #endif

		if(volt<4.2){			                                                                //will enter here if battery isn't charging
			volt-=BATTERY_MIN_VOLTAGE;
			remain_battery = volt*BATTERY_PERCENT_GAUGE;

		}else if(volt>=4.2 && last_rb <100) {	                                                //leave a 0.1V hysteresis
																	                            //will enter here if it is detected a battery and it is being charged
			if(initial_bat==0)initial_bat = last_rb;
			else{
				if(millis()>last_battery_actualization + 60*1000){                               //this modification wil create a virtual battery load counter. JT 300417
					float increaser = BATTERY_SIZE/(float)(BATTERY_CARGING_INTENSITY/60);		//this value depends of the battery charging configuration circuitry
					initial_bat+=increaser;
					initial_bat+=1.6;
					initial_bat>100 ? initial_bat=100:initial_bat=initial_bat;
					last_battery_actualization=millis();
				}
			}

			remain_battery = initial_bat;
		}else{
			remain_battery= 100;
		}
		last_rb = remain_battery;

		return remain_battery;
	}

	/*
	*	NTP client refresh: fill Hour, minute, second and date variables.
	*/

	// get time in hours, minutes and seconds
	Time get_time(){
        Time time;
		time.hour = timeClient.getHours();
		time.minute = timeClient.getMinutes();
		time.second = timeClient.getSeconds();
		return time;
	}

	// get current timestamp
	unsigned long get_timestamp(){
		return timeClient.getEpochTime();
	}

	// sync NTP time
	bool ntp_update(){
		if(WiFi.status() == WL_CONNECTED){
			return timeClient.update();
		}
		return false;
	}
 
	/*
	*	BUILTIN RGB LED FUNCTIONS
	*/

	// set current rgb color in led
	void set_rgb(int r, int g, int b){
		this->r = r;
		this->g = g;
		this->b = b;
		analogWrite(R, r);
		analogWrite(G, g);
		analogWrite(B, b);
	}

	// set current rgb color in led
	void set_rgb(const RGB& rgb){
		set_rgb(rgb.r, rgb.g, rgb.b);
	}

	// set custom RGB color based on name
	void set_rgb(const String& colorName){
		if(colorName.equalsIgnoreCase("BLUE"))			set_rgb(0,0,200);
		else if(colorName.equalsIgnoreCase("GREEN"))  	set_rgb(0,150,0);
		else if(colorName.equalsIgnoreCase("YELLOW")) 	set_rgb(50,150,0);
		else if(colorName.equalsIgnoreCase("ORANGE")) 	set_rgb(100,150,0);
		else if(colorName.equalsIgnoreCase("RED"))  	set_rgb(150,30,0);
		else if(colorName.equalsIgnoreCase("PURPLE")) 	set_rgb(200,0,50);
		else if(colorName.equalsIgnoreCase("WHITE"))  	set_rgb(50,200,200);
	}

	// set current rgb color in led
	RGB get_rgb(){
        RGB rgb;
		rgb.r = r;
		rgb.g = g;
		rgb.b = b;
		return rgb;
	}

    // initialize button resource
    void init_button_resource(){
        (*this)["button"] >> invertedDigitalPin(ClimaStick::BUTTON);
    }

    // initialize battery resource
    void init_battery_resource(){
        (*this)["battery"] >> [](pson &out) {
            out["voltage"] =  ClimaStick::get().get_battery_voltage();
            out["load"] = ClimaStick::get().get_battery_load();
        };
    }

    // initialize accelerometer resource
    void init_accelerometger_resource(){
        (*this)["accelerometer"] >> [](pson &out) {
            Accelerometer accel = ClimaStick::get().get_acceleration();
            out["ax"] = accel.ax;
            out["ay"] = accel.ay;
            out["az"] = accel.az;
        };
    }

    // initialize gyroscope resource
    void init_gyroscope_resource(){
        (*this)["gyroscope"] >> [](pson &out) {
            Gyroscope gyro = ClimaStick::get().get_gyroscope();
            out["gx"] = gyro.gx;
            out["gy"] = gyro.gy;
            out["gz"] = gyro.gz;
        };
    }

    // initialize compass resource
    void init_compass_resource(){
        (*this)["compass"] >> [](pson &out) {
            Compass compass = ClimaStick::get().get_compass();
            out["heading"] = compass.heading;
            out["degHeading"] = compass.headingDegrees;
        };
    }

    // initialize magnetometer resource
    void init_magnetometer_resource(){
        (*this)["magnetometer"] >> [](pson &out) {
            Magnetometer magnet = ClimaStick::get().get_magnetometer();
            out["x"] = magnet.x;
            out["y"] = magnet.y;
            out["z"] = magnet.z;
        };
    }

    // initialize environment resource
    void init_environment_resource(){
        (*this)["environment"] >> [](pson &out) {
            Environmental clima = ClimaStick::get().get_clima();
            out["temperature"] = clima.temperature;
            out["humidity"] = clima.humidity;
            out["altitude"] = clima.altitude;
            out["pressure"] = clima.pressure;
            out["lux"] = clima.lux;
        };
    }

    // initialize rgb resources
    void init_rgb_resources(){
        (*this)["led_rgb"] << [](pson &in) {
            if(in.is_empty()){
                RGB rgb =  ClimaStick::get().get_rgb();
                in["r"] = rgb.r;
                in["g"] = rgb.g;
                in["b"] = rgb.b;
            }else{
                ClimaStick::get().set_rgb(in["r"], in["g"], in["b"]);
            }
        };

        (*this)["led_color"] << [](pson &in) {
            String color = in["ColorName"];
            ClimaStick::get().set_rgb(color);
        };
    }

    // helper function to initialize all resources
	void init_resources() {
        init_button_resource();
		init_battery_resource();
        init_accelerometger_resource();
        init_gyroscope_resource();
		init_compass_resource();
        init_magnetometer_resource();
        init_environment_resource();
        init_rgb_resources();
	}

	void sleep(unsigned long seconds){
        sleep_sensors();        //added by JT to enabled low power consumiption 300417
		ESP.deepSleep(seconds * 1000000);
	}

    void sleep_sensors(){ //added by JT to enabled low power consumption 300417
        tsl.disable();
        mpu.setSleepEnabled(true);
        compass.setMeasurementMode(HMC5883L_IDLE);
        bme.sleep();
    }


};

ClimaStick* ClimaStick::clima_ = NULL;


#endif

