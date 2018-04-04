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

#ifndef CLIMA_STICK_V2_H
#define CLIMA_STICK_V2_H

#ifndef _DISABLE_BATTERY_SW_PROTECTION_
#define _BATTERY_SW_PROTECTION_
#endif

#include <ESP8266WiFi.h>		//GENERIC ESP8266
#include <ThingerESP8266.h>		//THINGER.IO IOT PLATFORM
#include <Wire.h>				//I2C BUS
#include <WiFiUdp.h>            // Wifi UDP for NTP
 
#include "Adafruit_Sensor.hpp"
  
#include "MPU9250.h"			//ACELEROMETER GYRO &MAGNETOMETER
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
typedef struct magnetometer {float mx, my, mz;} Magnetometer;
typedef struct environmental {float temperature, humidity, pressure, altitude; uint32_t lux;} Environmental;
typedef struct time{int hour, minute, second, day;} Time;
 
class ClimaStick : public ThingerESP8266{

public:
	//   led pins
	#define BUILTIN_LED 13
	#define LED_BUILTIN 13
	#define BuiltinLed 13
	 
    static const uint8_t redLed = 13;
    static const uint8_t blueLed = 1;

	// battery voltage to ADC pin, 400K pullup 100K pulldown voltage divider
    static const uint8_t BAT   = 0;

	// user button to GPIO_0
    static const uint8_t BUTTON = 0;

    // Expansion ports
    static const uint8_t P1 = 12;
    //static const uint8_t A0 = ADC;

	// GPIO_16 is connected to RESET circuitry
    static const uint8_t WUP   = 16;
    static const uint8_t WAKEUP   = 16;

	// reference pressure
    static constexpr double SEALEVELPRESSURE_HPA = 1013.25;

protected:

	// sensor instances
	MPU9250 mpu;
	Adafruit_BME280 bme; 
  	TSL2561 tsl = TSL2561(0x49);

 	// other instances
 	WiFiUDP ntpUDP;
	NTPClient timeClient;

	// current rgb led state
	int r=0, b=0;

    // self reference
    static ClimaStick* clima_;

public:

    ClimaStick(const char* username, const char* device, const char* password) :
          
        //tsl(0x49),
      
		timeClient(ntpUDP),
		ThingerESP8266(username, device, password)
	{
        clima_ = this;
    }

	// default constructor (not using thinger)
    ClimaStick() :
       
        //tsl(TSL2561_ADDR_HIGH),
     
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

		byte c = mpu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    	bool mpu_begin_ok =  (c == 0x71);

    	if(mpu_begin_ok){
        	mpu.MPU9250SelfTest(mpu.SelfTest);
        		Serial.print("x-axis self test: acceleration trim within : ");
			    Serial.print(mpu.SelfTest[0],1); Serial.println("% of factory value");
			    Serial.print("y-axis self test: acceleration trim within : ");
			    Serial.print(mpu.SelfTest[1],1); Serial.println("% of factory value");
			    Serial.print("z-axis self test: acceleration trim within : ");
			    Serial.print(mpu.SelfTest[2],1); Serial.println("% of factory value");
			    Serial.print("x-axis self test: gyration trim within : ");
			    Serial.print(mpu.SelfTest[3],1); Serial.println("% of factory value");
			    Serial.print("y-axis self test: gyration trim within : ");
			    Serial.print(mpu.SelfTest[4],1); Serial.println("% of factory value");
			    Serial.print("z-axis self test: gyration trim within : ");
			    Serial.print(mpu.SelfTest[5],1); Serial.println("% of factory value");
        	mpu.calibrateMPU9250(mpu.gyroBias, mpu.accelBias);
        	mpu.initMPU9250();
        	mpu.initAK8963(mpu.magCalibration);
        	
    	}
	    #ifdef _DEBUG_
	    	Serial.println(mpu_begin_ok ? "MPU9250 connection successful" : "MPU9250 connection failed");
	    #endif		
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
	
        return(mpu_begin_ok && bme_begin_ok && tsl_begin_ok );

    }


	// get motion (accelerometer and gyroscope measures)
	Motion get_motion(){
		Motion motion;

			if (mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){ 
		  	  	mpu.readAccelData(mpu.accelCount);  // Read the x/y/z adc values
		    	mpu.getAres();

		    	// Now we'll calculate the accleration value into actual g's
		    	// This depends on scale being set
		    	motion.ax = (float)mpu.accelCount[0]*mpu.aRes; // - accelBias[0];
		    	motion.ay = (float)mpu.accelCount[1]*mpu.aRes; // - accelBias[1];
		    	motion.az = (float)mpu.accelCount[2]*mpu.aRes; // - accelBias[2];

		    	mpu.readGyroData(mpu.gyroCount);  // Read the x/y/z adc values
			    mpu.getGres();

			    // Calculate the gyro value into actual degrees per second
			    // This depends on scale being set
			    motion.gx = (float)mpu.gyroCount[0]*mpu.gRes;
			    motion.gy = (float)mpu.gyroCount[1]*mpu.gRes;
			    motion.gz = (float)mpu.gyroCount[2]*mpu.gRes;
			}
			mpu.updateTime();
			mpu.count = millis();
	        mpu.sumCount = 0;
	        mpu.sum = 0;
		
	   return motion; 	

	}

	// get gyroscope
	Gyroscope get_gyroscope(){
        Gyroscope gyro;
		
       		if (mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){ 
	  	  	  	mpu.readGyroData(mpu.gyroCount);  // Read the x/y/z adc values
		    	mpu.getGres();
			    // Calculate the gyro value into actual degrees per second
			    // This depends on scale being set
 
			    gyro.gx = (float)mpu.gyroCount[0]*mpu.gRes; 
			    gyro.gy = (float)mpu.gyroCount[1]*mpu.gRes;
			    gyro.gz = (float)mpu.gyroCount[2]*mpu.gRes;
			}
			mpu.updateTime();
			mpu.count = millis();
	        mpu.sumCount = 0;
	        mpu.sum = 0;

		return gyro;
	}

	// get accelerometer and gyroscope measures
	Accelerometer get_acceleration(){
        Accelerometer accel;
		
       		if (mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){ 
	  	  	  	mpu.readAccelData(mpu.accelCount);  // Read the x/y/z adc values
	    		mpu.getAres();
		    	// Now we'll calculate the accleration value into actual g's
		    	// This depends on scale being set
		    	 
		    	mpu.ax= (float)mpu.accelCount[0]*mpu.aRes; // - accelBias[0];
				mpu.ay = (float)mpu.accelCount[1]*mpu.aRes; // - accelBias[1];
		    	mpu.az = (float)mpu.accelCount[2]*mpu.aRes; // - accelBias[2];
			
				accel.ax =((int)1000*mpu.ax);
				accel.ay =((int)1000*mpu.ay);
				accel.az =((int)1000*mpu.az); 

			}
			mpu.updateTime();
		    mpu.count = millis();
	        mpu.sumCount = 0;
	        mpu.sum = 0;

	
		return accel;
	}
 
	// get compass measure
	Compass get_compass(){
		Magnetometer magnet;
        Compass mCompass;
        
       		mpu.readMagData(mpu.magCount);  // Read the x/y/z adc values
   			mpu.getMres();
		    // User environmental x-axis correction in milliGauss, should be
		    // automatically calculated
		    mpu.magbias[0] = +470.;
		    // User environmental x-axis correction in milliGauss TODO axis??
		    mpu.magbias[1] = +120.;
		    // Get actual magnetometer value, this depends on scale being set
		    magnet.mx = (float)mpu.magCount[0]*mpu.mRes*mpu.magCalibration[0]  ;
		    magnet.my = (float)mpu.magCount[1]*mpu.mRes*mpu.magCalibration[1]  ;
		    mCompass.heading = atan2(magnet.mx, magnet.my); // Calculate heading
	  

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
		 
			mpu.readMagData(mpu.magCount);  // Read the x/y/z adc values
		    mpu.getMres();
		    // User environmental x-axis correction in milliGauss, should be
		    // automatically calculated
		    mpu.magbias[0] = +470.;
		    // User environmental x-axis correction in milliGauss TODO axis??
		    mpu.magbias[1] = +120.;
		    // User environmental x-axis correction in milliGauss
		    mpu.magbias[2] = +125.;
  			// Get actual magnetometer value, this depends on scale being set
		    magnet.mx = (float)mpu.magCount[0]*mpu.mRes*mpu.magCalibration[0] - mpu.magbias[0];
		    magnet.my = (float)mpu.magCount[1]*mpu.mRes*mpu.magCalibration[1] - mpu.magbias[1];
		    magnet.mz = (float)mpu.magCount[2]*mpu.mRes*mpu.magCalibration[2] - mpu.magbias[2];
		 
		return magnet;
	}

    // get magnetometer measure
    Magnetometer get_raw_magnetometer(){
        Magnetometer magnet;
       
			mpu.readMagData(mpu.magCount);  // Read the x/y/z adc values
		    mpu.getMres();
		    // User environmental x-axis correction in milliGauss, should be
		    // automatically calculated
		    mpu.magbias[0] = +470.;
		    // User environmental x-axis correction in milliGauss TODO axis??
		    mpu.magbias[1] = +120.;
		    // User environmental x-axis correction in milliGauss
		    mpu.magbias[2] = +125.;
  			// Get actual magnetometer value, this depends on scale being set
		    magnet.mx = (float)mpu.magCount[0];
		    magnet.my = (float)mpu.magCount[1];
		    magnet.mz = (float)mpu.magCount[2];
		
        return magnet;
    }
	 
	// get clima
	Environmental get_clima(){
        Environmental clima;
 		clima.temperature = bme.readTemperature();
		clima.humidity = bme.readHumidity();
		clima.pressure = bme.readPressure() / 100.0F;
		clima.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        clima.lux = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
  
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
		time.day = timeClient.getDay();
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
            Magnetometer magnet = ClimaStick::get().get_raw_magnetometer();
            out["mx"] = magnet.mx;
            out["my"] = magnet.my;
            out["mz"] = magnet.mz;
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
            out["dew_point"] = (pow((clima.humidity/100),0.125)*(float)(112+0.9*clima.temperature)+(float)(0.1*clima.temperature)-112);
       
        };
    }

 
     void init_led_resource(){   //aki hay algo q no va
     	pinMode(BuiltinLed,OUTPUT);
        (*this)["Led"] << [](pson &in) {
             
                digitalWrite(BuiltinLed,(in? 1 : 0));
            
        };
    }


    // helper function to initialize all resources
	void init_resources() {
        init_button_resource();
	    //init_battery_resource(); //falla esta rutina
        init_accelerometger_resource();
        init_gyroscope_resource();
	 	init_compass_resource();
	 	//falla la rutina normal mag
        init_magnetometer_resource();
        init_environment_resource();
       	init_led_resource();
      
	}

	void sleep(unsigned long seconds){
     	 sleep_sensors();        //added by JT to enabled low power consumiption 300417
         Serial.println("going to sleep"); 
		 ESP.deepSleep(seconds * 1000000);
	}

    void sleep_sensors(){ //added by JT to enabled low power consumption 300417
      
	     tsl.disable();
	     mpu.sleep();
	     bme.sleep();
        
    }


};

ClimaStick* ClimaStick::clima_ = NULL;


#endif

