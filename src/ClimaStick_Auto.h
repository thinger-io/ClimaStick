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


#include <ClimaStick.h>
 
 void init_resources(ThingerESP8266 &thing) {
   
    Serial.begin(115200);

    timeClient.begin();		//Inicializing NTP 

    builtinSensorsBegin();  //Inicializing sensors
        
    thing["button"] >> outputValue((int)digitalRead(BUTTON));
      
    thing["battery"]  >> [](pson& out){
        out["voltaje"]=(float)batteryVoltaje();
        out["load"]=(float)batteryLoad();
        };
      
    thing["accelerometer"] >> [](pson& out){
        getMotion();
        out["ax"]=accel.x;
        out["ay"]=accel.y;
        out["az"]=accel.z;
        };
      
    thing["gyroscope"] >> [](pson& out){
        getMotion();
        out["gx"]=gyro.x;
        out["gy"]=gyro.y;
        out["gz"]=gyro.z;
        };
    
    thing["compass"] >> [](pson& out){
        getCompass();
        out["heading"]=compass.heading;
        out["degHeading"]=compass.headingDegrees;
        };
     
    thing["magnetometer"] >> [] (pson& out){
        getMagnet();
        out["rawX"]=magnet.x;
        out["rawY"]=magnet.y;
        out["rawZ"]=magnet.z;
        out["normX"]=magnet.nx;
        out["normY"]=magnet.ny;
        out["normZ"]=magnet.nz;
        };

    thing["environment"] >> [] (pson& out){
        getClima(); 
        out["temperature"]=clima.temperature;
        out["humidity"]=clima.humidity;
        out["altitude"]=clima.altitude;
        out["pressure"]=clima.pressure;
        out["lux"]=clima.lux;
        out["luminosity"]=clima.lux;
        };

    thing["RGB"] << [](pson& in){
        rgb(in["r"],in["g"],in["b"]);
        };

    thing["setColorByName"] << [](pson& in){
        String colorName = in["ColorName"];
        rgb(colorName);
        };
 	
}


 
void streamAll(ThingerESP8266 &thing){
	thing.stream(thing["button"]);
	thing.stream(thing["battery"]);
	thing.stream(thing["accelerometer"]);
	thing.stream(thing["gyroscope"]);
	thing.stream(thing["compass"]);
	thing.stream(thing["environment"]);

}

 