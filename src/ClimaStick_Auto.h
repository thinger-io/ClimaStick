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

    builtinSensors_begin();  //Inicializing sensors
        
    thing["button"] >> outputValue((int)digitalRead(BUTTON));
      
    thing["battery"]  >> [](pson& out){
        out["voltaje"]=(float)batteryVoltaje();
        out["load"]=(float)batteryLoad();
        };
      
    thing["accelerometer"] >> [](pson& out){
        struct accelerometer accelgyro = get_acceleration();
        out["ax"]=accelgyro.ax;
        out["ay"]=accelgyro.ay;
        out["az"]=accelgyro.az;
        };
      
    thing["gyroscope"] >> [](pson& out){
        struct accelerometer accelgyro = get_acceleration();
        out["gx"]=accelgyro.gx;
        out["gy"]=accelgyro.gy;
        out["gz"]=accelgyro.gz;
        };
    
    thing["compass"] >> [](pson& out){
        struct compass myCompass = get_compass();
        out["heading"]=myCompass.heading;
        out["degHeading"]=myCompass.headingDegrees;
        };
     
    thing["magnetometer"] >> [] (pson& out){
        struct magnetometer myMagnet = get_magnet();
        out["rawX"]=myMagnet.x;
        out["rawY"]=myMagnet.y;
        out["rawZ"]=myMagnet.z;
        out["normX"]=myMagnet.nx;
        out["normY"]=myMagnet.ny;
        out["normZ"]=myMagnet.nz;
        };

    thing["environment"] >> [] (pson& out){
        struct environmental myClima = get_clima(); 
        out["temperature"]=myClima.temperature;
        out["humidity"]=myClima.humidity;
        out["altitude"]=myClima.altitude;
        out["pressure"]=myClima.pressure;
        out["lux"]=myClima.lux;
        out["luminosity"]=myClima.lux;
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

 