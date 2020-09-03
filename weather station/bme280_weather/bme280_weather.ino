/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
///lol

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
unsigned long cur_millis;
unsigned char i = 0;
const unsigned char arr_size = 144;//160
short int avg_press = 0;
unsigned long pressure_into_mas_interval = 300000;//300000
short int press_mas[arr_size];

void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    //Serial.println("BME280 test");

    unsigned status;
    cur_millis = millis();
    // default settings
    //status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    status = bme.begin(0x76, &Wire);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    //Serial.println("-- Default Test --");
    delayTime = 3000;
    

    //Serial.println();
}


void loop() { 
    
    if (millis()> cur_millis + pressure_into_mas_interval){
      if (i == arr_size){
        for (int j=0; j<arr_size-1;j++){ //arr_size-1
          press_mas[j]=press_mas[j+1];
          }
        //int p = bme.readPressure() / 100.0F * 0.75006;
        
      } 
   
      cur_millis = millis();
      //float p_float = bme.readPressure() / 100.0F * 0.75006;
      int p = roundf(bme.readPressure() / 100.0F * 0.75006);
      if(i<arr_size){
          press_mas[i] = p;
          i++;
      }else {
          press_mas[arr_size-1] = p;
      }
        
      long int sum=0; 
      for(int j = 0; j < i; j++)
      {
          sum = sum+press_mas[j];
          //Serial.print("mas");
          //Serial.print(j);
          //Serial.print(" = ");
          //Serial.print(press_mas[j]);
          //Serial.print(" ");
      }

      float sum_float = sum;
      avg_press= roundf(sum_float/i);
      //Serial.print("end avg_press=");
      //Serial.println(avg_press);
        
      
      
    }
      
    printValues();
    delay(delayTime);
}


void printValues() {
    //Serial.print("Temperature = ");
    int t = bme.readTemperature();
    int p = roundf(bme.readPressure() / 100.0F * 0.75006);
    int h = bme.readHumidity();
    if (millis() < 3600000){ //3600000
      Serial.print(t);
      Serial.print(p);
      Serial.print(h);
      Serial.println("000");
      
    }else {
      Serial.print(t);
      Serial.print(p);
      Serial.print(h);
      Serial.println(avg_press);
     }
    /*Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F * 0.75006);
    Serial.println(" mm Hg");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
    */
}
