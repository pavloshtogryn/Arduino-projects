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
#include <avr/wdt.h>
#include <avr/sleep.h>


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
unsigned long cur_millis;
unsigned long cur_millis_delay_transmitt;
unsigned char i = 0;
const unsigned char arr_size = 144;//160
short int avg_press = 0;
unsigned long pressure_into_mas_interval = 270000;//300000
short int press_mas[arr_size];
String str="";
unsigned long int delay_counter=0;
volatile bool break_flag = 0;
volatile bool can_transmit_flag = 1;
unsigned long cur_time;

void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    //Serial.println("BME280 test");

    unsigned status;
    cur_millis = millis();
    cur_millis_delay_transmitt = millis();

    cur_time = millis();
    pinMode(2, INPUT_PULLUP);
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
    //Serial.print("AT+SLEEP");
}


void loop() { 
    //Serial.println("loop");
    //((millis()+ delay_counter*4000) - cur_time) > 60000)
    if ( (millis() + delay_counter*4000 )> cur_time + pressure_into_mas_interval){ //allow adding to mas if spent more than 4.5 min (sleeping 5 min)
      //Serial.println("if");
      if (i == arr_size){
        for (int j=0; j<arr_size-1;j++){ //arr_size-1
          press_mas[j]=press_mas[j+1];
          }
        //int p = bme.readPressure() / 100.0F * 0.75006;
        
      } 
   
      cur_time = millis() + (delay_counter*4000);
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
      //Serial.println(" ");
      float sum_float = sum;
      avg_press= roundf(sum_float/i);
      //Serial.print("end avg_press=");
      //Serial.println(avg_press);

      if (can_transmit_flag == 0){ //засинаємо лише якщо до цього спали
        sleep_manager();
      }
      

      BME280_Wake(0x76);
      
    }

    command_disconnect();
    //sleep_manager();
    
    
    
    if (millis() - cur_millis_delay_transmitt > 3000){
      cur_millis_delay_transmitt = millis();
      printValues();
  
    }

    
    
    //printValues();
    //delay(delayTime);
}


void printValues() {
   //Serial.print(" H");
   //Serial.println(can_transmit_flag);
  if (can_transmit_flag == 1){
    //Serial.print("Temperature = ");
    int t = bme.readTemperature();
    int p = roundf(bme.readPressure() / 100.0F * 0.75006);
    int h = bme.readHumidity();
    if (millis()+ delay_counter * 4000 <  3600000){ //3600000
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
  }/*
  else{
    Serial.print("I'm awake but can't transmit");
  }
  */
}

void command_disconnect(){
    if(Serial.available()>0){     
      char data= Serial.read(); // reading the data received from the bluetooth module
      str += data;
      //Serial.println(str);
      if (str=="OK+LOST"){
        can_transmit_flag = 0;
        Serial.print("AT+SLEEP");
        //digitalWrite(13,HIGH);
        str="";
        sleep_manager();

        BME280_Wake(0x76);
      }
      //else if (str=="OK+SLEEP"){
        //Serial.print("SLEEPING");
        //}
      delay(50);
    }else{
      str="";
    }
}

void sleep_manager(){
  
  //Serial.println(" S");
  
  BME280_Sleep(0x76);  
  
  delay(70);
  attachInterrupt(digitalPinToInterrupt(2), inter_handler, FALLING);
  //detachInterrupt(0);
  if ( (delay_counter % 70) == 0){
    
    break_flag = 0;
    can_transmit_flag = 0;
    for (int i=0; i<70;i++){ //sleep ~5min
      wdt_enable(WDTO_4S); //Задаем интервал сторожевого таймера (2с)
      WDTCSR |= (1 << WDIE); //Устанавливаем бит WDIE регистра WDTCSR для разрешения прерываний от сторожевого таймера
      set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Устанавливаем интересующий нас режим
      sleep_mode(); // Переводим МК в спящий режим
      //Serial.println(i);
      //delay(100);
      if (break_flag == 1){
        break;
        }
      delay_counter = delay_counter+1;
    }
  }else{
    
    break_flag = 0;
    can_transmit_flag = 0;
    int tmp = 70 - (delay_counter % 70);
    for (int i=0; i< tmp;i++){ //if interrupt sleep other time
      wdt_enable(WDTO_4S); //Задаем интервал сторожевого таймера (2с)
      WDTCSR |= (1 << WDIE); //Устанавливаем бит WDIE регистра WDTCSR для разрешения прерываний от сторожевого таймера
      set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Устанавливаем интересующий нас режим
      sleep_mode(); // Переводим МК в спящий режим
      //Serial.println(i);
      //delay(100);
      if (break_flag == 1){
        break;
        }
      delay_counter = delay_counter+1;
    }
    
    
   }
  detachInterrupt(digitalPinToInterrupt(2));
  //Serial.println("WAKING");
  
  }

ISR (WDT_vect) {
  wdt_disable();
  //f = !f;
}

void inter_handler () {
  //Serial.println("I");
  break_flag = 1;
  can_transmit_flag = 1;
}

void BME280_Sleep(int device_address) {
  // BME280 Register 0xF4 (control measurement register) sets the device mode, specifically bits 1,0
  // The bit positions are called 'mode[1:0]'. See datasheet Table 25 and Paragraph 3.3 for more detail.
  // Mode[1:0]  Mode
  //    00      'Sleep'  mode
  //  01 / 10   'Forced' mode, use either '01' or '10'
  //    11      'Normal' mode
  Wire.beginTransmission(device_address);
  Wire.write((uint8_t)0xF4);       // Select Control Measurement Register
  Wire.write((uint8_t)0b00000000); // Send '00' in bits 0-1 for Sleep mode
  Wire.endTransmission();
}

void BME280_Wake(int device_address){
  Wire.beginTransmission(device_address);
  Wire.write((uint8_t)0xF4);       // Select Control Measurement Register
  Wire.write((uint8_t)0b00100111); // Send '11' in bits 0-1 for Sleep mode and '1' in bits 2 and 5 
                                   //for turning on temperature and pressure oversampling (without it temp and press will be 0)
  Wire.endTransmission();
  bme.begin(0x76, &Wire);//not neccecary
  }
