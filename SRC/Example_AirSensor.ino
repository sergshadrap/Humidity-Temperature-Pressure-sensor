#include <Arduino.h>
#include <Wire.h>
#include <ShadrapEsensor.h>

MyEsensor Airsensor;
float pr=0, tem=0, hu=0;
unsigned long next;

boolean checkI2Cdevice(uint8_t dev_addr)  // Check if the device response at their address
{
   Serial.printf("Device address:%#x\n",dev_addr);
    delay(50);
    Wire.beginTransmission(dev_addr);
        if (Wire.endTransmission()==0)  return true;
    else return false;
}

//The setup function is called once at startup of the sketch
void setup() {

 
//--------------------------------------- I2C initialisation
  Wire.begin();
  Wire.setClockStretchLimit(2500); 
//---------------Air sensor detection-------------
 if(checkI2Cdevice(0x40)&& checkI2Cdevice(0x77) ) 
{
        Serial.printf("Found H&T ShadrapEsensor on 0x40 address  and Pressure sensor on 0x77!\n"); 
   
    Airsensor.begin(Wire);  // Init sensor
      delay(50);
     pr=Airsensor.readPressure();    // Read pressure data
      tem=Airsensor.readTemperature();    // Read temperature data
        Serial.printf("Temperature:02%f\n",tem);
          hu=Airsensor.readHumidity();  //Read humidity data
            Serial.printf("Humidity:%f\n",hu);
               Serial.printf("Pressure:%f\n",pr);
   }
   else  
        Serial.println("I2c Air-Enviroment sensor Not Found!"); 
}

// The loop function is called in an endless loop
void loop() {
unsigned long now = millis();

 if (now >= next) {
hu=Airsensor.readHumidity();
     tem=Airsensor.readTemperature();
         pr=Airsensor.readPressure(); 
                     Serial.printf("Humidity:%f\n",hu);
                        Serial.printf("Pressure:%f\n",pr);
                          Serial.printf("Temperature:02%f\n",tem);
   next = now + 3000;        //check sensor every 3 sec

 }
}
