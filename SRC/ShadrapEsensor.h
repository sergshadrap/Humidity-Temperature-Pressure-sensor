/*
	ShadrapEsensor.h
	ShadrapEsensor.h originally created by: Sergey Sinitsyn, Jan 1st 2021 // shadrap@yandex.ru
	
	This code is release AS-IS into the public domain, no guarantee or warranty is given.

	I2C only Driver for the HDC2080 Temperature and Humidity Sensor and the Infineon DPS368 Pressure and Temperature sensor
	
	Description: This header file accompanies ShadrapEsensor.cpp, and declares all methods, fields,
	and constants used in the source code. 
	Have a look at the datasheets for more information. 
	There are only basic functionality being represented this files.
	For full functionality please use original libraries.
	!!!Note!!! Due to high current consumption (90ma) of internal Heater HDC2080 don't use it over pure GPIO connection
	to avoid burning your Soc.
*/


#include <Arduino.h>
#include <Wire.h>


#define HDC2080_ADDRESS 0x40U
#define DPS368_ADDRESS 0x77U

//Define  Map HDC2080
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
#define CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
#define TEMP_AND_HUMID 0
#define TEMP_ONLY 1
#define HUMID_ONLY 2


static const int32_t scaling_facts[8]= {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};


class MyEsensor
{
public:
//used for I2C
  TwoWire *m_i2cbus;
  uint8_t m_slaveAddress;
  uint8_t slaveAddress;


	void begin(TwoWire &bus);				  // Join I2C bus, init both sensors set default constants
	float readTemperature(void);			  // Returns the temperature in degrees C
	float readHumidity(void);				  // Returns the relative humidity %
	float readPressure(void);				  // Returns the absolute pressure Pa
	void enableHeater(void);				  // Enables the heating element
	void disableHeater(void);				  // Disables the heating element
	void resetHDC2080(void);				  // Triggers a software reset for HDC2080 sensor
	void triggerMeasurement(void);			  // Triggers measurment of HDC2080 
	void setTempResHDC2080(int resolution);	   // Set temperature Resolution for HDC2080 "0 - 14 bit,1 - 11 bit,2 - 9 bit
		   							   // default - 14 bit
	void setMeasurementModeHDC2080(int mode);		   //Set measurment mode 0 - Temperature&Humidity ; 1-Temperature only;2-Humidity only
	void setHumidResHDC2080(int resolution);			   // Set humidity Resolution for HDC2080 "0 - 14 bit,1 - 11 bit,2 - 9 bit
		   							   // default - 14 bit
	int16_t measureTempOnce(float &result);	    // Triggers one temperature measurment by DPS368 (for pressure compensation)
	int16_t standby(void);

private:
//Define  Map DPS368
typedef struct
{
    uint8_t regAddress;
    uint8_t mask;
    uint8_t shift;
} RegMask_t;

typedef struct
{
    uint8_t regAddress;
    uint8_t length;
} RegBlock_t;


enum Registers_e
{
    PROD_ID = 0,
    REV_ID,
    TEMP_SENSOR,    // internal vs external
    TEMP_SENSORREC, //temperature sensor recommendation
    TEMP_SE,        //temperature shift enable (if temp_osr>3)
    PRS_SE,         //pressure shift enable (if prs_osr>3)
    FIFO_FL,        //FIFO flush
    FIFO_EMPTY,     //FIFO empty
    FIFO_FULL,      //FIFO full
    INT_HL,
    INT_SEL,         //interrupt select
};

enum Config_Registers_e
{
    TEMP_MR = 0, // temperature measure rate
    TEMP_OSR,    // temperature measurement resolution
    PRS_MR,      // pressure measure rate
    PRS_OSR,     // pressure measurement resolution
    MSR_CTRL,    // measurement control
    FIFO_EN,

    TEMP_RDY,
    PRS_RDY,
    INT_FLAG_FIFO,
    INT_FLAG_TEMP,
    INT_FLAG_PRS,
};

const RegMask_t config_registers[16] = {
    {0x07, 0x70, 4}, // TEMP_MR
    {0x07, 0x07, 0}, // TEMP_OSR
    {0x06, 0x70, 4}, // PRS_MR
    {0x06, 0x07, 0}, // PRS_OSR
    {0x08, 0x07, 0}, // MSR_CTRL
    {0x09, 0x02, 1}, // FIFO_EN

    {0x08, 0x20, 5}, // TEMP_RDY
    {0x08, 0x10, 4}, // PRS_RDY
    {0x0A, 0x04, 2}, // INT_FLAG_FIFO
    {0x0A, 0x02, 1}, // INT_FLAG_TEMP
    {0x0A, 0x01, 0}, // INT_FLAG_PRS
};


const RegMask_t registers[16] = {
    {0x0D, 0x0F, 0}, // PROD_ID
    {0x0D, 0xF0, 4}, // REV_ID
    {0x07, 0x80, 7}, // TEMP_SENSOR
    {0x28, 0x80, 7}, // TEMP_SENSORREC
    {0x09, 0x08, 3}, // TEMP_SE
    {0x09, 0x04, 2}, // PRS_SE
    {0x0C, 0x80, 7}, // FIFO_FL
    {0x0B, 0x01, 0}, // FIFO_EMPTY
    {0x0B, 0x02, 1}, // FIFO_FULL
    {0x09, 0x80, 7}, // INT_HL
    {0x09, 0x70, 4}, // INT_SEL
};

enum Mode
{
    IDLE = 0x00,
    CMD_PRS = 0x01,
    CMD_TEMP = 0x02,
    CMD_BOTH = 0x03, // only for DPS422
    CONT_PRS = 0x05,
    CONT_TMP = 0x06,
    CONT_BOTH = 0x07
};

enum RegisterBlocks_e
{
    PRS = 0, // pressure value
    TEMP,    // temperature value
};

const RegBlock_t registerBlocks[2] = {
    {0x00, 3},
    {0x03, 3},
};


const RegBlock_t coeffBlock = {0x10, 18};

  Mode m_opMode;
 // uint8_t m_opMode=0;

  //flags
  uint8_t m_initFail;

  uint8_t m_productID;
  uint8_t m_revisionID;
  //settings
  uint8_t m_tempSensor;
  uint8_t m_tempMr;
  uint8_t m_tempOsr;
  uint8_t m_prsMr;
  uint8_t m_prsOsr;
  // compensation coefficients 
  int32_t m_c00;
  int32_t m_c10;
  int32_t m_c01;
  int32_t m_c11;
  int32_t m_c20;
  int32_t m_c21;
  int32_t m_c30;
  int32_t m_c0Half;
  int32_t m_c1;
  // last measured scaled temperature (necessary for pressure compensation)
  float m_lastTempScal;


	void _initDPS368(uint8_t sensor_address);		// 		Init DPS368 
	void _initHDC2080(uint8_t sensor_address);  	//		Init HDC2080
	int16_t correctTemp(void);					// Tempretature correction for internal DPS368 temp sensor
	int16_t startMeasurePressureOnce(uint8_t oversamplingRate);  
	int16_t startMeasurePressureOnce(void);
	int16_t measurePressureOnce(float &result, uint8_t oversamplingRate);
	int16_t startMeasureTempOnce(uint8_t oversamplingRate);
	int16_t startMeasureTempOnce(void);
	int16_t measureTempOnce(float &result, uint8_t oversamplingRate);
	uint16_t calcBusyTime(uint16_t mr, uint16_t osr);
	int16_t getSingleResult(float &result);
	float calcPressure(int32_t raw);
	float calcTemp(int32_t raw);
	int16_t getRawResult(int32_t *raw, RegBlock_t reg);
	int16_t configPressure(uint8_t prsMr, uint8_t prsOsr);
	int16_t configTemp(uint8_t tempMr, uint8_t tempOsr);
	int16_t disableFIFO();
	int16_t enableFIFO();
	int16_t flushFIFO();
	int16_t setOpMode(uint8_t opMode);
	int16_t readcoeffs(void);
	void getTwosComplement(int32_t *raw, uint8_t length);
	int16_t readBlock(RegBlock_t regBlock, uint8_t *buffer);
	int16_t readByteBitfield(RegMask_t regMask);
	int16_t readByte(uint8_t regAddress);
	int16_t writeByteBitfield(uint8_t data,uint8_t regAddress,uint8_t mask,uint8_t shift,uint8_t check);
	int16_t writeByteBitfield(uint8_t data, RegMask_t regMask);
	int16_t writeByte(uint8_t regAddress, uint8_t data, uint8_t check);
	int16_t writeByte(uint8_t regAddress, uint8_t data);
};



