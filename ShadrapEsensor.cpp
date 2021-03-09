/*
	ShadrapEsensor.cpp
	ShadrapEsensor.cpp originally created by: Sergey Sinitsyn, Jan 1st 2021
	
	This code is release AS-IS into the public domain, no guarantee or warranty is given.
	
	Description: This library facilitates communication with, and configuration of,
	the HDC2080 Temperature and Humidity Sensor and the Infineon DPS368 Pressure and Temperature sensor. It makes extensive use of the 
	Wire.H library. 
*/


#include <ShadrapEsensor.h>
#include <Wire.h>




int16_t MyEsensor::writeByte(uint8_t regAddress, uint8_t data)
{
  return writeByte(regAddress, data, 0U);
}

int16_t MyEsensor::writeByte(uint8_t regAddress, uint8_t data, uint8_t check)
{
  m_i2cbus->beginTransmission(m_slaveAddress);
  m_i2cbus->write(regAddress);      //Write Register number to buffer
  m_i2cbus->write(data);          //Write data to buffer
  if (m_i2cbus->endTransmission() != 0) //Send buffer content to slave
  {
    return -1;
  }
  else
  {
    if (check == 0)
      return 0;           //no checking
    if (readByte(regAddress) == data) //check if desired by calling function
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }
}

int16_t MyEsensor::writeByteBitfield(uint8_t data, RegMask_t regMask)
{
  return writeByteBitfield(data, regMask.regAddress, regMask.mask, regMask.shift, 0U);
}

int16_t MyEsensor::writeByteBitfield(uint8_t data,
                  uint8_t regAddress,
                  uint8_t mask,
                  uint8_t shift,
                  uint8_t check)
{
  int16_t old = readByte(regAddress);
  if (old < 0)
  {
    //fail while reading
    return old;
  }
  return writeByte(regAddress, ((uint8_t)old & ~mask) | ((data << shift) & mask), check);
}

int16_t MyEsensor::readByte(uint8_t regAddress)
{
  m_i2cbus->beginTransmission(m_slaveAddress);
  m_i2cbus->write(regAddress);
  m_i2cbus->endTransmission(false);
  //request 1 byte from slave
  if (m_i2cbus->requestFrom(m_slaveAddress, 1U, 1U) > 0)
  {
    return m_i2cbus->read(); //return this byte on success
  }
  else
  {
    return -1; //if 0 bytes were read successfully
  }
}

int16_t MyEsensor::readByteBitfield(RegMask_t regMask)
{
  int16_t ret = readByte(regMask.regAddress);
  if (ret < 0)
  {
    return ret;
  }
  return (((uint8_t)ret) & regMask.mask) >> regMask.shift;
}

int16_t MyEsensor::readBlock(RegBlock_t regBlock, uint8_t *buffer)
{
  //do not read if there is no buffer
  if (buffer == NULL)
  {
    return 0; //0 bytes read successfully
  }

  m_i2cbus->beginTransmission(m_slaveAddress);
  m_i2cbus->write(regBlock.regAddress);
  m_i2cbus->endTransmission(false);
  //request length bytes from slave
  int16_t ret = m_i2cbus->requestFrom(m_slaveAddress, regBlock.length, 1U);
  //read all received bytes to buffer
// Serial.printf("retcode:%d address:%#x register:%#x\n",ret,m_slaveAddress,regBlock.regAddress);
  for (int16_t count = 0; count < ret; count++)
  {
    buffer[count] = m_i2cbus->read();
  }
  return ret;
}

void MyEsensor::getTwosComplement(int32_t *raw, uint8_t length)
{
  if (*raw & ((uint32_t)1 << (length - 1)))
  {
    *raw -= (uint32_t)1 << length;
  }
}

int16_t MyEsensor::readcoeffs(void)
{
  m_slaveAddress =DPS368_ADDRESS;
  // TODO: remove magic number
  uint8_t buffer[18];
  //read COEF registers to buffer
  int16_t ret = readBlock(coeffBlock, buffer);

  //compose coefficients from buffer content
  m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
  getTwosComplement(&m_c0Half, 12);
  //c0 is only used as c0*0.5, so c0_half is calculated immediately
  m_c0Half = m_c0Half / 2U;

  //now do the same thing for all other coefficients
  m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
  getTwosComplement(&m_c1, 12);
  m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
  getTwosComplement(&m_c00, 20);
  m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
  getTwosComplement(&m_c10, 20);

  m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
  getTwosComplement(&m_c01, 16);

  m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
  getTwosComplement(&m_c11, 16);
  m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
  getTwosComplement(&m_c20, 16);
  m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
  getTwosComplement(&m_c21, 16);
  m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
// Serial.printf("Coeff func m_c0Half:%d m_c1:%d\n",m_c0Half,m_c1);
  getTwosComplement(&m_c30, 16);
  return 0;
}

int16_t MyEsensor::setOpMode(uint8_t opMode)
{
  m_slaveAddress =DPS368_ADDRESS;
  if (writeByteBitfield(opMode, config_registers[MSR_CTRL]) == -1)
  {
    return -1;
  }
  m_opMode = (Mode)opMode;
// Serial.printf("Operation mode: %d\n",opMode);
  return 0;
}

int16_t MyEsensor::flushFIFO()
{
  m_slaveAddress =DPS368_ADDRESS;
  return writeByteBitfield(1U, registers[FIFO_FL]);
}

int16_t MyEsensor::enableFIFO()
{
  m_slaveAddress =DPS368_ADDRESS;
  return writeByteBitfield(1U, config_registers[FIFO_EN]);
}

int16_t MyEsensor::disableFIFO()
{
  m_slaveAddress =DPS368_ADDRESS;
  int16_t ret = flushFIFO();
  ret = writeByteBitfield(0U, config_registers[FIFO_EN]);
  return ret;
}

int16_t MyEsensor::standby(void)
{
  m_slaveAddress =DPS368_ADDRESS;
  //abort if initialization failed
  if (m_initFail)
  {
    return -1;
  }
  //set device to idling mode
  int16_t ret = setOpMode(IDLE);
  if (ret != 0)
  {
    return ret;
  }
  ret = disableFIFO();
  return ret;
}

int16_t MyEsensor::configTemp(uint8_t tempMr, uint8_t tempOsr)
{
  m_slaveAddress =DPS368_ADDRESS;
//  int16_t ret = configTemp(tempMr, tempOsr);
int16_t ret;
  writeByteBitfield(m_tempSensor, registers[TEMP_SENSOR]);
  //set TEMP SHIFT ENABLE if oversampling rate higher than eight(2^3)
  if (tempOsr > 3U)
  {
    ret = writeByteBitfield(1U, registers[TEMP_SE]);
  }
  else
  {
    ret = writeByteBitfield(0U, registers[TEMP_SE]);
  }
  return ret;
}

int16_t MyEsensor::configPressure(uint8_t prsMr, uint8_t prsOsr)
{
  m_slaveAddress =DPS368_ADDRESS;
  int16_t ret;
//  int16_t ret = configPressure(prsMr, prsOsr);
  //set PM SHIFT ENABLE if oversampling rate higher than eight(2^3)
  if (prsOsr > 3U)
  {
    ret = writeByteBitfield(1U, registers[PRS_SE]);
  }
  else
  {
    ret = writeByteBitfield(0U, registers[PRS_SE]);
  }
  return ret;
}

int16_t MyEsensor::getRawResult(int32_t *raw, RegBlock_t reg)
{
  m_slaveAddress =DPS368_ADDRESS;
  uint8_t buffer[3] = {0};
  if (readBlock(reg, buffer) != 3)
    return -1;
//  Serial.printf("Raw temperature byte: %#x %#x %#x\n",buffer[0],buffer[1],buffer[2]);

  *raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
// Serial.printf("United in byte : %d , %x\n",raw,raw );
  getTwosComplement(raw, 24);
  return 0;
}

float MyEsensor::calcTemp(int32_t raw)
{
  m_slaveAddress =DPS368_ADDRESS;
  float temp = raw;

  //scale temperature according to scaling table and oversampling
  temp /= scaling_facts[m_tempOsr];

  //update last measured temperature
  //it will be used for pressure compensation
  m_lastTempScal = temp;

  //Calculate compensated temperature
  temp = m_c0Half + m_c1 * temp;

  return temp;
}

float MyEsensor::calcPressure(int32_t raw)
{
  m_slaveAddress =DPS368_ADDRESS;
  float prs = raw;

  //scale pressure according to scaling table and oversampling
  prs /= scaling_facts[m_prsOsr];

  //Calculate compensated pressure
  prs = m_c00 + prs * (m_c10 + prs * (m_c20 + prs * m_c30)) + m_lastTempScal * (m_c01 + prs * (m_c11 + prs * m_c21));

  //return pressure
  return prs;
}

int16_t MyEsensor::getSingleResult(float &result)
{
  m_slaveAddress =DPS368_ADDRESS;
  //abort if initialization failed
  if (m_initFail)
  {
    return -2;
  }

  //read finished bit for current opMode
  int16_t rdy;
  switch (m_opMode)
  {
  case CMD_TEMP: //temperature
    rdy = readByteBitfield(config_registers[TEMP_RDY]);
    break;
  case CMD_PRS: //pressure
    rdy = readByteBitfield(config_registers[PRS_RDY]);
    break;
  default: //DPS310 not in command mode
    return -3;
  }
  //read new measurement result
  switch (rdy)
  {
  case -1: //could not read ready flag
    return -1;
  case 0: //ready flag not set, measurement still in progress
    return -4;
  case 1: //measurement ready, expected case
    Mode oldMode = m_opMode;
    m_opMode = IDLE; //opcode was automatically reseted by DPS310
    int32_t raw_val;
    switch (oldMode)
    {
    case CMD_TEMP: //temperature
      getRawResult(&raw_val, registerBlocks[TEMP]);
      result = calcTemp(raw_val);
      return 0; // TODO
    case CMD_PRS:        //pressure
      getRawResult(&raw_val, registerBlocks[PRS]);
      result = calcPressure(raw_val);
      return 0; // TODO
    default:
      return -1; //should already be filtered above
    }
  }
  return -1;
}

uint16_t MyEsensor::calcBusyTime(uint16_t mr, uint16_t osr)
{
  //formula from datasheet (optimized)
  return ((uint32_t)20U << mr) + ((uint32_t)16U << (osr + mr));
}

int16_t MyEsensor::measureTempOnce(float &result)
{
  m_slaveAddress =DPS368_ADDRESS;
  return measureTempOnce(result, m_tempOsr);
}

int16_t MyEsensor::measureTempOnce(float &result, uint8_t oversamplingRate)
{
  //Start measurement
  int16_t ret = startMeasureTempOnce(oversamplingRate);
  if (ret != 0)
  {
    return ret;
  }

  //wait until measurement is finished
  delay(calcBusyTime(0U, m_tempOsr) / 10U);
  delay(10U);
// Serial.printf("MeasureTempOnce delay1:%d  delay2:%d \n",calcBusyTime(0U, m_tempOsr) / DPS__BUSYTIME_SCALING,DPS310__BUSYTIME_FAILSAFE);
  ret = getSingleResult(result);
  if (ret != 0)
  {
    standby();
  }
  return ret;
}

int16_t MyEsensor::startMeasureTempOnce(void)
{
  m_slaveAddress =DPS368_ADDRESS;
  return startMeasureTempOnce(m_tempOsr);
}

int16_t MyEsensor::startMeasureTempOnce(uint8_t oversamplingRate)
{
  m_slaveAddress =DPS368_ADDRESS;
  //abort if initialization failed
  if (m_initFail)
  {
    return -1;
  }
  //abort if device is not in idling mode
  if (m_opMode != IDLE)
  {
    return -3;
  }

  if (oversamplingRate != m_tempOsr)
  {
    //configuration of oversampling rate
    if (configTemp(0U, oversamplingRate) != 0)
    {
      return -1;
    }
  }

  //set device to temperature measuring mode
  return setOpMode(CMD_TEMP);
}

float MyEsensor::readPressure(void)
{
  float pressure;
  if(measurePressureOnce(pressure, 3)<0) return -1;
  return pressure;
}

int16_t MyEsensor::measurePressureOnce(float &result, uint8_t oversamplingRate)
{
  m_slaveAddress =DPS368_ADDRESS;
  //start the measurement
  int16_t ret = startMeasurePressureOnce(oversamplingRate);
  if (ret != 0)
  {
    return ret;
  }

  //wait until measurement is finished
  delay(calcBusyTime(0U, m_prsOsr) / 10U);
  delay(10U);

  ret = getSingleResult(result);
  if (ret != 0)
  {
    standby();
  }
  return ret;
}

int16_t MyEsensor::startMeasurePressureOnce(void)
{
  m_slaveAddress =DPS368_ADDRESS;
  return startMeasurePressureOnce(m_prsOsr);
}

int16_t MyEsensor::startMeasurePressureOnce(uint8_t oversamplingRate)
{
  m_slaveAddress =DPS368_ADDRESS;
  //abort if initialization failed
  if (m_initFail)
  {
    return -2;
  }
  //abort if device is not in idling mode
  if (m_opMode != IDLE)
  {
    return -3;
  }
  //configuration of oversampling rate, lowest measure rate to avoid conflicts
  if (oversamplingRate != m_prsOsr)
  {
    if (configPressure(0U, oversamplingRate))
    {
      return -1;
    }
  }
  //set device to pressure measuring mode
  return setOpMode(CMD_PRS);
}

int16_t MyEsensor::correctTemp(void)
{
  m_slaveAddress =DPS368_ADDRESS;
  if (m_initFail)
  {
    return -2;
  }
  writeByte(0x0E, 0xA5);
  writeByte(0x0F, 0x96);
  writeByte(0x62, 0x02);
  writeByte(0x0E, 0x00);
  writeByte(0x0F, 0x00);

  //perform a first temperature measurement (again)
  //the most recent temperature will be saved internally
  //and used for compensation when calculating pressure
  float trash;
  measureTempOnce(trash);

  return 0;
}

void MyEsensor::_initDPS368(uint8_t sensor_address)
{
   m_slaveAddress =sensor_address;
  int16_t prodId = readByteBitfield(registers[PROD_ID]);
  if (prodId < 0)
  {
    //Connected device is not a Dps368
    m_initFail = 1U;
    return;
  }
  m_productID = prodId;

  int16_t revId = readByteBitfield(registers[REV_ID]);
  if (revId < 0)
  {
    m_initFail = 1U;
    return;
  }
  m_revisionID = revId;

  //find out which temperature sensor is calibrated with coefficients...
  int16_t sensor = readByteBitfield(registers[TEMP_SENSORREC]);
  if (sensor < 0)
  {
    m_initFail = 1U;
    return;
  }

  //...and use this sensor for temperature measurement
  m_tempSensor = sensor;
  if (writeByteBitfield((uint8_t)sensor, registers[TEMP_SENSOR]) < 0)
  {
    m_initFail = 1U;
    return;
  }


  //read coefficients
  if (readcoeffs() < 0)
  {
    m_initFail = 1U;
    return;
  }
 
  //set to standby for further configuration
  standby();

  //set measurement precision and rate to standard values;
  configTemp(2, 3);
  configPressure(2, 3);

  //perform a first temperature measurement
  //the most recent temperature will be saved internally
  //and used for compensation when calculating pressure
  float trash;

  measureTempOnce(trash);

  //make sure the Dps368 is in standby after initialization
  standby();

  // Fix IC with a fuse bit problem, which lead to a wrong temperature
  // Should not affect ICs without this problem
  correctTemp();
}

void MyEsensor::_initHDC2080(uint8_t sensor_address)
{
  m_slaveAddress =sensor_address;
  resetHDC2080();
  triggerMeasurement();
  return;
}

float MyEsensor::readTemperature(void)
{
  uint8_t byte[2];
  uint16_t temp;
   m_slaveAddress =HDC2080_ADDRESS;
   triggerMeasurement();
  byte[0] = readByte(TEMP_LOW);
  byte[1] = readByte(TEMP_HIGH);
  temp = byte[1];
  temp = (temp << 8) | byte[0];
  float f = temp;
  f = ((f * 165.0f) / 65536.0f) - 40.0f;
  return f;
}

float MyEsensor::readHumidity(void)
{
  uint8_t byte[2];
  uint16_t humidity;
  triggerMeasurement();
  m_slaveAddress =HDC2080_ADDRESS;
  byte[0] = readByte(HUMID_LOW);
  byte[1] = readByte(HUMID_HIGH);
  humidity = byte[1];
  humidity = (humidity << 8) | byte[0];
  float f = humidity;
  f = (f / 65536.0f) * 100.0f;

  return f;
}

void MyEsensor::enableHeater(void)
{
  uint8_t configContents; //Stores current contents of config register
  m_slaveAddress =HDC2080_ADDRESS;
  
  configContents = readByte(CONFIG);

  //set bit 3 to 1 to enable heater
  configContents = (configContents | 0x08);

  writeByte(CONFIG, configContents);
}

void MyEsensor::disableHeater(void)
{
  uint8_t configContents; //Stores current contents of config register
  m_slaveAddress =HDC2080_ADDRESS;

  configContents = readByte(CONFIG);

  //set bit 3 to 0 to disable heater (all other bits 1)
  configContents = (configContents & 0xF7);
  writeByte(CONFIG, configContents);
}

void MyEsensor::resetHDC2080(void)
{
  uint8_t configContents;
  m_slaveAddress =HDC2080_ADDRESS;
  
  configContents = readByte(CONFIG);

  configContents = (configContents | 0x80);
  writeByte(CONFIG, configContents);
  delay(50);
}

/* Upper two bits of the MEASUREMENT_CONFIG register controls
   the temperature resolution*/
void MyEsensor::setTempResHDC2080(int resolution)
{
  uint8_t configContents;
  m_slaveAddress =HDC2080_ADDRESS;
  
  configContents = readByte(MEASUREMENT_CONFIG);

  switch (resolution)
  {
  case 0:
    configContents = (configContents & 0x3F);
    break;

  case 1:
    configContents = (configContents & 0x7F);
    configContents = (configContents | 0x40);
    break;

  case 2:
    configContents = (configContents & 0xBF);
    configContents = (configContents | 0x80);
    break;

  default:
    configContents = (configContents & 0x3F);
  }

  writeByte(MEASUREMENT_CONFIG, configContents);
}

/*  Bits 5 and 6 of the MEASUREMENT_CONFIG register controls
    the humidity resolution*/
void MyEsensor::setHumidResHDC2080(int resolution)
{
	uint8_t configContents;
	configContents = readByte(MEASUREMENT_CONFIG);

	switch (resolution)
	{
	case 0:
		configContents = (configContents & 0xCF);
		break;

	case 1:
		configContents = (configContents & 0xDF);
		configContents = (configContents | 0x10);
		break;

	case 2:
		configContents = (configContents & 0xEF);
		configContents = (configContents | 0x20);
		break;

	default:
		configContents = (configContents & 0xCF);
	}

	writeByte(MEASUREMENT_CONFIG, configContents);
}


void MyEsensor::triggerMeasurement(void)
{
  m_slaveAddress =HDC2080_ADDRESS;
  uint8_t configContents;
  configContents = readByte(MEASUREMENT_CONFIG);

  configContents = (configContents | 0x01);
  writeByte(MEASUREMENT_CONFIG, configContents);
}

void MyEsensor::setMeasurementModeHDC2080(int mode)
{
	uint8_t configContents;
	configContents = readByte(MEASUREMENT_CONFIG);

	switch (mode)
	{
	case TEMP_AND_HUMID:
		configContents = (configContents & 0xF9);
		break;

	case TEMP_ONLY:
		configContents = (configContents & 0xFC);
		configContents = (configContents | 0x02);
		break;

	case HUMID_ONLY:
		configContents = (configContents & 0xFD);
		configContents = (configContents | 0x04);
		break;

	default:
		configContents = (configContents & 0xF9);
	}

	writeByte(MEASUREMENT_CONFIG, configContents);
}

void MyEsensor::begin(TwoWire &bus)
{

  //this flag will show if the initialization was successful
  m_initFail = 0U;

  //Set I2C bus connection
   m_i2cbus = &bus;
//   m_slaveAddress = slaveAddress;

  // Init bus
  m_i2cbus->begin();

  delay(50); //startup time of Dps310
  _initDPS368(DPS368_ADDRESS);
  _initHDC2080(HDC2080_ADDRESS);
}

