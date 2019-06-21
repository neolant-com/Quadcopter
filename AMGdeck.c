/*
 * AMGdeck.c
 *
 *  Created on: Jun 21, 2019
 *      Author: bitcraze
 */


#define DEBUG_MODULE "AMG8833"

#include "amg8833.h"
#include "deck.h"
#include "debug.h"
#include "param.h"
//#include "stm32f4xx.h"

const float AMG88xx_TEMP_CONVERSION = 0.25;
const float AMG88xx_THRM_CONVERSION = 0.0625;

static uint8_t mode = 1;

static bool isInit = false;

static void amg8833init(AMG8833_Dev_t *dev)
{
	  // Set I2C parameters
	  dev->I2Cx = I2C1_DEV;
	  dev->devAddr = AMG88xx_ADDRESS;
	  bool i2c_complete = i2cdevInit(dev->I2Cx);
	  // Enter normal mode
	  bool mode_selected = write8(dev, AMG88xx_PCTL, AMG88xx_NORMAL_MODE);
	  // Software reset
	  bool software_resetted = write8(dev, AMG88xx_RST, AMG88xx_INITIAL_RESET);
	  //disable interrupts by default
	  bool interrupts_set = disableInterrupt(dev);
	  //set to 10 FPS
	  bool fps_set = write8(dev, AMG88xx_FPSC, (AMG88xx_FPS_10 & 0x01));
	  vTaskDelay(M2T(10));
	  DEBUG_PRINT("AMG8833 deck driver initialized!\n");
	  isInit = i2c_complete && mode_selected && software_resetted &&
	    interrupts_set && fps_set;
}

static bool amg8833test()
{
	DEBUG_PRINT("AMG8833 driver tested!\n");
	return true;
}

/**************************************************************************
 Read Infrared sensor values

 @param  pdev Thermal camera struct
 @param  buf the array to place the pixels in
 @param  size Optional number of bytes to read (up to 64). Default is 64 bytes.
 @return up to 64 bytes of pixel data in buf
**************************************************************************/
void readPixels(AMG8833_Dev_t *dev, float *buf, uint8_t size)
{
  uint16_t recast;
  float converted;
  uint8_t bytesToRead = min((uint8_t) (size << 1), (uint8_t) (AMG88xx_PIXEL_ARRAY_SIZE << 1));
  uint8_t rawArray[bytesToRead];
  read(dev, AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);

  for (int i = 0; i < size; i++) {
    uint8_t pos = i << 1;
    recast = ((uint16_t) rawArray[pos + 1] << 8) | ((uint16_t) rawArray[pos]);
    converted = int12ToFloat(recast) * AMG88xx_TEMP_CONVERSION;
    buf[i] = converted;
  }
}

/**************************************************************************
 Read the onboard thermistor

 @param  pdev Thermal camera struct
 @returns a the floating point temperature in degrees Celsius
**************************************************************************/
float readThermistor(AMG8833_Dev_t *dev)
{
  uint8_t raw[2];
  read(dev, AMG88xx_TTHL, raw, 2);
  uint16_t recast = ((uint16_t) raw[1] << 8) | ((uint16_t) raw[0]);
  return signedMag12ToFloat(recast) * AMG88xx_THRM_CONVERSION;
}

/**************************************************************************
 Enables the interrupt pin on the device.

 @param  pdev Thermal camera struct
**************************************************************************/
bool enableInterrupt(AMG8833_Dev_t *dev)
{
  // 0 = Difference interrupt mode
  // 1 = absolute value interrupt mode
  return write8(dev, AMG88xx_INTC, (mode << 1 | 1) & 0x03);
}

/**************************************************************************
 Disables the interrupt pin on the device

 @param  pdev Thermal camera struct
**************************************************************************/
bool disableInterrupt(AMG8833_Dev_t *dev)
{
  // 0 = Difference interrupt mode
  // 1 = absolute value interrupt mode
  return write8(dev, AMG88xx_INTC, (mode << 1 | 0) & 0x03);
}

/**************************************************************************
 Set the interrupt to either absolute value or difference mode

 @param  pdev Thermal camera struct
 @param  mode passing AMG88xx_DIFFERENCE sets the device to difference
 mode, AMG88xx_ABSOLUTE_VALUE sets to absolute value mode.
**************************************************************************/
void setInterruptMode(AMG8833_Dev_t *dev, uint8_t m)
{
  mode = m;
  write8(dev, AMG88xx_INTC, (mode << 1 | 1) & 0x03);
}

/**************************************************************************
 Read the state of the triggered interrupts on the device. The full
 interrupt register is 8 bytes in length.

 @param  pdev Thermal camera struct
 @param  buf the pointer to where the returned data will be stored
 @param  size Optional number of bytes to read. Default is 8 bytes.
 @returns up to 8 bytes of data in buf
**************************************************************************/
void getInterrupt(AMG8833_Dev_t *dev, uint8_t *buf, uint8_t size)
{
  uint8_t bytesToRead = min(size, (uint8_t) 8);
  read(dev, AMG88xx_INT_OFFSET, buf, bytesToRead);
}

/**************************************************************************
 Clear any triggered interrupts

 @param  pdev Thermal camera struct
**************************************************************************/
void clearInterrupt(AMG8833_Dev_t *dev)
{
  write8(dev, AMG88xx_RST, AMG88xx_FLAG_RESET);
}

/**************************************************************************
 Set the interrupt levels. The hysteresis value defaults to .95 * high

 @param  pdev Thermal camera struct
 @param  high the value above which an interrupt will be triggered
 @param  low the value below which an interrupt will be triggered
**************************************************************************/
void setInterruptLevels_N(AMG8833_Dev_t *dev, float high, float low)
{
  setInterruptLevels_H(dev, high, low, high * 0.95f);
}

/**************************************************************************
 Set the interrupt levels

 @param  pdev Thermal camera struct
 @param  high the value above which an interrupt will be triggered
 @param  low the value below which an interrupt will be triggered
 @param  hysteresis the hysteresis value for interrupt detection
**************************************************************************/
void setInterruptLevels_H(AMG8833_Dev_t *dev, float high, float low,
  float hysteresis)
{
  int highConv = high / AMG88xx_TEMP_CONVERSION;
  highConv = constrain(highConv, -4095, 4095);
  write8(dev, AMG88xx_INTHL, (highConv & 0xFF));
  write8(dev, AMG88xx_INTHH, ((highConv & 0xF) >> 4));

  int lowConv = low / AMG88xx_TEMP_CONVERSION;
  lowConv = constrain(lowConv, -4095, 4095);
  write8(dev, AMG88xx_INTLL, (lowConv & 0xFF));
  write8(dev, AMG88xx_INTLH, (((lowConv & 0xF) >> 4) & 0xF));

  int hysConv = hysteresis / AMG88xx_TEMP_CONVERSION;
  hysConv = constrain(hysConv, -4095, 4095);
  write8(dev, AMG88xx_IHYSL, (hysConv & 0xFF));
  write8(dev, AMG88xx_IHYSH, (((hysConv & 0xF) >> 4) & 0xF));
}

/**************************************************************************
 Set the moving average mode.

 @param  pdev Thermal camera struct
 @param  mode If false, no moving average. If true, twice the moving average
**************************************************************************/
void setMovingAverageMode(AMG8833_Dev_t *dev, bool mode)
{
  write8(dev, AMG88xx_AVE, (mode << 5));
}

/**************************************************************************
 Read one byte of data from the specified register

 @param  dev Thermal camera struct
 @param  reg the register to read
 @returns one byte of register data
**************************************************************************/
uint8_t read8(AMG8833_Dev_t *dev, uint8_t reg)
{
  uint8_t ret;
  read(dev, reg, &ret, 1);
  return ret;
}

/**************************************************************************
 Read a chunk of bytes of data from the specified register

 @param  dev Thermal camera struct
 @param  reg the register to read
 @param  buf integer buffer to save read bytes
 @param  num number of bytes need to be read
**************************************************************************/
void read(AMG8833_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t num)
{
  i2cdevReadReg8(dev->I2Cx, dev->devAddr, reg, num, buf);
}

/**************************************************************************
 Write one byte of data to the specified register

 @param  dev Thermal camera struct
 @param  reg the register to write to
 @param  value the value to write
 @returns result of the write operation
**************************************************************************/
bool write8(AMG8833_Dev_t *dev, uint16_t reg, uint8_t value)
{
  return (i2cdevWrite16(dev->I2Cx, dev->devAddr, reg, 1, &value));
}

/**************************************************************************
 Read one byte of data from the specified register

 @param  dev Thermal camera struct
 @param  reg the register to write
 @param  buf integer buffer containing data to write
 @param  num number of bytes need to be written
**************************************************************************/
void write(AMG8833_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t num)
{
  for (int i = 0; i < num; i++) {
    write8(dev, reg, buf[i]);
  }
}

/**************************************************************************
 Convert a 12-bit signed magnitude value to a floating point number

 @param  val the 12-bit signed magnitude value to be converted
 @returns the converted floating point value
**************************************************************************/
float signedMag12ToFloat(uint16_t val)
{
  // Take first 11 bits as absolute val
  uint16_t absVal = (val & 0x7FF);
  return (val & 0x800) ? 0 - (float) absVal : (float) absVal;
}

/**************************************************************************
 Convert a 12-bit integer two's complement value to a floating point number

 @param  val the 12-bit integer  two's complement value to be converted
 @returns the converted floating point value
**************************************************************************/
float int12ToFloat(uint16_t val)
{
  // Shift to left so that sign bit of 12 bit integer number is placed on
  // sign bit of 16 bit signed integer number
  int16_t sVal = (val << 4);
  // Shift back the signed number, return converts to float
  return sVal >> 4;
}

/**************************************************************************
 Finds the minimum value between two integers

 @param  a first integer value
 @param  b second integer value
 @returns the minimum of a and b integers
**************************************************************************/
uint8_t min(uint8_t a, uint8_t b)
{
  return (a < b) ? a : b;
}



static const DeckDriver amg8833Driver = {
		.vid = 0xBC,
		.pid = 0x69,
		.name = "amg8833deck",

		.usedGpio = 0,
		.init = amg8833init,
		.test = amg8833test,
};

DECK_DRIVER(amg8833Driver);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, amg8833deck, &isInit)
PARAM_GROUP_STOP(deck)
