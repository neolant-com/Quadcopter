/*
 * mlx90614.c
 *
 *  Created on: Jul 19, 2019
 *      Author: bitcraze
 *
 *  mlx90614.c - driver for mlx90614
 */

/***************************************************
  This is a library for the MLX90614 Temp Sensor
 ****************************************************/
#include "stm32f4xx_i2c.h"
#include "mlx90614.h"
#include "i2cdev.h"

//static uint8_t devAddr;
//static I2C_Dev *I2Cx;
//
////mlx90614(uint8_t i2caddr) {
////  _addr = i2caddr;
////}
//
//
//bool mlx90614_begin() {
//	i2cdevInit(I2C1_DEV);
//	I2Cx = I2C1_DEV;
//	devAddr = MLX90614_I2CADDR;
//
//	return true;
//}
//
////////////////////////////////////////////////////////
//
//
//double mlx90614_readObjectTempF(void) {
//  return (readTemp(MLX90614_TOBJ1) * 9 / 5) + 32;
//}
//
//
//double mlx90614_readAmbientTempF(void) {
//  return (readTemp(MLX90614_TA) * 9 / 5) + 32;
//}
//
//double mlx90614_readObjectTempC(void) {
//  return readTemp(MLX90614_TOBJ1);
//}
//
//
//double mlx90614_readAmbientTempC(void) {
//  return readTemp(MLX90614_TA);
//}
//
//float mlx90614_readTemp(uint8_t reg) {
//  float temp;
//
//  temp = mlx90614_ReadReg(reg);
//  temp *= .02;
//  temp  -= 273.15;
//  return temp;
//}
//
///*********************************************************************/
//
//uint16_t mlx90614_ReadReg(uint8_t a) {
//  uint16_t ret;
//  I2C_GenerateSTART();
//  i2c_Start();
//  Wire.beginTransmission(_addr); // start transmission to device
//  Wire.write(a); // sends register address to read from
//  Wire.endTransmission(false); // end transmission
//
//  Wire.requestFrom(_addr, (uint8_t)3);// send data n-bytes read
//  ret = Wire.read(); // receive DATA
//  ret |= Wire.read() << 8; // receive DATA
//
//  uint8_t pec = Wire.read();
//
//  return ret;
//}

void mlx90614_init(I2C_TypeDef* I2Cx)
{
  i2cm_init(I2Cx, MLX9061X_I2C_CLOCK);
}


uint8_t mlx90614_crc8_byte(uint8_t Data, uint8_t Crc)
{
  Crc ^= Data;
  for (uint8_t i = 0; i < 8; i++)
    Crc = Crc & 0x80 ? Crc << 1 ^ 0x07 : Crc << 1;

  return Crc;
}

uint8_t mlx90614_crc8_buff(uint8_t *p, uint8_t len)
{
  uint8_t crc = 0;

  while (len--)
    crc = mlx90614_crc8_byte(*(p++), crc);

  return crc;
}

int8_t mlx9061x_ReadReg(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t RamAddr, float *pTemp)
{
  int8_t err;
  uint8_t crc = 0;
  uint8_t ReadBuff[3];

  // Выдаём первый START на шину
  err = i2cm_Start(I2Cx, MLX9061X_I2C_ADDR, 0, MLX9061X_I2C_TO);
  crc = mlx90614_crc8_byte(MLX9061X_I2C_ADDR, crc);
  if (err) return err;
  // Выдаём команду (внутренний адрес) в mlx9061x
  err = i2cm_WriteBuff(I2Cx, &RamAddr, 1, MLX9061X_I2C_TO);
  crc = mlx90614_crc8_byte(RamAddr, crc);
  if (err) return err;

  // Выдаём повторный START перед началом чтения
  err = i2cm_Start(I2Cx, MLX9061X_I2C_ADDR, 1, MLX9061X_I2C_TO);
  crc = mlx90614_crc8_byte(MLX9061X_I2C_ADDR | 1, crc);
  if (err) return err;

  // Читаем 2 байта и CRC (PEC)
  err = i2cm_ReadBuffAndStop(I2Cx, ReadBuff, 3, MLX9061X_I2C_TO);
  if (err) return err;

  // Считаем CRC и сверяем с байтом PEC
  for (uint8_t i = 0; i < 3; i++)
    crc = mlx90614_crc8_byte(ReadBuff[i], crc);
  if (crc)
    return MLX9061X_ERR_BadChksum;

  // Пересчитываем в физ. величины
  uint16_t Temp16 = ReadBuff[0] | (((uint16_t)ReadBuff[1]) << 8);

  if (Temp16 & 0x8000)
    return MLX9061X_ERR_MeasErr;

  *pTemp = (float)(Temp16 - 13658) / 50;       // 273.16 * 50 = 13658

  return MLX9061X_ERR_Ok;
}


int8_t mlx9061x_ReadTa(I2C_TypeDef* I2Cx, uint8_t slave_addr, float *pTemp)
{
  return mlx9061x_ReadReg(I2Cx, slave_addr, 6, pTemp);
}


int8_t mlx9061x_ReadTo1(I2C_TypeDef* I2Cx, uint8_t slave_addr, float *pTemp)
{
  return mlx9061x_ReadReg(I2Cx, slave_addr, 7, pTemp);
}


int8_t mlx9061x_ReadTo2(I2C_TypeDef* I2Cx, uint8_t slave_addr, float *pTemp)
{
  return mlx9061x_ReadReg(I2Cx, slave_addr, 8, pTemp);
}
