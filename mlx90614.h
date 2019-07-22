///*
// * mlx90614.h
// *
// *  Created on: Jul 19, 2019
// *      Author: bitcraze
// */
//
//#ifndef SRC_DRIVERS_INTERFACE_MLX90614_H_
//#define SRC_DRIVERS_INTERFACE_MLX90614_H_
//
//#include <stdint.h>
//
///***************************************************
//  This is a library for the MLX90614 Temp Sensor
// ****************************************************/
//
//
//#define MLX90614_I2CADDR 0x5A
//
//// RAM
//#define MLX90614_RAWIR1 0x04
//#define MLX90614_RAWIR2 0x05
//#define MLX90614_TA 0x06
//#define MLX90614_TOBJ1 0x07
//#define MLX90614_TOBJ2 0x08
//// EEPROM
//#define MLX90614_TOMAX 0x20
//#define MLX90614_TOMIN 0x21
//#define MLX90614_PWMCTRL 0x22
//#define MLX90614_TARANGE 0x23
//#define MLX90614_EMISS 0x24
//#define MLX90614_CONFIG 0x25
//#define MLX90614_ADDR 0x0E
//#define MLX90614_ID1 0x3C
//#define MLX90614_ID2 0x3D
//#define MLX90614_ID3 0x3E
//#define MLX90614_ID4 0x3F
//
//#define MLX90614_ERR_Ok 0
//#define MLX90614_ERR_NotConnect -1
//#define MLX90614_ERR_BadChksum -2
//#define MLX90614_ERR_HWerr -3
//#define MLX90614_ERR_MeasErr -10
//
////void mlx90614(uint8_t addr = MLX90614_I2CADDR);
//bool begin();
//uint32_t readID(void);
//
//double readObjectTempC(void);
//double readAmbientTempC(void);
//double readObjectTempF(void);
//double readAmbientTempF(void);
//
//float readTemp(uint8_t reg);
//
//uint8_t _addr;
//uint16_t mlx90614_ReadReg(uint8_t addr);
//void write16(uint8_t addr, uint16_t data);
//
//
//#endif /* SRC_DRIVERS_INTERFACE_MLX90614_H_ */

#ifndef _MLX9061X_H
#define _MLX9061X_H

#include "i2cm.h"


#define MLX9061X_I2C_ADDR       0xB4
#define MLX9061X_I2C_CLOCK      100000
#define MLX9061X_I2C_TO         1000


#define MLX9061X_ERR_Ok         I2C_ERR_Ok
#define MLX9061X_ERR_NotConnect I2C_ERR_NotConnect
#define MLX9061X_ERR_BadChksum  I2C_ERR_BadChksum
#define MLX9061X_ERR_HWerr      I2C_ERR_HWerr
#define MLX9061X_ERR_MeasErr    -10




void mlx90614_init(I2C_TypeDef* I2Cx);

int8_t mlx9061x_ReadReg(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t RamAddr, float *pTemp);

int8_t mlx9061x_ReadTa(I2C_TypeDef* I2Cx, uint8_t slave_addr, float *pTemp);
int8_t mlx9061x_ReadTo1(I2C_TypeDef* I2Cx, uint8_t slave_addr, float *pTemp);
int8_t mlx9061x_ReadTo2(I2C_TypeDef* I2Cx, uint8_t slave_addr, float *pTemp);


#endif
