#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_rcc.h>
#include "i2cm.h"

void i2cm_init(I2C_TypeDef* I2Cx, uint32_t i2c_clock)
{
  if (I2Cx == I2C1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  I2C_InitTypeDef i2c_InitStruct;
  i2c_InitStruct.I2C_Mode = I2C_Mode_I2C;
  i2c_InitStruct.I2C_ClockSpeed = i2c_clock;
  i2c_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  i2c_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  i2c_InitStruct.I2C_Ack = I2C_Ack_Enable;
  i2c_InitStruct.I2C_OwnAddress1 = 0;
  I2C_Cmd(I2Cx, ENABLE);
  I2C_Init(I2Cx, &i2c_InitStruct);

  GPIO_InitTypeDef InitStruct;
  InitStruct.GPIO_Mode = GPIO_Mode_AF;
  InitStruct.GPIO_OType = GPIO_OType_OD;
  InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  if (I2Cx == I2C1)
    InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

  GPIO_Init(GPIOB, &InitStruct);
}

int8_t i2cm_Start(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t IsRead, uint16_t TimeOut)
{
  uint16_t TOcntr;

  I2C_GenerateSTART(I2Cx, ENABLE);
  TOcntr = TimeOut;
  while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) && TOcntr) {TOcntr--;}
  if (!TOcntr)
    return I2C_ERR_HWerr;

  if (IsRead)
  {
    I2C_Send7bitAddress(I2Cx, slave_addr, I2C_Direction_Receiver);
    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && TOcntr) {TOcntr--;}
  }
  else
  {
    I2C_Send7bitAddress(I2Cx, slave_addr, I2C_Direction_Transmitter);
    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && TOcntr) {TOcntr--;}
  }

  if (!TOcntr)
      return I2C_ERR_NotConnect;

  return I2C_ERR_Ok;
}

int8_t i2cm_Stop(I2C_TypeDef* I2Cx, uint16_t TimeOut)
{
  I2C_GenerateSTOP(I2Cx, ENABLE);
  uint16_t TOcntr = TimeOut;
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) && TOcntr);
  if (!TOcntr)
    return I2C_ERR_HWerr;

  return I2C_ERR_Ok;
}


int8_t i2cm_WriteBuff(I2C_TypeDef* I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut)
{
  uint16_t TOcntr;

  while (len--)
  {
    I2C_SendData(I2Cx, *(pbuf++));
    TOcntr = TimeOut;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF) && TOcntr) {TOcntr--;}
    if (!TOcntr)
      return I2C_ERR_NotConnect;
  }

  return I2C_ERR_Ok;
}


int8_t i2cm_ReadBuffAndStop(I2C_TypeDef* I2Cx, uint8_t *pbuf, uint16_t len, uint16_t TimeOut)
{
  uint16_t TOcntr;


  I2C_AcknowledgeConfig(I2Cx, ENABLE);

  if (len == 1)
  {

    I2C_AcknowledgeConfig(I2Cx, DISABLE);


    __disable_irq();
    (void) I2Cx->SR2;                           // ADDR
    I2C_GenerateSTOP(I2Cx,ENABLE);              // STOP
    __enable_irq();


    TOcntr = TimeOut;
    while ((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE)) && TOcntr) {TOcntr--;}
    *pbuf++ = I2C_ReceiveData(I2Cx);
  }
  else if (len == 2)
  {

    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);


    __disable_irq();
    (void) I2Cx->SR2;                           // ADDR
    I2C_AcknowledgeConfig(I2Cx, DISABLE);       //  ACK
    __enable_irq();


    TOcntr = TimeOut;
    while ((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF)) && TOcntr) {TOcntr--;}

    __disable_irq();
    I2C_GenerateSTOP(I2Cx, ENABLE);
    *pbuf++ = I2Cx->DR;
    __enable_irq();


    *pbuf++ = I2Cx->DR;
  }
  else
  {
    (void) I2Cx->SR2;                           // ADDR
    while (len-- != 3)
    {

      TOcntr = TimeOut;
      while ((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF)) && TOcntr) {TOcntr--;}
      *pbuf++ = I2C_ReceiveData(I2Cx);
    }

    TOcntr = TimeOut;
    while ((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF)) && TOcntr) {TOcntr--;}

    // EV7_2
   __disable_irq();
    *pbuf++ = I2C_ReceiveData(I2Cx);
    I2C_GenerateSTOP(I2Cx,ENABLE);               // STOP
    __enable_irq();

    *pbuf++ = I2C_ReceiveData(I2Cx);



    I2C_AcknowledgeConfig(I2Cx, DISABLE);


    TOcntr = TimeOut;
    while ((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) && TOcntr) {TOcntr--;}
    *pbuf++ = I2C_ReceiveData(I2Cx);

    len = 0;
  }

  TOcntr = TimeOut;
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) && TOcntr);
  if (!TOcntr)
    return I2C_ERR_HWerr;

  return I2C_ERR_Ok;
}
//==============================================================================
