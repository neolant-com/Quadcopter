/*
 * mlx90614deck.c
 *
 *  Created on: Jul 22, 2019
 *      Author: bitcraze
 */
#include "debug.h"
#include "log.h"
#include "param.h"
#include "deck.h"
#include "mlx90614.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"


#define MLX_TASK_NAME "MLX"
#define MLX_TASK_STACKSIZE (2 * configMINIMAL_STACK_SIZE)
#define MLX_TASK_PRI 3

static bool isInit = false;
static float Ta, To1;

static void mlxTask(void *param)
{
	systemWaitStart();
	TickType_t lastWakeTime = xTaskGetTickCount();

	while(1)
	{
        vTaskDelayUntil(&lastWakeTime, M2T(100));

        int8_t ErrTa = mlx9061x_ReadTa(I2C1, MLX9061X_I2C_ADDR, &Ta);
        int8_t ErrTo1 = mlx9061x_ReadTo1(I2C1, MLX9061X_I2C_ADDR, &To1);
	if (ErrTa)
		DEBUG_PRINT("Tamb = Err%d \r\n", ErrTa);
	else
		DEBUG_PRINT("Tamb = %3.1f \r\n", Ta);
	
	if (ErrTo1)
		DEBUG_PRINT("Tobj = Err%d ", ErrTo1);
   	else
		DEBUG_PRINT("Tobj = %3.1f ", To1);
	}

}


void mlxInit(DeckInfo *info)
{
	if (isInit)
		return;
	mlx90614_init(I2C1);
	isInit = true;
	DEBUG_PRINT("MLX90614 initialized!");

    xTaskCreate(mlxTask, MLX_TASK_NAME, MLX_TASK_STACKSIZE, NULL, MLX_TASK_PRI, NULL);
}

bool mlxTest()
{
	if (ErrTa)
		DEBUG_PRINT("Tamb = Err%d \r\n", ErrTa);
	else
		DEBUG_PRINT("Tamb = %3.1f \r\n", Ta);
	
	if (ErrTo1)
		DEBUG_PRINT("Tobj = Err%d ", ErrTo1);
   	else
		DEBUG_PRINT("Tobj = %3.1f ", To1);
    	DEBUG_PRINT("MLX90614 tested!");
	return true;
}

const DeckDriver mlxDriver = {
  .vid = 0xBC,
  .pid = 0x0,
  .name = "MLX90614",


  .usedGpio = 0,

  .init = mlxInit,
  .test = mlxTest,
};

DECK_DRIVER(mlxDriver);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, mlx90614deck, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(temperature)
LOG_ADD(LOG_FLOAT, ambient, &Ta)
LOG_ADD(LOG_FLOAT, object, &To1)
LOG_GROUP_STOP(temperature)
