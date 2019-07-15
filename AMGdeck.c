/*
 * AMGdeck.c
 *
 *  Created on: Jun 21, 2019
 *      Author: bitcraze
 */



#define DEBUG_MODULE "AMG8833"
#define AMG_TASK_NAME "AMG"
#define AMG_TASK_STACKSIZE (2 * configMINIMAL_STACK_SIZE)
#define AMG_TASK_PRI 3


#include "system.h"
#include "amg8833.h"
#include "deck.h"
#include "debug.h"
#include "param.h"
#include "log.h"

//#include "stm32f4xx.h"
//typedef AMG8833_Dev_t amgDev;
static bool isInit = false;

static float pixelLog[64];

static void amgTask(AMG8833_Dev_t *amgDev)
{
	systemWaitStart();

    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
    	vTaskDelayUntil(&lastWakeTime, M2T(100));
    	readPixels(amgDev, pixelLog, 64);
    }
}

static void amg8833init()
{
	DEBUG_PRINT("AMG8833 driver initialized!");
	isInit = true;
}

static bool amg8833test()
{
	AMG8833_Dev_t *amgDev;
	amgDev->I2Cx = I2C1_DEV;
	amgDev->devAddr = AMG88xx_ADDRESS;
	if (isInit)
			{
				begin(amgDev, I2C1_DEV);
				readPixels(amgDev, pixelLog, 64);
				DEBUG_PRINT("AMG8833 driver tested!\n");
			}
	return true;
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

LOG_GROUP_START(amg)
LOG_ADD(LOG_FLOAT, bottomRight, &pixelLog[0])
LOG_ADD(LOG_FLOAT, bottomLeft, &pixelLog[7])
LOG_ADD(LOG_FLOAT, topRight, &pixelLog[55])
LOG_ADD(LOG_FLOAT, topLeft, &pixelLog[63])
LOG_ADD(LOG_FLOAT, picture, &pixelLog)
LOG_GROUP_STOP(amg)
