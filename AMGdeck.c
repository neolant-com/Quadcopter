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

#include "task.h"
//#include "stm32f4xx.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;


static bool isInit = false;

static float pixelLog[64];

static void amgTask(AMG8833_Dev_t *dev)
{
	systemWaitStart();

    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
    	vTaskDelayUntil(&lastWakeTime, M2T(100));
    	readPixels(dev, pixelLog, 64);
    }
}

static void amg8833init(AMG8833_Dev_t *dev)
{
	  // Set I2C parameters
	  I2Cx = I2C1_DEV;
	  devAddr = AMG88xx_ADDRESS;
	  bool i2c_complete = i2cdevInit(I2C1_DEV);
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
	  readPixels(dev, pixelLog, 64);

	  xTaskCreate(amgTask, AMG_TASK_NAME, AMG_TASK_STACKSIZE, NULL,
	        MULTIRANGER_TASK_PRI, NULL);
}

static bool amg8833test()
{
	if (isInit)
			{
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
LOG_ADD(LOG_FLOAT, onePixel, &pixelLog[0])
LOG_ADD(LOG_FLOAT, picture, &pixelLog)
LOG_GROUP_STOP(amg)
