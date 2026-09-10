#include "cooling.h"
#include "stm32h7xx_hal.h"


void cooling_initialize()
{

}

void cooling_tick(float dt)
{

}

void turn_on_fan()
{
	// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
}

void turn_off_fan()
{
	// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
}
