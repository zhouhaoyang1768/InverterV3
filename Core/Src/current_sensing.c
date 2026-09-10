#include "current_sensing.h"
#include "stm32h7xx_hal.h"

const double max_current = 198;

uint32_t initial_adc_result[3];
uint32_t adc_result[2];
double current_coef[2];

extern ADC_HandleTypeDef hadc1, hadc2;

void do_adc()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	while (!(ADC1->ISR & ADC_ISR_EOC_Msk));
	while (!(ADC2->ISR & ADC_ISR_EOC_Msk));
	adc_result[0] = ADC1->DR;
	adc_result[1] = ADC2->DR;
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);

}
void initialize_current_sensing()
{
	HAL_Delay(500);
	do_adc();
	for (int i = 0; i < 2; ++i)
	{
		initial_adc_result[i] = adc_result[i];
		current_coef[i] = max_current / adc_result[i];
	}
}
void get_current(double current[3])
{
	do_adc();
	current[1] = ((int)adc_result[0] - (int)initial_adc_result[0]) * current_coef[0];
	current[2] = ((int)adc_result[1] - (int)initial_adc_result[1]) * current_coef[1];
	current[0] = 0 - current[2] - current[1];
}
