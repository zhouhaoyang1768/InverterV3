#include "svpwm.h"
#include "stm32h7xx_hal.h"
#include "my_math.h"
#include "current_sensing.h"

#define sqrt3_2 0.866025403784
#define sin60 0.866025403784
#define cos60 0.5
#define sqrt3 1.73205080757
#define pi ((double)3.1415926535897932384626433)

void apply_voltage_vector(int vector)
{
	switch (vector) {
		case 0:
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			break;
		case 1:
			TIM1->CCR1 = 2;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			break;
		case 2:
			TIM1->CCR1 = 2;
			TIM1->CCR2 = 2;
			TIM1->CCR3 = 0;
			break;
		case 3:
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 2;
			TIM1->CCR3 = 0;
			break;
		case 4:
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 2;
			TIM1->CCR3 = 2;
			break;
		case 5:
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 2;
			break;
		case 6:
			TIM1->CCR1 = 2;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 2;
			break;
		case 7:
			TIM1->CCR1 = 2;
			TIM1->CCR2 = 2;
			TIM1->CCR3 = 2;
			break;
		default:
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			break;
	}
}

void get_svpwm_time(float voltage_angle, float modulation, struct svpwm_time* out) {
	const static float conv = pi / 180.0f;
	const static float inv_sin60 = 1.0 / sin60;
	// float total_time;
	if (modulation > 1) modulation = 1;
	out->vector1 = 1;
	while (voltage_angle >= 360) voltage_angle -= 360;
	while (voltage_angle >= 60) {
		voltage_angle -= 60;
		++out->vector1;
	}
	out->vector2 = out->vector1 + 1;
	if (out->vector2 > 6) out->vector2 -= 6;

	out->time2 = arm_sin_f32(voltage_angle * conv) * inv_sin60;
	out->time1 = arm_cos_f32(voltage_angle * conv) - cos60 * out->time2;

	out->time2 *= sqrt3_2 * modulation;
	out->time1 *= sqrt3_2 * modulation;

	/*
	total_time = out->time2 + out->time1;
	if (total_time > 1) {
		out->time2 /= total_time;
		out->time1 /= total_time;
	}
	*/

	out->off_time = 1 - out->time1 - out->time2;
}

int dtc(double dt, float rotor_angle, float modulation, float target_frequency)
{
	const static double conv = pi / 180.0f;

	const double max_torque = 120;
	static char was_off = 1;
	static double pulse_duration = 0;
    static int torque_op = 0;
    static double torque_tolerance_coef = 0.8;

    float frequency;

	// const int P = 8;
	const double Ls = 0.14;
	const double rotor_flux = 1.4;

	double target_torque = max_torque * modulation;
	double torque_tolerance = target_torque * torque_tolerance_coef;

    double current[3];
    double ial;
    double ibe;
    double flux_al;
    double flux_be;

    double torque;

    float angle;
    int rotor_sector;
    int voltage_vector;


    get_current(current);
    ial = current[0] - cos60 * (current[1] + current[2]);
    ibe = (current[1] - current[2]) * sin60;

    flux_al = Ls * ial + rotor_flux * arm_cos_f32(rotor_angle * conv);
    flux_be = Ls * ibe + rotor_flux * arm_sin_f32(rotor_angle * conv);

    torque = 1.5 * (flux_al * ibe - flux_be * ial);
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)(current[0] / 40 * 2000) + 2000);

    if (torque < target_torque - torque_tolerance)
	{
    	torque_op = 1;
	}
    if (torque > target_torque + torque_tolerance)
    {
    	torque_op = 0;
    }

    angle = rotor_angle + 30;
    if (angle >= 360) angle -= 360;
    rotor_sector = (int)(angle / 60.0) + 1;


    if (torque_op == 1)
    {
    	voltage_vector = rotor_sector + 2;
    	if (voltage_vector > 6) voltage_vector -= 6;
    }
    else
    {
    	if (TIM1->CCR1 + TIM1->CCR2 + TIM1->CCR3 > 2)
    		voltage_vector = 7;
    	else
    		voltage_vector = 0;
    }

    pulse_duration += dt;
    if (was_off && (voltage_vector != 0 && voltage_vector != 7))
    {
    	frequency = 1.0 / pulse_duration;
    	if (frequency < target_frequency)
    	{
    		torque_tolerance_coef -= pulse_duration * 4;
    		if (torque_tolerance_coef < 0) torque_tolerance_coef = 0;
    	}
    	if (frequency > target_frequency)
    	{
    		torque_tolerance_coef += pulse_duration * 4;
    		if (torque_tolerance_coef > 1) torque_tolerance_coef = 1;
    	}
    	pulse_duration = 0;
    }
    was_off = (voltage_vector == 0 || voltage_vector == 7);
    return voltage_vector;
}
