/*
 * svpwm.h
 *
 *  Created on: Dec 21, 2025
 *      Author: zhouh
 */

#ifndef INC_SVPWM_H_
#define INC_SVPWM_H_

struct svpwm_time;
struct svpwm_time {
	int vector1;
	float time1;
	int vector2;
	float time2;
	float off_time;
};

void apply_voltage_vector(int vector);
void get_svpwm_time(float voltage_angle, float modulation, struct svpwm_time* out);
int dtc(double dt, float rotor_angle, float modulation, float target_frequency);

#endif /* INC_SVPWM_H_ */
