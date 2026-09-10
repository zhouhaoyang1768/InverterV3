#ifndef INC_MY_MATH_H_
#define INC_MY_MATH_H_

float arm_sin_f32(float x);
float arm_cos_f32(float x);
float sine_THI(float x);
float sine_dpwm2(float x, float modulation);
float triangle_wave(float Angle, float Period);
float triangle_wave_2(float Angle, float Period);
float intermediate_60_wave(float angle, int alphas);

float map(float lo_1, float hi_1, float x, float lo_2, float hi_2);
float mod(float a, float b);

int rand(int bound);


#endif /* INC_MY_MATH_H_ */
