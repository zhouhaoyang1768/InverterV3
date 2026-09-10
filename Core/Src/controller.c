#include "controller.h"

#include "stm32h7xx_hal.h"
#include "my_math.h"
#include "math.h"
#include "svpwm.h"
#include "angles.h"
#include "rotor_position_sensing.h"

#define _ 0

#define MODE_OFF 0
#define MODE_ASYNC_SVPWM 1
#define MODE_ASYNC_THI 2
#define MODE_SYNC_SVPWM 3
#define MODE_DTC 4
#define MODE_WIDE3 5
#define MODE_SQUARE 6
#define MODE_SYNC_SHEPWM 7
#define MODE_SYNC_CHMPWM 8
#define MODE_SYNC_CHMPWM_1F 9


#define ACCL_OFF '0'
#define ACCL_ACCL '1'
#define ACCL_KEEP '2'
#define ACCL_DECL '3'

#define pi ((float)3.1415926535897932384626433)

void modulation_async_svpwm(double dt);
void modulation_async_svpwm_thi();
void modulation_sync_svpwm(float modulation, int pulses);
void modulation_dtc(float modulation, float pulses);
void modulation_wide3(float modulation);
void modulation_square();
void modulation_shepwm(float modulation, int pulses);
void modulation_chmpwm(float modulation, int pulses);
void modulation_chmpwm_1fold(float modulation, int pulses);

float get_target_modulation(void);



struct modulation_properties
{
	uint8_t modulation_type;
	float frequency_low_bound;
	float frequency_high_bound;
	float pulses_low;
	float pulses_high;
	float modulation_low;
	float modulation_high;
	uint8_t random;
};


static const struct modulation_properties modulation_off = {MODE_OFF};


// position, velocity, acceleration
float rotor_acceleration;	// Hz/s
float rotor_speed;			// Hz
float rotor_angle;			// degree 0 - 360

int previous_rotor_sector;
float previous_rotor_speed;
float previous_rotor_angle;

float time_between_sectors;

// controls
struct modulation_properties const *current_modulation_props;
char accelerate;
float target_modulation;
float modulation;

// triangle wave for asynchronous mode
float async_pulses;
float trig_wave_angle;

// time lapsed
double dt;




// ADTranz GTO
/**

#define accl_speed 3.5f;
#define decl_speed -3.5f;
float get_target_modulation() {
	float target = .9 * rotor_speed/100.0 + 0.02;
	return target;
}
static const struct modulation_properties modulation_list_accelerate[] = {
//   Mode               Freq(L)   Freq(H) Pulses(L) Pulese(H) Random
	{MODE_ASYNC_SVPWM,    0.0f,    11.0f,      250},
	{MODE_DTC,           11.0f,   155.0f,      720},
	{MODE_WIDE3,         55.0f,    62.0f},
	{MODE_SQUARE,        62.0f,   350.0f},

};
static const struct modulation_properties modulation_list_decelerate[] = {
//   Mode               Freq(L)   Freq(H) Pulses(L) Pulese(H) Random
	{MODE_ASYNC_SVPWM,    0.0f,    11.0f,      250},
	{MODE_DTC,           11.0f,   155.0f,      720},
	{MODE_WIDE3,         55.0f,    62.0f},
	{MODE_SQUARE,        62.0f,   350.0f},
};
/**/

/**/

#define accl_speed 3.5f;
#define decl_speed -3.5f;
float get_target_modulation() {
	float target = .5 * rotor_speed/100.0;
	if (target < 0.02) target = 0.02;
	// if (target > 0.1) target = 0.1;
	return target;
}
static const struct modulation_properties modulation_list_accelerate[] = {
//   Mode               Freq(L)   Freq(H) Pulses(L) Pulese(H) Random
	{MODE_ASYNC_SVPWM,    0.0f,     35.0f,     1000},
	{MODE_SYNC_SVPWM,    35.0f,     60.0f,       15},
	{MODE_SYNC_SVPWM,    60.0f,   1100.0f,        9}

};
static const struct modulation_properties modulation_list_decelerate[] = {
//   Mode               Freq(L)   Freq(H) Pulses(L) Pulese(H) Random
	{MODE_ASYNC_SVPWM,    0.0f,    35.0f,     1000},
	{MODE_SYNC_SVPWM,    35.0f,    60.0f,       15},
	{MODE_SYNC_SVPWM,    60.0f,  1100.0f,        9}
};
/**/
/**
// Siemons GTO

#define accl_speed  6.0f;
#define decl_speed -6.0f;
float get_target_modulation() {
	return 0.7 * rotor_speed/100.0;
}

static const struct modulation_properties modulation_list_accelerate[] = {
//   Mode               Freq(L)   Freq(H)   Pulses
	{MODE_ASYNC_THI,      0.0f,     1.0f,      100},
	{MODE_ASYNC_THI,      1.0f,    15.0f,      100,    450},
	{MODE_ASYNC_THI,     15.0f,    30.0f,      450},
	{MODE_SYNC_CHMPWM,   30.0f,    45.0f,      9},
	{MODE_SYNC_CHMPWM,   45.0f,    65.0f,      7},
	{MODE_SYNC_CHMPWM,   65.0f,    172.0f,      5},
	//{MODE_SYNC_CHMPWM,   72.0f,    85.0f,      3},
	//{MODE_SYNC_WIDE3,    85.0f,    92.0f,      _,        _,    0.9,    0.9},
	//{MODE_SYNC_SQUARE,   92.0f,   300.0f},


};
static const struct modulation_properties modulation_list_decelerate[] = {
//   Mode               Freq(L)   Freq(H)   Pulses
	{MODE_ASYNC_THI,      0.0f,     1.0f,      100},
	{MODE_ASYNC_THI,      1.0f,    15.0f,      100,    450},
	{MODE_ASYNC_THI,     15.0f,    30.0f,      450,    450},
	{MODE_SYNC_CHMPWM,   30.0f,    45.0f,      9},
	{MODE_SYNC_CHMPWM,   45.0f,    65.0f,      7},
	{MODE_SYNC_CHMPWM,   65.0f,    172.0f,      5},
	{MODE_SYNC_CHMPWM,   72.0f,    85.0f,      3},
	//{MODE_SYNC_WIDE3,    85.0f,    92.0f,      _,        _,    0.9,    0.9},
	//{MODE_SYNC_SQUARE,   92.0f,   300.0f},
};
/**/

static const uint8_t accelerate_list_length = sizeof(modulation_list_accelerate) / sizeof(modulation_list_accelerate[0]);
static const uint8_t decelerate_list_length = sizeof(modulation_list_decelerate) / sizeof(modulation_list_decelerate[0]);





void controller_initialize()
{
	// The current modulation type
	current_modulation_props = &modulation_off;

	// Fields related to PWM modulation index
	target_modulation = 0.0f;
	modulation = 0.0f;

	// Fields related to async modulation
	trig_wave_angle = 0.0f;
	async_pulses = 1;

	// Fields related to the Remote Controller
	accelerate = ACCL_OFF;

	// Fields related to rotor position sensing
	previous_rotor_sector = get_rotor_sector();
	previous_rotor_angle = (previous_rotor_sector - 1) * 60.0;
	previous_rotor_speed = 0;

	rotor_angle = previous_rotor_angle;
	rotor_speed = 0.0f;
	rotor_acceleration = 0;

	time_between_sectors = 0;
}

void modulate()
{
	float _modulation = modulation;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
	if (current_modulation_props->modulation_low != 0 || current_modulation_props->modulation_high != 0) {
		_modulation = map(current_modulation_props->frequency_low_bound,
						current_modulation_props->frequency_high_bound,
						rotor_speed,
						current_modulation_props->modulation_low,
						current_modulation_props->modulation_high);
	}
	switch(current_modulation_props->modulation_type)
	{
		case MODE_OFF:
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			break;
		case MODE_ASYNC_SVPWM:
			modulation_async_svpwm(dt);
			break;
		case MODE_ASYNC_THI:
			modulation_async_svpwm_thi();
			break;
		case MODE_SYNC_SVPWM:
			modulation_sync_svpwm(_modulation, current_modulation_props->pulses_low);
			break;
		case MODE_DTC:
			modulation_dtc(modulation, current_modulation_props->pulses_low);
			break;
		case MODE_WIDE3:
			modulation_wide3(modulation);
			break;
		case MODE_SQUARE:
			modulation_square();
			break;
		case MODE_SYNC_SHEPWM:
			modulation_shepwm(_modulation, current_modulation_props->pulses_low);
			break;
		case MODE_SYNC_CHMPWM:
			modulation_chmpwm(_modulation, current_modulation_props->pulses_low);
			break;
		case MODE_SYNC_CHMPWM_1F:
			modulation_chmpwm_1fold(_modulation, current_modulation_props->pulses_low);
			break;
		default:
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
	}
}

inline static void choose_modulation_mode()
{
	uint8_t i;
	static int previous_sector = 8;
	int sector;
	char mode = accelerate;
	const struct modulation_properties* modulation_list;
	int list_length;

	if (target_modulation == 0 && modulation == 0)
	{
		current_modulation_props = &modulation_off;
		return;
	}

	sector = (int)(rotor_angle / 60);
	if (sector == previous_sector) return;
	else previous_sector = sector;

	switch(mode)
	{
		case ACCL_ACCL:
			modulation_list = modulation_list_accelerate;
			list_length = accelerate_list_length;
			break;
		case ACCL_DECL:
			modulation_list = modulation_list_decelerate;
			list_length = decelerate_list_length;
			break;
		case ACCL_KEEP:
			modulation_list = modulation_list_decelerate;
			list_length = decelerate_list_length;
			break;
		case ACCL_OFF:
			modulation_list = modulation_list_decelerate;
			list_length = decelerate_list_length;
			break;
		default:
			modulation_list = modulation_list_decelerate;
			list_length = decelerate_list_length;
			break;
	}

	for(i = 0; i < list_length; ++i)
	{
		if (rotor_speed >= modulation_list[i].frequency_low_bound &&
			rotor_speed <  modulation_list[i].frequency_high_bound)
		{
			current_modulation_props = &modulation_list[i];
			break;
		}
	}
}


inline static void update_target_modulation(double dt)
{
	if (accelerate == ACCL_OFF) {
		target_modulation = 0;
	} else {
		target_modulation = get_target_modulation();
	}

	if (modulation < target_modulation)
	{
		modulation += dt * 2;
		if (modulation > target_modulation) modulation = target_modulation;
	}
	if (modulation > target_modulation)
	{
		modulation -= dt * 2;
		if (modulation < target_modulation) modulation = target_modulation;
	}
}

inline static void update_rotor(double dt)
{
	float rotor_stats[3];
	int detected_rotor_direction;
	int detected_rotor_sector;
	float detected_rotor_angle;
	float detected_rotor_speed;
	float detected_rotor_acceleration;
	float inv_sector_dt;
	float sector_angle_lower_bound;
	float sector_angle_upper_bound;
	float sector_tolerance = 5.0f;


	// First do an update anyway
	rotor_speed += rotor_acceleration * dt;
	if (rotor_speed < .2) rotor_speed = 0.2;

	rotor_angle += rotor_speed * 360.0f * dt;
	if (rotor_angle >= 360) rotor_angle -= 360;

	//
	time_between_sectors += dt;
	detected_rotor_sector = get_rotor_sector();

	// If the rotor switched from one sector to another sector,
	// we update the position, speed, and acceleration of the rotor
	if (detected_rotor_sector != previous_rotor_sector) {
		// Calculate
		inv_sector_dt = 1.0f / time_between_sectors;

		detected_rotor_direction = get_rotor_direction(detected_rotor_sector, previous_rotor_sector);
		detected_rotor_angle = get_rotor_angle(detected_rotor_sector, previous_rotor_sector);
		detected_rotor_speed = inv_sector_dt * get_rotor_displacement(detected_rotor_direction, detected_rotor_angle, previous_rotor_angle);
		detected_rotor_acceleration = inv_sector_dt * (detected_rotor_speed - previous_rotor_speed);

		// Update
		previous_rotor_angle = detected_rotor_angle;
		previous_rotor_speed = detected_rotor_speed;

		rotor_angle = detected_rotor_angle;
		rotor_speed = detected_rotor_speed;
		rotor_acceleration = detected_rotor_acceleration;

		time_between_sectors = 0;
	}
	else
	{
		// Check if our simulated rotor is within the current detected sector
		get_sector_boundry_angles(detected_rotor_sector, &sector_angle_lower_bound, &sector_angle_upper_bound);
		if ((sector_angle_lower_bound < rotor_angle - sector_tolerance && rotor_angle + sector_tolerance < sector_angle_upper_bound)
			|| (sector_angle_lower_bound < rotor_angle - sector_tolerance - 360 && rotor_angle + sector_tolerance - 360 < sector_angle_upper_bound))
		{
			// Simulated rotor is within the detected sector, nothing to do.
		}
		else
		{
			rotor_angle = (sector_angle_lower_bound + sector_angle_upper_bound) * 0.5;

			if (rotor_angle < 0) rotor_angle += 360;
			if (360 <= rotor_angle) rotor_angle -= 360;
		}
	}



//	rotor_angle = 0;
//	rotor_speed = 0;
//	rotor_acceleration = 0;
}

void update_accelerate(double dt)
{
	uint32_t input_reading;
	char direction = 1;
	int32_t power;

	static double time = 0;
	time += dt;

	input_reading = (GPIOG->IDR >> 2) & 0b1111111;

	direction = (input_reading & 0b1000000) ? 1 : 0;
	power = input_reading & 0b1111;

	if (power > 0b1000) {
		accelerate = ACCL_ACCL;
		rotor_acceleration = accl_speed;
	}
	else {
		accelerate = ACCL_DECL;
		rotor_acceleration = decl_speed;
	}

	/*
	if (time < 1) {
		rotor_acceleration = 0;
		accelerate = ACCL_KEEP;
	}
	else if (time < 17) {
		accelerate = ACCL_ACCL;
		rotor_acceleration = accl_speed;
	}
	else if (time < 19) {
		accelerate = ACCL_KEEP;
		rotor_acceleration = 0;
	}
	else {
		accelerate = ACCL_DECL;
		rotor_acceleration = decl_speed;
	}
	*/
}

void update_stats(double _dt)
{
	dt = _dt;
	update_rotor(dt);
	choose_modulation_mode();
	update_target_modulation(dt);
	update_accelerate(dt);
}


const float svpwm_voltage_angle_offset = 0;

void modulation_async_svpwm(double dt) {
	const static float d = 1.0/360.0;

	float voltage_angle = rotor_angle + svpwm_voltage_angle_offset; //150
	float carrier_angle = trig_wave_angle * d;
	int vector;
	float temp;
	int temp2;
	static float zones[7];
	static struct svpwm_time svpwm_times;

	// Update async pulse
	async_pulses = current_modulation_props->pulses_low;
	if (current_modulation_props->modulation_type == MODE_ASYNC_SVPWM) {
		if (current_modulation_props->pulses_high == 0) {
			async_pulses = current_modulation_props->pulses_low;
		} else {
			async_pulses = map(current_modulation_props->frequency_low_bound,
							   current_modulation_props->frequency_high_bound,
							   rotor_speed,
							   current_modulation_props->pulses_low,
							   current_modulation_props->pulses_high);
		}

		if (current_modulation_props->random > 0)
			async_pulses = async_pulses + rand(current_modulation_props->random);
	}
	trig_wave_angle += async_pulses * 360.0 * dt;
	if (trig_wave_angle >= 360) {
		trig_wave_angle -= 360;
	}

	get_svpwm_time(voltage_angle, modulation, &svpwm_times);

	if ((svpwm_times.vector1 & 1) == 0) {
		temp = svpwm_times.time1;
		svpwm_times.time1 = svpwm_times.time2;
		svpwm_times.time2 = temp;

		temp2 = svpwm_times.vector1;
		svpwm_times.vector1 = svpwm_times.vector2;
		svpwm_times.vector2 = temp2;
	}

	zones[0] = svpwm_times.off_time * 0.25;
	zones[1] = zones[0] + svpwm_times.time1 * 0.5;
	zones[2] = zones[1] + svpwm_times.time2 * 0.5;
	zones[3] = zones[2] + svpwm_times.off_time * 0.5;
	zones[4] = zones[3] + svpwm_times.time2 * 0.5;
	zones[5] = zones[4] + svpwm_times.time1 * 0.5;
	zones[6] = 1;


	if (carrier_angle < zones[0]) vector = 0;
	else if (carrier_angle < zones[1]) vector = svpwm_times.vector1;
	else if (carrier_angle < zones[2]) vector = svpwm_times.vector2;
	else if (carrier_angle < zones[3]) vector = 7;
	else if (carrier_angle < zones[4]) vector = svpwm_times.vector2;
	else if (carrier_angle < zones[5]) vector = svpwm_times.vector1;
	else vector = 0;

	apply_voltage_vector(vector);
}

void modulation_sync_svpwm(float _modulation, int pulses) {

	const static float d = 1.0/360.0;

	float voltage_angle = rotor_angle + svpwm_voltage_angle_offset;
	float carrier_angle = voltage_angle * pulses * d;
	int vector;
	float temp;
	int temp2;
	float zones[7];
	static struct svpwm_time svpwm_times;

	carrier_angle -= (int)carrier_angle;


	get_svpwm_time(voltage_angle, _modulation, &svpwm_times);

	if ((svpwm_times.vector1 & 1) == 0) {
		temp = svpwm_times.time1;
		svpwm_times.time1 = svpwm_times.time2;
		svpwm_times.time2 = temp;

		temp2 = svpwm_times.vector1;
		svpwm_times.vector1 = svpwm_times.vector2;
		svpwm_times.vector2 = temp2;
	}

	zones[0] = svpwm_times.off_time * 0.25;
	zones[1] = zones[0] + svpwm_times.time1 * 0.5;
	zones[2] = zones[1] + svpwm_times.time2 * 0.5;
	zones[3] = zones[2] + svpwm_times.off_time * 0.5;
	zones[4] = zones[3] + svpwm_times.time2 * 0.5;
	zones[5] = zones[4] + svpwm_times.time1 * 0.5;
	zones[6] = 1;


	if (carrier_angle < zones[0]) vector = 0;
	else if (carrier_angle < zones[1]) vector = svpwm_times.vector1;
	else if (carrier_angle < zones[2]) vector = svpwm_times.vector2;
	else if (carrier_angle < zones[3]) vector = 7;
	else if (carrier_angle < zones[4]) vector = svpwm_times.vector2;
	else if (carrier_angle < zones[5]) vector = svpwm_times.vector1;
	else vector = 0;
	apply_voltage_vector(vector);
}



void modulation_dtc(float _modulation, float pulses)
{
	int voltage_vector;
	voltage_vector = dtc(dt, rotor_angle, _modulation / rotor_speed * 100.0, current_modulation_props->pulses_low);
	apply_voltage_vector(voltage_vector);
}



void modulation_wide3(float _modulation)
{
	float voltage_angle = rotor_angle + 90;
	float sector_angle = voltage_angle;
	int sector;

	_modulation = 12;

	if (voltage_angle >= 360) voltage_angle -= 360;
	while(sector_angle >= 60)
	{
		sector_angle -= 60;
	}

	sector = voltage_angle / 60 + 1;
	if (sector_angle < _modulation) sector = sector - 1;
	if (sector_angle > 60 - _modulation) sector = sector + 1;

	if (sector == 0) sector = 6;
	if (sector == 7) sector = 1;

	apply_voltage_vector(sector);
}


void modulation_square()
{
	float voltage_angle = rotor_angle + 90;
	int sector;

	if (voltage_angle >= 360) voltage_angle -= 360;
	sector = voltage_angle / 60 + 1;

	apply_voltage_vector(sector);
}





///////////////////////////        Angle based

float angle_offset = 0;

void modulation_async_svpwm_thi() {
	float T, S1, S2, S3;

	async_pulses = current_modulation_props->pulses_low;
	if (current_modulation_props->pulses_high == 0) {
		async_pulses = current_modulation_props->pulses_low;
	} else {
		async_pulses = map(current_modulation_props->frequency_low_bound,
						   current_modulation_props->frequency_high_bound,
						   rotor_speed,
						   current_modulation_props->pulses_low,
						   current_modulation_props->pulses_high);
	}

	if (current_modulation_props->random > 0)
		async_pulses = async_pulses + rand(current_modulation_props->random);
	trig_wave_angle += async_pulses * 360.0 * dt;
	if (trig_wave_angle >= 360) {
		trig_wave_angle -= 360;
	}

	T = triangle_wave_2(trig_wave_angle, 360);

	S1 = sine_THI(rotor_angle / 180 * pi);
	S2 = sine_THI(rotor_angle / 180 * pi + 4.0/3 * pi);
	S3 = sine_THI(rotor_angle / 180 * pi + 2.0/3 * pi);

	S1 *= modulation;
	S2 *= modulation;
	S3 *= modulation;

	TIM1->CCR1 = T > S1 ? 0 : 2;
	TIM1->CCR2 = T > S2 ? 0 : 2;
	TIM1->CCR3 = T > S3 ? 0 : 2;
	return;
}

char get_polarity(float angle, const float *alphas, unsigned int len, char flip) {
	float sine_angle = 0;
	int8_t polarity = -1;

	while (angle >= 360) angle -= 360;
	if (angle < 0) angle += 360;

	sine_angle = angle;
	if (sine_angle >= 180) sine_angle -= 180;
	if (sine_angle >= 90) sine_angle = 180 - sine_angle;

	for (int i = 0; i < len - 1; i += 2) {
		if (alphas[i] <= sine_angle && sine_angle < alphas[i + 1]) {
			polarity = 1;
			break;
		}
	}

	if (len & 1 && alphas[len - 1] <= sine_angle) polarity = 1;


	if (angle >= 180) polarity = -polarity;
	if (flip) polarity = -polarity;

	return polarity + 1;
}

void modulation_shepwm(float modulation, int pulses){
	int modulation_index;
	float alphas[5];
	float voltage_angle = rotor_angle + angle_offset;


	if (modulation > 1.14) modulation = 1.14;
	modulation_index = (int)(modulation * 1000);

	switch(pulses)
	{
		case 11:
			alphas[0] = SHE_5_alpha_1[modulation_index];
			alphas[1] = SHE_5_alpha_2[modulation_index];
			alphas[2] = SHE_5_alpha_3[modulation_index];
			alphas[3] = SHE_5_alpha_4[modulation_index];
			alphas[4] = SHE_5_alpha_5[modulation_index];

			TIM1->CCR1 = get_polarity(voltage_angle,       alphas, 5, 0);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, alphas, 5, 0);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, alphas, 5, 0);
			return;

		case 7:
			alphas[0] = SHE_3_alpha_1[modulation_index];
			alphas[1] = SHE_3_alpha_2[modulation_index];
			alphas[2] = SHE_3_alpha_3[modulation_index];

			TIM1->CCR1 = get_polarity(voltage_angle,       alphas, 3, 0);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, alphas, 3, 0);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, alphas, 3, 0);
			return;

		case 3:
			alphas[0] = SHE_1_alpha_1[modulation_index];

			TIM1->CCR1 = get_polarity(voltage_angle,       alphas, 1, 0);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, alphas, 1, 0);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, alphas, 1, 0);
			return;


	}
}




char chmpwm_3p(float angle, float modulation) {
	const char shift = 0; // or true
	const int pulse_num = 3;

	float carrier_saw = -triangle_wave(pulse_num * angle, 360);
	float saw = -triangle_wave(angle, 360);
	float pwm;

	if (shift) saw = -saw;
	pwm = (saw > 0) ? modulation : -modulation;
	return pwm > carrier_saw ? 2 : 0;


}



char CHM_4Alpha_Polarity(int index) {
	if (index >= 1143) return 0;
	else if (index >= 1022) return 1;
	else if (index >= 675) return 0;
	else return 1;
}
char CHM_3Alpha_Polarity(int index) {
	if (index >= 1113) return 1;
	else if (index >= 840) return 0;
	else return 1;
}
char CHM_2Alpha_Polarity(int index) {
	if (index < 1023) return 1;
	else return 0;
}

void modulation_chmpwm(float modulation, int pulses) {
	int modulation_index;
	char flip;
	float voltage_angle = rotor_angle + angle_offset;

	if (modulation > 1.1) modulation = 1.1;
	modulation_index = (int)(modulation * 1000);

	switch(pulses) {
		case 3:
			TIM1->CCR1 = chmpwm_3p(voltage_angle      , modulation);
			TIM1->CCR2 = chmpwm_3p(voltage_angle + 240, modulation);
			TIM1->CCR3 = chmpwm_3p(voltage_angle + 120, modulation);
			return;
		case 5:
			flip = CHM_2Alpha_Polarity(modulation_index);

			TIM1->CCR1 = get_polarity(voltage_angle,       CHM_2Alpha[modulation_index], 2, flip);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, CHM_2Alpha[modulation_index], 2, flip);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, CHM_2Alpha[modulation_index], 2, flip);
			return;
		case 7:
			flip = CHM_3Alpha_Polarity(modulation_index);

			TIM1->CCR1 = get_polarity(voltage_angle,       CHM_3Alpha[modulation_index], 3, flip);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, CHM_3Alpha[modulation_index], 3, flip);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, CHM_3Alpha[modulation_index], 3, flip);
			return;
		case 9:
			flip = CHM_4Alpha_Polarity(modulation_index);

			TIM1->CCR1 = get_polarity(voltage_angle,       CHM_4Alpha[modulation_index], 4, flip);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, CHM_4Alpha[modulation_index], 4, flip);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, CHM_4Alpha[modulation_index], 4, flip);
			return;
	}
}
void modulation_chmpwm_1fold(float modulation, int pulses) {
	int modulation_index;
	float voltage_angle = rotor_angle + angle_offset;

	if (modulation > 1.1) modulation = 1.1;

	switch(pulses) {
		case 9:
			modulation_index = (int)(modulation * 1000) - 799;
			if (modulation_index < 0) modulation_index = 0;
			TIM1->CCR1 = get_polarity(voltage_angle,       CHM_4Wide_Alpha[modulation_index], 4, 1);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, CHM_4Wide_Alpha[modulation_index], 4, 1);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, CHM_4Wide_Alpha[modulation_index], 4, 1);
			return;
		case 7:
			modulation_index = (int)(modulation * 1000) - 799;
			if (modulation_index < 0) modulation_index = 0;
			TIM1->CCR1 = get_polarity(voltage_angle,       CHM_3Wide_Alpha[modulation_index], 3, 0);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, CHM_3Wide_Alpha[modulation_index], 3, 0);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, CHM_3Wide_Alpha[modulation_index], 3, 0);
			return;
		case 5:
			modulation_index = (int)(modulation * 1000);
			if (modulation_index < 0) modulation_index = 0;
			TIM1->CCR1 = get_polarity(voltage_angle,       CHM_2Wide_Alpha[modulation_index], 2, 1);
			TIM1->CCR2 = get_polarity(voltage_angle + 240, CHM_2Wide_Alpha[modulation_index], 2, 1);
			TIM1->CCR3 = get_polarity(voltage_angle + 120, CHM_2Wide_Alpha[modulation_index], 2, 1);
			return;
	}
}
