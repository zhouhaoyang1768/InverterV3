#include "rotor_position_sensing.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"

float recorded_acceleration;
float recorded_speed;
float recorded_position;
int recorded_sector;

float previous_acceleration;
float previous_speed;
float previous_position;
int previous_sector;

int get_rotor_sector()
{
	char U, V, W;
	const static char sector[] = {		// W V U
									0,
									5,	// 0 0 1
									1,	// 0 1 0
									6,	// 0 1 1
									3,	// 1 0 0
									4,	// 1 0 1
									2,	// 1 1 0
	};

	U = (GPIOA->IDR & GPIO_PIN_1) != 0;
	V = (GPIOA->IDR & GPIO_PIN_2) != 0;
	W = (GPIOA->IDR & GPIO_PIN_3) != 0;

	int sum = U + (V << 1) + (W << 2);
	return sector[sum];
}

float get_rotor_angle(char sector1, char sector2)
{
	const static float angles[7][7] = {
			// To       1      2      3      4      5      6
			{  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, // From
			{  0.0f,  0.0f, 30.0f,  0.0f,  0.0f,  0.0f,330.0f}, // 1
			{  0.0f, 30.0f,  0.0f, 90.0f,  0.0f,  0.0f,  0.0f}, // 2
			{  0.0f,  0.0f, 90.0f,  0.0f,150.0f,  0.0f,  0.0f}, // 3
			{  0.0f,  0.0f,  0.0f,150.0f,  0.0f,210.0f,  0.0f}, // 4
			{  0.0f,  0.0f,  0.0f,  0.0f,210.0f,  0.0f,270.0f}, // 5
			{  0.0f,330.0f,  0.0f,  0.0f,  0.0f,270.0f,  0.0f}, // 6

	};
	return angles[(unsigned int)sector1][(unsigned int)sector2];
}
// 1 for counter-clockwise, -1 for clockwise
int get_rotor_direction(char sector1, char sector2)
{
	const static int directions[7][7] = {
		// To     1  2  3  4  5  6
			{  0, 0, 0, 0, 0, 0, 0}, // From
			{  0, 0, 1, 0, 0, 0,-1}, // 1
			{  0,-1, 0, 1, 0, 0, 0}, // 2
			{  0, 0,-1, 0, 1, 0, 0}, // 3
			{  0, 0, 0,-1, 0, 1, 0}, // 4
			{  0, 0, 0, 0,-1, 0, 1}, // 5
			{  0, 1, 0, 0, 0,-1, 0}, // 6

	};
	return directions[(unsigned int)sector1][(unsigned int)sector2];
}

void get_sector_boundry_angles(int sector, float* lower, float* upper)
{
	//			                     Sector    1       2       3       4       5       6
	const static float lowers[7] = { 0.0f, -30.0f,  30.0f,  90.0f, 150.0f, 210.0f, 270.0f};
	const static float uppers[7] = { 0.0f,  30.0f,  90.0f, 150.0f, 210.0f, 270.0f, 330.0f};

	*lower = lowers[sector];
	*upper = uppers[sector];
}

float get_rotor_displacement(int direction, float current_angle, float previous_angle)
{
	if (direction == 1 && current_angle < previous_angle)
	{
		current_angle += 360;
	}
	if (direction == -1 && current_angle > previous_angle)
	{
		previous_angle += 360;
	}
	return current_angle - previous_angle;
}

char update_rotor_position(float dt, float output[3])
{
	const static float sector_length = 1/6.0f;
	float displacement;
	float inv_dt = 1.0 / dt;

	previous_acceleration = recorded_acceleration;
	previous_speed = recorded_speed;
	previous_position = recorded_position;
	previous_sector = recorded_sector;

	recorded_sector = get_rotor_sector();

	// if the rotor is still in the same sector, do not update
	if (recorded_sector == previous_sector)
	{
		return 0;
	}

	// When the rotor switches sector, we can calculate its angle
	recorded_position = get_rotor_angle(recorded_sector, previous_sector);

	// Calculate speed
	displacement = recorded_sector - previous_sector;
	if (displacement == -5) displacement = 1;
	else if (displacement == 5) displacement = -1;

	recorded_speed = (displacement * sector_length) * inv_dt;

	// Calculate acceleration
	recorded_acceleration = (recorded_speed - previous_speed) * inv_dt;

	output[0] = recorded_position;
	output[1] = recorded_speed;
	output[2] = recorded_acceleration;
	return 1;
;}
