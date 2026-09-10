#ifndef INC_ROTOR_POSITION_SENSING_H_
#define INC_ROTOR_POSITION_SENSING_H_

char update_rotor_position(float dt, float output[3]);
int get_rotor_sector();
// 1 for counter-clockwise, -1 for clockwise
int get_rotor_direction();
float get_rotor_angle(char sector1, char sector2);
float get_rotor_displacement(int direction, float current_angle, float previous_angle);
void get_sector_boundry_angles(int sector, float* lower, float* upper);

#endif /* INC_ROTOR_POSITION_SENSING_H_ */
