#ifndef __COOLING_H
#define __COOLING_H

void cooling_initialize();
void cooling_tick(float dt);
void turn_on_fan();
void turn_off_fan();

#endif
