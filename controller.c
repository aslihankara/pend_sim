#include "controller.h"

int flop;

int get_action(float x, float x_dot, float theta, float theta_dot, float reinforcement)
{
	flop = !flop;
	
	return flop;
}

void reset_controller(void)
{
	flop = 0;
}
