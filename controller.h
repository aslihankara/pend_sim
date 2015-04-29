#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

int get_action(float x, float x_dot, 
               float theta, float theta_dot, float reinforcement);  

void reset_controller(void);
void init_controller(void);
int read_states(char* filename);
int write_states(char *filename);

#endif
