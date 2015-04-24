#include <stdlib.h>
#include <stdio.h>
#include "controller.h"


#define MAX_ANGLE 12.0
#define MAX_POSITION 2.4

#define NUM_ANGLES 100
#define NUM_POSITIONS 100
#define NUM_ACTIONS 2


#define EPSILON .01
#define GAMMA .02
#define ALPHA .4

#define NUM_STATES (NUM_ANGLES * NUM_POSITIONS * NUM_ACTIONS)

#define PI 3.14159265359


float q_values[(int)(NUM_ANGLES*MAX_ANGLE*2)][(int)(NUM_POSITIONS*MAX_POSITION*2)][NUM_ACTIONS] = {{{0.0}}};

float angle_div = 1.0/NUM_ANGLES;
float pos_div = 1.0/NUM_POSITIONS;

int epsilon = EPSILON * 100;
float gamma = GAMMA;
float alpha = ALPHA;

int prev_action, prev_x, prev_theta;

int init = 1;

int get_action(float x, float x_dot, float theta, float theta_dot, float reinforcement)
{
	int i;
	float max_value;
	int max_action;
	int x_index, theta_index;
	//int x_dot_index, theta_dot_index;
	int current_value;

	theta = theta * 180.0/PI;

	x_index = (int) (((x+MAX_POSITION/2)/pos_div)+0.5);
	theta_index = (int) (((theta+MAX_ANGLE/2)/angle_div)+0.5);


	if(reinforcement != 0)
	{
	//	fprintf(stderr, "state, x: %lf theta %lf\n", x, theta);
	//	fprintf(stderr, "indexes, x: %d theta %d\n\n", x_index, theta_index);
	//	getchar();
	}

	
	if(x_index < 0 || theta_index < 0)
	{
		fprintf(stderr, "invalid index, x: %d theta %d\n", x_index, theta_index);
		exit(1);
	}

	i = rand() % 100;

	if (i >= epsilon)
	{
		for (i=0, max_value=0.0, max_action=0; i < NUM_ACTIONS; ++i)
		{
			current_value = q_values[theta_index][x_index][i];

			if (current_value > max_value)
			{
				max_value = current_value;
				max_action = i;
			}
		}
	}
	else
	{
		max_action = rand() % NUM_ACTIONS;
	}

	if(init)
	{
		init = 0;
		prev_action = max_action;
		prev_x = x_index;
		prev_theta = theta_index;
		return max_action;
	}


	q_values[prev_theta][prev_x][prev_action] = q_values[prev_theta][prev_x][prev_action] + 
												alpha*(reinforcement + 
													   gamma*q_values[theta_index][x_index][max_action] - 
													   q_values[prev_theta][prev_x][prev_action]);

	prev_theta = theta_index;
	prev_x = x_index;
	prev_action = max_action;



	return max_action;
}

void init_controller(void)
{
	
}

void reset_controller(void)
{
	init = 1;
}


int read_states(char *filename) 
{



	return 0;
}

int write_states(char *filename) 
{
	int theta, x, action;
	char *default_filename = "log";
	FILE *fh;

	if(filename == 0)
	{
		filename = default_filename;
	}

	fh = fopen(filename, "w");
	
	printf("writing states to \'%s\'...", filename);
	fflush(stdout);

	for (theta = 0; theta < (int)(NUM_ANGLES*MAX_ANGLE*2); ++theta)
	{
		for(x = 0; x <  (int)(NUM_POSITIONS*MAX_POSITION*2); ++x)
		{
			for(action=0; action < NUM_ACTIONS; ++action)
			{
				fprintf(fh, "theta: %d\tpos: %d\taction: %d\tvalue: %lf\n",
						theta, x, action, q_values[theta][x][action]);

			}
		}
	}

	printf("done!\n");

	fclose(fh);

	return 0;
}
