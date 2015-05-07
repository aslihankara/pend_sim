#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "controller.h"


#define MAX_T 12.0
#define MAX_X 2.4
#define MAX_TD 180
#define MAX_XD 2

#define EPSILON_INIT .3
#define LAMBDA .1
#define GAMMA .2
#define ALPHA .2

#define NUM_T 10
#define NUM_X 1
#define NUM_TD 12
#define NUM_XD 10
#define NUM_ACTIONS 2

#define INIT_VALUE 0.0

#define PI 3.14159265359

typedef struct QVALS {
	int t, td, x, xd, a;
	float value;
}Qvalues;

float q_values[NUM_T][NUM_X][NUM_TD][NUM_XD][NUM_ACTIONS];

const int num_t = NUM_T,
	   	num_x = NUM_X,
		num_td = NUM_TD,
		num_xd = NUM_XD,
		num_actions = NUM_ACTIONS;


float mepsilon;
const float mgamma = GAMMA;
const float malpha = ALPHA;
float *current_state_value;
float *prev_state_value;
static double myrandmax;

//forward declarations
int myrand(float prob);
void decay_epsilon(int init);


int get_index(float x, float max, int num)
{
	int index;
	x = x *(num/2)/max;

	index = (int)x + (num/2);

	
	if (index >= num)
	{
	//	printf("index %d is out of range, truncating to %d\n", index, num-1);
		index = num-1;
	}
	if(index < 0)
	{
	//	printf("index %d is out of range, truncating to %d\n", index, 0);
		index = 0;
	}

	return index; 
}

int get_action(float x, float x_dot, float theta, float theta_dot, float reinforcement)
{
	int i;
	float max_value;
	int max_action;
	float current_value;
	int ix, ixd, it, itd;

	//convert radians to degrees
	theta = theta * 180.0/PI;
	theta_dot = theta_dot * 180.0/PI;

	ix = get_index(x, MAX_X, num_x);
	ixd = get_index(x_dot, MAX_XD, num_xd);
	it = get_index(theta, MAX_T, num_t);
	itd = get_index(theta_dot, MAX_TD, num_td);

	if (!myrand(mepsilon))
	{
		max_value = -50.0; //large negative number, pessimistic initialzation
		max_action = 0;
		for (i=0; i < num_actions; ++i)
		{
			current_value = q_values[it][ix][itd][ixd][i];

			if (current_value > max_value)
			{
				max_value = current_value;
				max_action = i;
			}
		}
	}
	else
	{
		max_action = rand() % num_actions;
	}
	
	current_state_value = &q_values[it][ix][itd][ixd][max_action];
	
	if(!prev_state_value)
	{
		prev_state_value = current_state_value;
		return max_action;
	}
	
	
	//check if terminating state
	if (reinforcement  <  -0.1)
	{
		for (i=0; i < num_actions; ++i)
		{
			q_values[it][ix][itd][ixd][i] = -1;
		}
	}

	*prev_state_value = *prev_state_value + malpha*(reinforcement + 
													mgamma * (*current_state_value) - 
													*prev_state_value);

	prev_state_value = current_state_value;

	return max_action;
}

void init_controller(void)
{
	myrandmax = (double)RAND_MAX;
	read_states(0);
	decay_epsilon(1);
}

void reset_controller(void)
{
	prev_state_value = 0;
	decay_epsilon(0);
}

void decay_epsilon(int init)
{
	static int t;
	
	if (init)
	{
		t = 0;
	}

	mepsilon = EPSILON_INIT * exp(-1*LAMBDA*t);
	
	++t;
}

void init_state_values(void)
{
	int t, x, td, xd, action;

	for (t = 0; t < num_t; ++t)
		for (x = 0; x < num_x; ++x)
			for (td = 0; td < num_td; ++td)
				for (xd = 0; xd < num_xd; ++xd)
					for (action = 0; action < num_actions; ++action)
					{
						q_values[t][x][td][xd][action] = INIT_VALUE;
					}
}


int read_states(char *filename) 
{
	char *default_filename = "log/a.values";
	FILE *fh;
	Qvalues cq;
	long int i;

	if(filename == 0)
	{
		filename = default_filename;
	}

	fh = fopen(filename, "r");
	if(fh == 0)
	{
		printf("could not open\'%s\' for reading!\n", filename);
		printf("initalizing state values...");
		fflush(stdout);
		init_state_values();
		printf("done!\n");
		return 1;
	}

	printf("reading states from \'%s\'...", filename);
	fflush(stdout);

	i = 0;	
	while(1)
	{
		if(!fread(&cq, sizeof(cq), 1, fh))
			break;
		++i;

		q_values[cq.t][cq.x][cq.td][cq.xd][cq.a] =	cq.value;
	}


	printf("done!\n%ld states read\n", i);

	fclose(fh);

	return 0;
}

int write_states(char *filename) 
{
	int t, x, td, xd, action;
	char *default_filename = "log/a.values";
	FILE *fh;
	Qvalues cq;
	long int i;

	if(filename == 0)
	{
		filename = default_filename;
	}

	fh = fopen(filename, "w");
	if(fh == 0)
	{
		printf("could not open\'%s\' for writing!\n", filename);
		return 1;
	}

	printf("writing states to \'%s\'...", filename);
	fflush(stdout);

	i = 0;	
	for (t = 0; t < num_t; ++t)
		for (x = 0; x < num_x; ++x)
			for (td = 0; td < num_td; ++td)
				for (xd = 0; xd < num_xd; ++xd)
					for (action = 0; action < num_actions; ++action)
					{
						cq.t = t;
						cq.x = x;
						cq.td = td;
						cq.xd = xd;
						cq.a = action;
						cq.value = q_values[t][x][td][xd][action];

						fwrite(&cq, sizeof(cq), 1, fh);
						++i;
					}


	printf("done!\n%ld states written\n", i);

	fclose(fh);

	return 0;
}

int myrand(float prob)
{
	double value;
	
	value = (double) rand() / myrandmax;

	if(value > prob)
		return 0;

	return 1;
}

