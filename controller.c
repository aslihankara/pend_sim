#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "controller.h"


#define MAX_T 12.0
#define MAX_X 2.4
#define MAX_TD 180
#define MAX_XD 2

#define EPSILON_INIT .5
#define LAMBDA .01
#define GAMMA .4
#define ALPHA .03
#define ETA ALPHA

#define NUM_T 10
#define NUM_X 1
#define NUM_TD 10
#define NUM_XD 1
#define NUM_ACTIONS 2

#define INIT_VALUE 0.0


typedef struct QVALS {
	int t, td, x, xd, a;
	float value;
}Qvalues;

typedef struct STATE {
	float t, td, x, xd;
	int a;
}State;

typedef struct FEATURES{
	char rr, rl, hw, fl, fr, c;
}Features;

float q_values[NUM_T][NUM_X][NUM_TD][NUM_XD][NUM_ACTIONS];

const int num_t = NUM_T,
	   	num_x = NUM_X,
		num_td = NUM_TD,
		num_xd = NUM_XD,
		num_actions = NUM_ACTIONS;


float mepsilon;
const float mgamma = GAMMA;
const float malpha = ALPHA;
const float eta = ETA;
State current_state;
State prev_state;
static double myrandmax;
char init;

double weights[8];

//forward declarations
int myrand(float prob);
void decay_epsilon(int init);
double calc_q(State s);
void update_q(float reward);
Features get_features(State s);


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
	State s;

	//convert radians to degrees
	theta = theta * 180.0/PI;
	theta_dot = theta_dot * 180.0/PI;

	ix = get_index(x, MAX_X, num_x);
	ixd = get_index(x_dot, MAX_XD, num_xd);
	it = get_index(theta, MAX_T, num_t);
	itd = get_index(theta_dot, MAX_TD, num_td);

	
	s.x = ix;
	s.t = it;
	s.xd = ixd;
	s.td = itd;
	if (!myrand(mepsilon))
	{
		max_value = -50.0; //large negative number, pessimistic initialzation
		max_action = 0;
		

		for (i=0; i < num_actions; ++i)
		{
			s.a = i;
			current_value = calc_q(s);

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
	s.a = max_action;
		
	current_state = s;
	
	if(init)
	{
		init = 0;
		prev_state= current_state;
		return max_action;
	}
		
	//check if terminating state
//	if (reinforcement  <  -0.1)
//	{
//		for (i=0; i < num_actions; ++i)
//		{
//			q_values[it][ix][itd][ixd][i] = -1;
//		}
//	}

	update_q(reinforcement);

	prev_state = current_state;

	return max_action;
}

Features get_features(State s)
{
	Features f = {0};


	//check for high w
	if (fabs(s.td) > 5)
	{
		f.hw = 1;
	}

	//check if pole is close to center
	if( fabs(s.t) < 1)
	{
		f.c = 1;
	}

	if (s.t > 0 ) //if angle is to the right
	{
		//check if restoring or falling
		if (s.td > 0)
		{
			f.fr = 1;
		}
		else
		{
			f.rr = 1;
		}
		
	}
	else //angle is to the left
	{
		//check if restoring or falling
		if (s.td < 0)
		{
			f.fl = 1;
		}
		else
		{
			f.rl = 1;
		}
	}

	return f;
}
double calc_q(Features f)
{
	double value;

	value = weights[0] + 
			s.rr*weights[1] +
			s.rl*weights[2] +
			s.hw*weights[3] +
			s.fl*weights[4] +
			s.fr*weights[5]
			f.c*weights[6] + 
			;

	return value;
}

void update_q(float reward)
{
	double delta;
	double current_value, prev_value;
	int i;

	current_value = calc_q(current_state);
	prev_value = calc_q(prev_state);	

	delta = reward + mgamma*current_value - prev_value;

	weights[0] = weights[0] + eta*delta * 1;
	weights[1] = weights[1] + eta*delta * prev_state.t;
	weights[2] = weights[2] + eta*delta * prev_state.x;
	weights[3] = weights[3] + eta*delta * prev_state.td;
	weights[4] = weights[4] + eta*delta * prev_state.xd;
	weights[5] = weights[5] + eta*delta * prev_state.a;

//	printf("weights:\n");
//	for (i=0; i < 6; ++i)
//	{
//		printf("w[%d]: %lf\n", i, weights[i]);
//	}
//	putchar('\n');
//	getchar();
}


void init_controller(void)
{
	int i;
	myrandmax = (double)RAND_MAX;
	read_states(0);
	decay_epsilon(1);

	for (i=0; i < 6; ++i)
		weights[i] = 0.0;
}

void reset_controller(void)
{
	init = 1;

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
//	char *default_filename = "log/a.values";
//	FILE *fh;
//	Qvalues cq;
//	long int i;

	return 0;

//	if(filename == 0)
//	{
//		filename = default_filename;
//	}
//
//	fh = fopen(filename, "r");
//	if(fh == 0)
//	{
//		printf("could not open\'%s\' for reading!\n", filename);
//		printf("initalizing state values...");
//		fflush(stdout);
//		init_state_values();
//		printf("done!\n");
//		return 1;
//	}
//
//	printf("reading states from \'%s\'...", filename);
//	fflush(stdout);
//
//	i = 0;	
//	while(1)
//	{
//		if(!fread(&cq, sizeof(cq), 1, fh))
//			break;
//		++i;
//
//		q_values[cq.t][cq.x][cq.td][cq.xd][cq.a] =	cq.value;
//	}
//
//
//	printf("done!\n%ld states read\n", i);
//
//	fclose(fh);
//
//	return 0;
}

int write_states(char *filename) 
{
	char *default_filename = "log/a.values";
	FILE *fh;
	int i;

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

	fwrite(weights, sizeof(double), 6, fh);


	printf("done!\n");

	printf("weights:\n");
	for (i=0; i < 6; ++i)
	{
		printf("w[%d]: %lf\n", i, weights[i]);
	}

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

