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
#define GAMMA .001
#define ALPHA .4

#define GAMMAT GAMMA
#define GAMMATD GAMMA/150

#define NUM_T 10
#define NUM_X 1
#define NUM_TD 12
#define NUM_XD 10
#define NUM_ACTIONS 2

#define NUM_INPUTS 4

#define INIT_VALUE 0.0


typedef struct QVALS {
	int t, td, x, xd, a;
	float value;
}Qvalues;

typedef struct STATE {
	double t, td;
}State;

typedef struct TDATA {
	State current, next;
	int a;
}Tdata;


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
int init;
char learning = 1;


double tweights[NUM_INPUTS]; 
double tdweights[NUM_INPUTS];

Tdata training_data[200000];
int valid_trials = 0;

//forward declarations
int myrand(float prob);
void decay_epsilon(int init);
State calc_nstate(State cstate, int action);
double sigmoid(double x);
void update_weights(State prev, int action, State actual);
double get_qvalue(State cstate, int action);
void show_error(State prev, int action, State actual);



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
	int best_action;
	float current_value;
	static State prev_state;
	static int prev_action;
	State current_state;

	//convert radians to degrees
	theta = theta * 180.0/PI;
	theta_dot = theta_dot * 180.0/PI;

	current_state.t = theta;
	current_state.td = theta_dot;

	if (!myrand(mepsilon))
	{
		max_value = 50000.0; //large  number, pessimistic initialzation
		best_action = 0;
		for (i=0; i < num_actions; ++i)
		{
			current_value = get_qvalue(current_state, i);

			if (current_value < max_value)
			{
				max_value = current_value;
				best_action = i;
			}
		}
	}
	else
	{
		best_action = rand() % num_actions;
	}
	
	if(init)
	{
		init = 0;
		prev_state = current_state;
		prev_action = best_action; 
		return best_action; 
	}

	training_data[valid_trials].current = prev_state;
	training_data[valid_trials].next = current_state;
	training_data[valid_trials].a = prev_action;
	++valid_trials;
	
	//show_error( prev_state, prev_action, current_state);
	
		
	//update_weights(prev_state, prev_action, current_state);

	/*
	for(i=0; i < NUM_INPUTS; ++i)
	{
		printf("t[%d]: %lf ", i, tweights[i]);
	}
	putchar('\n');
	for(i=0; i < NUM_INPUTS; ++i)
	{
		printf("td[%d]: %lf ", i, tdweights[i]);
	}
	putchar('\n');
	putchar('\n');
	getchar();
	*/

	prev_state = current_state;
	prev_action = best_action; 

	return best_action;
}


double get_qvalue(State cstate, int action)
{
	State nstate;
	//double tmp;

	nstate = calc_nstate(cstate, action);

	nstate.t *= 1.0/MAX_T;
	//nstate.t *= nstate.t;
	nstate.td *= 1.0/MAX_TD;

	//tmp = nstate.td * fabs(nstate.td)/ nstate.t;

	return sqrt(nstate.t*nstate.t + nstate.td*nstate.td*0.01);	
}

State calc_nstate(State cstate, int action)
{
	State next_state;
	double x;
	int i;
	double input[NUM_INPUTS];

	if(!action)
	{
		action = -1;
	}

	input[0] = 1; //bias
	input[1] = cstate.t;
	input[2] = cstate.td;
	input[3] = action;
	
	for (i=0, x=0; i < NUM_INPUTS; ++i)
	{
		x += tweights[i]*input[i];
	}
	next_state.t = 2*MAX_T*sigmoid(x) - MAX_T;


	for (i=0, x=0; i < NUM_INPUTS; ++i)
	{
		x += tdweights[i]*input[i];
	}
	next_state.td = 2*MAX_TD*sigmoid(x) - MAX_TD;


	return next_state;
}

void myweightupdate(double error, double weights[NUM_INPUTS], double inputs[NUM_INPUTS], double mygamma)
{
	int i;

	for (i=0; i < NUM_INPUTS; ++i)
	{
		weights[i] = weights[i] + mygamma*error*inputs[i];
	}
}

void show_error(State prev, int action, State actual)
{
	double error;
	State expected;
	double inputs[NUM_INPUTS];

	if(!action)
	{
		action = -1;
	}

	inputs[0] = 1;
	inputs[1] = prev.t;
	inputs[2] = prev.td;
	inputs[3] = action;

	expected = calc_nstate(prev, action);

	error = actual.t - expected.t;
	printf("t error: %lf ", error);

	error = actual.td - expected.td;
	printf("td error: %lf\n", error);
}

void update_weights(State prev, int action, State actual)
{
	double error;
	State expected;
	double inputs[NUM_INPUTS];

	if(!action)
	{
		action = -1;
	}

	inputs[0] = 1;
	inputs[1] = prev.t;
	inputs[2] = prev.td;
	inputs[3] = action;

	expected = calc_nstate(prev, action);

	error = actual.t - expected.t;
	myweightupdate(error, tweights, inputs, GAMMAT);

	error = actual.td - expected.td;
	myweightupdate(error, tdweights, inputs, GAMMATD);
}


double sigmoid(double x)
{
	return 1.0/(1+exp(-x));
}


void init_controller(void)
{
	int i;

	myrandmax = (double)RAND_MAX;
	read_states(0);
	decay_epsilon(1);

	for (i=0; i < NUM_INPUTS; ++i)
	{
		tweights[i] = -0.1;
		tdweights[i] = -0.1;
	}

}

void disable_learning()
{
	learning = 0;
}

void save_run(void)
{
	FILE* ft = fopen("log/a.t", "w");
	FILE* ftd = fopen("log/a.td", "w");
	FILE* fa = fopen("log/a.a", "w");
	int i;

	printf("saving run info...");
	fflush(stdout);
	for (i=0; i < valid_trials; ++i)
	{
		fprintf(ft, "%d %lf\n", i, training_data[i].current.t );
		fprintf(ftd, "%d %lf\n", i, training_data[i].current.td);
		fprintf(ftd, "%d %d\n", i, training_data[i].a*2 - 1);
	}
	fclose(ft);
	fclose(ftd);
	fclose(fa);
	printf("done\n");
}

void reset_controller(void)
{
	int i,j;
	prev_state_value = 0;
	init = 1;
	decay_epsilon(0);

if(valid_trials && learning)
{
	for(j=0; j < 100; ++j)
	for(i=0; i < valid_trials; ++i)
	{
		update_weights(training_data[i].current, training_data[i].a, training_data[i].next);
	}

	save_run();
}
valid_trials = 0;

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

	for(i=0; i < NUM_INPUTS; ++i)
	{
		printf("t[%d]: %lf ", i, tweights[i]);
	}
	putchar('\n');
	for(i=0; i < NUM_INPUTS; ++i)
	{
		printf("td[%d]: %lf ", i, tdweights[i]);
	}
	putchar('\n');
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

