#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

#include "controller.h"

#define JUPITER_GRAV 0             /* If set, use bigger gravity const */
#define TILTED 1                   /* If set, pole is given an initial tilt */
#define MAX_TRIALS    500000000       /* Termination criterion */
#define MAX_STEPS       100000     /* about 33 minutes of balancing */
#define MAX_SUCCESS     500  //number of successful balances before termination



//function declarations
void write_scores(char *filename);
void reset_state(float *x, float *x_dot, float *theta, float *theta_dot);
void cart_pole(int action, float *x, float *x_dot, float *theta, float *theta_dot);
int fail(float x, float x_dot, float theta, float theta_dot);
extern int get_action(float x, float x_dot, float theta, float theta_dot, float reinforcement);
extern void reset_controller(void);
void sig_handler(int signum);
/* extern void print_controller_info(); */




int scores[MAX_TRIALS]={0};
int ECHO_STATE = 0;                /* save state parameters to a file */
FILE *echo_file = NULL; 
int RND_SEED = 0;

int main(int argc, char *argv[])
{
   float x,                         /* cart position, meters */
         x_dot,                     /* cart velocity */
         theta,                     /* pole angle, radians */
         theta_dot;                 /* pole angular velocity */
   int action;                      /* 0 for push-left, 1 for push-right */
   int steps = 0;                   /* duration of trial, in 0.02 sec steps */
   //int failures = 0;                /* number of failed trials */
   int best_steps = 0;              /* number of steps in best trial */
   int best_trial = 0;              /* trial number of best trial */
   int num_success = 0;
   int num_trials = 0;


   signal(SIGINT, sig_handler);

   if (TILTED)
      printf("Pole will have tilted reset,");
   else
      printf("Pole has normal reset,");
   if (JUPITER_GRAV)
      printf(" and \"Jupiter\" gravity.\n");
   else
      printf(" and normal gravity.\n");

   if (ECHO_STATE)
   {
      echo_file = fopen("log/poledata", "w");
      if (echo_file == NULL)
         printf("ERROR: Cannot open \"poledata\" for output.\n");
   }

   if (argc > 1)
      RND_SEED = atoi(argv[1]);
   else
      RND_SEED = 0;

   init_controller();
   reset_controller();
   reset_state(&x, &x_dot, &theta, &theta_dot);

   /*--- Iterate through the action-learn loop. ---*/
   while (num_success < MAX_SUCCESS && num_trials < MAX_TRIALS)
   {
	  ++steps;
      action = get_action(x, x_dot, theta, theta_dot, 0.0);  
	  //usleep(10000);
      
      /*--- Apply action to the simulated cart-pole ---*/
      cart_pole(action, &x, &x_dot, &theta, &theta_dot);

      if (fail(x, x_dot, theta, theta_dot))
      {
		scores[num_trials] = steps;
	  	num_trials++;
		if(num_trials % 100000 == 0)
		{	
			printf("Trial %d was %d steps.\n", num_trials, steps);
		}
        if (steps > best_steps)
        {
			printf("new best: Trial %d was %d steps.\n", num_trials, steps);
			best_steps = steps;
            best_trial = num_trials;
        }

        /* Call controller with negative feedback for learning */
        get_action(x, x_dot, theta, theta_dot, -1.0);

        reset_controller();
        reset_state(&x, &x_dot, &theta, &theta_dot);
	    steps = 0;
      }
	  if(steps > MAX_STEPS)
	  {
		disable_learning();
		scores[num_trials] = steps;
	  	num_trials++;
		num_success++;
        
		printf("*******pole successfuly balanced %d times\n", num_success);	

        /* Call controller with negative feedback for learning */
        get_action(x, x_dot, theta, theta_dot, 0);

        reset_controller();
        reset_state(&x, &x_dot, &theta, &theta_dot);
	    steps = 0;
		best_steps = 0;
	  } 
   }
   scores[num_trials] = steps;

   /* Diagnose result */
   if (num_trials == MAX_TRIALS) 
   {
      printf("Pole not balanced. Stopping after %d trials.\n",num_trials);
      printf("High water mark: %d steps in trial %d.\n\n", 
             best_steps, best_trial);
   }
   printf("Pole balanced successfully for %d trials.\n\n", num_success);

   write_states(0);
   write_scores(0);

/* print_controller_info();*/
   if (echo_file != NULL)
      fclose(echo_file);
   return 0;
}


/*--------------------------------------------------------------------*/
typedef struct SCORE_STRUCT{
	int score;
}Score_struct;

void write_scores(char *filename)
{
	FILE *f;
	int i;
	char *default_filename = "log/a.score";
	long int n;

	if(filename == 0)
	{
		filename = default_filename;
	}

	f = fopen(filename, "a");


	printf("writing scores to \'%s\'...", filename);
	fflush(stdout);
	n = 0;
	for (i=0; i < MAX_TRIALS && scores[i] != 0; ++i)
	{
		fprintf(f, "%d\n", scores[i]);
		++n;
	}
	printf("done\n%ld scores written\n", n);
	fclose(f);
	return;
}

double rand_range(int min, int max)
{
	float value;

	value = (double) rand() / (double) RAND_MAX;

	return min + (value * (max-min));

}
void reset_state(float *x, float *x_dot, float *theta, float *theta_dot)
{
#define SIX_DEGREES     0.1047198
#define SEVEN_DEGREES   0.1221730
#define TEN_DEGREES     0.1745329

   float angle_degrees;

   *x = 0.0;
   *x_dot = 0.0;
   *theta_dot = 0.0;
   if (TILTED)
   {
	 angle_degrees = rand_range(-6, 6);
     *theta = angle_degrees * PI/180; 
   }
   else
      *theta = 0.0;

}

 
/*----------------------------------------------------------------------
   cart_pole:  Takes an action (0 or 1) and the current values of the
 four state variables and updates their values by estimating the state
 TAU seconds later.
----------------------------------------------------------------------*/

/*** Parameters for simulation ***/
#if JUPITER_GRAV
#define GRAVITY 30.0
#else
#define GRAVITY 9.8
#endif
#define MASSCART 1.0
#define MASSPOLE 0.1
#define TOTAL_MASS (MASSPOLE + MASSCART)
#define LENGTH 0.5		  /* actually half the pole's length */
#define POLEMASS_LENGTH (MASSPOLE * LENGTH)
#define FORCE_MAG 10.0
#define TAU 0.02		  /* seconds between state updates */
#define FOURTHIRDS 1.3333333333333


void cart_pole(int action, float *x, float *x_dot, 
               float *theta, float *theta_dot)
{
    float xacc,thetaacc,force,costheta,sintheta,temp;
    int fail(float x, float x_dot, float theta, float theta_dot);

    force = (action>0)? FORCE_MAG : -FORCE_MAG;  
    costheta = cos(*theta);
    sintheta = sin(*theta);

    temp = (force + POLEMASS_LENGTH * *theta_dot * *theta_dot * sintheta)
		         / TOTAL_MASS;

    thetaacc = (GRAVITY * sintheta - costheta* temp)
	       / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta
                                              / TOTAL_MASS));

    xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;

/*** Update the four state variables, using Euler's method. ***/

    *x  += TAU * *x_dot;
    *x_dot += TAU * xacc;
    *theta += TAU * *theta_dot;
    *theta_dot += TAU * thetaacc;
}


/*----------------------------------------------------------------------
       fail:    returns 1 if the pole falls more than 12 degrees
                from vertical, or if the cart moves beyond the
                limits of its track, 2.4 meters in either direction
                from its starting position
----------------------------------------------------------------------*/

int fail(float x, float x_dot, float theta, float theta_dot)
{
   float twelve_degrees = 0.2094384;

   if ( theta < -twelve_degrees || theta > twelve_degrees)
   {	
		return 1;
   }
   if ( x < -2.4 || x > 2.4 )
	{
	//	return 1;  /* fail == true */
	}
   
   return 0;
}

void sig_handler(int signum)
{
	printf("caught signal %d\n", signum);

	write_states(0);
	write_scores(0);

	exit(0);
}
