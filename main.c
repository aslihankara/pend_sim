#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "controller.h"

#define JUPITER_GRAV 0             /* If set, use bigger gravity const */
#define TILTED 0                   /* If set, pole is given an initial tilt */
#define MAX_FAILURES    3000000       /* Termination criterion */
#define MAX_STEPS       100000     /* about 33 minutes of balancing */



//function declarations
void reset_state(float *x, float *x_dot, float *theta, float *theta_dot);
void cart_pole(int action, float *x, float *x_dot, float *theta, float *theta_dot);
int fail(float x, float x_dot, float theta, float theta_dot);
extern int get_action(float x, float x_dot, float theta, float theta_dot, float reinforcement);
extern void reset_controller(void);
/* extern void print_controller_info(); */



int ECHO_STATE = 0;                /* save state parameters to a file */
FILE *echo_file = NULL; 
int RND_SEED = 0;

static char rcs_driver_id[] = "$Id: driver.c,v 2.0 1994/11/17 19:45:38 finton Exp $";

int main(int argc, char *argv[])
{
   float x,                         /* cart position, meters */
         x_dot,                     /* cart velocity */
         theta,                     /* pole angle, radians */
         theta_dot;                 /* pole angular velocity */
   int action;                      /* 0 for push-left, 1 for push-right */
   int steps = 0;                   /* duration of trial, in 0.02 sec steps */
   int failures = 0;                /* number of failed trials */
   int best_steps = 0;              /* number of steps in best trial */
   int best_trial = 0;              /* trial number of best trial */


   printf("Driver: %s\n", rcs_driver_id);
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
   reset_state(&x, &x_dot, &theta, &theta_dot);

   /*--- Iterate through the action-learn loop. ---*/
   while (steps++ < MAX_STEPS && failures < MAX_FAILURES)
   {
      action = get_action(x, x_dot, theta, theta_dot, 0.0);  
	  //usleep(10000);
      
      /*--- Apply action to the simulated cart-pole ---*/
      cart_pole(action, &x, &x_dot, &theta, &theta_dot);

      if (fail(x, x_dot, theta, theta_dot))
      {
	  	failures++;
		if(failures % 100000 == 0)
		{	
			printf("Trial %d was %d steps.\n", failures, steps);
		}
        if (steps > best_steps)
        {
			best_steps = steps;
            best_trial = failures;
        }

        /* Call controller with negative feedback for learning */
        get_action(x, x_dot, theta, theta_dot, -1.0);

        reset_controller();
        reset_state(&x, &x_dot, &theta, &theta_dot);
	    steps = 0;
      }
   }

   /* Diagnose result */
   if (failures == MAX_FAILURES) 
   {
      printf("Pole not balanced. Stopping after %d failures.\n",failures);
      printf("High water mark: %d steps in trial %d.\n\n", 
             best_steps, best_trial);
   }
   else
    printf("Pole balanced successfully for at least %d steps in trial %d.\n\n",
            steps - 1, failures + 1);

   write_states(0);

/* print_controller_info();*/
   if (echo_file != NULL)
      fclose(echo_file);
   return 0;
}


/*--------------------------------------------------------------------*/

void reset_state(float *x, float *x_dot, float *theta, float *theta_dot)
{
#define SIX_DEGREES     0.1047198
#define SEVEN_DEGREES   0.1221730
#define TEN_DEGREES     0.1745329

   float plus_or_minus(float val);

   *x = 0.0;
   *x_dot = 0.0;
   *theta_dot = 0.0;
   if (TILTED)
      *theta = plus_or_minus(SIX_DEGREES);
   else
      *theta = 0.0;
}


/*---------------------------------------------------------------------+
| plus_or_minus: takes a value and randomly returns either that value  |
|     or its negation                                                  |
+---------------------------------------------------------------------*/

float plus_or_minus(float val)
{
   long random(void);            /* system random number generator */

/* if RAND_MAX is undefined, try (random() / (float) ((1 << 31) - 1)) */
   if ((random() / (float)RAND_MAX)  >  0.5)
      return val;
   else
      return -val;
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
             return 1;  /* fail == true */
	}
   
   return 0;
}
