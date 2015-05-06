#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


typedef struct SCORE_STRUCT{
	int score;
}Score_struct;

#define DEFAULT_AVERAGE 1000

int main (int argc, char* argv[])
{
	int *cvalues;
	int AVG_COUNT = DEFAULT_AVERAGE;
	float caverage;
	int i,j;
	FILE *df;
 	char *infilename = "log/a.score";  
 	char *outfilename = "log/average.score";  
	char linebuf[64];
	FILE *outfile = fopen("log/average.score", "w");
	char done, tally;
	int iaverage;

	switch(argc) //handle cmd line parameters
	{
		case 1:
			//don't modify any defaults
			break;
		case 4: //modify the outfile
			outfilename = argv[3];
		case 3: //modify the infile
			infilename = argv[2];
		case 2: //modify the average
			AVG_COUNT = atoi(argv[1]);	
			break;
		default:
			printf("invalid number of input parameters\n");
			exit(1);
	}

	df = fopen(infilename, "r");
	outfile = fopen(outfilename, "w");

	if(df == 0)
	{
		printf("could not open \'%s\'", infilename);
		return 1;
	}

	cvalues = malloc(AVG_COUNT*sizeof(int));

	printf("processing scores from \'%s\'...", infilename);
	fflush(stdout);
	caverage = 0;
	i = 0;
	j = 0;
	done = 0;
	while (!done)
	{
		tally = 0;
		if(!fgets(linebuf, sizeof(linebuf), df))
		{
			--i;
			done = 1;
			tally = 1;
		}
		else
			sscanf(linebuf, "%d\n", &(cvalues[i]));

		++i;
		if(i == AVG_COUNT)
			tally = 1;
		if(tally)
		{
			caverage = 0;
			j = i;
			for(i=0; i < j; ++i)
			{
				caverage += (float)cvalues[i]/j;
			}
			iaverage = (int)caverage;
			fprintf(outfile, "%d\n", iaverage);
			
			i=0;
		}
	}
	printf("done\n");
	free(cvalues);
	fclose(df);
	fclose(outfile);
	return 0;
}
