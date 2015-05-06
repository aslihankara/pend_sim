#include <stdio.h>


typedef struct SCORE_STRUCT{
	int score;
}Score_struct;

#define AVG_COUNT 100
int read_scores(char *filename)
{
	int cvalues[AVG_COUNT];
	float caverage;
	int i,j;
	FILE *df;
 	char *default_filename = "log/a.score";  
	char linebuf[64];
	FILE *outfile = fopen("log/average.score", "w");
	char exit, tally;
	int iaverage;

	if(filename == 0)
		filename = default_filename;

	df = fopen(filename, "r");

	if(df == 0)
	{
		printf("could not open \'%s\'", filename);
		return 1;
	}

	printf("processing scores from \'%s\'...", filename);
	fflush(stdout);
	caverage = 0;
	i = 0;
	j = 0;
	exit = 0;
	while (!exit)
	{
		tally = 0;
		if(!fgets(linebuf, sizeof(linebuf), df))
		{
			--i;
			exit = 1;
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
	fclose(df);
	fclose(outfile);
	return;
}


int main (void)
{
	read_scores(0);

	return 0;
}
