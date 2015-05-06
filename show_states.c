#include <stdio.h>



typedef struct QVALS {
	int t, td, x, xd, a;
	float value;
}Qvalues;


int main (void)
{
	FILE *fh;
	char *filename = "log/a.values";
	Qvalues cq;

	fh = fopen(filename, "r");
	if(fh == 0)
	{
		printf("could not open\'%s\' for reading!\n", filename);
		return 1;
	}

	while(1)
	{
		if(!fread(&cq, sizeof(cq), 1, fh))
			break;

		printf("%d %d %d %d %d %f\n", cq.t, cq.x, cq.td, cq.xd, cq.a, cq.value);
	}

	fclose(fh);
	
	return 0;
}
