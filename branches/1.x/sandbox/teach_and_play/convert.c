#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Convert CSV to T&P file */
int main(int argc, char** argv){
	double scale, x_trans, y_trans, z_trans;
	FILE *in, *out;
	char line[100];
	double t, x, y, z, rp, rr, ry;
	double qw, qx, qy, qz;
	double c1, c2, c3, s1, s2, s3;
	
	if(argc < 6){
		printf("Usage: convert <file> <scale> <x_trans> <y_trans> <z_trans>\n");
		exit(0);
	}
	printf("argv1 = %s, argv2 = %s, argv3 = %s\n", argv[1], argv[2], argv[3]);
	scale = atof(argv[2]);
	x_trans = atof(argv[3]);
	y_trans = atof(argv[4]);
	z_trans = atof(argv[5]);
	printf("scale: %lf, x_trans: %lf\n", scale, x_trans);
	
	if((in = fopen(argv[1], "r")) == NULL){
		printf("Failed to open file for reading: %s\n", argv[1]);
		exit(0);
	}
	
	if((out = fopen("pose.csv", "w")) == NULL){
		printf("Could not create output file: pose.csv\n");
		exit(0);
	}
	fprintf(out, "pose_type\n");
	
	while(fgets(line, 99, in) != NULL){
		sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf, %lf", &t, &x, &y, &z, &rp, &rr, &ry);
		
		c1 = cos(ry*3.14/180/2);
		c2 = cos(rp*3.14/180/2);
		c3 = cos(rr*3.14/180/2);
		s1 = sin(ry*3.14/180/2);
		s2 = sin(rp*3.14/180/2);
		s3 = sin(rr*3.14/180/2);
		
		qw = c1 * c2 * c3 - s1 * s2 * s3;
		qx = s1 * s2 * c3 + c1 * c2 * s3;
		qy = s1 * c2 * c3 + c1 * s2 * s3;
		qz = c1 * s2 * c3 - s1 * c2 * s3;
		
		fprintf(out, "%f, %f, %f, %f, %f, %f, %f, %f\n", t * scale, x + x_trans, y + y_trans, z + z_trans, qw, qx, qy, qz);
	}
	
	fclose(in);
	fclose(out);
	
	
}


