#include <stdio.h>
#include "point.h"

int atox(char *p,int n) {
	int ans=0;
	for(int i=0; i<n; i++) {
		int a=('0'<=*p && *p<='9') ? *p-'0': *p-'a'+10;
		ans=ans*16+a;
		++p;
	}
	return ans;
}

void get_minmax(Point *dp,int dn, AreaLimits *a) {
	a->xmin=a->xmax=dp[0].x;
	a->ymin=a->ymax=dp[0].y;
	a->zmin=a->zmax=dp[0].z;
	for(int i=1; i<dn; i++) {
//printf("%f,%f,%f\n",dp[0].x,dp[0].y,dp[0].z);	
		if(a->xmin>dp[i].x) a->xmin=dp[i].x;
		if(a->xmax<dp[i].x) a->xmax=dp[i].x;
		if(a->ymin>dp[i].y) a->ymin=dp[i].y;
		if(a->ymax<dp[i].y) a->ymax=dp[i].y;
		if(a->zmin>dp[i].z) a->zmin=dp[i].z;
		if(a->zmax<dp[i].z) a->zmax=dp[i].z;
	}
	a->xmin=(int)a->xmin-1; a->xmax=(int)a->xmax+1;
	a->ymin=(int)a->ymin-1; a->ymax=(int)a->ymax+1;
	a->zmin=(int)a->zmin-1; a->zmax=(int)a->zmax+1;
	printf("xmin=%g\nxmax=%g\nymin=%g\nymax=%g\nzmin=%g\nzmax=%g\n"
			,a->xmin,a->xmax,a->ymin,a->ymax,a->zmin,a->zmax);
}

