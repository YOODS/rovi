#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include "cparser.h"

static char *cparsed[16];
static char buffer[512];

char **cparser(FILE *f){
	char *s1=buffer;
	char *s2=s1;
	int cn=0;
	int pn=0;
	for(;;){
		int a=fgetc(f);
		if(iscntrl(a)){
			if(cn>0){
				ungetc(a,stdin);
				*s1=0;
				break;
			}
			else return NULL;
		}
		else if(a==';'){
			if(cn>0){
				*s1=0;
				break;
			}
		}
		else if(a==' '){
			if(pn==cn) continue;
			else{
				*s1=0;
				s1++;
				pn=cn;
			}
		}
		else{
			if(pn==cn) cparsed[cn++]=s1;
			*s1=a;
			s1++;
		}
	}
	for(int i=cn;i<16;i++){
		cparsed[i]=NULL;
	}
	return cparsed;
}
/*
int main(int argc,char **argv){
	for(int i=0;;i++){
		char **argv=cparser(stdin);
		if(argv==NULL) break;
		printf("%d) %s %s %s\n",i,argv[0],argv[1],argv[2]);
	}
	return 0;
}
*/