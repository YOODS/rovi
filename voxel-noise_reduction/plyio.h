#pragma once 

extern Point *read_ply(char *fname,int *dn);
extern void write_ply(Point *dp,int dn,char *outfn,int ascf,int notexf,unsigned char *color);