#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#define SENSOR_NUMBER   4
#define CALIB_X 'X'
#define CALIB_Y 'Y'
unsigned long coords_x[SENSOR_NUMBER];
unsigned long coords_y[SENSOR_NUMBER];

unsigned int gray2bin(unsigned int num)
{
    unsigned int mask;
    for (mask = num >> 1; mask != 0; mask = mask >> 1)
    {
        num = num ^ mask;
    }
    return num;
}

void calib(char* buf)
{
    char* xy=strtok(buf,":");
    char* coords=strtok(NULL,":");
    char *s[SENSOR_NUMBER];

    s[0]=strtok(coords,",");
    int i;
    for(i=1;i<SENSOR_NUMBER;i++) s[i]=strtok(NULL,",");

    if(xy[0]==CALIB_X){
        for(i=0;i<SENSOR_NUMBER;i++) coords_x[i]=gray2bin(strtoul(s[i],NULL,16));
    }else if(xy[0]==CALIB_Y){
        for(i=0;i<SENSOR_NUMBER;i++) coords_y[i]=gray2bin(strtoul(s[i],NULL,16));
    }
}

int main(int argc, char* argv[])
{
    unsigned int num;
    if(argc>1){
        num = strtoul(argv[1],NULL,16);
        num = gray2bin(num);
        printf("%d\n",num);
        return;
    }
    char buf[]="Y:10,0C,12,DD";
    calib(buf);
    int i;
    for(i=0;i<SENSOR_NUMBER;i++){
        printf("%lu\n",coords_y[i]);
    }
}
