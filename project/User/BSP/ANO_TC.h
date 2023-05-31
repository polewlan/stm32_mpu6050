#ifndef __ANO_TC_H_
#define __ANO_TC_H_

#include "usart.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void ANO_TC_Send(int16_t data1,int16_t data2,int16_t data3,int16_t data4, int16_t data5,int16_t data6);
void ANO_TC_SendEuler(float roll, float pitch, float yaw);




#endif
