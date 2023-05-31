#include "ANO_TC.h"

uint8_t DataStore[50];

void ANO_TC_Send(int16_t data1,int16_t data2,int16_t data3,int16_t data4, int16_t data5,int16_t data6)
{
    
	uint8_t _cnt=0;
	
	DataStore[_cnt++]=0xAA;   
	DataStore[_cnt++]=0xFF;   
	DataStore[_cnt++]=0xF1;     
	DataStore[_cnt++]=0;
    
    DataStore[_cnt++]=BYTE0(data1);
	DataStore[_cnt++]=BYTE1(data1);
	
	
	DataStore[_cnt++]=BYTE0(data2);
	DataStore[_cnt++]=BYTE1(data2);
	
	DataStore[_cnt++]=BYTE0(data3);
	DataStore[_cnt++]=BYTE1(data3);
	
	DataStore[_cnt++]=BYTE0(data4);
	DataStore[_cnt++]=BYTE1(data4);
	
	DataStore[_cnt++]=BYTE0(data5);
	DataStore[_cnt++]=BYTE1(data5);
	
	DataStore[_cnt++]=BYTE0(data6);
	DataStore[_cnt++]=BYTE1(data6);

 
	DataStore[3] = _cnt-4;
	
	uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    for(uint8_t i = 0; i < DataStore[3]+4 ; i++){
        sumcheck += DataStore[i];
        addcheck += sumcheck;
    }
    
    DataStore[_cnt++] = sumcheck;
    DataStore[_cnt++] = addcheck;

	HAL_UART_Transmit (&huart1, DataStore , _cnt ,0xff ) ;


}

void ANO_TC_SendEuler(float roll, float pitch, float yaw){
    uint8_t _cnt=0;
	int16_t _temp = 0;

	DataStore[_cnt++]=0xAA;   
	DataStore[_cnt++]=0xFF;   
	DataStore[_cnt++]=0xF1;     
	DataStore[_cnt++]=0;
    
    _temp = (int16_t)(roll * 100);
    DataStore[_cnt++]=BYTE0(_temp);
	DataStore[_cnt++]=BYTE1(_temp);
	
	_temp = (int16_t)(pitch * 100);
	DataStore[_cnt++]=BYTE0(_temp);
	DataStore[_cnt++]=BYTE1(_temp);
	
    _temp = (int16_t)(yaw * 100);
	DataStore[_cnt++]=BYTE0(_temp);
	DataStore[_cnt++]=BYTE1(_temp);
	
	DataStore[3] = _cnt-4;
	
	uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    for(uint8_t i = 0; i < DataStore[3]+4 ; i++){
        sumcheck += DataStore[i];
        addcheck += sumcheck;
    }
    
    DataStore[_cnt++] = sumcheck;
    DataStore[_cnt++] = addcheck;

	HAL_UART_Transmit (&huart1, DataStore , _cnt ,0xff ) ;
}

