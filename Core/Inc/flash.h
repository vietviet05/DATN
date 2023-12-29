#ifndef __FLASH_H
#define __FLASH_H
#include "stdint.h"
#include "string.h"  


#pragma pack(1)
typedef struct{
		uint8_t no; 
		uint8_t ssid[30];
		uint8_t pass[30];
}wifi_infor_t;
#pragma pack()
void flash_Erase(uint32_t address);
void flash_Write_Int (uint32_t address, int value); 
void flash_Write_Float (uint32_t address, float f); 
void flash_Write_Array (uint32_t address, uint8_t *arr, uint16_t lengh); 
void flash_Write_Struct (uint32_t address, wifi_infor_t dta); 


int flash_Read_Int (uint32_t address); 
float flash_Read_Float (uint32_t address); 
void flash_Read_Array (uint32_t address, uint8_t *arr, uint16_t lengh); 
void flash_Read_Struct (uint32_t address, wifi_infor_t *dta); 



#endif