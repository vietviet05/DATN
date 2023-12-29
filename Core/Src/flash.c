#include "flash.h"
#include "stm32f1xx.h" 



void flash_Erase(uint32_t address)
{ 
	HAL_FLASH_Unlock ();
	FLASH_EraseInitTypeDef EraseInitStruct;  
	EraseInitStruct.Banks = 1; 
	EraseInitStruct.NbPages = 1; 
	EraseInitStruct.PageAddress = address; 
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	uint32_t pageerr;
	HAL_FLASHEx_Erase (&EraseInitStruct, &pageerr);
	HAL_FLASH_Lock(); 
}
void flash_Write_Int (uint32_t address, int value)
{
	HAL_FLASH_Unlock ();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value);
	HAL_FLASH_Lock(); 
}
void flash_Write_Float (uint32_t address, float f) 
{ 
	HAL_FLASH_Unlock ();
	uint8_t data[4]; 
	*(float*) data = f;
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *(uint32_t*)data);
	HAL_FLASH_Lock(); 
}
void flash_Write_Array (uint32_t address, uint8_t *arr, uint16_t lengh)
{
 HAL_FLASH_Unlock ();
 uint16_t *pt = (uint16_t *)arr;
 for(uint16_t i = 0; i < (lengh+1)/2; i++)
{
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2*i, *pt); 
	pt++;
}
 HAL_FLASH_Lock();
}	
void flash_Write_Struct (uint32_t address, wifi_infor_t dta)
{
 flash_Write_Array(address, (uint8_t*)&dta, sizeof(dta));

}	

int flash_Read_Int (uint32_t address)
{
	return *(__IO uint16_t *) (address);

}	
float flash_Read_Float (uint32_t address)
{
	uint32_t data = *(__IO uint32_t *) (address);
	return *(float*)(&data);
}	
void flash_Read_Array (uint32_t address, uint8_t *arr, uint16_t lengh)
{
	uint16_t * pt = (uint16_t*)arr;
	for(uint16_t i = 0; i < (lengh+1)/2; i++) 
	{
		*pt = *(__IO uint16_t *) (address + 2*i);
		pt++;
	}

}	
void flash_Read_Struct (uint32_t address, wifi_infor_t *dta) 
{
	flash_Read_Array(address, (uint8_t *)dta,sizeof(wifi_infor_t));

}