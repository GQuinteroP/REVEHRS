/*
 * EEPROM.c
 *
 *  Created on: Jul 4, 2023
 *      Author: ubuntu
 */

#include "EEPROM.h"
#include "m95p32.h"


uint16_t get_eeprom_data_len()
{
	uint8_t		status;
	uint16_t	data_len = 0;

	for(int ii=0;ii<eeprom_size;ii++)
	{
		Single_Read(&status, ii*M95P32_PAGESIZE ,1);
		kappa("\r\n [%d]: %x", ii, status);
		if(status!=0xFF)
			data_len++;
	}
	kappa("\r\nData_len:%d", data_len);
	return data_len;
}
