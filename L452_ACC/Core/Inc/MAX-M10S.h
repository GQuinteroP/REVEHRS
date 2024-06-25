/*
 * MAX-M10S.h
 *
 *  Created on: Apr 5, 2023
 *      Author: ubuntu
 */

#ifndef INC_MAX_M10S_H_
#define INC_MAX_M10S_H_

#include "stm32l4xx_hal.h"
#include "stdint.h"
#include "stdio.h"
#include "arm_math.h"

#define	NMEA_dec_resolution	100000	//ddmm.mmmmm

typedef struct {
	// GGA - Global Positioning System Fixed Data
	uint32_t nmea_latitude_deg;
	uint32_t nmea_latitude_min;
	uint32_t nmea_longitude_deg;
	uint32_t nmea_longitude_min;

	float32_t lat_dec;
	float32_t lon_dec;

	uint32_t utc_time;
	uint32_t utc_time_ms;
	uint8_t ns, ew;
	uint8_t lock;
	uint8_t satellites;
	float32_t hdop;
	float32_t msl_altitude;
	uint8_t msl_units;

	// RMC - Recommended Minimmum Specific GNS Data
	uint8_t rmc_status;
	float32_t speed_k;
	float32_t course_d;
	uint32_t date;

	// GLL
	uint8_t gll_status;

	// VTG - Course over ground, ground speed
	float32_t course_t; // ground speed true
	uint8_t course_t_unit;
	float32_t course_m; // magnetic
	uint8_t course_m_unit;
	uint8_t speed_k_unit;
	float32_t speed_km; // speek km/hr
	uint8_t speed_km_unit;

} MAX_M10S;

extern UART_HandleTypeDef hlpuart1;
extern MAX_M10S GPS;

void M10Schecksum(uint8_t *cc, uint8_t size);
void M10ChangeBaudrate(uint32_t baudrate);
void M10GGAOnly();
void M10PSMCT_RAM();
void M10PSMCT_BBR();
void M10Reset();
void M10ProcessPackets(uint8_t *nmea_str);
float32_t nmeaToDec(float32_t deg_coord, float32_t min_coord, uint8_t nsew);

#endif /* INC_MAX_M10S_H_ */
