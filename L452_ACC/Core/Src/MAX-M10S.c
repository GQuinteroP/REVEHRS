/*
 * MAX-M10S.c
 *
 *  Created on: Apr 5, 2023
 *      Author: ubuntu
 */
#include "MAX-M10S.h"

void M10ProcessPackets(uint8_t *nmea_str)
{
	//TODO: A better way to convert float to integer should be found since left zeros may not be accounted
	if (sscanf((const char *)nmea_str, "NGGA,%lu.%lu,%lu.%lu,%c,%lu.%lu,%c,%hu,%hu,",
			&GPS.utc_time,  &GPS.utc_time_ms,
			&GPS.nmea_latitude_deg, &GPS.nmea_latitude_min, &GPS.ns,
			&GPS.nmea_longitude_deg, &GPS.nmea_longitude_min, &GPS.ew,
			(short unsigned int *)&GPS.lock, (short unsigned int *)&GPS.satellites) >= 1)
	        {
				GPS.lat_dec = nmeaToDec(GPS.nmea_latitude_deg, GPS.nmea_latitude_min, GPS.ns);
				GPS.lon_dec = nmeaToDec(GPS.nmea_longitude_deg, GPS.nmea_longitude_min, GPS.ew);
	        }

	/*if (sscanf(nmea_str, "NGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns,
			&GPS.nmea_longitude, &GPS.ew, &GPS.lock, &GPS.satellites, &GPS.hdop, &GPS.msl_altitude, &GPS.msl_units) >= 1)
        {}
    // Check if it is a GPRMC msg
    else if (sscanf(nmea_str, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%d", &utc_time,&rmc_status,&nmea_latitude, &ns, &nmea_longitude, &ew, &speed_k, &course_d, &date) >= 1)
        {}
    // GLL - Geographic Position-Lat/Lon
    else if (sscanf(nmea_str, "$GPGLL,%f,%c,%f,%c,%f,%c", &nmea_latitude, &ns, &nmea_longitude, &ew, &utc_time, &gll_status) >= 1)
        {}*/
}

float32_t nmeaToDec(float32_t deg_coord, float32_t min_coord, uint8_t nsew)
{
    uint32_t degree = (uint32_t)(deg_coord/100);
    float32_t minutes = deg_coord - degree*100 + min_coord/NMEA_dec_resolution;
    float32_t dec_deg = minutes / 60;
    float32_t decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}

void M10Schecksum(uint8_t *cmd, uint8_t size)
{
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	for (int ii = 2; ii < size - 2; ii++)
	{
		CK_A = CK_A + cmd[ii];
		CK_B = CK_B + CK_A;
	}
	cmd[size - 2]= CK_A;
	cmd[size - 1]= CK_B;
}

//TODO: Change to multiple values (only 115200 for now)
void M10ChangeBaudrate(uint32_t baudrate)
{
	uint8_t command[20] = {0xB5, 0x62, 0x06, 0x8A,
							0x0C, 0x00,
							0x01, 0x01, 0x00, 0x00,
							0x01, 0x00, 0x52, 0x40,
							0x00, 0xC2, 0x01, 0x00, //115200
							0xF4, 0xB1};
	HAL_UART_Transmit(&hlpuart1, command, 20, -1);
}

void M10GGAOnly()
{
	uint8_t command[117] = {0xB5, 0x62, 0x06, 0x8A, 0x6D, 0x00, 0x01, 0x01, 0x00, 0x00, 0xBA, 0x00, 0x91,
							0x20, 0x00, 0xBE, 0x00, 0x91, 0x20, 0x00, 0xC9, 0x00, 0x91, 0x20, 0x00, 0xCD,
							0x00, 0x91, 0x20, 0x00, 0xCA, 0x00, 0x91, 0x20, 0x00, 0xBF, 0x00, 0x91, 0x20,
							0x00, 0xC3, 0x00, 0x91, 0x20, 0x00, 0xC0, 0x00, 0x91, 0x20, 0x00, 0xC4, 0x00,
							0x91, 0x20, 0x00, 0xC8, 0x00, 0x91, 0x20, 0x00, 0xC5, 0x00, 0x91, 0x20, 0x00,
							0xAB, 0x00, 0x91, 0x20, 0x00, 0xAF, 0x00, 0x91, 0x20, 0x00, 0xAC, 0x00, 0x91,
							0x20, 0x00, 0xB0, 0x00, 0x91, 0x20, 0x00, 0xB4, 0x00, 0x91, 0x20, 0x00, 0xB1,
							0x00, 0x91, 0x20, 0x00, 0x03, 0x00, 0x51, 0x10, 0x00, 0x06, 0x00, 0x92, 0x20,
							0x00, 0x07, 0x00, 0x92, 0x20, 0x00, 0x0A, 0x00, 0x92, 0x20, 0x00, 0xD7, 0xA0};
	HAL_UART_Transmit(&hlpuart1, command, 117, -1);
}

//TODO: The time should be specified (t<=10), at the moment is 7 s
void M10PSMCT_RAM()
{
	uint8_t command[23] = {0xB5, 0x62, 0x06, 0x8A, 0x0F, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01,
			0x00, 0x21, 0x30, 0x58, 0x1B, 0x01, 0x00, 0xD0, 0x20, 0x02, 0x59, 0xB7};//Change rate in RAM
	HAL_UART_Transmit(&hlpuart1, command, 23, -1);
}

void M10PSMCT_BBR()
{
	uint8_t command[23] = {0xB5, 0x62, 0x06, 0x8A, 0x0F, 0x00, 0x01, 0x02, 0x00, 0x00,
			0x01, 0x00, 0x21, 0x30, 0x58, 0x1B, 0x01, 0x00, 0xD0, 0x20, 0x02, 0x5A, 0xC5};//Change rate in BBR
	HAL_UART_Transmit(&hlpuart1, command, 23, -1);
}
//TODO: Change to multiple values (only 10s for now)
void M10ChangeUpdateRate(uint32_t baudrate)
{
	uint8_t command[18] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0x88, 0x13, 0x89, 0x1E};
	HAL_UART_Transmit(&hlpuart1, command, 18, -1);
}

void M10Reset()
{
	uint8_t N=12;
	uint8_t rst_cmd[N];

	//Preamble bytes 1 and 2
	rst_cmd[0]= 0xb5;
	rst_cmd[1]= 0x62;

	//Class ID bytes 3 and 4
	rst_cmd[2]= 0x06;
	rst_cmd[3]= 0x04;

	//Length (payload) bytes 5 and 6 (unsigned little-endian 16-bit integer)
	rst_cmd[4]= 0x04;
	rst_cmd[5]= 0x00;

	//Payload (bytes 7-10)
	rst_cmd[6]= 0x00;
	rst_cmd[7]= 0x00;
	rst_cmd[8]= 0x00;
	rst_cmd[9]= 0x00;

	//Checksum
	M10Schecksum(rst_cmd, N);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)rst_cmd, N, -1);
}
