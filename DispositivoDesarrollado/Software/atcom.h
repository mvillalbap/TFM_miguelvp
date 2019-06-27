/*
 * atcom.h
 *
 *  Created on: 14 ene. 2019
 *      Author: miguelvp
 */

#ifndef ATCOM_H_
#define ATCOM_H_

#include "stm32l4xx_hal.h"

// Comandos disponibles
typedef enum command {
	// Comandos STN
	STN_REPEAT, STN_LOW_POWER, STN_ECHO, STN_HEADER,
	STN_SERIAL_NUMBER, STN_BAUD_SPEED,
	STN_SLEEP, STN_GET_PROTOCOL, STN_DEFAULT_FILTERS,

	// Asociados a OBD
	STN_PIDS_1_1, STN_PIDS_1_2, STN_PIDS_1_3,
	STN_PIDS_1_4, STN_PIDS_1_5, STN_PIDS_1_6,
	STN_PIDS_9,
	STN_GET_RPM, STN_GET_SPEED,
	STN_GET_TIMESTART, STN_GET_AIR_TEMPERATURE,
	STN_GET_FUEL,
	STN_GET_DPF_1, STN_GET_DPF_2,
	STN_GET_NOx_NTE, STN_GET_PM_NTE,
	STN_GET_NOx_SENSOR, STN_GET_PM_SENSOR,
	STN_GET_NOx_SENSOR_CORRECTED, STN_GET_MONITOR,
	STN_GET_VIN, STN_USER_OBD,

	// Comandos NB-IoT
	NB_CONNECT, NB_CHECK_CONNECTION,
	NB_SOCKET_CREATION,

	// Comandos GNSS
	GNSS_RESTART, GNSS_GALILEO, GNSS_GPS, GNSS_GLONASS,
	GNSS_MULTIPLE, GNSS_STATIC, GNSS_START,

	// Ninguno de ellos
	NO_CMD,

} command;

// Definción del puerto de comunicaciones con un módulo
typedef struct atCom {
	UART_HandleTypeDef *huart;
	uint8_t *lastReq;
	command lastCom;
	uint8_t sendNum;
} atCom;

// Definción de un comando a enviar
typedef struct comando {
	command type;
	uint32_t params;
} comando;

uint8_t initSTNCom(void);
uint8_t initGNSSCom(void);
uint8_t initNBCom(void);
uint8_t STN_sendCMD(uint8_t *mssg);
uint8_t GNSS_sendCMD(uint8_t *mssg);
uint8_t NB_sendCMD(uint8_t *mssg);
uint8_t* STN_getLastCommand(command *sent);

// STN Commands to UART
#define STN_STRUCT					4
#define STN_LOW_POWER_CMD			"ATLP"
#define STN_ECHO_CMD				"ATE "
#define STN_HEADER_CMD				"ATH "
#define STN_SERIAL_NUMBER_CMD		"STSN"
#define STN_BAUD_SPEED_CMD			"STBR "
#define STN_SLEEP_CMD				"STSLEEP "
#define STN_GET_PROTOCOL_CMD		"ATDP"
#define STN_DEFAULT_FILTERS_CMD		"ATCRA"

#endif /* ATCOM_H_ */
