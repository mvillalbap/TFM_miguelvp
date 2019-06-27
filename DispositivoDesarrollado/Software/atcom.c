/*
 * atcom.c
 *
 *  Created on: 14 ene. 2019
 *      Author: miguelvp
 */

#include "atcom.h"
#include "FreeRTOS.h"
#include "usart.h"
#include "pids.h"
#include "printf.h"

// Tamaño de las cabeceras de los comandos
#define STN_COMMAND			4
#define NB_COMMAND			3
#define GNSS_COMMAND		5

// Posición de la carga de la tasa de transferencia
#define BAUD_SLEEP_PAYLOAD	10

// Timeout del envío de la UART
#define TIMEOUT_UART		500

// Umbrales tabla ASCII
#define ASCII_NUMBER_THRESHOLD	48
#define ASCII_LETTER_THRESHOLD	65

// Instrucciones disponibles
#define N_INSTRUCCIONES			48
#define MAX_CHAR_INST			15

// Comandos del módulo NB-IoT
#define CONFIG_CONNECTION	"AT+CEREG=2;+CSCON=1;+CFUN=1;+CGDCONT=0,\"IP\",\"\";+COPS=1,2,\"21401\";\r"
#define CONFIG_NUM_B		66
#define CHECK_CONNECTION	"AT+CGDCONT?\r"
#define CHECK_NUM_B			12
#define SOCKET_CREATION		"AT+NSOCR=DGRAM,17,16666,1\r"
#define CREATION_NUM_B		26

// Comandos del módulo GNSS
#define RESTART_COMMAND		"$PMTK103*30\r\n"
#define START_COMMAND		"$PMTK324,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*37\r\n"
#define GALILEO_COMMAND		"$PMTK353,0,0,1,1,0*2B\r\n"
#define GPS_COMMAND			"$PMTK353,1,0,1,1,0*2A\r\n"
#define GLONASS_COMMAND		"$PMTK353,0,1,1,1,0*2A\r\n"
#define MULTIPLE_COMMAND	"$PMTK353,1,1,0,0,0*2B\r\n"
#define STATIC_COMMAND		"$PMTK386,1.4*38\r\n"
#define RESTART_CMD_NUM_B	13
#define START_CMD_NUM_B		55
#define MULTIPLE_CMD_NUM_B	23
#define STATIC_CMD_NUM_B	17

// Instrucciones disponibles a ejecutar
static const uint8_t inst[N_INSTRUCCIONES][MAX_CHAR_INST] = {
		// STN conf
		{"\r\0"},
		{"lp\r\0"},
		{"echo\r\0"},
		{"header\r\0"},
		{"sn\r\0"},
		{"brate\r\0"},
		{"sleep\r\0"},
		{"getProt\r\0"},
		{"default\r\0"},

		// OBD data
		{"s11\r\0"},
		{"s12\r\0"},
		{"s13\r\0"},
		{"s14\r\0"},
		{"s15\r\0"},
		{"s16\r\0"},
		{"s9\r\0"},
		{"rpm\r\0"},
		{"speed\r\0"},
		{"timestart\r\0"},
		{"air\r\0"},
		{"fuel\r\0"},
		{"dpf1\r\0"},
		{"dpf2\r\0"},
		{"noxNTE\r\0"},
		{"pmNTE\r\0"},
		{"noxSensor\r\0"},
		{"pmSensor\r\0"},
		{"noxSensorC\r\0"},
		{"monitor\r\0"},
		{"vin\r\0"},
		{"obd\r\0"},

		// NB instructions
		{"connect\r\0"},
		{"check\r\0"},
		{"create\r\0"},

		// GNSS instructions
		{"restart\r\0"},
		{"gal\r\0"},
		{"gps\r\0"},
		{"glo\r\0"},
		{"multiple\r\0"},
		{"static\r\0"},
		{"start\r\0"},

		// Test commands
		{"test speed\r\0"},
		{"test stop\r\0"},

		// Print car data
		{"data\r\0"},

		// Error
		{"error\r\0"}
};

// Comunicaciones con cada uno de los módulos
static atCom *stnPort, *gnssPort, *nbPort;

// UARTs
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

static uint8_t sendCMD(comando cmd, atCom *comPort);
static uint8_t decodeCommand(comando cmd, atCom *port);
static uint8_t decodeOBD(command cmd);
static void importPID(uint8_t *request, uint8_t PID);

static uint8_t str_cmp(uint8_t *str1, const uint8_t *str2);

/*
 * @brief	Inicialización de la comunicación con el STN
 * @param	Nada
 * @retval	1 -> Todo correcto
 * 			0 -> No se ha podido reservar memoria
 */
uint8_t initSTNCom(void)
{
	if ((stnPort = (atCom*) pvPortMalloc(sizeof(atCom))) == NULL)
		return 0;

	if ((stnPort->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t))) == NULL) {
		vPortFree(stnPort);
		return 0;
	}

	stnPort->huart = &huart1;
	stnPort->lastCom = NO_CMD;

	return 1;
}

/*
 * @brief	Inicialización de la comunicación con el GNSS
 * @param	Nada
 * @retval	1 -> Todo correcto
 * 			0 -> No se ha podido reservar memoria
 */
uint8_t initGNSSCom(void)
{
	if ((gnssPort = (atCom*) pvPortMalloc(sizeof(atCom))) == NULL)
		return 0;

	gnssPort->huart = &huart2;

	return 1;
}

/*
 * @brief	Inicialización de la comunicación con el módulo de NB-IoT
 * @param	Nada
 * @retval	1 -> Todo correcto
 * 			0 -> No se ha podido reservar memoria
 */
uint8_t initNBCom(void)
{
	if ((nbPort = (atCom*) pvPortMalloc(sizeof(atCom))) == NULL)
		return 0;

	nbPort->huart = &huart3;

	nbPort->lastReq = NULL;
	return 1;
}

/*
 * @brief	Envío de comando al módulo STN
 * @param	mssg: puntero al mensaje que hay que enviar
 * @retval	1 -> Se ha enviado
 * 			0 -> Error en el envío
 */
uint8_t STN_sendCMD(uint8_t *mssg)
{
	comando cmd;
	uint8_t i, j, match;
	uint32_t max;

	// Comprobamos cuál es el mensaje a enviar
	for (cmd.type = 0; cmd.type < NB_CONNECT; cmd.type++) {
		match = str_cmp(&mssg[STN_COMMAND], inst[cmd.type]);
		if (match)
			break;
	}

	if (cmd.type >= NB_CONNECT)
		return 0;

	// Recibimos los parámteros del mensaje
	cmd.params = 0;
	if ((cmd.type == STN_ECHO && mssg[9] == '1') || (cmd.type == STN_HEADER && mssg[11] == '1')) {
		cmd.params = 1;
	} else if (cmd.type == STN_BAUD_SPEED || cmd.type == STN_SLEEP) {
		max = 1;
		for (i = BAUD_SLEEP_PAYLOAD; mssg[i] != '\0' && mssg[i] != '\r' ; i++) {
			max = max * 10;
		}
		max = max/10;
		for (j = BAUD_SLEEP_PAYLOAD; j < i; j++) {
			cmd.params = cmd.params + (mssg[j] - ASCII_NUMBER_THRESHOLD)*max;
			max = max / 10;
		}
	}

	// Distinguimos entre si es mensaje de configuración u OBD
	stnPort->lastCom = cmd.type;

	// Mensaje para el vehículo
	if (cmd.type > STN_DEFAULT_FILTERS) {
		vPortFree(stnPort->lastReq);
		if (cmd.type == STN_USER_OBD) {
			i = 8;
			while (mssg[i] != '\r'){
				i++;
			}
			stnPort->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(i-8+2));
			sprintf_((char*) stnPort->lastReq, "%s\r", (char*) &(mssg[8]));
			stnPort->sendNum = i-8+1;
		} else if (!(stnPort->sendNum = decodeOBD(cmd.type))) {
			return 0;
		}

		if(HAL_UART_Transmit(stnPort->huart, stnPort->lastReq, stnPort->sendNum, TIMEOUT_UART) == HAL_ERROR) {
			return 0;
		}

		return 1;

	// Mensaje de configuración
	} else {
		return sendCMD(cmd, stnPort);
	}
}

/*
 * @brief	Envío de comando al módulo GNSS
 * @param	mssg: puntero al mensaje que hay que enviar
 * @retval	1 -> Se ha enviado
 * 			0 -> Error en el envío
 */
uint8_t GNSS_sendCMD(uint8_t *mssg)
{
	comando cmd;
	uint8_t match;

	// Comprobamos cuál es el mensaje a enviar
	for (cmd.type = GNSS_RESTART; cmd.type < NO_CMD; cmd.type++) {
		match = str_cmp(&mssg[GNSS_COMMAND], inst[cmd.type]);
		if (match)
			break;
	}

	if (cmd.type >= NO_CMD)
		return 0;

	// Según el tipo, realizamos la distinción de los comandos
	cmd.params = 0;
	if (cmd.type == GNSS_RESTART) {

			if(HAL_UART_Transmit_IT(gnssPort->huart, (uint8_t *) RESTART_COMMAND, RESTART_CMD_NUM_B) == HAL_ERROR) {
				return 0;
			}

			return 1;

	} else if (cmd.type == GNSS_GALILEO) {

		if(HAL_UART_Transmit_IT(gnssPort->huart, (uint8_t *) GALILEO_COMMAND, MULTIPLE_CMD_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	} else if (cmd.type == GNSS_GPS) {

		if(HAL_UART_Transmit_IT(gnssPort->huart, (uint8_t *) GPS_COMMAND, MULTIPLE_CMD_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	} else if (cmd.type == GNSS_GLONASS) {

		if(HAL_UART_Transmit_IT(gnssPort->huart, (uint8_t *) GLONASS_COMMAND, MULTIPLE_CMD_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	} else if (cmd.type == GNSS_MULTIPLE) {

		if(HAL_UART_Transmit_IT(gnssPort->huart, (uint8_t *) MULTIPLE_COMMAND, MULTIPLE_CMD_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	} else if (cmd.type == GNSS_STATIC) {

		if(HAL_UART_Transmit_IT(gnssPort->huart, (uint8_t *) STATIC_COMMAND, STATIC_CMD_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	} else if (cmd.type == GNSS_START) {

		if(HAL_UART_Transmit_IT(gnssPort->huart, (uint8_t *) START_COMMAND, START_CMD_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;
	}
	return 0;
}

/*
 * @brief	Envío de comando al módulo NB-IoT
 * @param	mssg: puntero al mensaje que hay que enviar
 * @retval	1 -> Se ha enviado
 * 			0 -> Error en el envío
 */
uint8_t NB_sendCMD(uint8_t *mssg)
{
	comando cmd;
	uint8_t match;

	// Comprobamos cuál es el mensaje a enviar
	for (cmd.type = NB_CONNECT; cmd.type < GNSS_MULTIPLE; cmd.type++) {
		match = str_cmp(&mssg[NB_COMMAND], inst[cmd.type]);
		if (match)
			break;
	}

	if (cmd.type >= GNSS_RESTART)
		return 0;

	// Según el tipo, realizamos la distinción de los comandos a transmitir
	cmd.params = 0;
	if (cmd.type == NB_CONNECT) {

		if(HAL_UART_Transmit_IT(nbPort->huart, (uint8_t *) CONFIG_CONNECTION, CONFIG_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	} else if (cmd.type == NB_CHECK_CONNECTION) {

		if(HAL_UART_Transmit_IT(nbPort->huart, (uint8_t *) CHECK_CONNECTION, CHECK_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	} else if (cmd.type == NB_SOCKET_CREATION) {

		if(HAL_UART_Transmit_IT(nbPort->huart, (uint8_t *) SOCKET_CREATION, CREATION_NUM_B) == HAL_ERROR) {
			return 0;
		}

		return 1;

	}
	return 0;
}

/*
 * @brief	Devuelve la información del último comando enviado por el módulo STN en
 * 			formato de comando y su equivalencia en orden para el módulo
 * @param	sent: puntero donde se indica el tipo de comando enviado
 * @retval	Puntero a la orden enviada al módulo
 */
uint8_t* STN_getLastCommand(command *sent)
{
	*sent = stnPort->lastCom;
	return stnPort->lastReq;
}

/*
 * @brief	Envío de un comando de configuración a cualquiera de los módulos
 * @param	cmd: comando a enviar
 * 			comPort: puntero al puerto de comunicaciones por el que se mandará el mensaje
 * @retval	1 -> Todo correcto
 * 			0 -> No se ha podido transmitir el mensaje
 */
static uint8_t sendCMD(comando cmd, atCom *comPort)
{
	vPortFree(comPort->lastReq);
	if (!(comPort->sendNum = decodeCommand(cmd, comPort))) {
		return 0;
	}

	if(HAL_UART_Transmit(comPort->huart, comPort->lastReq, comPort->sendNum, TIMEOUT_UART) == HAL_ERROR) {
		return 0;
	}

	return 1;
}

/*
 * @brief	Decodificación del comando en los bytes asociados para el envío
 * @param	cmd: comando a enviar
 * 			port: puntero al puerto de comunicaciones por el que se mandará el mensaje
 * @retval	número de bytes a transmitir
 */
static uint8_t decodeCommand(comando cmd, atCom *port)
{
	uint8_t i, tipo, numOrden, sendNum;
	uint32_t j, value = 0;
	tipo = 1;
	switch(cmd.type) {

	// Comandos STN
	case STN_REPEAT:
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*2)) == NULL)
			return 0;
		port->lastReq[0] = '\r';
		port->lastReq[1] = '\0';
		sendNum = 2;
		return sendNum;

	case STN_LOW_POWER:
		numOrden = 2;
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s\r", STN_LOW_POWER_CMD);
		break;

	case STN_BAUD_SPEED:
		tipo = 0;
		value = cmd.params;
		for (i = 0, j = 1000000; i < 7; i++, j = j/10) {
			if (value / j != 0)
				break;
		}
		numOrden = 3+(7-i);
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s%u\r", STN_BAUD_SPEED_CMD, (unsigned int) cmd.params);
		break;

	case STN_ECHO:
		numOrden = 3;
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s%u\r", STN_ECHO_CMD, (unsigned int) cmd.params);
		break;

	case STN_HEADER:
		numOrden = 3;
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s%u\r", STN_HEADER_CMD, (unsigned int) cmd.params);
		break;

	case STN_SERIAL_NUMBER:
		tipo = 0;
		numOrden = 2;
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s\r", STN_SERIAL_NUMBER_CMD);
		break;

	case STN_SLEEP:
		value = cmd.params;
		tipo = 0;
		for (i = 0, j = 100; i < 3; i++, j = j/10) {
			if (value / j != 0)
				break;
		}
		numOrden = 6+(3-i);
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s%u\r", STN_SLEEP_CMD, (unsigned int) cmd.params);
		break;

	case STN_GET_PROTOCOL:
		numOrden = 2;
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s\r", STN_GET_PROTOCOL_CMD);
		break;

	case STN_DEFAULT_FILTERS:
		numOrden = 3;
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + STN_STRUCT))) == NULL) {
			return 0;
		}
		sprintf_((char*) port->lastReq, "%s\r", STN_DEFAULT_FILTERS_CMD);
		break;

	default:
		return 0;
	}

	// Gestion de comandos AT y ST
	if (tipo < 2) {
		sendNum = numOrden + 3;
		return sendNum;

	// Gestion de comandos NMEA de ML865C1
	} else {
		if ((port->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*(numOrden + 8))) == NULL) {
			return 0;
		}
		port->lastReq[0] = '$';
		port->lastReq[1] = 'P';
		if (tipo == 2) {
			port->lastReq[2] = 'M';
			port->lastReq[3] = 'T';
			port->lastReq[4] = 'K';
		} else {
			port->lastReq[2] = 'T';
			port->lastReq[3] = 'W';
			port->lastReq[4] = 'S';
		}
		port->lastReq[numOrden+5] = '\r';
		port->lastReq[numOrden+6] = '\n';
		port->lastReq[numOrden+7] = '\0';
		sendNum = numOrden + 8;

		return sendNum-1;
	}
}

/*
 * @brief	Decodificación de los PIDs de OBD II
 * @param	cmd: comando a transmitir
 * @retval	Número de bytes a transmitir
 */
static uint8_t decodeOBD(command cmd)
{
	uint8_t *request;
	if ((stnPort->lastReq = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*8)) == NULL)
		return 0;
	request = stnPort->lastReq;
	request[0] = '0';
	request[1] = '1';
	request[4] = '\r';
	request[5] = '\0';

	switch(cmd) {

	// PIDs de parámetros soportados
	case STN_PIDS_1_1:
		importPID(&(request[2]), FIRST_PIDS);
		break;
	case STN_PIDS_1_2:
		importPID(&(request[2]), SECOND_PIDS);
		break;
	case STN_PIDS_1_3:
		importPID(&(request[2]), THIRD_PIDS);
		break;
	case STN_PIDS_1_4:
		importPID(&(request[2]), FOURTH_PIDS);
		break;
	case STN_PIDS_1_5:
		importPID(&(request[2]), FIFTH_PIDS);
		break;
	case STN_PIDS_1_6:
		importPID(&(request[2]), SIXTH_PIDS);
		break;
	case STN_PIDS_9:
		request[1] = '9';
		importPID(&(request[2]), FIRST_PIDS);
		break;

	// PIDS del modo de datos del vehículo
	case STN_GET_RPM:
		importPID(&(request[2]), ENGINE_RPM);
		break;
	case STN_GET_SPEED:
		importPID(&(request[2]), VEHICLE_SPEED);
		break;
	case STN_GET_TIMESTART:
		importPID(&(request[2]), TIME_SINCE_START);
		break;
	case STN_GET_AIR_TEMPERATURE:
		importPID(&(request[2]), AMBIENT_AIR_TEMP);
		break;
	case STN_GET_FUEL:
		importPID(&(request[2]), FUEL_TYPE);
		break;
	case STN_GET_DPF_1:
		importPID(&(request[2]), DPF_1);
		break;
	case STN_GET_DPF_2:
		importPID(&(request[2]), DPF_2);
		break;
	case STN_GET_NOx_NTE:
		importPID(&(request[2]), NOX_NTE);
		break;
	case STN_GET_PM_NTE:
		importPID(&(request[2]), PM_NTE);
		break;
	case STN_GET_NOx_SENSOR:
		importPID(&(request[2]), NOX_SENSOR);
		break;
	case STN_GET_PM_SENSOR:
		importPID(&(request[2]), PM_SENSOR);
		break;
	case STN_GET_NOx_SENSOR_CORRECTED:
		importPID(&(request[2]), NOX_SENSOR_CORRECTED);
		break;

	// PIDs del modo de información del vehículo
	case STN_GET_MONITOR:
		request[1] = '9';
		importPID(&(request[2]), MONITOR);
		break;
	case STN_GET_VIN:
		request[1] = '9';
		importPID(&(request[2]), VIN_NUMBER);
		break;
	default:
		return 0;
	}

	return 5;
}

/*
 * @brief	Reinicia el STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void importPID(uint8_t *request, uint8_t PID)
{
	// Pasamos a ASCII el primer número
	if ((PID >> 4) >= 0xA) {
		request[0] = (ASCII_LETTER_THRESHOLD-10) + (PID >> 4);
	} else {
		request[0] = ASCII_NUMBER_THRESHOLD + (PID >> 4);
	}

	// Pasamos a ASCII el segundo número
	if ((PID % 0x10) >= 0xA) {
		request[1] = (ASCII_LETTER_THRESHOLD-10) + (PID % 0x10);
	} else {
		request[1] = ASCII_NUMBER_THRESHOLD + (PID % 0x10);
	}
}

/*
 * @brief	Comparación de dos strings para ver si el primero es contenido en el segundo
 * @param	str1: string a comparar 1
 * 			str2: string a comparar 2
 * @retval	1 -> Contenido
 * 			0 -> No contenido
 */
static uint8_t str_cmp(uint8_t *str1, const uint8_t *str2)
{
	uint8_t i;
	for (i = 0; i < 20; i++) { // El 20 es arbitrario, aumentar si hay instrucciones con más de esos caracteres
		if (str2[i] == '\r' && (str1[i] == '\r' || str1[i] == ' '))
			return 1;
		else if (str1[i] != str2[i])
			return 0;
	}
	return 1;
}
