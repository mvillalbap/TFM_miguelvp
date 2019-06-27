/*
 * micro.c
 *
 *  Created on: 15 ene. 2019
 *      Author: miguelvp
 */

#include "micro_fsm.h"
#include "FreeRTOS.h"
#include "atcom.h"
#include "main.h"
#include "pids.h"
#include "copert.h"
#include <string.h>
#include "printf.h"

#if DEBUG
#define TIMER_STN	60	//segundos
#else
#define TIMER_STN	5
#endif
#define FIRST_TIMER_GNSS	5	//segundos
#define TIMER_GNSS			3	//segundos
#define TIMER_NB			60	//segundos

// Conversion de segundos a milisegundos
#define SEC_TO_MILL			1000

// Funciones de comprobación
static uint8_t alwaysRead (fsm_t *this);
static uint8_t setupState (fsm_t *this);
static uint8_t respondSTN (fsm_t *this);
static uint8_t newDataSTN (fsm_t *this);
static uint8_t respondNB (fsm_t *this);
static uint8_t newDataNB (fsm_t *this);
static uint8_t setupFinished (fsm_t *this);
static uint8_t tryAgain (fsm_t *this);
static uint8_t dataRead (fsm_t *this);
static uint8_t stnMssg (fsm_t *this);
static uint8_t gnssMssg (fsm_t *this);
static uint8_t nbMssg (fsm_t *this);
static uint8_t timeout (fsm_t *this);

// Funciones de transición
static void read (fsm_t *this);
static void sendSTN (fsm_t *this);
static void sendGNSS (fsm_t *this);
static void setupGNSS (fsm_t *this);
static void sendNB (fsm_t *this);
static void translateOBD (fsm_t *this);
static void back (fsm_t *this);
static void resetSTN (fsm_t *this);
static void translateNB (fsm_t *this);
static void resetNB (fsm_t *this);
static void setupSTN (fsm_t *this);
static void setupNB (fsm_t *this);
static void successConnection (fsm_t *this);
static void checkConexion (fsm_t *this);
static void clearNewSTN (fsm_t *this);
static void clearSetupSTN (fsm_t *this);
static void clearNewNB (fsm_t *this);
static void clearSetupNB (fsm_t *this);
static void clearAll (fsm_t *this);

// Funciones auxiliares
static uint32_t decodeNumber (uint8_t *data, uint8_t len);
static void setPosition (car *coche);

// Mensajes de debug
#if DEBUG
static const uint8_t mssg[9][22] = {
		// USB messages
		{"STN communication\0"},
		{"STN timeout. Reset\r\0"},
		{"GNSS communication\0"},
		{"GNSS timeout. Reset\r\0"},
		{"NB communication\0"},
		{"NB timeout. Reset\r\0"},
		{"Toggle Test:\0"},
		{"Orden no valida\r\0"},

		{"error\0"}
};
#endif

// Estados de la máquina
static enum uCstates {
	PREV,
	SETUP_STN,
	SETUP_GNSS,
	SETUP_NB,
	CONEXION,
	SOCKET,
	IDLE,
	COM_STN,
	COM_GNSS,
	COM_NB,
};

// Tabla de transiciones
static fsm_trans_t uC_tt[] = {
	{PREV,			alwaysRead,		SETUP_STN,		setupSTN},
	{SETUP_STN,		setupState,		SETUP_GNSS,		setupGNSS},
	{SETUP_STN,		respondSTN,		SETUP_STN,		clearSetupSTN},
	{SETUP_STN,		newDataSTN,		SETUP_STN,		setupSTN},
	{SETUP_GNSS,	setupState,		SETUP_NB,		setupNB},
	{SETUP_GNSS,	timeout,		SETUP_GNSS,		setupGNSS},
	{SETUP_NB,		respondNB,		SETUP_NB,		clearSetupNB},
	{SETUP_NB,		newDataNB,		SETUP_NB,		clearNewNB},
	{SETUP_NB,		timeout,		CONEXION,		checkConexion},
	{CONEXION,		setupFinished,	SOCKET,			clearAll},
	{CONEXION,		tryAgain,		SETUP_NB,		setupNB},
	{CONEXION,		respondNB,		CONEXION,		successConnection},
	{SOCKET,		respondNB,		SOCKET,			clearSetupNB},
	{SOCKET,		newDataNB,		IDLE,			NULL},
	{IDLE,			dataRead,		IDLE,			read},
	{IDLE,			stnMssg,		COM_STN,		sendSTN},
	{IDLE,			gnssMssg,		COM_GNSS,		sendGNSS},
	{IDLE,			nbMssg,			COM_NB, 		sendNB},
	{COM_STN, 		respondSTN, 	COM_STN, 		translateOBD},
	{COM_STN, 		newDataSTN, 	IDLE, 			back},
	{COM_STN,		timeout,		IDLE,			resetSTN},
	{COM_GNSS,		timeout,		IDLE,			NULL},
	{COM_NB,		respondSTN, 	IDLE, 			translateNB},
	{COM_NB,		timeout,		IDLE,			resetNB},
	{-1, NULL, -1, NULL},
};

/*
 * @brief 	Inicialización de la máquina de estados
 * @param 	flags: puntero a la variable compartida de la señalización por flags
 * @retval 	Máquina de estados
 */
fsm_t* init_micro(car *coche)
{
	fsm_t *fsm;
	fsm = fsm_new(uC_tt, coche);

#if USE_STN
	initSTNCom();
	HAL_GPIO_WritePin(STN_RST_GPIO_Port, STN_RST_Pin, 0);
#endif

#if USE_GNSS
	initGNSSCom();
#endif

#if USE_NB
	initNBCom();
#endif

	return fsm;
}

/*
 * @brief	Siempre ejecuta al próximo estado
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Siempre ejecuta
 */
static uint8_t alwaysRead (fsm_t *this)
{
	return 1;
}

/*
 * @brief	Comprueba si ha terminado de completarse la configuración
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> La configuración ha terminado
 * 			 0 -> La configuración no ha terminado
 */
static uint8_t setupState (fsm_t *this)
{
	return !(((car*)(this->data))->setupState);
}

/*
 * @brief	Comprueba si el módulo de STN ha respondido
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> El módulo STN tiene una nueva respuesta
 * 			 0 -> El módulo se mantiene
 */
static uint8_t respondSTN (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & RESPOND_STN);
}


/*
 * @brief	Comprueba si el módulo de STN solicita nuevos datos
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> El módulo STN espera nuevos datos
 * 			 0 -> El módulo se mantiene commo estaba
 */
static uint8_t newDataSTN (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & NEW_DATA_STN);
}

/*
 * @brief	Comprueba si el módulo de NB-IoT ha respondido
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> El módulo NB-IoT tiene una nueva respuesta
 * 			 0 -> El módulo se mantiene
 */
static uint8_t respondNB (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & RESPOND_NB) != 0;
}

/*
 * @brief	Comprueba si el módulo de NB-IoT solicita nuevos datos
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> El módulo NB-IoT espera nuevos datos
 * 			 0 -> El módulo se mantiene commo estaba
 */
static uint8_t newDataNB (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & NEW_DATA_NB) != 0;
}

/*
 * @brief	Comprueba si el módulo NB-IoT se ha configurado correctamente con el socket creado
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> La configuración ha sido un éxito
 * 			 0 -> La configuración está indeterminada
 */
static uint8_t setupFinished (fsm_t *this)
{
	return ((*(((car*)(this->data))->communication->flags) & RESPOND_NB) != 0) && ((*(((car*)(this->data))->communication->flags) & TEST_MSSG) != 0);
}

/*
 * @brief	Comprueba si el módulo NB-IoT se ha configurado correctamente con el socket creado
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> La configuración ha fracasado
 * 			 0 -> La configuración está indeterminada
 */
static uint8_t tryAgain (fsm_t *this)
{
	return ((*(((car*)(this->data))->communication->flags) & RESPOND_NB) != 0) && ((*(((car*)(this->data))->communication->flags) & NB_MSSG) != 0);
}

/*
 * @brief	Comprobación de si hay datos recibidos no leídos
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos para leer
 * 			 0 -> No hay datos para leer
 */
static uint8_t dataRead (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & B_NOT_READ);
}

/*
 * @brief	Comprobación de si hay un mensaje para el STN
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos para transmitir al módulo
 * 			 0 -> No hay datos para transmitir al módulo
 */
static uint8_t stnMssg (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & STN_MSSG);
}

/*
 * @brief	Comprobación de si hay un mensaje para el GNSS
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos para transmitir al módulo
 * 			 0 -> No hay datos para transmitir al módulo
 */
static uint8_t gnssMssg (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & GNSS_MSSG);
}

/*
 * @brief	Comprobación de si hay un mensaje para el NB-IoT
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos para transmitir al módulo
 * 			 0 -> No hay datos para transmitir al módulo
 */
static uint8_t nbMssg (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & NB_MSSG);
}

/*
 * @brief	Comprobación de si ha saltado el timeout
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Ha saltado el timeout
 * 			 0 -> No ha saltado el timeout
 */
static uint8_t timeout (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & TIMEOUT);
}

/*
 * @brief	Lee los datos recibidos que hay en el buffer de recepción
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void read (fsm_t *this)
{
	uint8_t resp[32];
	uint16_t tipo;
	uint16_t *flags = (((car*)(this->data))->communication->flags);

	// Comprobamos los mensajes recibidos
	lockRX();
	if (lenFirstMssgRX() == notReadRX())
		*flags = *flags & ~B_NOT_READ;
	tipo = typeNextMssgRX();
	unlockRX();

	// Modificamos los flags según el tipo de comando
	lockTX();
	switch (tipo) {

	case STN_MSSG:
#if DEBUG
		sprintf_((char*) resp, "%s\r", mssg[0]);
		putTX(resp, strlen((char*) resp));
#endif
		*flags = *flags | STN_MSSG;
		break;

	case GNSS_MSSG:
#if DEBUG
		sprintf_((char*) resp, "%s\r", mssg[2]);
		putTX(resp, strlen((char*) resp));
#endif
		*flags = *flags | GNSS_MSSG;
		break;

	case NB_MSSG:
#if DEBUG
		sprintf_((char*) resp, "%s\r", mssg[4]);
		putTX(resp, strlen((char*) resp));
#endif
		*flags = *flags | NB_MSSG;
		break;

	case TEST_MSSG:
#if DEBUG
		sprintf_((char*) resp, "%s\r", mssg[6]);
		putTX(resp, strlen((char*) resp));
#endif
		if (*flags & TEST_MSSG)
			*flags = *flags & ~TEST_MSSG;
		else
			*flags = *flags | TEST_MSSG;
		jumpMssgRX();
		break;

	// Devuelve los datos recogidos y almacenados
	case TX_DATA:
		jumpMssgRX();
		sprintf_((char*) resp, "%s,%3u,%4u,%3d\r",
				((car*)(this->data))->vin,
				((car*)(this->data))->speed,
				((car*)(this->data))->rpm,
				((car*)(this->data))->air);

		putTX(resp, strlen((char*) resp));
		*flags = *flags | TX_DATA;
		break;

	// Establecemos los parámetros de COPERT según la norma Euro
	case SETUP_MSSG:
		getRX(resp, 7);
		if (resp[4] == '3')
			tipo = EURO3_GAS;
		else if (resp[4] == '6')
			tipo = EURO6_DIESEL_2016;
		setParams((car*)(this->data), (uint8_t) tipo);
		break;

	default:
#if DEBUG
		sprintf_((char*) resp, "%s\r", mssg[7]);
		putTX(resp, strlen((char*) resp));
#endif
		break;
	}

#if DEBUG
	*flags = *flags | TX_DATA;
#endif
	unlockTX();
}

/*
 * @brief	Envía datos al STN que haya en el buffer de recepción
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void sendSTN (fsm_t *this)
{
	uint16_t len;
	uint8_t *mens;
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~STN_MSSG;

	// Recogemos el mensaje y lo transmitimos como comando al módulo STN
	lockRX();
	len = lenFirstMssgRX();
	mens = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*len);
	getRX(mens, len);
	unlockRX();

	STN_sendCMD(mens);
	vPortFree(mens);
	if (!launchTimer(TIMER_STN*SEC_TO_MILL)){
		while(1){}
	}

}

/*
 * @brief	Envía datos al GNSS que haya en el buffer de recepción
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void sendGNSS (fsm_t *this)
{
	uint16_t len;
	uint8_t *mens;
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~GNSS_MSSG;

	// Recogemos el mensaje y lo transmitimos como comando al módulo GNSS
	lockRX();
	len = lenFirstMssgRX();
	mens = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*len);
	getRX(mens, len);
	unlockRX();

	GNSS_sendCMD(mens);
	vPortFree(mens);
	if (!launchTimer(1*SEC_TO_MILL)){
		while(1){}
	}
}

/*
 * @brief	Envía datos al NB-IoT que haya en el buffer de recepción
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void sendNB (fsm_t *this)
{
	uint16_t len;
	uint8_t *mens;
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~NB_MSSG;

	// Recogemos el mensaje y lo transmitimos como comando al módulo NB-IoT
	lockRX();
	len = lenFirstMssgRX();
	mens = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*len);
	getRX(mens, len);
	unlockRX();

	NB_sendCMD(mens);
	vPortFree(mens);
	if (!launchTimer(TIMER_NB*SEC_TO_MILL)){
		while(1){}
	}
}

/*
 * @brief	Traduce el mensaje que se recibe por parte del módulo STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void translateOBD (fsm_t *this)
{
	command lastCom;
	uint8_t i, j, nLin;
	uint8_t data[30], dev[20];
	uint16_t head, pos;
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~RESPOND_STN;
	STN_getLastCommand(&lastCom);
	i = 0;

	// Recogemos los datos almacenados en el buffer asociado a la UART1
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	pos = ((car*)this->data)->communication->pileUART1->tail;
	head = ((car*)this->data)->communication->pileUART1->head;
	do {
		data[i] = ((car*)this->data)->communication->pileUART1->buffer[pos];
		i++;
		pos = (pos + 1) % ((car*)this->data)->communication->pileUART1->size;
	} while (pos != head);
	((car*)this->data)->communication->pileUART1->tail = pos;
	osMutexRelease(((car*)this->data)->communication->pileLock);

	// Según el tipo de comando que mandamos, decodificamos de una forma u otra
	switch (lastCom) {

	// Numero de bastidor: Transformación al abecedario
	case STN_GET_VIN:
		if (data[0] == '7' && data[1] == 'E') {
		switch (data[5]) {
		// Línea 1
		case '0':
			for (i = VIN_INIT_NUMBER; i < VIN_MAX_NUMBER; i += NEXT_NUMBER) {
				((car*)(this->data))->vin
						[(i-VIN_INIT_NUMBER)/NEXT_NUMBER] = 0;
				if (data[i] >= ASCII_LETTER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_INIT_NUMBER)/NEXT_NUMBER] += (((data[i]-ASCII_LETTER_THRESHOLD)+10) << 4);	// Valores en hexadecimal A-F
				} else if (data[i] >= ASCII_NUMBER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_INIT_NUMBER)/NEXT_NUMBER] += ((data[i]-ASCII_NUMBER_THRESHOLD) << 4);	// Valores en hexadecimal 0-9
				}
				if (data[i+1] >= ASCII_LETTER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_INIT_NUMBER)/NEXT_NUMBER] += ((data[i+1]-ASCII_LETTER_THRESHOLD)+10);	// Valores en hexadecimal A-F
				} else if (data[i+1] >= ASCII_NUMBER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_INIT_NUMBER)/NEXT_NUMBER] += (data[i+1]-ASCII_NUMBER_THRESHOLD);	// Valores en hexadecimal 0-9
				}
			}

			break;
		// Líneas 2 y 3
		case '1':
		case '2':
			nLin = data[5] - ASCII_NUMBER_THRESHOLD;
			for (i = VIN_NEXT_LINE; i < VIN_MAX_NUMBER; i += NEXT_NUMBER) {
				((car*)(this->data))->vin
						[(i-VIN_NEXT_LINE)/NEXT_NUMBER + (VIN_MAX_NUMBER-VIN_INIT_NUMBER)/NEXT_NUMBER + ((VIN_MAX_NUMBER-VIN_NEXT_LINE)/NEXT_NUMBER*(nLin-1))] = 0;
				if (data[i] >= ASCII_LETTER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_NEXT_LINE)/NEXT_NUMBER + (VIN_MAX_NUMBER-VIN_INIT_NUMBER)/NEXT_NUMBER + ((VIN_MAX_NUMBER-VIN_NEXT_LINE)/NEXT_NUMBER*(nLin-1))] += (((data[i]-ASCII_LETTER_THRESHOLD)+10) << 4);	// Valores en hexadecimal A-F
				} else if (data[i] >= ASCII_NUMBER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_NEXT_LINE)/NEXT_NUMBER + (VIN_MAX_NUMBER-VIN_INIT_NUMBER)/NEXT_NUMBER + ((VIN_MAX_NUMBER-VIN_NEXT_LINE)/NEXT_NUMBER*(nLin-1))] += ((data[i]-ASCII_NUMBER_THRESHOLD) << 4);	// Valores en hexadecimal 0-9
				}
				if (data[i+1] >= ASCII_LETTER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_NEXT_LINE)/NEXT_NUMBER + (VIN_MAX_NUMBER-VIN_INIT_NUMBER)/NEXT_NUMBER + ((VIN_MAX_NUMBER-VIN_NEXT_LINE)/NEXT_NUMBER*(nLin-1))] += ((data[i+1]-ASCII_LETTER_THRESHOLD)+10);	// Valores en hexadecimal A-F
				} else if (data[i+1] >= ASCII_NUMBER_THRESHOLD) {
					((car*)(this->data))->vin
							[(i-VIN_NEXT_LINE)/NEXT_NUMBER + (VIN_MAX_NUMBER-VIN_INIT_NUMBER)/NEXT_NUMBER + ((VIN_MAX_NUMBER-VIN_NEXT_LINE)/NEXT_NUMBER*(nLin-1))] += (data[i+1]-ASCII_NUMBER_THRESHOLD);	// Valores en hexadecimal 0-9
				}
			}
#if DEBUG || TEST
			if (nLin == 2) {
				((car*)(this->data))->vin[VIN_LENGTH] = '\0';
				sprintf_((char*) data, "%s\r", ((car*)(this->data))->vin);
				putTX(data, strlen((char*) data));
				*flags = *flags | TX_DATA;
			}
#endif
			break;
		}
		}
		break;

	// Revoluciones por minuto: Pasar a número y aplicar corrección
	case STN_GET_RPM:
		if (data[0] == '7' && data[1] == 'E') {
		((car*)(this->data))->rpm[0] = 0;
		((car*)(this->data))->rpm[0] = decodeNumber(&(data[PAYLOAD]), data[NUM_BYTES] - ASCII_NUMBER_THRESHOLD - LEN_HEADER_OBD);
		((car*)(this->data))->rpm[0] /= CORRECCION_RPM;
#if DEBUG || TEST
		sprintf_((char*) dev, "%d\r", ((car*)(this->data))->rpm[0]);
		putTX(dev, strlen((char*) dev));
		*flags = *flags | TX_DATA;
#endif
		}
		break;

	// Velocidad: Pasar a número
	case STN_GET_SPEED:
		if (data[0] == '7' && data[1] == 'E') {
		((car*)(this->data))->speed[0] = 0;
		((car*)(this->data))->speed[0] = decodeNumber(&(data[PAYLOAD]), data[NUM_BYTES] - ASCII_NUMBER_THRESHOLD - LEN_HEADER_OBD);	// Nunca es mayor a 8 (al menos en la velocidad)

#if DEBUG || TEST
		setPosition((car*)(this->data));
		sprintf_((char*) dev, "%d\r", ((car*)(this->data))->speed[0]);
		putTX(dev, strlen((char*) dev));
		*flags = *flags | TX_DATA;
#endif
		}
		break;

	// Tipo de combustible: TODO con la tabla de conversiones dado por los PIDs
	case STN_GET_FUEL:
		((car*)(this->data))->fuel = NONE;
		((car*)(this->data))->fuel = decodeNumber(&(data[PAYLOAD]), data[NUM_BYTES] - ASCII_NUMBER_THRESHOLD - LEN_HEADER_OBD);
		break;

	// Temperatura del aire ambiente: Pasar a número y aplicar corrección
	case STN_GET_AIR_TEMPERATURE:
		if (data[0] == '7' && data[1] == 'E') {
		((car*)(this->data))->air[0] = 0;
		((car*)(this->data))->air[0] = decodeNumber(&(data[PAYLOAD]), data[NUM_BYTES] - ASCII_NUMBER_THRESHOLD - LEN_HEADER_OBD);
		((car*)(this->data))->air[0] += CORRECCION_AIR;
#if DEBUG || TEST
		sprintf_((char*) dev, "%3d\r", ((car*)(this->data))->air[0]);
		putTX(dev, strlen((char*) dev));
		*flags = *flags | TX_DATA;
#endif
		}
		break;

	// Principalmente usado para el test
	case STN_USER_OBD:
		if (data[0] == '7' && data[1] == 'E') {
		if (*flags & TEST_MSSG) {
#if TEST
			i = 0;
			j = ((car*)(this->data))->times;
			pos = PAYLOAD;
#if SPEED_TEST
			((car*)(this->data))->speed[j] = 0;
			((car*)(this->data))->speed[j] = decodeNumber(&(data[pos]), 1);
			calcCO(((car*)(this->data)), SEND_PERIOD/NUM_VAL_CALC);
			calcNOx(((car*)(this->data)), SEND_PERIOD/NUM_VAL_CALC);
			calcPM(((car*)(this->data)), SEND_PERIOD/NUM_VAL_CALC);
			pos += 6;		// "0D XX "
#endif
#if RPM_TEST
			((car*)(this->data))->rpm[j] = 0;
			((car*)(this->data))->rpm[j] = decodeNumber(&(data[pos]), 2);
			((car*)(this->data))->rpm[j] /= CORRECCION_RPM;
			pos += 9;
#endif
			if (j == NUM_VAL_CALC-1) {
				setPosition((car*)(this->data));
				lockTX();
				sendMssg(((car*)(this->data)));
				unlockTX();
				((car*)(this->data))->times = 0;
				((car*)(this->data))->co = 0;
				((car*)(this->data))->nox = 0;
				((car*)(this->data))->pm = 0;
			} else
				((car*)(this->data))->times++;
#endif
			} else {
				data[i] = '\0';
				putTX(data, i+1);
				*flags = *flags | TX_DATA;
			}
		*flags = *flags | TX_DATA;
		}
		break;
	default:
#if DEBUG || TEST
		data[i] = '\0';
		putTX(data, i+1);
		*flags = *flags | TX_DATA;
#endif
		break;

	}

}


/*
 * @brief	Retorno de la solicitud de un nuevo comando por parte del módulo STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void back (fsm_t *this)
{
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~NEW_DATA_STN;
	stopTimer();
	*flags = *flags & ~TIMEOUT;
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART1->tail = ((car*)this->data)->communication->pileUART1->head;
	osMutexRelease(((car*)this->data)->communication->pileLock);
}

/*
 * @brief	Reinicia el STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void resetSTN (fsm_t *this)
{
#if DEBUG
	uint8_t resp[21];
#endif
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~TIMEOUT;
#if DEBUG
	sprintf_((char*) resp, "%s\r", mssg[1]);
	putTX(resp, strlen((char*) resp));
	*flags = *flags | TX_DATA;
#endif
}

/*
 * @brief	Traduce el mensaje que se recibe por parte del NB-IoT (Hecho en otro puntos del código en estos momentos).
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void translateNB (fsm_t *this)
{
	return;
}

/*
 * @brief	Reinicia el NB (Hecho en otro puntos del código en estos momentos).
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void resetNB (fsm_t *this)
{
	return;
}

/*
 * @brief	Realiza la configuración del módulo STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void setupSTN (fsm_t *this)
{

	uint8_t mssg[14];
	uint8_t state = (((car*)(this->data))->setupState);
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~(NEW_DATA_STN | RESPOND_STN);
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART1->tail = ((car*)this->data)->communication->pileUART1->head;
	osMutexRelease(((car*)this->data)->communication->pileLock);
	if (state == 4) {
		HAL_GPIO_WritePin(STN_RST_GPIO_Port, STN_RST_Pin, 1);
	} else if (state == 3) {
		sprintf_((char*) mssg, "stn echo 0\r");
		STN_sendCMD(mssg);
	} else if (state == 2) {
		sprintf_((char*) mssg, "stn header 1\r");
		STN_sendCMD(mssg);
		enciendeLED(AZUL);
	}
	(((car*)(this->data))->setupState)--;
}

/*
 * @brief	Realiza la configuración del módulo GNSS
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void setupGNSS (fsm_t *this)
{
	uint8_t mssg[10];
	uint8_t state = (((car*)(this->data))->setupState);
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~TIMEOUT;
	if (!state) {
		(((car*)(this->data))->setupState) = 4;
		state = 4;
	}
	stopTimer();
	(state % 2)? apagaLED(AZUL) : enciendeLED(AZUL) ;
	if (state == 4) {
		launchTimer(FIRST_TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 3) {
		sprintf_((char*) mssg, "gnss start\r");
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 2) {
		sprintf_((char*) mssg, "gnss multiple\r");
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 1) {
		sprintf_((char*) mssg, "gnss static\r");
		GNSS_sendCMD(mssg);
	}
	(((car*)(this->data))->setupState)--;
}

/*
 * @brief	Realiza la configuración del módulo NB-IoT
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void setupNB (fsm_t *this)
{
	uint8_t mssg[12];
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~(TIMEOUT | NB_MSSG | RESPOND_STN | RESPOND_NB | TEST_MSSG | NEW_DATA_STN | NEW_DATA_NB);
	((car*)(this->data))->setupState = 5;
	enciendeLED(AZUL);
	sprintf_((char*) mssg, "nb connect\r");
	stopTimer();
	launchTimer(TIMER_NB*SEC_TO_MILL);
	NB_sendCMD(mssg);
}

/*
 * @brief	Continua el almacenamiento en el buffer asociado al UART1 que corresponde al módulo STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void clearSetupSTN (fsm_t *this)
{
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~RESPOND_STN;
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART1->tail = ((((car*)this->data)->communication->pileUART1->tail+1) % ((car*)this->data)->communication->pileUART1->size);
	osMutexRelease(((car*)this->data)->communication->pileLock);
}

/*
 * @brief	Continua el almacenamiento en el buffer asociado al UART3 que corresponde al módulo NB-IoT
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void clearSetupNB (fsm_t *this)
{
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~RESPOND_NB;
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART3->tail = ((((car*)this->data)->communication->pileUART3->tail+1) % ((car*)this->data)->communication->pileUART3->size);
	osMutexRelease(((car*)this->data)->communication->pileLock);
}

/*
 * @brief	Comprueba si se ha creado correctamente el socket con el servidor
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void successConnection (fsm_t *this)
{
	uint8_t i, data;
	uint16_t head, pos;
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	i = 0;
	data = '\"';
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	pos = ((car*)this->data)->communication->pileUART3->tail;
	head = ((car*)this->data)->communication->pileUART3->head;
	do {
		if (i == 19)
			data = ((car*)this->data)->communication->pileUART3->buffer[pos];
		i++;
		pos = (pos + 1) % ((car*)this->data)->communication->pileUART3->size;
	} while (pos != head);
	((car*)this->data)->communication->pileUART3->tail = pos;
	osMutexRelease(((car*)this->data)->communication->pileLock);
	if(data != '\"') {
		apagaLED(AZUL);
		*flags = *flags | TEST_MSSG;
	} else {
		*flags = *flags | NB_MSSG;
	}
	((car*)(this->data))->setupState = 5;
}

/*
 * @brief	Comprueba si se ha realizado correctamente la conexión a la celda
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void checkConexion (fsm_t *this)
{
	uint8_t mssg[10];
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART3->tail = ((car*)this->data)->communication->pileUART3->head;
	*flags = *flags & ~(TIMEOUT | RESPOND_STN);
	osMutexRelease(((car*)this->data)->communication->pileLock);
	sprintf_((char*) mssg, "nb check\r");
	NB_sendCMD(mssg);
}

/*
 * @brief	Limpia el buffer de recepción del módulo STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void clearNewSTN (fsm_t *this)
{
	uint8_t mssg[14];
	uint8_t state = (((car*)(this->data))->setupState);
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~(NEW_DATA_STN | RESPOND_STN);
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART1->tail = ((car*)this->data)->communication->pileUART1->head;
	osMutexRelease(((car*)this->data)->communication->pileLock);
	if (state == 3) {
		sprintf_((char*) mssg, "stn echo 0\r");
		STN_sendCMD(mssg);
	} else if (state == 2) {
		sprintf_((char*) mssg, "stn header 1\r");
		STN_sendCMD(mssg);
	}
	(((car*)(this->data))->setupState)--;
}

/*
 * @brief	Limpia el buffer de recepción del módulo NB-IoT
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void clearNewNB (fsm_t *this)
{
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~NEW_DATA_NB;
	*flags = *flags & ~RESPOND_NB;
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART3->tail = ((car*)this->data)->communication->pileUART3->head;
	osMutexRelease(((car*)this->data)->communication->pileLock);
	(((car*)(this->data))->setupState)--;
}

/*
 * @brief	Limpia los buffer de recepción de los módulos STN y NB-IoT
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void clearAll (fsm_t *this)
{
	uint8_t mssg[11];
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = 0;
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART1->tail = ((car*)this->data)->communication->pileUART1->head;
	((car*)this->data)->communication->pileUART3->tail = ((car*)this->data)->communication->pileUART3->head;
	osMutexRelease(((car*)this->data)->communication->pileLock);
	sprintf_((char*) mssg, "nb create\r");
	NB_sendCMD(mssg);
}

/*
 * @brief	Realiza un toggle del LED indicado
 * @param	color: color del LED a cambiar
 * @retval	Nada
 */
void cambiaLED(leds color)
{
	HAL_GPIO_TogglePin((color==VERDE)?USER_LED_1_GPIO_Port:USER_LED_2_GPIO_Port,(color==VERDE)?USER_LED_1_Pin:USER_LED_2_Pin);
}

/*
 * @brief	Enciende el LED indicado
 * @param	color: color del LED a encender
 * @retval	Nada
 */
void enciendeLED(leds color)
{
	HAL_GPIO_WritePin((color==VERDE)?USER_LED_1_GPIO_Port:USER_LED_2_GPIO_Port,(color==VERDE)?USER_LED_1_Pin:USER_LED_2_Pin, 1);
}

/*
 * @brief	Apaga el LED indicado
 * @param	color: color del LED a apagar
 * @retval	Nada
 */
void apagaLED(leds color)
{
	HAL_GPIO_WritePin((color==VERDE)?USER_LED_1_GPIO_Port:USER_LED_2_GPIO_Port,(color==VERDE)?USER_LED_1_Pin:USER_LED_2_Pin, 0);
}

/*
 * @brief	Pasa el string a los valores numéricos con formato de byte [NUM_1_ASCII][NUM_2_ASCII][Espacio]
 * @param	data: string a convertir
 * 			bytes: número de bytes contenidos en el string
 * @retval	Número equivalente al string pasado
 */
static uint32_t decodeNumber(uint8_t *data, uint8_t bytes)
{
	uint32_t dev = 0;
	uint8_t i = 0;

	for (i = 0; i < bytes*NEXT_NUMBER; i += NEXT_NUMBER) {
		if (data[i] >= ASCII_LETTER_THRESHOLD) {
			dev = dev << 4;
			dev = dev + ((data[i] - ASCII_LETTER_THRESHOLD)+10);
		} else if (data[i] >= ASCII_NUMBER_THRESHOLD) {
			dev = dev << 4;
			dev = dev + (data[i] - ASCII_NUMBER_THRESHOLD);
		}
		if (data[i+1] >= ASCII_LETTER_THRESHOLD) {
			dev = dev << 4;
			dev = dev + ((data[i+1] - ASCII_LETTER_THRESHOLD)+10);
		} else if (data[i+1] >= ASCII_NUMBER_THRESHOLD) {
			dev = dev << 4;
			dev = dev + (data[i+1] - ASCII_NUMBER_THRESHOLD);
		}
	}

	return dev;
}

/*
 * @brief	Determina y almacena las posiciones recogidas por geolocalización
 * @param	car: coche a posicionar
 * @retval	Nada
 */
static void setPosition (car *coche)
{
	uint8_t pos;
	uint8_t *coor;
	float decAux;
	if (osMutexWait(coche->communication->pileLock, 2) == osOK) {
		coor = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*13);

		// Set Latitude
		getLatitud(coor);
		coche->lastLat = 0;
		// Grados dd
		for (pos = 0; pos < 2; pos++) {
			coche->lastLat = coche->lastLat*10;
			coche->lastLat += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}

		// Minutos a grados en formato mm.mmmm
		decAux = 0;
		for (pos = 2; pos < 4; pos++) {
			decAux = decAux*10;
			decAux += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}

		for (pos = 5; pos < 9; pos++) {
			decAux = decAux*10;
			decAux += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}
		coche->lastLat = coche->lastLat + decAux/600000;

		// Comprobar la polaridad
		if (coor[10] == 'S')
			coche->lastLat = coche->lastLat * (-1);

		// Set Longitude
		getLongitud(coor);
		coche->lastLong = 0;
		// Grados ddd
		for (pos = 0; pos < 3; pos++) {
			coche->lastLong = coche->lastLong*10;
			coche->lastLong += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}

		// Minutos a grados en formato mm.mmmm
		decAux = 0;
		for (pos = 3; pos < 5; pos++) {
			decAux = decAux*10;
			decAux += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}

		for (pos = 6; pos < 10; pos++) {
			decAux = decAux*10;
			decAux += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}
		coche->lastLong = coche->lastLong + decAux/600000;

		//Comprobar la polaridad
		if (coor[11] == 'W')
			coche->lastLong = coche->lastLong * (-1);

		vPortFree(coor);

		osMutexRelease(coche->communication->pileLock);
	}

}
