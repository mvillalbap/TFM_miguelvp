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
#define TIMER_GNSS	2	//segundos
#define TIMER_NB	60	//segundos

#define SEC_TO_MILL			1000

#define CORRECCION_RPM		4
#define CORRECCION_AIR		-40


// Copia del string
static void str_cpy(uint8_t *str1, const uint8_t *str2);

// Funciones de comprobación
static uint8_t alwaysRead (fsm_t *this);
static uint8_t setupDoneSTN (fsm_t *this);
static uint8_t setupState (fsm_t *this);
static uint8_t setupDoneNB (fsm_t *this);
static uint8_t setupNewNB (fsm_t *this);
static uint8_t setupFinished (fsm_t *this);
static uint8_t tryAgain (fsm_t *this);
static uint8_t dataRead (fsm_t *this);
static uint8_t stnMssg (fsm_t *this);
static uint8_t gnssMssg (fsm_t *this);
static uint8_t nbMssg (fsm_t *this);
static uint8_t timeout (fsm_t *this);
static uint8_t respond (fsm_t *this);
static uint8_t newData (fsm_t *this);

// Funciones de transición
static void read (fsm_t *this);
static void sendSTN (fsm_t *this);
static void sendGNSS (fsm_t *this);
static void setupGNSS (fsm_t *this);
static void sendNB (fsm_t *this);
static void translateOBD (fsm_t *this);
static void back (fsm_t *this);
static void resetSTN (fsm_t *this);
static void translateGNSS (fsm_t *this);
static void resetGNSS (fsm_t *this);
static void translateNB (fsm_t *this);
static void resetNB (fsm_t *this);
#if !DEPLOYMENT
static void setupSTN (fsm_t *this);
#endif
static void setupNB (fsm_t *this);
static void successConnection (fsm_t *this);
static void checkConexion (fsm_t *this);
static void clearNewSTN (fsm_t *this);
static void clearSetupSTN (fsm_t *this);
static void clearNewNB (fsm_t *this);
static void clearSetupNB (fsm_t *this);
static void clearAll (fsm_t *this);

static uint32_t decodeNumber (uint8_t *data, uint8_t len);
static void setPosition (car *coche);

// Mensajes
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

// Estados de la máquina
static enum uCstates {
	PREV,
	SETUP_STN,
	SETUP_GNSS,
	SETUP_NB,
	CONEXION,
	SOCKET,
	IDLE,
	//INSTRUCTION,
	COM_STN,
	COM_GNSS,
	COM_NB,
};

// Tabla de transiciones
static fsm_trans_t uC_tt[] = {
#if !DEPLOYMENT
	{PREV,			alwaysRead,		SETUP_STN,		setupSTN},
#else
	{PREV,			alwaysRead,		SETUP_STN,		setupSTN},
#endif
	{SETUP_STN,		setupState,		SETUP_GNSS,		setupGNSS},
	{SETUP_STN,		setupDoneSTN,	SETUP_STN,		clearSetupSTN},
	{SETUP_STN,		newData,		SETUP_STN,		setupSTN},
	{SETUP_GNSS,	setupState,		SETUP_NB,		setupNB},
	{SETUP_GNSS,	timeout,		SETUP_GNSS,		setupGNSS},
	{SETUP_NB,		setupDoneNB,	SETUP_NB,		clearSetupNB},
	{SETUP_NB,		setupNewNB,		SETUP_NB,		clearNewNB},
	{SETUP_NB,		timeout,		CONEXION,		checkConexion},
	{CONEXION,		setupFinished,	SOCKET,			clearAll},
	{CONEXION,		tryAgain,		SETUP_NB,		setupNB},
	{CONEXION,		setupDoneNB,	CONEXION,		successConnection},
	{IDLE,			dataRead,		IDLE,			read},
	{SOCKET,		setupDoneNB,	SOCKET,			clearSetupNB},
	{SOCKET,		setupNewNB,		IDLE,			NULL},
	{SETUP_GNSS,	setupState,		IDLE,			clearAll},
	{IDLE,			stnMssg,		COM_STN,		sendSTN},
	{IDLE,			gnssMssg,		COM_GNSS,		sendGNSS},
	{IDLE,			nbMssg,			COM_NB, 		sendNB},
	//{INSTRUCTION,	alwaysRead,		IDLE,			NULL},
	{COM_STN, 		respond, 		COM_STN, 		translateOBD},
	{COM_STN, 		newData, 		IDLE, 			back},
	{COM_STN,		timeout,		IDLE,			resetSTN},
	{COM_GNSS,		timeout,		IDLE,			NULL},
	{COM_NB,		respond, 		IDLE, 			translateNB},
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
 * @brief	Siempre ejecuta al próximo estado
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Siempre ejecuta
 */
static uint8_t setupDoneSTN (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & RESPOND_STN);
}


/*
 * @brief	Siempre ejecuta al próximo estado
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Siempre ejecuta
 */
static uint8_t setupState (fsm_t *this)
{
	return !(((car*)(this->data))->setupState);
}

/*
 * @brief	Siempre ejecuta al próximo estado
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Siempre ejecuta
 */
static uint8_t setupDoneNB (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & RESPOND_NB) != 0;
}

/*
 * @brief	Siempre ejecuta al próximo estado
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Siempre ejecuta
 */
static uint8_t setupNewNB (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & NEW_DATA_NB) != 0;
}

/*
 * @brief	Siempre ejecuta al próximo estado
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Siempre ejecuta
 */
static uint8_t setupFinished (fsm_t *this)
{
	return ((*(((car*)(this->data))->communication->flags) & RESPOND_NB)) && ((*(((car*)(this->data))->communication->flags) & TEST_MSSG));
}

/*
 * @brief	Siempre ejecuta al próximo estado
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Siempre ejecuta
 */
static uint8_t tryAgain (fsm_t *this)
{
	return ((*(((car*)(this->data))->communication->flags) & RESPOND_NB)) && ((*(((car*)(this->data))->communication->flags) & NB_MSSG));
}

/*
 * @brief	Comprobación de si hay datos recibidos no leídos
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Hay datos para leer
 * 			0 -> No hay datos para leer
 */
static uint8_t dataRead (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & B_NOT_READ);
}

/*
 * @brief	Comprobación de si hay un mensaje para el STN
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Hay datos para transmitir al módulo
 * 			0 -> No hay datos para transmitir al módulo
 */
static uint8_t stnMssg (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & STN_MSSG);
}

/*
 * @brief	Comprobación de si hay un mensaje para el GNSS
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Hay datos para transmitir al módulo
 * 			0 -> No hay datos para transmitir al módulo
 */
static uint8_t gnssMssg (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & GNSS_MSSG);
}

/*
 * @brief	Comprobación de si hay un mensaje para el NB
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Hay datos para transmitir al módulo
 * 			0 -> No hay datos para transmitir al módulo
 */
static uint8_t nbMssg (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & NB_MSSG);
}

/*
 * @brief	Comprobación de si ha saltado el timeout
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Ha saltado el timeout
 * 			0 -> No ha saltado el timeout
 */
static uint8_t timeout (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & TIMEOUT);
}

/*
 * @brief	Comprobación de si el último módulo ha respondido
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Ha respondido el módulo
 * 			0 -> No ha respondido el módulo
 */
static uint8_t respond (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & RESPOND_STN);
}


/*
 * @brief	Comprobación de si el último módulo ha respondido
 * @param	this: máquina de estados a evaluar
 * @retval	1 -> Ha respondido el módulo
 * 			0 -> No ha respondido el módulo
 */
static uint8_t newData (fsm_t *this)
{
	return (*(((car*)(this->data))->communication->flags) & NEW_DATA_STN);
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

	lockRX();

	if (lenFirstMssgRX() == notReadRX())
		*flags = *flags & ~B_NOT_READ;

	tipo = typeNextMssgRX();
	unlockRX();

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
 * @brief	Envía datos al NB que haya en el buffer de recepción
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void sendNB (fsm_t *this)
{
	uint16_t len;
	uint8_t *mens;
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~NB_MSSG;

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
 * @brief	Traduce el mensaje que se recibe por parte del STN
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void translateOBD (fsm_t *this)
{
	command lastCom;
	float cont;
	uint8_t i, j, nLin;
	uint8_t data[30], dev[20];
	uint16_t head, pos;
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~RESPOND_STN;
	STN_getLastCommand(&lastCom);
	i = 0;
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
	switch (lastCom) {
	// Numero de bastidor
	case STN_GET_VIN:
		if (data[0] == '7' && data[1] == 'E') {
		switch (data[5]) {
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
	// Revoluciones por minuto
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
	// Velocidad
	case STN_GET_SPEED:
		if (data[0] == '7' && data[1] == 'E') {
		((car*)(this->data))->speed[0] = 0;
		((car*)(this->data))->speed[0] = decodeNumber(&(data[PAYLOAD]), data[NUM_BYTES] - ASCII_NUMBER_THRESHOLD - LEN_HEADER_OBD);	// Nunca es mayor a 8 (al menos en la velocidad)
#if TEST
		//cont = calcNOx((car*)(this->data), SEND_PERIOD/NUM_VAL_CALC);
#else
		cont = calcNOx((car*)(this->data), 500);
#endif
#if DEBUG || TEST
		setPosition((car*)(this->data));
		sprintf_((char*) dev, "%d\r", ((car*)(this->data))->speed[0]);
		putTX(dev, strlen((char*) dev));
		*flags = *flags | TX_DATA;
#endif
		}
		break;
	// Tipo de combustible
	case STN_GET_FUEL:
		((car*)(this->data))->fuel = NONE;
		((car*)(this->data))->fuel = decodeNumber(&(data[PAYLOAD]), data[NUM_BYTES] - ASCII_NUMBER_THRESHOLD - LEN_HEADER_OBD);
		break;
	// Temperatura del aire ambiente
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
	case STN_USER_OBD:
		if (data[0] == '7' && data[1] == 'E') {
		if (*flags & TEST_MSSG) {
			/*if (getPositionReady()) {
				osMutexWait(((car*)this->data)->communication->pileLock, 0);
				setPosition(((car*)(this->data)));
				osMutexRelease(((car*)this->data)->communication->pileLock);
			}*/
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
			/*
			data[i++] = (((car*)(this->data))->speed / 100)+ASCII_NUMBER_THRESHOLD;
			data[i++] = ((((car*)(this->data))->speed / 10)%10)+ASCII_NUMBER_THRESHOLD;
			data[i++] = (((car*)(this->data))->speed % 10)+ASCII_NUMBER_THRESHOLD;
			data[i++] = (uint8_t) ',';*/
			pos += 6;		// "0D XX "
#endif
#if RPM_TEST
			((car*)(this->data))->rpm[j] = 0;
			((car*)(this->data))->rpm[j] = decodeNumber(&(data[pos]), 2);
			((car*)(this->data))->rpm[j] /= CORRECCION_RPM;
			/*
			data[i++] = (((car*)(this->data))->rpm / 1000)+ASCII_NUMBER_THRESHOLD;
			data[i++] = ((((car*)(this->data))->rpm / 100)%10)+ASCII_NUMBER_THRESHOLD;
			data[i++] = ((((car*)(this->data))->rpm / 10)%10)+ASCII_NUMBER_THRESHOLD;
			data[i++] = (((car*)(this->data))->rpm %10)+ASCII_NUMBER_THRESHOLD;
			data[i++] = (uint8_t)',';*/
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
			//data[i++] = '\r';
			}
		//putTX(data, i);
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
//	uint8_t wait;
#if DEBUG
	uint8_t resp[21];
#endif
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~TIMEOUT;
/*	HAL_GPIO_WritePin(STN_RST_GPIO_Port,STN_RST_Pin, 0);

	// Espera de 2 us (HSE de 8 MHz) mínimo
	for (wait = 0; wait < 20; wait++){}

	HAL_GPIO_WritePin(STN_RST_GPIO_Port,STN_RST_Pin, 1);
	//HAL_UART_AbortReceive_IT();*/
#if DEBUG
	sprintf_((char*) resp, "%s\r", mssg[1]);
	putTX(resp, strlen((char*) resp));
	*flags = *flags | TX_DATA;
#endif
}

/*
 * @brief	Traduce el mensaje que se recibe por parte del GNSS
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void translateGNSS (fsm_t *this)
{

}

/*
 * @brief	Reinicia el GNSS
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void resetGNSS (fsm_t *this)
{

}

/*
 * @brief	Traduce el mensaje que se recibe por parte del NB
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void translateNB (fsm_t *this)
{

}

/*
 * @brief	Reinicia el NB
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void resetNB (fsm_t *this)
{

}

/*
 *
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
	}
	(((car*)(this->data))->setupState)--;
}

/*
 *
 */
static void setupGNSS (fsm_t *this)
{
	uint8_t mssg[10];
	uint8_t state = (((car*)(this->data))->setupState);
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~TIMEOUT;
	if (!state) {
		(((car*)(this->data))->setupState) = 6;
		state = 6;
	}
	stopTimer();
	if (state == 6) {
		sprintf_((char*) mssg, "gnss static\r");
		//HAL_GPIO_WritePin(NB_RST_GPIO_Port, NB_RST_Pin, 1);
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 5) {
		sprintf_((char*) mssg, "gnss static\r");
		//HAL_GPIO_WritePin(NB_RST_GPIO_Port, NB_RST_Pin, 1);
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 4) {
		sprintf_((char*) mssg, "gnss multiple\r");
		//HAL_GPIO_WritePin(NB_RST_GPIO_Port, NB_RST_Pin, 1);
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 3) {
		sprintf_((char*) mssg, "gnss multiple\r");
		//HAL_GPIO_WritePin(NB_RST_GPIO_Port, NB_RST_Pin, 1);
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 2) {
		sprintf_((char*) mssg, "gnss start\r");
		//HAL_GPIO_WritePin(NB_RST_GPIO_Port, NB_RST_Pin, 1);
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	} else if (state == 1) {
		sprintf_((char*) mssg, "gnss start\r");
		//HAL_GPIO_WritePin(NB_RST_GPIO_Port, NB_RST_Pin, 0);
		GNSS_sendCMD(mssg);
		launchTimer(TIMER_GNSS*SEC_TO_MILL);
	/*} else if (state == 3) {
		sprintf_((char*) mssg, "gnss gsa\r");
		GNSS_sendCMD(mssg);
	} else if (state == 2) {
		sprintf_((char*) mssg, "gnss gsv\r");
		GNSS_sendCMD(mssg);
	} else if (state == 1) {
		sprintf_((char*) mssg, "gnss stop\r");
		GNSS_sendCMD(mssg);*/
	}
	(((car*)(this->data))->setupState)--;
}

/*
 *
 */
static void setupNB (fsm_t *this)
{
	uint8_t mssg[12];
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~(TIMEOUT | NB_MSSG | RESPOND_STN | RESPOND_NB | TEST_MSSG | NEW_DATA_STN | NEW_DATA_NB);
	((car*)(this->data))->setupState = 5;
	sprintf_((char*) mssg, "nb connect\r");
	stopTimer();
	launchTimer(TIMER_NB*SEC_TO_MILL);
	NB_sendCMD(mssg);
}

/*
 *
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
 *
 */
static void clearSetupNB (fsm_t *this)
{
	uint16_t *flags = (((car*)(this->data))->communication->flags);
	*flags = *flags & ~RESPOND_NB;
	osMutexWait(((car*)this->data)->communication->pileLock, 0);
	((car*)this->data)->communication->pileUART3->tail = ((((car*)this->data)->communication->pileUART3->tail+1) % ((car*)this->data)->communication->pileUART3->size);
	osMutexRelease(((car*)this->data)->communication->pileLock);
}


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
		*flags = *flags | TEST_MSSG;
	} else {
		*flags = *flags | NB_MSSG;
	}
	((car*)(this->data))->setupState = 5;
}


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
 *
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
 *
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
		for (pos = 0; pos < 2; pos++) {
			coche->lastLat = coche->lastLat*10;
			coche->lastLat += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}

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

		if (coor[10] == 'S')
			coche->lastLat = coche->lastLat * (-1);

		// Set Longitude
		getLongitud(coor);
		coche->lastLong = 0;
		for (pos = 0; pos < 3; pos++) {
			coche->lastLong = coche->lastLong*10;
			coche->lastLong += (coor[pos] - ASCII_NUMBER_THRESHOLD);
		}

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

		if (coor[11] == 'W')
			coche->lastLong = coche->lastLong * (-1);

		osMutexRelease(coche->communication->pileLock);
	}

}

/*
 * @brief	Realiza una copia del string
 * @param	str1: puntero a la zona de memoria donde se copia
 * 			str2: string a copiar
 * @retval	Nada
 */
static void str_cpy(uint8_t *str1, const uint8_t *str2)
{
	uint8_t i;
	for(i = 0; str2[i] != '\0'; i++)
		str1[i] = str2[i];
	str1[i+1] = '\0';
}
