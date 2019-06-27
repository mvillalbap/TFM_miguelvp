/*
 * usb_fsm.c
 *
 *  Created on: 24 ene. 2019
 *      Author: miguelvp
 */

#include "usb_fsm.h"
#include "FreeRTOS.h"
#include "usbd_cdc_if.h"
#include "stm32l4xx_hal_uart.h"
#include <string.h>

// UARTs
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

// USB
USBD_HandleTypeDef hUsbDeviceFS;

// Dato recibido nuevo
static uint8_t new;

// Funciones de comprobación
static uint8_t firstStep (fsm_t *this);
static uint8_t sendData (fsm_t *this);
static uint8_t allSent (fsm_t *this);
static uint8_t checkUSB (fsm_t *this);
static uint8_t checkUART1 (fsm_t *this);
static uint8_t checkUART2 (fsm_t *this);
static uint8_t checkUART3 (fsm_t *this);

// Funciones de transición
static void setup (fsm_t *this);
static void sendUSB (fsm_t *this);
static void flush (fsm_t *this);
static void update (fsm_t *this);
static void dataUART1 (fsm_t *this);
static void dataUART2 (fsm_t *this);
static void dataUART3 (fsm_t *this);

// Estados de la máquina
static enum USBstates {
	PREV,
	IDLE,
	TX_USB,
};

// Tabla de transiciones
static fsm_trans_t USB_tt[] = {
	{PREV,		firstStep,		IDLE,		setup},
	{IDLE,		checkUSB,		IDLE,		update},
	{IDLE,		sendData,		TX_USB,		sendUSB},
	{IDLE,		checkUART1,		IDLE, 		dataUART1},
	{IDLE,		checkUART2,		IDLE, 		dataUART2},
	{IDLE,		checkUART3,		IDLE, 		dataUART3},
	{TX_USB,	allSent,		IDLE,		flush},
	{TX_USB,	firstStep,		TX_USB,		sendUSB},
	{-1, NULL, -1, NULL}
};

/*
 * @brief 	Inicialización de la máquina de estados
 * @param 	flags: puntero a la variable compartida de la señalización por flags
 * @retval 	Máquina de estados
 */
fsm_t* init_usb(pilePointers_t *data)
{
	fsm_t *fsm;
	fsm = fsm_new(USB_tt, data);
	return fsm;
}

static uint8_t firstStep (fsm_t *this)
{
	return 1;
}

/*
 * @brief	Comprobación de si hay datos a enviar
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos a enviar
 * 			 0 -> No hay datos a enviar
 */
static uint8_t sendData (fsm_t *this)
{
	return ((*((pilePointers_t*)this->data)->flags) & TX_DATA);
}

/*
 * @brief	Comprobación de si se han enviado todos los datos
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Se han enviado todos los datos
 * 			 0 -> No se han enviado todos los datos
 */
static uint8_t allSent (fsm_t *this)
{
	return !((*((pilePointers_t*)this->data)->flags) & TX_DATA);
}

/*
 * @brief	Comprobación de si hay datos nuevos recibidos por USB
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos nuevos
 * 			 0 -> No hay datos nuevos
 */
static uint8_t checkUSB (fsm_t *this)
{
	return new;
}

/*
 * @brief	Comprobación de si hay datos nuevos recibidos por la UART1
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos nuevos
 * 			 0 -> No hay datos nuevos
 */
static uint8_t checkUART1 (fsm_t *this)
{
	return ((BUFFER_UART1 - ((pilePointers_t*)this->data)->pileUART1->tail) != ((huart1.hdmarx)->Instance->CNDTR));
}

/*
 * @brief	Comprobación de si hay datos nuevos recibidos por la UART2
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos nuevos
 * 			 0 -> No hay datos nuevos
 */
static uint8_t checkUART2 (fsm_t *this)
{
	return (BUFFER_UART2 - ((pilePointers_t*)this->data)->pileUART2->head) != ((huart2.hdmarx)->Instance->CNDTR);
}

/*
 * @brief	Comprobación de si hay datos nuevos recibidos por la UART3
 * @param	this: máquina de estados a evaluar
 * @retval	!0 -> Hay datos nuevos
 * 			 0 -> No hay datos nuevos
 */
static uint8_t checkUART3 (fsm_t *this)
{
	return (BUFFER_UART3 - ((pilePointers_t*)this->data)->pileUART3->tail) != ((huart3.hdmarx)->Instance->CNDTR);
}

/*
 * @brief	Configuración de las DMA para asignarlas a los buffer creados previamente
 * 			para cada una de las UART
 * @param	this: máquina de estados a evaluar
 * @retval	Nada
 */
static void setup (fsm_t *this)
{
	HAL_UART_Receive_DMA(&huart1, ((pilePointers_t*)this->data)->pileUART1->buffer, BUFFER_UART1);
	HAL_UART_Receive_DMA(&huart2, ((pilePointers_t*)this->data)->pileUART2->buffer, BUFFER_UART2);
	HAL_UART_Receive_DMA(&huart3, ((pilePointers_t*)this->data)->pileUART3->buffer, BUFFER_UART3);
}

/*
 * @brief	Envía datos por USB y para el módulo NB-IoT que haya en el buffer de transmisión
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void sendUSB (fsm_t *this)
{
	uint8_t *data, result;
	uint16_t *flags;

	flags = ((pilePointers_t*)this->data)->flags;

	if (((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState == 0) {

		// Recuperamos los datos a transmitir
		lockTX();
		if (getTX(&data, 10) == osEventMail) {
			result = CDC_Transmit_FS(data, strlen((char*) data));

			if (result != USBD_OK) {
				putTX(data, strlen((char*) data));
			}
			vPortFree(data);
		} else {
			*flags = *flags & ~TX_DATA;
		}

		unlockTX();
	}
	if (huart3.gState != HAL_UART_STATE_BUSY_TX && huart3.gState != HAL_UART_STATE_BUSY_RX) {
		if (getNB(&data, 10) == osEventMail) {
			result = HAL_UART_Transmit_IT(&huart3, data, strlen((char*) data));

			if (result == HAL_ERROR) {
				putNB(data, strlen((char*) data));
			}
			vPortFree(data);
		} else {
			*flags = *flags & ~TX_DATA;
		}
	}

}

/*
 * @brief	Vuelta del envío de información
 * @param	this: máquina de estados a evaluar
 * @retval	Nada
 */
static void flush (fsm_t *this)
{
	// Por ahora no tiene uso la vuelta a RX
	return;
}

/*
 * @brief	Actualiza el estado de la señal de datos en el buffer de recepción
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void update (fsm_t *this)
{
	uint16_t *flags;
	flags = ((pilePointers_t*)this->data)->flags;
	*flags = *flags | B_NOT_READ;
	new = 0;
}

/*
 * @brief	Actualiza el buffer circular asociado a la UART1
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void dataUART1 (fsm_t *this)
{
	uint16_t *flags;
	uint16_t tail;

	osMutexWait(((pilePointers_t*)this->data)->pileLock, 0);
	tail = ((pilePointers_t*)this->data)->pileUART1->tail;
	do {
		if (((pilePointers_t*)this->data)->pileUART1->buffer[tail] == '\r') {
				flags = ((pilePointers_t*)this->data)->flags;
				*flags = *flags | RESPOND_STN;
				((pilePointers_t*)this->data)->pileUART1->head = (tail+1) % BUFFER_UART1;
			osMutexRelease(((pilePointers_t*)this->data)->pileLock);
			return;
		}
		if (((pilePointers_t*)this->data)->pileUART1->buffer[tail] == '>') {
			flags = ((pilePointers_t*)this->data)->flags;
			*flags = *flags | NEW_DATA_STN;
			((pilePointers_t*)this->data)->pileUART1->head = (tail+1) % BUFFER_UART1;
			osMutexRelease(((pilePointers_t*)this->data)->pileLock);
			return;
		}
		tail = (tail+1) % BUFFER_UART1;
	} while ((BUFFER_UART1 - tail) != ((huart1.hdmarx)->Instance->CNDTR));
	osMutexRelease(((pilePointers_t*)this->data)->pileLock);
}

/*
 * @brief	Actualiza el buffer circular asociado a la UART2
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void dataUART2 (fsm_t *this)
{
	uint16_t pos;
	uint8_t j, charVal = 0;
	static uint8_t flagCheck = 0;
	static uint8_t lastLat[13];
	static uint8_t lastLong[13];

	if(osMutexWait(((pilePointers_t*)this->data)->pileLock, 0) == osOK) {
		pos = ((pilePointers_t*)this->data)->pileUART2->head;
		do {
			charVal = ((pilePointers_t*)this->data)->pileUART2->buffer[pos];
			// Comienzo del mensaje
			if (charVal == '$') {
				flagCheck = 1;

			// Comprobación de si el valor es válido o no o si ha llegado al checksum
			} else if (flagCheck && (charVal == 'V' || charVal == '*')) {
				flagCheck = 0;

			// Análisis de los datos de posición recibidos
			} else if (flagCheck && (charVal == 'E' || charVal == 'W')) {
				flagCheck = 0;
				if (pos >= 23) {
					strcpy((char*) lastLat, (char*) &(((pilePointers_t*)this->data)->pileUART2->buffer[pos-23]));
					strcpy((char*) lastLong, (char*) &(((pilePointers_t*)this->data)->pileUART2->buffer[pos-11]));
					((pilePointers_t*)this->data)->pileUART2->tail = pos;
				} else if (pos >= 11) {
					for(j = (BUFFER_UART2 - (23 - pos)); (j < BUFFER_UART2) && ((BUFFER_UART2 - j) < 12); j++)
						lastLat[j - (BUFFER_UART2 - (23 - pos))] = ((pilePointers_t*)this->data)->pileUART2->buffer[j];
					for(j = (23 - pos); (j < pos) && (j < 12); j++)
						lastLat[j] = ((pilePointers_t*)this->data)->pileUART2->buffer[j - (23 - pos)];
					strcpy((char*) lastLong, (char*) &(((pilePointers_t*)this->data)->pileUART2->buffer[pos-11]));
					((pilePointers_t*)this->data)->pileUART2->tail = BUFFER_UART2 - (23 - pos);
				} else {
					strcpy((char*) lastLat, (char*) &(((pilePointers_t*)this->data)->pileUART2->buffer[(BUFFER_UART2 - (23 - pos))]));
					for(j = (BUFFER_UART2 - (11 - pos)); (j < BUFFER_UART2) && ((BUFFER_UART2 - j) < 12); j++)
						lastLong[j - (BUFFER_UART2 - (11 - pos))] = ((pilePointers_t*)this->data)->pileUART2->buffer[j];
					for(j = (11 - pos); (j < pos) && (j < 12); j++)
						lastLong[j] = ((pilePointers_t*)this->data)->pileUART2->buffer[j - (11 - pos)];
					((pilePointers_t*)this->data)->pileUART2->tail = BUFFER_UART2 - (11 - pos);
				}
				lastLat[11] = '\0';
				lastLong[12] = '\0';
				setLatitud(lastLat);
				setLongitud(lastLong);
			}
			pos = (pos+1) % BUFFER_UART2;
		} while ((BUFFER_UART2 - pos) != ((huart2.hdmarx)->Instance->CNDTR));
		((pilePointers_t*)this->data)->pileUART2->head = pos;
		osMutexRelease(((pilePointers_t*)this->data)->pileLock);
	}
}

/*
 * @brief	Actualiza el buffer circular asociado a la UART3
 * @param	this: máquina de estados de la acción
 * @retval	Nada
 */
static void dataUART3 (fsm_t *this)
{
	uint16_t *flags;
	uint16_t tail;
	uint8_t seqFlag = 0, start = 0;

	osMutexWait(((pilePointers_t*)this->data)->pileLock, 0);
	tail = ((pilePointers_t*)this->data)->pileUART3->tail;
	do {
		if (!start && (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == '\r')) {
			start = 1;
		} else if((start == 2) && (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == '\n')) {
			flags = ((pilePointers_t*)this->data)->flags;
			if (seqFlag == 4 || seqFlag == 10)
				*flags = *flags | NEW_DATA_NB;
			else
				*flags = *flags | RESPOND_NB;
			((pilePointers_t*)this->data)->pileUART3->head = (tail+1) % BUFFER_UART3;
			osMutexRelease(((pilePointers_t*)this->data)->pileLock);
			return;
		} else if (start && (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == '\n')) {
			start = 2;
			seqFlag = 1;

		// Secuencia de nuevo valor
		} else if(seqFlag) {
			switch (seqFlag) {
			case 1:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == 'O')
					seqFlag = 2;
				else if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == 'E')
					seqFlag = 4;
				else
					seqFlag = 0;
				break;
			case 2:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == 'K')
					seqFlag = 3;
				else
					seqFlag = 0;
				break;
			case 3:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == '\r')
					seqFlag = 4;
				else
					seqFlag = 0;
				break;
			case 4:
				seqFlag = 0;
				break;
			case 5:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == 'R')
					seqFlag = 6;
				else
					seqFlag = 0;
				break;
			case 6:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == 'R')
					seqFlag = 7;
				else
					seqFlag = 0;
				break;
			case 7:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == 'O')
					seqFlag = 8;
				else
					seqFlag = 0;
				break;
			case 8:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == 'R')
					seqFlag = 9;
				else
					seqFlag = 0;
				break;
			case 9:
				if (((pilePointers_t*)this->data)->pileUART3->buffer[tail] == '\r')
					seqFlag = 10;
				else
					seqFlag = 0;
				break;
			case 10:
				seqFlag = 0;
				break;
			}
		}
		tail = (tail+1) % BUFFER_UART3;
	} while ((BUFFER_UART3 - tail) != ((huart3.hdmarx)->Instance->CNDTR));
	osMutexRelease(((pilePointers_t*)this->data)->pileLock);
}

/*
 * @brief	Handler de la recepción de datos por USB
 * @param	Buf: buffer donde se encuentran los datos recibidos
 * @param	len: cantidad de datos recibidos
 * @retval	Nada
 */
void newData (uint8_t* Buf, uint32_t len)
{
	uint8_t i;
	lockRX();
	putRX(Buf, len);
	unlockRX();
	for (i = 0; i < len; i++)
		if (Buf[i] == '\r')
			new = 1;
}
