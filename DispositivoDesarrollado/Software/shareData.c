/*
 * shareData.c
 *
 *  Circular buffers from
 *      Author: Phillip Johnston
 *      Web: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
 *      Modifier: miguelvp
 */

#include "shareData.h"
#include "lptim.h"
#include "stm32l4xx_hal_lptim.h"
#include "printf.h"
#include <string.h>

// Tamaños de las pilas de los buffer de recepción y transmisión a módulo
#define RX_SIZE		48
#define TX_SIZE		32

// Tamaños de pila de los mensajes a enviar por USB y NB-IoT
#define TX_USB_NUM_MSSG		10
#define TX_NB_NUM_MSSG		10

// Tiempos máximos de espera para los Mutex
#define RX_TIMEOUT	2000	// ms
#define TX_TIMEOUT	1000	// ms

#if SPEED_TEST
	#define SPEED_TEMPLATE	"\"speed\":%1.2f,"
	#define CO_TEMPLATE		"\"co\":%E,"
	#define NOX_TEMPLATE	"\"nox\":%E,"
	#define PM_TEMPLATE		"\"pm\":%E,"
#else
	#define SPEED_TEMPLATE	""
	#define CO_TEMPLATE		""
	#define NOX_TEMPLATE	""
	#define PM_TEMPLATE		""
#endif

#if RPM_TEST
	#define RPM_TEMPLATE	"\"rpm\":%1.2f,"
#else
	#define RPM_TEMPLATE	""
#endif

#define TEMPLATE	"{" SPEED_TEMPLATE CO_TEMPLATE NOX_TEMPLATE PM_TEMPLATE RPM_TEMPLATE "}\r"

// Copia de los flags para el timer
static uint16_t *flags_cpy;

// Buffers y mutex de datos de recepción y transmisión por USB
static circular_buf_t *usbReceive;
static circular_buf_t *usbSend;
static osMailQId txUSB, txNB;
static osMutexId sendPileMutex, receivePileMutex;

// Almacenamiento de coordenadas
static uint8_t lastLat[13];
static uint8_t lastLong[13];

// Codificación de mensajes en texto plano a hexadecimal
static void string2hex (uint8_t *src, uint8_t *dst);

// Funciones de los buffer
static void circular_buf_free(circular_buf_t *cbuf);
static uint8_t circular_buf_reset(circular_buf_t *cbuf);
static uint8_t circular_buf_put(circular_buf_t *cbuf, uint8_t data);
static uint8_t circular_buf_get(circular_buf_t *cbuf, uint8_t *data);
static uint8_t circular_buf_empty(circular_buf_t cbuf);
static uint8_t circular_buf_full(circular_buf_t cbuf);

/*
 * @brief	Inicializa los valores y reserva en memoria de los punteros
 * @param	flags: dirección de la variable compartida por micro y USB
 * 				   para tener copia como variable global
 * @retval	1 -> Se ha reservado correctamente
 * 			0 -> Ha habido algún error en las reservas de memoria
 */
uint8_t shareData_init(uint16_t *flags)
{
	flags_cpy = flags;

	if (circular_buf_init(&usbReceive, RX_SIZE) == 0) {
		return 0;
	}

	if (circular_buf_init(&usbSend, TX_SIZE) == 0) {
		circular_buf_free(usbReceive);
		return 0;
	}

	osMutexDef(sendPile);
	if ((sendPileMutex = osMutexCreate(osMutex(sendPile))) == NULL) {
		circular_buf_free(usbReceive);
		circular_buf_free(usbSend);
		return 0;
	}

	osMutexDef(sendReceive);
	if ((receivePileMutex = osMutexCreate(osMutex(sendReceive))) == NULL) {
		circular_buf_free(usbReceive);
		circular_buf_free(usbSend);
		osMutexDelete(sendPileMutex);
		return 0;
	}

	osMailQDef(mssgPileTX, TX_USB_NUM_MSSG, uint8_t*);
	if ((txUSB = osMailCreate(osMailQ(mssgPileTX), NULL)) == NULL) {
		circular_buf_free(usbReceive);
		circular_buf_free(usbSend);
		osMutexDelete(sendPileMutex);
		osMutexDelete(receivePileMutex);
		return 0;
	}

	osMailQDef(nbPileTX, TX_NB_NUM_MSSG, uint8_t*);
	if ((txNB = osMailCreate(osMailQ(nbPileTX), NULL)) == NULL) {
		circular_buf_free(usbReceive);
		circular_buf_free(usbSend);
		osMutexDelete(sendPileMutex);
		osMutexDelete(receivePileMutex);
		return 0;
	}

	return 1;

}

/*
 * @brief	Reserva el mutex asociado al buffer de recepción
 * @retval	1 -> Se ha cumplido la reserva del mutex
 * 			0 -> Ha ido mal la reserva por salto de timeout o
 * 				 cualquier otro motivo
 */
uint8_t lockRX(void)
{
	osStatus devol;
	devol = osMutexWait(receivePileMutex, pdMS_TO_TICKS(RX_TIMEOUT));
	return devol == osOK;
}

/*
 * @brief	Libera el mutex asociado al buffer de recepción
 * @retval	1 -> Se ha cumplido la liberación del mutex
 * 			0 -> Ha ido mal la liberación por cualquier motivo
 */
uint8_t unlockRX(void)
{
	osStatus devol;
	devol = osMutexRelease(receivePileMutex);
	return devol == osOK;
}

/*
 * @brief	Indica la cantidad de datos que hay en el buffer de recepción
 * 			que todavía no se han leido
 * @retval	Número de datos sin leer
 */
uint16_t notReadRX(void)
{
	if (usbReceive->head >= usbReceive->tail)
		return usbReceive->head-(usbReceive->tail);
	else
		return (usbReceive->size)-((usbReceive->tail)-usbReceive->head);
}

/*
 * @brief	Indica la longitud del primer mensaje que hay en el buffer
 * 			de recepción. Esto es hasta que se encuentre un \0
 * @retval	Longitud del primer mensaje
 */
uint16_t lenFirstMssgRX(void)
{
	uint16_t devol, initTail;
	uint8_t lastByte;

	initTail = usbReceive->tail;
	devol = 0;
	do {
		if (!circular_buf_get(usbReceive, &lastByte)) {
			devol = 0;
		} else {
			devol++;
		}
	} while (lastByte != '\0' && devol != 0);

	usbReceive->tail = initTail;
	return devol;
}

/*
 * @brief	Indica el tipo del siguiente mensaje que hay en el buffer
 * 			de recepción.
 * @retval	Tipo de mensaje STN_MSSG, GNSS_MSSG o NB_MSSG
 * 			0 -> No coincide con ninguno de los tipos de mensaje
 */
uint16_t typeNextMssgRX(void)
{
	uint8_t lastByte, initTail;
	uint16_t type;
	type = 0;
	lastByte = '\0';
	initTail = usbReceive->tail;

	circular_buf_get(usbReceive, &lastByte);
	if (lastByte == 's')
		type = STN_MSSG;
	else if (lastByte == 'g')
		type = GNSS_MSSG;
	else if (lastByte == 'n')
		type = NB_MSSG;
	else if (lastByte == 't')
		type = TEST_MSSG;
	else if (lastByte == 'd')
		type = TX_DATA;
	else if (lastByte == 'e')
		type = SETUP_MSSG;
	else
		type = 0;

	circular_buf_get(usbReceive, &lastByte);
	switch (type) {
	case STN_MSSG:
		if (lastByte != 't')
			type = 0;
		break;
	case GNSS_MSSG:
		if (lastByte != 'n')
			type = 0;
		break;
	case NB_MSSG:
		if (lastByte != 'b')
			type = 0;
		break;
	case TEST_MSSG:
		if (lastByte != 'e')
			type = 0;
		break;
	case TX_DATA:
		if (lastByte != 'a')
			type = 0;
		break;
	case SETUP_MSSG:
		if (lastByte != 'u')
			type = 0;
		break;
	}

	circular_buf_get(usbReceive, &lastByte);
	switch (type) {
	case STN_MSSG:
		if (lastByte != 'n')
			type = 0;
		break;
	case GNSS_MSSG:
		if (lastByte != 's')
			type = 0;
		break;
	case TEST_MSSG:
		if (lastByte != 's')
			type = 0;
		break;
	case TX_DATA:
		if (lastByte != 't')
			type = 0;
		break;
	case SETUP_MSSG:
		if (lastByte != 'r')
			type = 0;
		break;
	}

	circular_buf_get(usbReceive, &lastByte);
	switch (type) {
	case GNSS_MSSG:
		if (lastByte != 's')
			type = 0;
		break;
	case TEST_MSSG:
		if (lastByte != 't')
			type = 0;
		break;
	case TX_DATA:
		if (lastByte != 'a')
			type = 0;
		break;
	case SETUP_MSSG:
		if (lastByte != 'o')
			type = 0;
		break;
	}

	usbReceive->tail = initTail;
	if (!type)
		jumpMssgRX();
	return type;
}

/*
 * @brief	Añade datos al buffer de recepción
 * @param	data: puntero a los datos a guardar
 * 			len: cantidad de datos del puntero a guardar
 * @retval	1 -> datos añadidos correctamente
 * 			0 -> datos no añadidos
 */
uint8_t putRX(uint8_t *data, uint16_t len)
{
	uint16_t i, hInit;
	hInit = usbReceive->head;
	for (i = 0; i < len; i++) {
		if (!circular_buf_put(usbReceive, data[i])) {
			usbReceive->head = hInit;
			return 0;
		}
	}
	if (data[len-1] == '\r'){
		if (!circular_buf_put(usbReceive, '\0')) {
			usbReceive->head = hInit;
			return 0;
		}
	}
	return 1;
}

/*
 * @brief	Toma un cierto número de datos del buffer de recepción
 * @param	data: puntero al lugar donde se almacenarán los datos del buffer
 * @param	len: número de datos a recoger
 * @retval	1 -> datos extraídos correctamente
 * 			0 -> datos no extraídos
 */
uint8_t getRX(uint8_t *data, uint16_t len)
{
	uint16_t i, tInit;
	tInit = usbReceive->tail;
	for (i = 0; i < len; i++) {
		if (!circular_buf_get(usbReceive, &data[i])) {
			usbReceive->tail = tInit;
			return 0;
		}
	}
	return 1;
}

/*
 * @brief	Salta el mensaje en el buffer de recepción al siguiente
 * @retval	1 -> salto correcto
 * 			0 -> fallo en el salto
 */
uint8_t jumpMssgRX(void)
{
	uint8_t i, data;
	i = 0;
	do {
		circular_buf_get(usbReceive, &data);
		if (i == 255) {
			return 0;
		}
		i++;
	} while (data != '\0');
	return 1;
}

/*
 * @brief	Reserva el mutex asociado al buffer de transmisión
 * @retval	1 -> Se ha cumplido la reserva del mutex
 * 			0 -> Ha ido mal la reserva por salto de timeout o
 * 				 cualquier otro motivo
 */
uint8_t lockTX(void)
{
	osStatus devol;
	devol = osMutexWait(sendPileMutex, pdMS_TO_TICKS(TX_TIMEOUT));
	return devol == osOK;
}

/*
 * @brief	Libera el mutex asociado al buffer de transmisión
 * @retval	1 -> Se ha cumplido la liberación del mutex
 * 			0 -> Ha ido mal la liberación por cualquier motivo
 */
uint8_t unlockTX(void)
{
	osStatus devol;
	devol = osMutexRelease(sendPileMutex);
	return devol == osOK;
}

/*
 * @brief	Indica la cantidad de datos que hay en el buffer de transmisión
 * 			que todavía no se han enviado (Sin uso)
 * @retval	Número de datos sin leer
 */
uint16_t notSendTX(void)
{
	return 0;
}

/*
 * @brief	Añade datos al buffer de transmisión por USB
 * @param	data: puntero a los datos a guardar
 * 			len: cantidad de datos del puntero a guardar
 * @retval	1 -> datos añadidos correctamente
 * 			0 -> datos no añadidos
 */
uint8_t putTX(uint8_t *data, uint16_t len)
{
	uint8_t *mssg, j;
	uint16_t memPos;
	for (j = 0; j <= (len-1)/32; j++) {
		memPos = ((len-32*j)>=32)?33:(len-32*j)+1;
		if ((mssg = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*memPos)) == NULL)
			return 0;
		strcpy((char*) mssg, (char*) &(data[32*j]));
		mssg[memPos - 1] = '\0';
		osMailPut(txUSB, mssg);
	}
	return 1;
}

/*
 * @brief	Añade datos al buffer de transmisión por NB-IoT
 * @param	data: puntero a los datos a guardar
 * 			len: cantidad de datos del puntero a guardar
 * @retval	1 -> datos añadidos correctamente
 * 			0 -> datos no añadidos
 */
uint8_t putNB(uint8_t *data, uint16_t len)
{
	uint8_t *mssg;
	if ((mssg = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*len)) == NULL)
		return 0;
	strcpy((char*) mssg, (char*) data);
	osMailAlloc(txNB, 0);
	osMailPut(txNB, mssg);
	return 1;
}

/*
 * @brief	Toma un puntero de los mensajes del buffer de transmisión de USB
 * @param	data: puntero al lugar donde se almacenarán los datos del buffer
 * @param	len: número de datos a recoger (Sin uso)
 * @retval	Estado de la extracción del buffer
 */
uint8_t getTX(uint8_t **data, uint16_t len)
{
	osEvent evt;
	evt = osMailGet(txUSB, 0);
	*data = evt.value.p;
	osMailFree(txUSB, evt.value.p);
	return evt.status;
}

/*
 * @brief	Toma un puntero de los mensajes del buffer de transmisión de NB-IoT
 * @param	data: puntero al lugar donde se almacenarán los datos del buffer
 * @param	len: número de datos a recoger
 * @retval	Estado de la extracción del buffer
 */
uint8_t getNB(uint8_t **data, uint16_t len)
{
	osEvent evt;
	evt = osMailGet(txNB, 0);
	*data = evt.value.p;
	osMailFree(txNB, evt.value.p);
	return evt.status;
}

/*
 * @brief	Lanza un temporizador LP con el periodo indicado con interrupción
 * @param	period: periodo por el cual salta el timeout
 * @retval	1 -> HAL_Status OK
 * 			0 -> HAL_Status not OK
 */
uint8_t launchTimer(uint32_t period)
{
	return !HAL_LPTIM_OnePulse_Start_IT(&hlptim2, period, period-1);
}

/*
 * @brief	Detiene el timer LP
 * @retval	1 -> HAL_Status OK
 * 			0 -> HAL_Status not OK
 */
uint8_t stopTimer(void)
{
	return !HAL_LPTIM_OnePulse_Stop_IT(&hlptim2);
}

/*
 * @brief	Callback del fin del timer que activa el flag de timeout
 * @param	hlptim: handler del timer de bajo consumo
 */
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	*flags_cpy = *flags_cpy | TIMEOUT;
}


void setLatitud(uint8_t *newLat)
{
	strcpy((char*) lastLat, (char*) newLat);
}
void getLatitud(uint8_t *newLat)
{
	strcpy((char*) newLat, (char*) lastLat);
}
void setLongitud(uint8_t *newLong)
{
	strcpy((char*) lastLong, (char*) newLong);
}
void getLongitud(uint8_t *newLong)
{
	strcpy((char*) newLong, (char*) lastLong);
}

/*
 * @brief	Envío de mensajes con los datos del vehículo a los buffer de USB y NB-IoT
 * @param	coche: coche caracterizado
 * @retval	1 -> siempre válido
 */
uint8_t sendMssg (car* coche)
{
	uint8_t mssg[60], str2hex[30];
	float av_speed, av_rpm;
	uint8_t i, len;
	// Medimos la longitud del mensaje a enviar
	len = 0;
	sprintf_((char*) mssg, "{\"id\":111012345678,\"vin\":\"VF1BG0A0524085422\"");
	len += strlen((char*) mssg);
	sprintf_((char*) mssg, ",\"lat\":%1.7E,\"long\":%1.8E", coche->lastLat, coche->lastLong);
	len += strlen((char*) mssg);
	sprintf_((char*) mssg, ",\"co\":%1.6E,\"nox\":%1.6E",coche->co,coche->nox);
	len += strlen((char*) mssg);
	sprintf_((char*) mssg, ",\"pm\":%1.6E}\r",coche->pm);
	len += strlen((char*) mssg) - 1;
	sprintf_((char*) mssg, "AT+NSOST=0,35.226.227.97,8888,%d,",len);
	putNB(mssg, strlen((char*) mssg));
	sprintf_((char*) str2hex, "{\"id\":111012345678,");
	string2hex(str2hex, mssg);
	putNB(mssg, strlen((char*) mssg));
#if TEST
	av_speed = 0;
	av_rpm = 0;
	for (i = 0; i < NUM_VAL_CALC; i++) {
#if SPEED_TEST
		av_speed += coche->speed[i];
#endif
#if RPM_TEST
		av_rpm += coche->rpm[i];
#endif
	}
#if SPEED_TEST
	av_speed = av_speed / NUM_VAL_CALC;
#endif
#if RPM_TEST
	av_rpm = av_rpm / NUM_VAL_CALC;
#endif

	sprintf_((char*) str2hex, "\"vin\":\"VF1BG0A0524085422\",");
	string2hex(str2hex, mssg);
	putNB(mssg, strlen((char*) mssg));
	sprintf_((char*) str2hex, "\"lat\":%1.7E,", coche->lastLat);
	string2hex(str2hex, mssg);
	putNB(mssg, strlen((char*) mssg));
	sprintf_((char*) str2hex, "\"long\":%1.8E,", coche->lastLong);
	string2hex(str2hex, mssg);
	putNB(mssg, strlen((char*) mssg));

	// Envío por USB
	sprintf_((char*) mssg, "{\"speed\":[%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d],",
			coche->speed[0],
			coche->speed[1],
			coche->speed[2],
			coche->speed[3],
			coche->speed[4],
			coche->speed[5],
			coche->speed[6],
			coche->speed[7],
			coche->speed[8],
			coche->speed[9]);
	putTX(mssg, strlen((char*) mssg));
	sprintf_((char*) mssg, "\"lat\":%1.7E,", coche->lastLat);
	putTX(mssg, strlen((char*) mssg));
	sprintf_((char*) mssg, "\"long\":%1.8E,", coche->lastLong);
	putTX(mssg, strlen((char*) mssg));
	sprintf_((char*) mssg, "\"co\":%E,",coche->co);
	putTX(mssg, strlen((char*) mssg));
	sprintf_((char*) mssg, "\"nox\":%E,",coche->nox);
	putTX(mssg, strlen((char*) mssg));
	sprintf_((char*) mssg, "\"pm\":%E}\r",coche->pm);
	putTX(mssg, strlen((char*) mssg));

	// Envío por NB
	sprintf_((char*) str2hex, "\"co\":%1.6E,",coche->co);
	string2hex(str2hex, mssg);
	putNB(mssg, strlen((char*) mssg));
	sprintf_((char*) str2hex, "\"nox\":%1.6E,",coche->nox);
	string2hex(str2hex, mssg);
	putNB(mssg, strlen((char*) mssg));
	sprintf_((char*) str2hex, "\"pm\":%1.6E}",coche->pm);
	string2hex(str2hex, mssg);
	len = strlen((char*) mssg);
	mssg[len] = '\r';
	mssg[len+1] = '\0';
	putNB(mssg, strlen((char*) mssg));
	return 1;
#else
	return 0;
#endif
}

/*
 * @brief	Inicializa el buffer circular
 * @param	cbuf: buffer circular
 * @param	size: tamaño del buffer inicializado
 * @retval	1 -> Reserva de memoria con éxito
 * 			0 -> Error al reservar memoria
 */
uint8_t circular_buf_init(circular_buf_t **cbuf, uint16_t size)
{
	if ((*cbuf = (circular_buf_t*) pvPortMalloc(sizeof(circular_buf_t))) == NULL)
		return 0;
	if (((*cbuf)->buffer = (uint8_t*) pvPortMalloc(size*sizeof(uint8_t))) == NULL)
		return 0;
	(*cbuf)->size = size;
	return circular_buf_reset(*cbuf);
}

/*
 * @brief	Libera el buffer circular
 * @param	cbuf: buffer circular
 * @retval	Nada
 */
static void circular_buf_free(circular_buf_t *cbuf)
{
	vPortFree(cbuf);
}

/*
 * @brief	Reinicia el buffer circular
 * @param	cbuf: buffer circular
 * @retval	1 -> Reinicio con éxito
 * 			0 -> Error al reiniciar el buffer
 */
static uint8_t circular_buf_reset(circular_buf_t *cbuf)
{
    int r = 0;

    if(cbuf)
    {
        cbuf->head = 0;
        cbuf->tail = 0;
        cbuf->full = 0;
        r = 1;
    }

    return r;
}

/*
 * @brief	Comprobación de si el buffer circular está vacío
 * @param	cbuf: buffer circular
 * @retval	1 -> El buffer está vacío
 * 			0 -> El buffer no está vacío
 */
static uint8_t circular_buf_empty(circular_buf_t cbuf)
{
    return (cbuf.head == cbuf.tail) && !cbuf.full;
}

/*
 * @brief	Comprobación de si el buffer circular está lleno
 * @param	cbuf: buffer circular
 * @retval	1 -> El buffer está lleno
 * 			0 -> El buffer no está lleno
 */
static uint8_t circular_buf_full(circular_buf_t cbuf)
{
    return cbuf.full;
}

/*
 * @brief	Añade datos al buffer circular
 * @param	cbuf: buffer circular
 * @param	data: datos a añadir
 * @retval	1 -> Datos añadidos correctamente
 * 			0 -> Error al añadir los datos
 */
static uint8_t circular_buf_put(circular_buf_t *cbuf, uint8_t data)
{
    int r = 0;

    if(cbuf)
    {
        cbuf->buffer[cbuf->head] = data;
        cbuf->head = (cbuf->head + 1) % cbuf->size;

        if(cbuf->head == cbuf->tail)
        {
            cbuf->tail = (cbuf->tail + 1) % cbuf->size;
            cbuf->full = 1;
        }

        r = 1;
    }

    return r;
}

/*
 * @brief	Conversión del string pasado a su codificación en hexadecimal
 * @param	src: string a codificar
 * 			dst: puntero al destino
 */
static void string2hex (uint8_t *src, uint8_t *dst)
{
	uint8_t i, j;

	i = 0;
	j = 0;

	while(src[i] != '\0')
	{
		sprintf((char*)(dst+j),"%02X", src[i]);
		i += 1;
		j += 2;
	}

	dst[j++] = '\0';
}

/*
 * @brief	Extrae datos del buffer circular
 * @param	cbuf: buffer circular
 * @param	data: puntero a los datos extraídos
 * @retval	1 -> Datos extraídos correctamente
 * 			0 -> Error al extraer los datos
 */
static uint8_t circular_buf_get(circular_buf_t * cbuf, uint8_t * data)
{
    int r = 0;

    if(cbuf && data && !circular_buf_empty(*cbuf))
    {
        *data = cbuf->buffer[cbuf->tail];
        cbuf->tail = (cbuf->tail + 1) % cbuf->size;

        r = 1;
    }

    return r;
}
