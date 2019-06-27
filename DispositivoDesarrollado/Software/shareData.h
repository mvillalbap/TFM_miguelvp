/*
 * shareData.h
 *
 *  Circular buffers from
 *      Author: Phillip Johnston
 *      Web: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
 *      Modifier: miguelvp
 */

#ifndef SHAREDATA_H_
#define SHAREDATA_H_

#include <stdint.h>
#include "cmsis_os.h"

// Flags
#define TX_DATA			0x0001

#define B_NOT_READ		0x0002
#define STN_MSSG		0x0004
#define GNSS_MSSG		0x0008
#define NB_MSSG			0x0010
#define RESPOND_STN		0x0020
#define NEW_DATA_STN	0x0040
#define TIMEOUT			0x0080

#define TEST_MSSG		0x0100
#define SETUP_MSSG		0x0200
#define RESPOND_NB		0x0400
#define NEW_DATA_NB		0x0800

#define VIN_LENGTH	17

#define ASCII_NUMBER_THRESHOLD	48
#define ASCII_LETTER_THRESHOLD	65

#define DEBUG			0
#define TEST			1
#if TEST
#define SPEED_TEST		1
#define RPM_TEST		0
#define AIR_TEST		0
#endif

#define DEPLOYMENT		0

#if DEPLOYMENT
#define NUM_SETUP		3
#else
#define NUM_SETUP		4
#endif

#if TEST
#define SEND_PERIOD		5000 // milliseconds
#define NUM_VAL_CALC	10
#endif


// Buffer circular
typedef struct circular_buf {
    uint8_t *buffer;
    uint16_t head;
    uint16_t tail;
    uint16_t size; //of the buffer
    uint16_t full;
} circular_buf_t;

typedef struct _pilePointers {
	uint16_t		*flags;
	osMutexId		pileLock;
	circular_buf_t	*pileUART1;
	circular_buf_t	*pileUART2;
	circular_buf_t	*pileUART3;
} pilePointers_t;

typedef enum _leds {
	VERDE,
	AZUL,
} leds;

typedef enum _fuelType {
	NONE,
	GAS, METH, ETH, DSL, LPG, CNG, PROP, ELEC,
	BI_GAS, BI_METH, BI_ETH, BI_LPG, BI_CNG, BI_PROP, BI_ELEC, BI_MIX,
	HYB_GAS, HYB_ETH, HYB_DSL, HYB_ELEC, HYB_MIX, HYB_REG,
	BI_DSL
} fuelType;

// COPERT equation (alpha*V^2+beta*V+gamma+delta/V)/(epsilon/eta*V^2+zita/eta*V+1)*(1-reductionFactor)/eta
typedef struct _emissionParams {
	float a;	// alpha
	float b;	// beta
	float c;	// gamma
	float d;	// delta
	float e;	// epsilon/eta
	float f;	// zeta/eta
	float reductionFactor;		// (1-reductionFactor)/eta
} emissionParams;

typedef struct _car {
	uint8_t			vin[VIN_LENGTH+1];
#if RPM_TEST
	uint16_t		rpm[NUM_VAL_CALC];
#else
	uint16_t		rpm[1];
#endif
#if SPEED_TEST
	uint8_t			speed[NUM_VAL_CALC];
#else
	uint8_t			speed[1];
#endif
#if AIR_TEST
	int16_t			air[NUM_VAL_CAL];
#else
	int16_t			air[1];
#endif
	fuelType		fuel;
	uint8_t			timestart;

	emissionParams	coParams;
	float			co;
	emissionParams	noxParams;
	float			nox;
	emissionParams	pmParams;
	float			pm;

	float			lastLat;
	float			lastLong;

	pilePointers_t	*communication;
	uint8_t			setupState;

#if TEST
	uint8_t			times;
#endif

} car;

// Inicialización de datos
uint8_t shareData_init(uint16_t *flags);
uint8_t circular_buf_init(circular_buf_t **cbuf, uint16_t size);

// Tratamiento del buffer de recepción
uint8_t lockRX(void);
uint8_t unlockRX(void);
uint16_t notReadRX(void);
uint16_t lenFirstMssgRX(void);
uint16_t typeNextMssgRX(void);
uint8_t putRX(uint8_t *data, uint16_t len);
uint8_t getRX(uint8_t *data, uint16_t len);
uint8_t jumpMssgRX(void);

// Tratamiento del buffer de transmisión
uint8_t lockTX(void);
uint8_t unlockTX(void);
uint16_t notSendTX(void);
uint8_t putTX(uint8_t *data, uint16_t len);
uint8_t getTX(uint8_t **data, uint16_t len);
uint8_t putNB(uint8_t *data, uint16_t len);
uint8_t getNB(uint8_t **data, uint16_t len);

// Envío de datos
uint8_t sendMssg(car* coche);

// Gestino de localización
void setLatitud(uint8_t *newLat);
void getLatitud(uint8_t *newLat);
void setLongitud(uint8_t *newLong);
void getLongitud(uint8_t *newLong);

// Manejo de timer HW
uint8_t launchTimer(uint32_t period);
uint8_t stopTimer(void);

#endif /* SHAREDATA_H_ */
