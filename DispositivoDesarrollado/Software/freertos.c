/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "micro_fsm.h"
#include "usb_fsm.h"
#include "copert.h"

// Periodos de la tareas
#define USB_PERIOD		12	// milliseconds
#define MICRO_PERIOD	8	// milliseconds
#define ALIVE_PERIOD	100	// milliseconds


/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId usbTaskHandle;
osThreadId microTaskHandle;
osThreadId aliveTaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartUsbTask(void const * argument);
void StartMicroTask(void const * argument);
void StartAliveTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	uint8_t i;
	uint16_t *flags;			// Debemos tener una variable compartida entre las máquinas USB y del micro
	pilePointers_t *serial;
	car *coche;

	// Inicialización de los flags
	if ((flags = (uint16_t*) pvPortMalloc(sizeof(uint16_t))) == NULL) {
		enciendeLED(AZUL);
		enciendeLED(VERDE);
		while(1) {}
	}
	*flags = 0;

	// Inicialización de los datos compartidos
	if (!shareData_init(flags)){
		enciendeLED(AZUL);
		enciendeLED(VERDE);
		while(1) {}
	}

	// Inicialización de los datos recibidos por las comunicaciones
	if ((serial = (pilePointers_t*) pvPortMalloc(sizeof(pilePointers_t))) == NULL) {
		enciendeLED(AZUL);
		enciendeLED(VERDE);
		while (1) {}
	}
	circular_buf_init(&(serial->pileUART1), BUFFER_UART1);
	circular_buf_init(&(serial->pileUART2), BUFFER_UART2);
	circular_buf_init(&(serial->pileUART3), BUFFER_UART3);
	serial->flags = flags;
	osMutexDef(pileLock);
	if ((serial->pileLock = osMutexCreate(osMutex(pileLock))) == NULL) {
		enciendeLED(AZUL);
		enciendeLED(VERDE);
		while (1) {}
	}

	// Inicialización de los datos del coche
	if ((coche = (car*) pvPortMalloc(sizeof(car))) == NULL) {
		enciendeLED(AZUL);
		enciendeLED(VERDE);
		while (1) {}
	}
	coche->timestart = 0;
	coche->fuel = NONE;
	coche->communication = serial;
	coche->setupState = NUM_SETUP;
	coche->lastLat = 0;
	coche->lastLong = 0;

#if TEST
	coche->times = 0;
	for (i = 0; i < NUM_VAL_CALC; i++) {
#if SPEED_TEST
		coche->speed[i] = 0;
#else
		coche->speed[0] = 0;
#endif
#if RPM_TEST
		coche->rpm[i] = 0;
#else
		coche->rpm[0] = 0;
#endif
#if AIR_TEST
		coche->air[i] = 0;
#else
		coche->air[0] = 0;
#endif
	}
#endif

	MX_USB_DEVICE_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of usbTask */
  osThreadDef(usbTask, StartUsbTask, osPriorityNormal, 0, 128);
  usbTaskHandle = osThreadCreate(osThread(usbTask), (void*) serial);

  /* definition and creation of microTask */
  osThreadDef(microTask, StartMicroTask, osPriorityAboveNormal, 0, 256);
  microTaskHandle = osThreadCreate(osThread(microTask), (void*) coche);

  /* definition and creation of aliveTask */
  osThreadDef(aliveTask, StartAliveTask, osPriorityBelowNormal, 0, 32);
  aliveTaskHandle = osThreadCreate(osThread(aliveTask), (void*) flags);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartUsbTask */
/**
  * @brief  Function implementing the usbTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartUsbTask */
void StartUsbTask(void const * argument)
{
	/* USER CODE BEGIN StartUsbTask */
	fsm_t *usb;
	uint32_t clk1, clk2;
	pilePointers_t *serial = (pilePointers_t*) argument;

	// Si no se puede crear la máquina, paramos hebra
	if ((usb = init_usb(serial)) == NULL) {
		enciendeLED(VERDE);
		while(1){}
	}

	clk1 = osKernelSysTick();
	/* Infinite loop */
	for(;;)
	{
		fsm_fire(usb);
		clk2 = osKernelSysTick();
		osDelay(USB_PERIOD - (clk2 - clk1));
		clk1 += USB_PERIOD;
	}
	/* USER CODE END startUsbTask */
}

/* USER CODE BEGIN Header_StartMicroTask */
/**
* @brief Function implementing the microTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMicroTask */
void StartMicroTask(void const * argument)
{
	/* USER CODE BEGIN startMicroTask */
	fsm_t *micro;
	uint32_t clk1, clk2;
	car *coche = (car*) argument;

	// Si no se puede crear la máquina, paramos hebra
	if ((micro = init_micro(coche)) == NULL) {
		enciendeLED(AZUL);
		while(1){}
	}
	clk1 = osKernelSysTick();
	/* Infinite loop */
	for(;;)
	{
		fsm_fire(micro);
		clk2 = osKernelSysTick();
		osDelay(MICRO_PERIOD - (clk2 - clk1));
		clk1 += MICRO_PERIOD;
	}
	/* USER CODE END startMicroTask */
}

/* USER CODE BEGIN Header_StartAliveTask */
/**
* @brief Function implementing the aliveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAliveTask */
void StartAliveTask(void const * argument)
{
	/* USER CODE BEGIN startAliveTask */
	uint32_t clk1, clk2;
	uint8_t mssgLen, mssgPos, cont = 0;
	uint8_t *mssg;
	uint16_t *flags = (uint16_t*) argument;
	clk1 = osKernelSysTick();
	/* Infinite loop */
	for(;;)
	{
		(cont == 5)? enciendeLED(VERDE) : apagaLED(VERDE);
		cont = (cont+1)%6;
#if TEST
		// Envío del mensaje de test de forma periódica
		if ((*flags) & TEST_MSSG) {
			mssgPos = 10;

			// Reservamos memoria en función de los datos que vayamos a pedir en el test
			mssgLen = 11
	#if SPEED_TEST
					+2
	#endif
	#if RPM_TEST
					+2
	#endif
					;

			mssg = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*mssgLen);

			// Creamos el mensaje con los datos a pedir en el test
			mssg[0] = 's'; mssg[1] = 't'; mssg[2] = 'n'; mssg[3] = ' '; mssg[4] = 'o'; mssg[5] = 'b'; mssg[6] = 'd'; mssg[7] = ' '; mssg[8] = '0'; mssg[9] = '1';
	#if SPEED_TEST
			mssg[mssgPos] = '0'; mssg[mssgPos+1] = 'd';
			mssgPos += 2;
	#endif
	#if RPM_TEST
			mssg[mssgPos] = '0'; mssg[mssgPos+1] = 'c';
			mssgPos += 2;
	#endif
			mssg[mssgPos] = '\r';

			lockRX();
			putRX(mssg, mssgLen);
			unlockRX();
			*flags = *flags | B_NOT_READ;
			vPortFree(mssg);
		}
#endif
		clk2 = osKernelSysTick();
		osDelay(SEND_PERIOD/NUM_VAL_CALC - (clk2 - clk1));
		clk1 += SEND_PERIOD/NUM_VAL_CALC;
	}
	/* USER CODE END startAliveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
