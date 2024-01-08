/***************************************************************************************
**  uart.h - Funciones generales de la UART
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/05/2019
**  Fecha de modificacion: 03/08/2020
**
**  El proyecto URpilot NO es libre. No se puede distribuir y/o modificar este fichero
**  bajo ningun concepto.
**
**  En caso de modificacion y/o solicitud de informacion pongase en contacto con
**  el grupo de investigacion ICON a traves de: www.unirioja.es/urpilot
**
**
**  Control de versiones del fichero
**
**  v1.0  Ramon Rico. Se ha liberado la primera version estable
**
****************************************************************************************/

#ifndef __UART_H
#define __UART_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if defined(STM32F767xx)
  #define NUM_MAX_UART               8
#elif defined(STM32F722xx)
  #define NUM_MAX_UART               6
#endif

#ifndef TAMANIO_BUFFER_RX_UART
  #define TAMANIO_BUFFER_RX_UART     512
#endif
#ifndef TAMANIO_BUFFER_TX_UART
  #define TAMANIO_BUFFER_TX_UART     256
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    UART_NINGUNO = -1,
    UART_1       =  0,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_6,
    UART_7,
    UART_8,
} numUART_e;

typedef enum {
    UART_BIT_STOP_1 = 0,
	UART_BIT_STOP_2,
} stop_e;

typedef enum {
    UART_NO_PARIDAD = 0,
	UART_PARIDAD_EVEN,
	UART_PARIDAD_ODD,
} paridad_e;

typedef enum {
	UART_LONGITUD_WORD_7 = 0,
	UART_LONGITUD_WORD_8,
	UART_LONGITUD_WORD_9,
} lWord_e;

typedef struct {
    uint32_t baudrate;
    lWord_e lWord;
    paridad_e paridad;
    stop_e stop;
} configIniUART_t;

typedef struct {
    bool asignado;
	UART_HandleTypeDef huart;
#ifdef USAR_DMA_UART
    DMA_HandleTypeDef hdmaTx;
    DMA_HandleTypeDef hdmaRx;
#endif
    uint8_t IRQ;
    uint8_t prioridadIRQ;
    pin_t pinTx;
    pin_t pinRx;
} halUART_t;

typedef void (*uartRxCallback)(uint8_t dato);

typedef struct {
	bool iniciado;
    halUART_t hal;
    uartRxCallback rxCallback;
    volatile uint16_t rxBuffer[TAMANIO_BUFFER_RX_UART];
    volatile uint16_t txBuffer[TAMANIO_BUFFER_TX_UART];
    volatile uint16_t cabezaRxBuffer;
    volatile uint16_t colaRxBuffer;
    volatile uint16_t cabezaTxBuffer;
    volatile uint16_t colaTxBuffer;
    volatile uint16_t numErrores;
} uart_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
uart_t *punteroUART(numUART_e numUART);
bool asignarHALuart(numUART_e numUART);
bool iniciarUART(numUART_e numUART, configIniUART_t configInicial, uartRxCallback rxCall);
bool uartIniciada(numUART_e numUART);
bool iniciarDriverUART(numUART_e numUART, configIniUART_t configInicial);
bool ajustarBaudRateUART(numUART_e numUART, uint32_t baudrate);
void errorCallbackUART(numUART_e numUART);
uint16_t contadorErrorUART(numUART_e numUART);
void resetearContadorErrorUART(numUART_e numUART);

void escribirUART(numUART_e numUART, uint8_t byteTx);
void escribirBufferUART(numUART_e numUART, uint8_t *datoTx, uint16_t longitud);
int16_t leerUART(numUART_e numUART);
void leerBufferUART(numUART_e numUART, int16_t *datoRx, uint16_t longitud);
uint16_t bytesRecibidosUART(numUART_e numUART);
bool bufferTxVacioUART(numUART_e numUART);
uint16_t bytesLibresBufferTxUART(numUART_e numUART);
void flushUART(numUART_e numUART);

#endif // __UART_H
