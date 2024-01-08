/***************************************************************************************
**  gp_uart.c - Funciones y grupo de parametros de la UART
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 04/07/2020
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "gp_uart.h"

#ifdef USAR_UART
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef USAR_DMA_UART
  #define USAR_DMA_DRIVER_UART   false
#else
  #define USAR_DMA_DRIVER_UART   true
#endif

#ifndef PIN_RX_UART_1
  #define PIN_RX_UART_1          NINGUNO
#endif

#ifndef PIN_TX_UART_1
  #define PIN_TX_UART_1          NINGUNO
#endif

#ifndef DMA_TX_UART_1
  #define DMA_TX_UART_1          NULL
#endif

#ifndef DMA_RX_UART_1
  #define DMA_RX_UART_1          NULL
#endif

#ifndef PIN_RX_UART_2
  #define PIN_RX_UART_2          NINGUNO
#endif

#ifndef PIN_TX_UART_2
  #define PIN_TX_UART_2          NINGUNO
#endif

#ifndef DMA_TX_UART_2
  #define DMA_TX_UART_2          0
#endif

#ifndef DMA_RX_UART_2
  #define DMA_RX_UART_2          0
#endif

#ifndef PIN_RX_UART_3
  #define PIN_RX_UART_3          NINGUNO
#endif

#ifndef PIN_TX_UART_3
  #define PIN_TX_UART_3          NINGUNO
#endif

#ifndef DMA_TX_UART_3
  #define DMA_TX_UART_3          0
#endif

#ifndef DMA_RX_UART_3
  #define DMA_RX_UART_3          0
#endif

#ifndef PIN_RX_UART_4
  #define PIN_RX_UART_4          NINGUNO
#endif

#ifndef PIN_TX_UART_4
  #define PIN_TX_UART_4          NINGUNO
#endif

#ifndef DMA_TX_UART_4
  #define DMA_TX_UART_4          0
#endif

#ifndef DMA_RX_UART_4
  #define DMA_RX_UART_4          0
#endif

#ifndef PIN_RX_UART_5
  #define PIN_RX_UART_5          NINGUNO
#endif

#ifndef PIN_TX_UART_5
  #define PIN_TX_UART_5          NINGUNO
#endif

#ifndef DMA_TX_UART_5
  #define DMA_TX_UART_5          0
#endif

#ifndef DMA_RX_UART_5
  #define DMA_RX_UART_5          0
#endif

#ifndef PIN_RX_UART_6
  #define PIN_RX_UART_6          NINGUNO
#endif

#ifndef PIN_TX_UART_6
  #define PIN_TX_UART_6          NINGUNO
#endif

#ifndef DMA_TX_UART_6
  #define DMA_TX_UART_6          0
#endif

#ifndef DMA_RX_UART_6
  #define DMA_RX_UART_6          0
#endif

#ifndef PIN_RX_UART_7
  #define PIN_RX_UART_7          NINGUNO
#endif

#ifndef PIN_TX_UART_7
  #define PIN_TX_UART_7          NINGUNO
#endif

#ifndef DMA_TX_UART_7
  #define DMA_TX_UART_7          0
#endif

#ifndef DMA_RX_UART_7
  #define DMA_RX_UART_7          0
#endif

#ifndef PIN_RX_UART_8
  #define PIN_RX_UART_8          NINGUNO
#endif

#ifndef PIN_TX_UART_8
  #define PIN_TX_UART_8          NINGUNO
#endif

#ifndef DMA_TX_UART_8
  #define DMA_TX_UART_8          0
#endif

#ifndef DMA_RX_UART_8
  #define DMA_RX_UART_8          0
#endif



/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configUART_t, NUM_MAX_UART, configUART, GP_CONFIGURACION_UART, 1);

static const configUART_t configUARTdefecto[] = {
    { DEFIO_TAG(PIN_TX_UART_1), DEFIO_TAG(PIN_RX_UART_1), USAR_DMA_DRIVER_UART, DMA_TX_UART_1, DMA_RX_UART_1},
    { DEFIO_TAG(PIN_TX_UART_2), DEFIO_TAG(PIN_RX_UART_2), USAR_DMA_DRIVER_UART, DMA_TX_UART_2, DMA_RX_UART_2},
    { DEFIO_TAG(PIN_TX_UART_3), DEFIO_TAG(PIN_RX_UART_3), USAR_DMA_DRIVER_UART, DMA_TX_UART_3, DMA_RX_UART_3},
    { DEFIO_TAG(PIN_TX_UART_4), DEFIO_TAG(PIN_RX_UART_4), USAR_DMA_DRIVER_UART, DMA_TX_UART_4, DMA_RX_UART_4},
    { DEFIO_TAG(PIN_TX_UART_5), DEFIO_TAG(PIN_RX_UART_5), USAR_DMA_DRIVER_UART, DMA_TX_UART_5, DMA_RX_UART_5},
    { DEFIO_TAG(PIN_TX_UART_6), DEFIO_TAG(PIN_RX_UART_6), USAR_DMA_DRIVER_UART, DMA_TX_UART_6, DMA_RX_UART_6},
#if defined(STM32F767xx)
    { DEFIO_TAG(PIN_TX_UART_7), DEFIO_TAG(PIN_RX_UART_7), USAR_DMA_DRIVER_UART, DMA_TX_UART_7, DMA_RX_UART_7},
    { DEFIO_TAG(PIN_TX_UART_8), DEFIO_TAG(PIN_RX_UART_8), USAR_DMA_DRIVER_UART, DMA_TX_UART_8, DMA_RX_UART_8},
#endif
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void fnResetGP_configUART(configUART_t *configUART)
**  Descripcion:    Funcion de reset de la configuracion de la UART
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configUART(configUART_t *configUART)
{
    for (uint8_t i = 0; i < NUM_MAX_UART; i++) {
        configUART[i].pinTx = configUARTdefecto[i].pinTx;
        configUART[i].pinRx = configUARTdefecto[i].pinRx;
        configUART[i].usarDMA = configUARTdefecto[i].usarDMA;
        configUART[i].dmaTx = configUARTdefecto[i].dmaTx;
        configUART[i].dmaRx = configUARTdefecto[i].dmaRx;
    }
}

#endif
