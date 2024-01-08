/***************************************************************************************
**  uart.c - Funciones generales de la UART
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdio.h>
#include <string.h>

#include "uart.h"

#ifdef USAR_UART


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static uart_t uart[NUM_MAX_UART];
static configIniUART_t configuracionUART[NUM_MAX_UART];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         uart_t *punteroUART(numUART_e numUART)
**  Descripcion:    Devuelve el puntero a una UART seleccionada
**  Parametros:     Dispositivo a devolver
**  Retorno:        Puntero
****************************************************************************************/
uart_t *punteroUART(numUART_e numUART)
{
    return &uart[numUART];
}


/***************************************************************************************
**  Nombre:         bool iniciarUART(numUART_e numUART, configIniUART_t configInicial, uartRxCallback rxCall)
**  Descripcion:    Inicia la UART
**  Parametros:     Dispositivo a iniciar, configuracion de la UART, callback de recepcion
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarUART(numUART_e numUART, configIniUART_t configInicial, uartRxCallback rxCall)
{
    if (numUART == UART_NINGUNO) {
#ifdef DEBUG
        printf("Fallo en la definicion de la UART\n");
#endif
        return false;
    }

    uart_t *driver = &uart[numUART];

	memset(driver, 0, sizeof(*driver));
    resetearContadorErrorUART(numUART);
    driver->iniciado = false;
    driver->rxCallback = rxCall;

    // Cargamos la configuracion inicial
    configuracionUART[numUART] = configInicial;

    if (iniciarDriverUART(numUART, configuracionUART[numUART])) {
    	driver->iniciado = true;
        return true;
    }
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion de la UART %u\n", numUART + 1);
#endif
        return false;
    }
}


/***************************************************************************************
**  Nombre:         bool uartIniciada(numUART_e numUART)
**  Descripcion:    Comprueba si la UART esta iniciada
**  Parametros:     Numero de UART
**  Retorno:        True si iniciada
****************************************************************************************/
bool uartIniciada(numUART_e numUART)
{
    return uart[numUART].iniciado;
}


/***************************************************************************************
**  Nombre:         void ajustarBaudRateUART(numUART_e numUART, uint32_t baudrate)
**  Descripcion:    Asigna el Baudrate
**  Parametros:     Dispositivo, baudrate
**  Retorno:        Ninguno
****************************************************************************************/
bool ajustarBaudRateUART(numUART_e numUART, uint32_t baudrate)
{
	uart_t *driver = &uart[numUART];

    // Resetea el Buffer
    driver->cabezaRxBuffer = 0;
    driver->colaRxBuffer = 0;
    driver->cabezaTxBuffer = 0;
    driver->colaTxBuffer = 0;

    configuracionUART[numUART].baudrate = baudrate;
    return iniciarDriverUART(numUART, configuracionUART[numUART]);
}


/***************************************************************************************
**  Nombre:         void errorCallbackUART(numUART_e numUART)
**  Descripcion:    Incrementa el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void errorCallbackUART(numUART_e numUART)
{
    uart[numUART].numErrores++;
}


/***************************************************************************************
**  Nombre:         uint16_t contadorErrorUART(numUART_e numUART)
**  Descripcion:    Devuelve el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Retorna el numero de errores
****************************************************************************************/
uint16_t contadorErrorUART(numUART_e numUART)
{
    return uart[numUART].numErrores;
}


/***************************************************************************************
**  Nombre:         void resetearContadorErrorUART(numUART_e numUART)
**  Descripcion:    Resetea el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void resetearContadorErrorUART(numUART_e numUART)
{
    uart[numUART].numErrores = 0;
}

#endif
