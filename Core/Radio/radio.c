/***************************************************************************************
**  rc.c - Funciones relativas al sistema receptor de radio
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/07/2019
**  Fecha de modificacion: 20/09/2020
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

#include "radio.h"

#ifdef USAR_RADIO
#include "GP/gp_radio.h"
#include "Drivers/uart.h"
#include "Scheduler/scheduler.h"
#include "Comun/matematicas.h"
#include "Drivers/tiempo.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
// Timeouts
#define TIMEOUT_MEDIDA_RADIO      100000     // timeout en us desde la ultima lectura


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static radio_t radio;
static tablaFnRadio_t *tablaFnRadio;
static bool failsafeExtRadio = false;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarDriverRadio(void);
bool nuevaRecepcionRadio(void);
void actualizarFailsafeRadio(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarRadio(void)
**  Descripcion:    Inicia la radio
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarRadio(void)
{
    // Reseteamos las variables de la radio
    memset(&radio, 0, sizeof(radio_t));

    for (uint8_t i = 0; i < NUM_MAX_CANALES_RADIO; i++)
    	radio.canales[i] = VALOR_MEDIO_RADIO;

    switch (configRadio()->protocolo) {
#ifdef USAR_RADIO_PPM
        case RX_PPM:
        	tablaFnRadio = &tablaFnRadioPPM;
            ajustarFrecuenciaEjecucionTarea(TAREA_LEER_RADIO, PERIODO_TAREA_HZ_SCHEDULER(50));
            break;
#endif
#ifdef USAR_RADIO_UART
        case RX_IBUS:
        	tablaFnRadio = &tablaFnRadioIBUS;
            ajustarFrecuenciaEjecucionTarea(TAREA_LEER_RADIO, PERIODO_TAREA_HZ_SCHEDULER(configRadio()->frecLeer));
            break;

        case RX_SBUS:
        	tablaFnRadio = &tablaFnRadioSBUS;
            ajustarFrecuenciaEjecucionTarea(TAREA_LEER_RADIO, PERIODO_TAREA_HZ_SCHEDULER(configRadio()->frecLeer));
            break;
#endif
        default:
#ifdef DEBUG
            printf("Fallo en la definicion del protocolo de la Radio\n");
#endif
            return false;
            break;
    }

    if (iniciarDriverRadio())
        radio.iniciada = true;
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion de la Radio\n");
#endif
    	radio.iniciada = false;
    }

    return radio.iniciada;
}


/***************************************************************************************
**  Nombre:         bool iniciarDriverRadio(void)
**  Descripcion:    Inicia el protocolo de la radio
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverRadio(void)
{
    if (tablaFnRadio->iniciarRadio())
        return true;

    return false;
}


#include "Sensores/IMU/imu.h"
#include "Drivers/tiempo.h"
extern reaction_t reaction;
extern uint32_t rTimeMax;
extern uint32_t rTimeMin;
extern uint32_t rTime1;
extern bool cambiarID;
extern bool iniR;

/***************************************************************************************
**  Nombre:         void leerRadio(uint32_t tiempoActual)
**  Descripcion:    Lee la radio y actualiza los canales
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void leerRadio(uint32_t tiempoActual)
{


    if (reaction.id == 'C') {
    	cambiarID = true;

		rTime1 = micros() - reaction.tiempo;
		if (rTime1 > rTimeMax)
			rTimeMax = rTime1;

		if (rTime1 < rTimeMin)
			rTimeMin = rTime1;
    }



    radio.nuevaEntrada = false;

    if (radio.iniciada) {
    	failsafeExtRadio = false;
        tablaFnRadio->leerRadio(tiempoActual);

        if (nuevaRecepcionRadio()) {
            radio.nuevaEntrada = true;
            radio.ultimaMedida = tiempoActual;
        }
    }

    actualizarFailsafeRadio();
}


/***************************************************************************************
**  Nombre:         void anadirRecepcionRadio(uint8_t numValores, uint16_t *valores)
**  Descripcion:    Anade una nueva recepcion de datos de radio
**  Parametros:     Numero de canales recibidos, valores de los canales, failsafe
**  Retorno:        Ninguno
****************************************************************************************/
void anadirRecepcionRadio(uint8_t numValores, uint16_t *valores)
{
    numValores = MIN(numValores, NUM_MAX_CANALES_RADIO);
    memcpy(radio.canales, valores, numValores * sizeof(uint16_t));

    radio.numCanales = numValores;
    radio.contadorEntradas++;
}


/***************************************************************************************
**  Nombre:         bool nuevaRecepcionRadio(void)
**  Descripcion:    Comprueba si ha habido una nueva recepcion de datos
**  Parametros:     Ninguno
**  Retorno:        True si se han recibido datos de radio
****************************************************************************************/
bool nuevaRecepcionRadio(void)
{
    bool recepcion = radio.contadorEntradas != radio.ultimoContadorEntradas;
    if (recepcion)
        radio.ultimoContadorEntradas = radio.contadorEntradas;

    return recepcion;
}


/***************************************************************************************
**  Nombre:         void actualizarFailsafeRadio(void)
**  Descripcion:    Actualiza la bandera del failsafe
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFailsafeRadio(void)
{
    if (!radio.iniciada || failsafeExtRadio) {   // Failsafe activado externamente
        radio.failsafe = true;
        return;
    }

    // Se considera que la radio esta operativa si se ha actualizado en X tiempo,
    const uint32_t tiempo = micros();
    radio.failsafe = (tiempo - radio.ultimaMedida) > TIMEOUT_MEDIDA_RADIO;

    if (radio.failsafe)
        return;

    if (radio.nuevaEntrada) {
    	radio.failsafe = false;

        for (uint8_t i = 0; i < radio.numCanales; i++) {
    	    if (radio.canales[i] < configRadio()->minFailsafe || radio.canales[i] > configRadio()->maxFailsafe) {
                radio.failsafe = true;
                return;
    	    }
        }
    }
}


/***************************************************************************************
**  Nombre:         void radioOperativa(void)
**  Descripcion:    Comprueba si la radio esta operativa
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool radioOperativa(void)
{
    return radio.iniciada && !radio.failsafe;
}


/***************************************************************************************
**  Nombre:         bool radioEnFailsafe(void)
**  Descripcion:    Comprueba si la radio esta en failsafe
**  Parametros:     Ninguno
**  Retorno:        True si esta
****************************************************************************************/
bool radioEnFailsafe(void)
{
    return radio.failsafe;
}


/***************************************************************************************
**  Nombre:         void activarFailsafeRadio(void)
**  Descripcion:    Activa el failsafe de la radio
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void activarFailsafeRadio(void)
{
	failsafeExtRadio = true;
}


/***************************************************************************************
**  Nombre:         void desactivarFailsafeRadio(void)
**  Descripcion:    Desactiva el failsafe de la radio
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void desactivarFailsafeRadio(void)
{
	failsafeExtRadio = false;
}


/***************************************************************************************
**  Nombre:         bool nuevaEntradaRadioValida(void)
**  Descripcion:    Comprueba si hay una entrada de radio valida
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool nuevaEntradaRadioValida(void)
{
    return radio.nuevaEntrada && !radio.failsafe;
}


/***************************************************************************************
**  Nombre:         uint16_t canalRadio(uint8_t canal)
**  Descripcion:    Devuelve el valor de un canal
**  Parametros:     Ninguno
**  Retorno:        Valor del canal
****************************************************************************************/
uint16_t canalRadio(uint8_t canal)
{
    if (canal < NUM_MAX_CANALES_RADIO)
        return radio.canales[canal];
    else
        return 0;
}


/***************************************************************************************
**  Nombre:         void canalesRadio(uint16_t *canales)
**  Descripcion:    Devuelve el valor de todos los canales
**  Parametros:     Puntero al array de canales
**  Retorno:        Ninguno
****************************************************************************************/
void canalesRadio(uint16_t *canales)
{
    canales = radio.canales;
}

#endif
