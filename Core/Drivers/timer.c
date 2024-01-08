/***************************************************************************************
**  timer.c - Funciones generales de los Timer
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 17/08/2019
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

#include "timer.h"

#ifdef USAR_TIMERS
#include "Core/led_estado.h"

/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static tim_t tim[NUM_CANALES_TIMER];
static timerHAL_t timer[NUM_TIMERS];
static uint8_t cntTimer = 0;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         tim_t *punteroTim(uint8_t numTim)
**  Descripcion:    Devuelve el puntero a un tim seleccionado
**  Parametros:     Dispositivo a devolver
**  Retorno:        Puntero
****************************************************************************************/
tim_t *punteroTim(uint8_t numTim)
{
    return &tim[numTim];
}


/***************************************************************************************
**  Nombre:         timerHAL_t *punteroTimer(numTimer_e numTimer)
**  Descripcion:    Devuelve el puntero a un timer seleccionado
**  Parametros:     Dispositivo a devolver
**  Retorno:        Puntero
****************************************************************************************/
timerHAL_t *punteroTimer(numTimer_e numTimer)
{
    return &timer[numTimer];
}


/***************************************************************************************
**  Nombre:         bool iniciarTimer(numTimer_e numTimer, uint8_t pin, usoTimer_e uso)
**  Descripcion:    Inicia el driver del Timer
**  Parametros:     Timer a iniciar, pin del timer, uso que se va a dar
**  Retorno:        True si OK
****************************************************************************************/
bool iniciarTimer(numTimer_e numTimer, uint8_t pin, usoTimer_e uso)
{
    if (numTimer == TIMER_NINGUNO) {
#ifdef DEBUG
        printf("Fallo en la definicion del timer\n");
#endif
        return false;
    }

    if (numTimer != TIMER_6 && numTimer != TIMER_7) {
	    if (cntTimer != 0) {
            for (uint8_t i = 0; i < cntTimer; i++) {
            	bool encontrado;
            	tim_t *timEnc = timerPorTag(pin, &encontrado);

            	if (encontrado && timEnc != NULL) {
    	            if (timerUsado(timEnc)) {
#ifdef DEBUG
    	                printf("Timer ya usado\n");
#endif
    	                return false;
                    }
            	}
            }
	    }

	    tim_t *driver = &tim[cntTimer];
        if (!driver->asignado) {
            if (asignarPinTimer(cntTimer, numTimer, pin)) {
                driver->asignado = true;
                driver->numTimer = numTimer;
                cntTimer++;
            }
            else {
#ifdef DEBUG
                printf("Fallo en la inicializacion de Tim %u\n", cntTimer + 1);
#endif
                return false;
            }
        }
    }

    if (!timer[numTimer].hal.asignado) {
        if (asignarHALtimer(numTimer)) {
            timer[numTimer].hal.asignado = true;
            timer[numTimer].uso = uso;
        }
        else {
    #ifdef DEBUG
            printf("Fallo en la inicializacion de Timer %u\n", numTimer + 1);
    #endif
            return false;
        }
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool timerUsado(tim_t *dTim)
**  Descripcion:    Comprueba si ese timer esta usado
**  Parametros:     Timer
**  Retorno:        Timer usado o no
****************************************************************************************/
bool timerUsado(tim_t *dTim)
{
    if (dTim == NULL)
        return true;

    if (!dTim->asignado)
        return false;
    else
        return true;
}


/***************************************************************************************
**  Nombre:         tim_t *timerPorTag(uint8_t tag, bool *encontrado)
**  Descripcion:    Obtiene el Timer dado el pin que lo utiliza
**  Parametros:     Pin, encontrado
**  Retorno:        Puntero al timer encontrado
****************************************************************************************/
tim_t *timerPorTag(uint8_t tag, bool *encontrado)
{
    if (tag == 0) {
    	*encontrado = false;
        return NULL;
    }

    for (uint8_t i = 0; i < NUM_CANALES_TIMER; i++) {
        if (tag == tim[i].pin.pin) {
        	*encontrado = true;
        	return &tim[i];
        }
    }

	*encontrado = false;
    return NULL;
}

#endif

