/***************************************************************************************
**  fc.c - Funciones generales de la controladora de vuelo
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/09/2019
**  Fecha de modificacion: 19/09/2020
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
#include "fc.h"
#include "GP/gp_fc.h"
#include "Sensores/IMU/imu.h"
#include "AHRS/ahrs.h"
#include "control.h"
#include "control_ModelMatching.h"
#include "mixer.h"
#include "Scheduler/scheduler.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
//#define MODEL_MATCHING

/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


/***************************************************************************************
**  Nombre:         bool iniciarFC(void)
**  Descripcion:    Inicia la controladora de vuelo
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool iniciarFC(void)
{
	// Ajustamos las frecuencias del scheduler
    ajustarFrecuenciaEjecucionTarea(TAREA_ACTUALIZAR_VEL_ANGULAR_FC, PERIODO_TAREA_HZ_SCHEDULER(configFC()->frecLazoVelAngular));
    ajustarFrecuenciaEjecucionTarea(TAREA_ACTUALIZAR_ACTITUD_FC, PERIODO_TAREA_HZ_SCHEDULER(configFC()->frecLazoActitud));
    ajustarFrecuenciaEjecucionTarea(TAREA_ACTUALIZAR_POSICION_FC, PERIODO_TAREA_HZ_SCHEDULER(configFC()->frecLazoPosicion));

#ifdef MODEL_MATCHING
    iniciarControladoresMM();
#else
    iniciarControladores();
#endif
    return true;
}

#include "Drivers/tiempo.h"
/***************************************************************************************
**  Nombre:         void actualizarLazoVelAngularFC(uint32_t tiempoActual)
**  Descripcion:    Actualiza el bucle del control de velocidad angular
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void actualizarLazoVelAngularFC(uint32_t tiempoActual)
{
    #ifdef MODEL_MATCHING
	    actualizarControlVelAngularMM();
    #else
	    actualizarControlVelAngular();
    #endif
    actualizarMixer();
}

extern reaction_t reaction;
extern reaction_t reaction;
extern uint32_t numReaction;
extern bool cambiarID;
extern bool iniR;

extern uint32_t rTimeMax;
extern uint32_t rTimeMin;
extern uint32_t rTime1;

/***************************************************************************************
**  Nombre:         void actualizarLazoActitudFC(uint32_t tiempoActual)
**  Descripcion:    Actualiza el lazo de orientacion
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarLazoActitudFC(uint32_t tiempoActual)
{

UNUSED(tiempoActual);

    actualizarActitudAHRS();

    #ifdef MODEL_MATCHING
        actualizarControlActitudMM();
    #else
        actualizarControlActitud();
    #endif

    if (reaction.id == 'A' && iniR) {
    	reaction.id = 'B';
    }
}


/***************************************************************************************
**  Nombre:         void actualizarLazoPosicionFC(uint32_t tiempoActual)
**  Descripcion:    Actualiza el lazo de posicion
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarLazoPosicionFC(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    actualizarPosicionAHRS();
}



