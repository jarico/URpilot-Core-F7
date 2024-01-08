/***************************************************************************************
**  rc.h - Funciones relativas al sistema de referencia de la Radio
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/08/2020
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

#ifndef __RC_H_
#define __RC_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    CANAL_NINGUNO = -1,
    CANAL1        =  0,
	CANAL2,
	CANAL3,
	CANAL4,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
} canalRC_e;

typedef enum {
	MODO_NINGUNO = -1,
    STABILIZE    =  0,
    ALT_HOLD,
    LOITER,
	AUTO,
    RTL,
	CNT_MODOS = RTL,
} modo_e;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float altura;
    modo_e modo;
    bool eStop;
} rc_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarRC(void);
void actualizarRC(uint32_t tiempoActual);
void activarEstopRC(void);
void desactivarEstopRC(void);
bool sistemaEnEStop(void);
uint8_t modoRC(void);
void refAngulosRC(float *ref);
float refAlturaRC(void);

#endif // __RC_H_
