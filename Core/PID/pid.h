/***************************************************************************************
**  pid.h - Funciones relativas al los PID
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

#ifndef __PID_H_
#define __PID_H_

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
typedef struct {
    float kp;
    float ki;
    float kd;
    float kff;
    float limIntegral;
    float limSalida;
} paramPID_t;

typedef struct {
	paramPID_t p;
    float integral;
    float u;           // Accion de control
} pid_t;



/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarPID(pid_t *pid, float kp, float ki, float kd, float kff, float limIntegral, float limSalida);
float actualizarPID(pid_t *pid, float setPoint, float sensor, float sensorDerivada, float dt, bool habIntegral);
void resetearIntegralPID(pid_t *pid);


#endif // __PID_H_
