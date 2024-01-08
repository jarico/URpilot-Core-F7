/***************************************************************************************
**  gp_calibrador.h - Funciones y grupo de parametros de los calibradores
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 18/05/2021
**  Fecha de modificacion: 18/05/2021
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

#ifndef __GP_CALIBRADOR_H
#define __GP_CALIBRADOR_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Sensores/IMU/imu.h"
#include "Sensores/Magnetometro/magnetometro.h"
#include "gp.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FREC_ACTUALIZAR_CALIBRADOR_IMU_HZ    20
#define FREC_ACTUALIZAR_CALIBRADOR_MAG_HZ    20


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    calIMU_t calIMU;
} configCalIMU_t;

typedef struct {
    calParamMag_t calMag;
} configCalMag_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_ARRAY_GP(configCalIMU_t, NUM_MAX_IMU, configCalIMU);
DECLARAR_ARRAY_GP(configCalMag_t, NUM_MAX_MAG, configCalMag);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_CALIBRADOR_H
