/***************************************************************************************
**  inicializacion.h - Funcion de inicializacion de la placa
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

#ifndef __INICIALIZACION_H
#define __INICIALIZACION_H

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
    ESTADO_SIS_INICIALIZANDO      = 0,
	ESTADO_SIS_CONFIG_CARGADA     = (1 << 0),
	ESTADO_SIS_DRIVERS_READY      = (1 << 1),
	ESTADO_SIS_PERIFERICOS_READY  = (1 << 2),
	ESTADO_SIS_FC_READY           = (1 << 3),
	ESTADO_SIS_SCHEDULER_READY    = (1 << 4),
	ESTADO_SIS_READY              = (1 << 5),
} estadoSistema_e;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarPlaca(void);

#endif // __INICIALIZACION_H
