/***************************************************************************************
**  fallo_sistema.h - Funciones para fallos de sistema
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/05/2019
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

#ifndef __FALLO_SISTEMA_H
#define __FALLO_SISTEMA_H

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
    FALLO_CONTENIDO_FLASH_CONFIG_INVALIDO = 1,
    FALLO_ESCRITURA_FLASH,
#ifdef USAR_POWER_MODULE
	FALLO_INICIAR_POWER_MODULE,
#endif
#ifdef USAR_SD
	FALLO_INICIAR_SD,
#endif
#ifdef USAR_IMU
    FALLO_INICIAR_IMU,
#endif
#ifdef USAR_BARO
    FALLO_INICIAR_BARO,
#endif
#ifdef USAR_MAG
    FALLO_INICIAR_MAG,
#endif
#ifdef USAR_GPS
    FALLO_INICIAR_GPS,
#endif
#ifdef USAR_RADIO
    FALLO_INICIAR_RADIO,
#endif
#ifdef USAR_MOTORES
	FALLO_INICIAR_MOTORES,
#endif
} falloSistema_e;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void falloSistema(falloSistema_e fallo);

#endif // __FALLO_SISTEMA_H
