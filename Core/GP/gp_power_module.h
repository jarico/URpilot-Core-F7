/***************************************************************************************
**  gp_power_module.h - Funciones y grupo de parametros del Power Module
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/12/2020
**  Fecha de modificacion: 11/12/2020
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

#ifndef __GP_POWER_MODULE_H
#define __GP_POWER_MODULE_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Sensores/PM/power_module.h"
#include "gp.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FREC_LEER_POWER_MODULE_HZ                  10
#define FREC_ACTUALIZAR_POWER_MODULE_HZ            50


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    tipoPowerModule_e tipo;
    uint8_t multV;
    uint8_t multI;
    uint8_t dispBus;
    uint8_t dirI2C;
    uint16_t frecLeer;
    uint16_t frecActualizar;
} configPowerModule_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_ARRAY_GP(configPowerModule_t, NUM_MAX_POWER_MODULE, configPowerModule);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_POWER_MODULE_H
