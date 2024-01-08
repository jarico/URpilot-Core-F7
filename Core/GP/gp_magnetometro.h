/***************************************************************************************
**  gp_magnetometro.h - Funciones y grupo de parametros del magnetometro
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 13/06/2020
**  Fecha de modificacion: 13/09/2020
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

#ifndef __GP_MAGNETOMETRO_H
#define __GP_MAGNETOMETRO_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Sensores/Magnetometro/magnetometro.h"
#include "gp.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FREC_ACTUALIZAR_MAG_HZ      100
#define FREC_LEER_MAG_HZ            50
#define LEER_MAG_SCHEDULER              // El sheduler se encarga de llamar a la lectura del sensor. Sino lo hace otra funcion


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    tipoMag_e tipoMag;
    bool auxiliar;
    tipoBus_e bus;
    uint8_t dispBus;
    uint8_t csSPI;
    uint8_t dirI2C;
    uint8_t drdy;
    uint8_t rangoFiltro;
    rotacionSensor_t rotacion;
    uint16_t frecActualizar;
    uint16_t frecLeer;
} configMag_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_ARRAY_GP(configMag_t, NUM_MAX_MAG, configMag);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_MAGNETOMETRO_H
