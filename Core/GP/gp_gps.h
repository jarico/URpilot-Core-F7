/***************************************************************************************
**  gp_gps.h - Funciones y grupo de parametros del GPS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 25/06/2020
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

#ifndef __GP_GPS_H
#define __GP_GPS_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Sensores/GPS/gps.h"
#include "gp.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FREC_LEER_GPS_HZ            50
#define LEER_GPS_SCHEDULER              // El sheduler se encarga de llamar a la lectura del sensor. Sino lo hace otra funcion


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    tipoGPS_e tipoGPS;
    bool auxiliar;
    int8_t dispUART;
    modoSBAS_e modoSBAS;
    modoConf_e modoConf;
    int8_t elevacionMin;
    configGNSS_e gnss;
    uint16_t periodoMuestreo;
    uint16_t frecLeer;
} configGPS_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_ARRAY_GP(configGPS_t, NUM_MAX_GPS, configGPS);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_GPS_H
