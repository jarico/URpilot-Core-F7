/***************************************************************************************
**  gp_ahrs.h - Funciones y grupo de parametros del AHRS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/09/2020
**  Fecha de modificacion: 06/10/2020
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

#ifndef __GP_AHRS_H
#define __GP_AHRS_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "AHRS/ahrs.h"
#include "Radio/radio.h"
#include "gp.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    float kpIni;        // Ganancia inicial para encontrar convergencia rapidamente
    float kiIni;
    float kp;
    float ki;
    float kpMarg;
    float kiMarg;
} configMahony_t;

typedef struct {
    float betaIni;      // Beta inicial para encontrar convergencia rapidamente
    float beta;
    float zeta;
    float betaMarg;
    float zetaMarg;
} configMadgwick_t;

typedef struct {
	ahrs_e filtro;
	bool habilitarMag;
	uint16_t fecFiltroAcelAng;
    float kFC;
    configMahony_t mahony;
    configMadgwick_t madgwick;
} configAHRS_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_GP(configAHRS_t, configAHRS);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_AHRS_H
