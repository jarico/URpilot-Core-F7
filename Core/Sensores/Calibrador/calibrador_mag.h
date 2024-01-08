/***************************************************************************************
**  calibrador_mag.h - Funciones y variables comunes a la calibracion del magnetometro
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/04/2021
**  Fecha de modificacion: 10/04/2021
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

#ifndef __CALIBRADOR_MAG_H_
#define __CALIBRADOR_MAG_H_

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
    float radio;
	float offset[3];
    float diag[3];
    float offDiag[3];
} calParamMag_t;



/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarCalMag(void);
void terminarCalMag(void);
void actualizarCalMag(uint32_t tiempoActual);
bool calibracionMagExitosa(uint8_t numCal);
calParamMag_t parametrosCalMAg(uint8_t numCal);

#endif // __CALIBRADOR_MAG_H_
