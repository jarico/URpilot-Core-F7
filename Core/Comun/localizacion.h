/***************************************************************************************
**  localizacion.h - Funciones de tratamiento de la localizacion GPS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 28/12/2020
**  Fecha de modificacion: 28/12/2020
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

#ifndef __LOCALIZACION_H
#define __LOCALIZACION_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    int32_t altitud;                     // Altitud en cm
    int32_t latitud;                     // Latitud en grados * 10.000.000
    int32_t longitud;                    // Longitud en grados * 10.000.000
} localizacion_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void distanciaNE(localizacion_t ne1, localizacion_t ne2, float *dist);
void distanciaNED(localizacion_t ne1, localizacion_t ne2, float *dist);
void anadirOffsetLoc(float ofsNorte, float ofsEste, float ofsAlt, localizacion_t *loc);

#endif // __LOCALIZACION_H
