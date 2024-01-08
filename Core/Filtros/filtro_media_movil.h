/***************************************************************************************
**  filtro_media_movil.h - Funciones del filtro de media movil
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/05/2020
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

#ifndef __FILTRO_MEDIA_MOVIL_H
#define __FILTRO_MEDIA_MOVIL_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TAM_MAX_FILTRO_MEDIA_MOVIL       16


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    float muestras[TAM_MAX_FILTRO_MEDIA_MOVIL];
    uint8_t tamFiltro;
    uint8_t numMuestras;
    uint8_t indiceMuestra;
} filtroMediaMovil_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void ajustarFiltroMediaMovil(filtroMediaMovil_t *filtro, uint8_t tamFiltro);
void resetearFiltroMediaMovil (filtroMediaMovil_t *filtro);
float actualizarFiltroMediaMovil(filtroMediaMovil_t *filtro, float muestra);

#endif // __FILTRO_MEDIA_MOVIL_H
