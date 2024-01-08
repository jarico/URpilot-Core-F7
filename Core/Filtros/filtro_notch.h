/***************************************************************************************
**  filtro_notch.h - Funciones del filtro Notch
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 26/05/2020
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

#ifndef __FILTRO_NOTCH_H
#define __FILTRO_NOTCH_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_ARMONICOS_FILTRO_NOTCH        8


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool operativo;
    float frecCentral;
    float frecMuestreo;
    float anchoBandaHz;
    float atenuacionDB;
    float b0, b1, b2, a1, a2, a0Inv;
    float notchSen, notchSen1, notchSen2, senal1, senal2;
    float valor;
} filtroNotch_t;

typedef struct {
    bool operativo;
    float frecCentral;
    float frecMuestreo;
    float anchoBandaHz;
    float atenuacionDB;
    float A, Q;
    filtroNotch_t filtros[NUM_MAX_ARMONICOS_FILTRO_NOTCH];
    uint8_t filtrosHabilitados;
    uint8_t armonicos;
    float valor;
} filtroNotchArmonicos_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void ajustarFiltroNotch(filtroNotch_t *filtro, float frecCentral, float frecMuestreo, float anchoBandaHz, float atenuacionDB);
void actualizarFrecFiltroNotch(filtroNotch_t *filtro, float frecCentral);
void resetearFiltroNotch(filtroNotch_t *filtro);
float actualizarFiltroNotch(filtroNotch_t *filtro, float muestra);

void ajustarFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float frecCentral, float frecMuestreo, float anchoBandaHz, float atenuacionDB, uint8_t armonicos);
void actualizarFrecFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float frecCentral);
void resetearFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro);
float actualizarFiltroNotchArmonicos(filtroNotchArmonicos_t *filtro, float muestra);

#endif // __FILTRO_NOTCH_H
