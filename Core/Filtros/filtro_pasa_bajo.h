/***************************************************************************************
**  filtro_pasa_bajo.h - Funciones del filtro pasa bajo
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 25/05/2020
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

#ifndef __FILTRO_PASA_BAJO_H
#define __FILTRO_PASA_BAJO_H

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
    float frecCorte;
    float frecMuestreo;
    float alpha;
    float valor;
} filtroPasaBajo_t;

typedef struct {
    bool operativo;
    float frecCorte;
    float frecMuestreo;
    float a1, a2, b0, b1, b2;
    float elemRetardo1;
    float elemRetardo2;
    float valor;
} filtroPasaBajo2P_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void ajustarFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte, float frecMuestreo);
void actualizarFrecFiltroPasaBajo(filtroPasaBajo_t *filtro, float frecCorte);
void resetearFiltroPasaBajo(filtroPasaBajo_t *filtro);
float actualizarFiltroPasaBajo(filtroPasaBajo_t *filtro, float muestra);

void ajustarFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte, float frecMuestreo);
void actualizarFrecFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float frecCorte);
void resetearFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro);
float actualizarFiltroPasaBajo2P(filtroPasaBajo2P_t *filtro, float muestra);

#endif // __FILTRO_PASA_BAJO_H
