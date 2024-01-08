/***************************************************************************************
**  filtro_derivada.h - Funciones del filtro derivada
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

#ifndef __FILTRO_DERIVADA_H
#define __FILTRO_DERIVADA_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TAM_MAX_FILTRO_DERIVADA      11


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    FILTRO_DERIVADA_GRADO_5  = 5,
    FILTRO_DERIVADA_GRADO_7  = 7,
    FILTRO_DERIVADA_GRADO_9  = 9,
    FILTRO_DERIVADA_GRADO_11 = 11,
} filtroDerivadaGrado_e;

typedef struct {
    uint8_t tamFiltro;
    uint8_t numMuestras;
    uint8_t indiceMuestra;
    float muestras[TAM_MAX_FILTRO_DERIVADA];
    uint32_t tiempos[TAM_MAX_FILTRO_DERIVADA];
    float ultimoValor;
    bool nuevoDato;
} filtroDerivada_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void ajustarFiltroDerivada(filtroDerivada_t *filtro, filtroDerivadaGrado_e gradoFiltro);
void resetearFiltroDerivada(filtroDerivada_t *filtro);
void actualizarFiltroDerivada(filtroDerivada_t *filtro, float muestra, uint32_t tiempoActual);
float derivarFiltroDerivada(filtroDerivada_t *filtro);

#endif // __FILTRO_DERIVADA_H
