/***************************************************************************************
**  sensor.h - Funciones generales para los sensores
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

#ifndef __SENSOR_H
#define __SENSOR_H

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
    float acumulado;
    uint8_t contador;
} acumulador_t;

typedef struct {
    float acumulado[3];
    uint8_t contador;
} acumulador3_t;

typedef struct {
    float acumulado[7];
    uint8_t contador;
} acumulador7_t;

typedef struct {
    int16_t rotacion;
    bool volteado;
} rotacionSensor_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void acumularLectura(acumulador_t *acumulador, float valor, uint8_t maxLecturas);
void acumularLecturas3(acumulador3_t *acumulador, float *valor, uint8_t maxLecturas);
void acumularLecturas7(acumulador7_t *acumulador, float *valor, uint8_t maxLecturas);

#endif // __SENSOR_H
