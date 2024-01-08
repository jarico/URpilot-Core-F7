/***************************************************************************************
**  matematicas.h - Funciones generales de matematicas
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2019
**  Fecha de modificacion: 23/09/2020
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
**  v1.1  Ramon Rico. Se ha anadido la funcion limitarUint16
**
****************************************************************************************/

#ifndef __MATEMATICAS_H
#define __MATEMATICAS_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "matriz.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef sq
  #define sq(x)     ((x)*(x))
#endif

#define power3(x)   ((x)*(x)*(x))

#define M_PIf       3.14159265358979323846f

#define MIN(a,b) \
                    __extension__ ({ __typeof__ (a) _a = (a); \
                    __typeof__ (b) _b = (b); \
                    _a < _b ? _a : _b; })

#define MAX(a,b) \
                    __extension__ ({ __typeof__ (a) _a = (a); \
                    __typeof__ (b) _b = (b); \
                    _a > _b ? _a : _b; })

#define ABS(x) \
                    __extension__ ({ __typeof__ (x) _x = (x); \
                    _x > 0 ? _x : -_x; })


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
uint16_t limitarUint16(uint16_t valor, uint16_t bajo, uint16_t alto);
int32_t limitarInt32(int32_t valor, int32_t bajo, int32_t alto);
float limitarFloat(float valor, float bajo, float alto);
float moduloVector2(float *vector);
float moduloVector3(float *vector);
void productoCruzado3F(float *vector1, float *vector2, float *resultado);
float envolverInt360(const int32_t angulo, float unidadMod);
float envolverFloat360(const float angulo, float unidadMod);
bool esPotenciaDeDos(uint32_t x);
void normalizar3Array(float *a);
void normalizar4Array(float *a);
uint16_t generarNumeroAleatorioUint16(void);

#endif // __MATEMATICAS_H
