/***************************************************************************************
**  matriz.h - Funciones generales de matrices
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 30/03/2021
**  Fecha de modificacion: 30/03/2021
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

#ifndef __MATRIZ_H
#define __MATRIZ_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_DIM_MATRIZ     10


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	float m[NUM_MAX_DIM_MATRIZ][NUM_MAX_DIM_MATRIZ];
} matriz_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void sumarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim);
void restarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim);
void multiplicarEscalarMatriz(matriz_t A, float B, matriz_t *R, uint8_t dim);
void multiplicarMatrices(matriz_t A, matriz_t B, matriz_t *R, uint8_t dim);
void traspuestaMatriz(matriz_t A, matriz_t *R, uint8_t dim);
bool inversaMatriz(matriz_t A, matriz_t *R, uint8_t dim);
void copiarMatriz(matriz_t A, matriz_t *B, uint8_t dim);
void resetearMatriz(matriz_t *M, uint8_t dim);
void asignarIdentidadMatriz(matriz_t *M, uint8_t dim);

#endif // __MATRIZ_H
