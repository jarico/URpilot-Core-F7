/***************************************************************************************
**  stack.c - Funciones de chequeo del stack
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/06/2019
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "stack.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define CARACTER_LLENO_STACK     0xA5


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern char _estack;            // Fin del stack declarado en el linker
extern char _Min_Stack_Size;    // Declarado en el linker
static uint32_t stackUsado;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/*
 * Los procesadores ARM usan un stack descendiente. Esto significa que el puntero al stack mantiene la direccion 
 * del ultimo item en memoria. Cuando el procesador publica un nuevo item en el stack, decrementa el puntero y
 * escribe el nuevo item en la nueva zona de memoria.
 */

/***************************************************************************************
**  Nombre:         void chequearStack(uint32_t tiempoActual)
**  Descripcion:    Comprueba que hay suficiente memoria para el stack
**  Parametros:     Tiempo actual (no se usa)
**  Retorno:        Ninguno
****************************************************************************************/
void chequearStack(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    char * const memAlta = &_estack;
    const uint32_t tam = (uint32_t)&_Min_Stack_Size;
    char * const memBaja = memAlta - tam;
    const char * const stackActual = (char *)&memBaja;

    char *p;
    for (p = memBaja; p < stackActual; ++p) {
        if (*p != CARACTER_LLENO_STACK)
            break;
    }

    stackUsado = (uint32_t)memAlta - (uint32_t)p;
}


/***************************************************************************************
**  Nombre:         uint32_t tamUsadoStack(void)
**  Descripcion:    Devuelve el tamanio del stack usado
**  Parametros:     Ninguno
**  Retorno:        Stack usado
****************************************************************************************/
uint32_t tamUsadoStack(void)
{
    return stackUsado;
}


/***************************************************************************************
**  Nombre:         uint32_t tamanioStack(void)
**  Descripcion:    Devuelve el tamanio del stack
**  Parametros:     Ninguno
**  Retorno:        Tamanio del stack
****************************************************************************************/
uint32_t tamanioStack(void)
{
    return (uint32_t)(intptr_t)&_Min_Stack_Size;
}

