/***************************************************************************************
**  atomico.h - Drivers para usar funciones atomicas
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2019
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

#ifndef __ATOMICO_H
#define __ATOMICO_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
/*
 * Ejecuta un bloque con un BASEPRI elevado (usando BASEPRI_MAX), devolviendo el valor del
 * BASEPRI a la salida.
 * La barrera de memoria se situa en el inicio y el fin. Se utiliza __unused__ attribute
 * para que no salte el CLang warning
 */
#define BLOQUE_ATOMICO(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (restaurarBasepriMem), __unused__)) = __get_BASEPRI(), \
                                  __ToDo = ajustarBasepriMax(prio); __ToDo ; __ToDo = 0 )


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/
/*
 * Funciones de manipulacion del BASEPRI
 * Las librerias solo implementan set_BASEPRI. El resto de funciones se tienen que
 * implementar aqui
 */

/***************************************************************************************
**  Nombre:         void restaurarBasepriMem(uint8_t *prio)
**  Descripcion:    Restaura el BASEPRI con barrera de memoria global
**  Parametros:     Prioridad
**  Retorno:        Ninguno
****************************************************************************************/
static inline void restaurarBasepriMem(uint8_t *prio)
{
    __set_BASEPRI(*prio);
}


/***************************************************************************************
**  Nombre:         uint8_t ajustarBasepriMax(uint8_t prio)
**  Descripcion:    Ajusta el BASEPRI al valor maximo
**  Parametros:     Prioridad
**  Retorno:        Retorna 1
****************************************************************************************/
static inline uint8_t ajustarBasepriMax(uint8_t prio)
{
    __set_BASEPRI_MAX(prio);
    return 1;
}


#endif // __ATOMICO_H
