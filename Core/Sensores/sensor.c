/***************************************************************************************
**  sensor.c - Funciones generales para los sensores
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


/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <math.h>

#include "sensor.h"
#include "Comun/matematicas.h"
#include "Comun/util.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


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

/***************************************************************************************
**  Nombre:         void acumularLectura(acumulador_t *acumulador, float valor, uint8_t maxLecturas)
**  Descripcion:    Acumula las medidas
**  Parametros:     Acumulador, valor a acumular, numero maximo de acumulaciones
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void acumularLectura(acumulador_t *acumulador, float valor, uint8_t maxLecturas)
{
    acumulador->acumulado = acumulador->acumulado + valor;
    acumulador->contador++;

    if (acumulador->contador == maxLecturas) {
        acumulador->contador = maxLecturas / 2;
        acumulador->acumulado = acumulador->acumulado / 2;
    }
}


/***************************************************************************************
**  Nombre:         void acumularLecturas3(acumulador3_t *acumulador, float *valor, uint8_t maxLecturas)
**  Descripcion:    Acumula las medidas
**  Parametros:     Acumulador, valor a acumular, numero maximo de acumulaciones
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void acumularLecturas3(acumulador3_t *acumulador, float *valor, uint8_t maxLecturas)
{
    acumulador->acumulado[0] = acumulador->acumulado[0] + valor[0];
    acumulador->acumulado[1] = acumulador->acumulado[1] + valor[1];
    acumulador->acumulado[2] = acumulador->acumulado[2] + valor[2];
    acumulador->contador++;

    if (acumulador->contador == maxLecturas) {
        acumulador->contador = maxLecturas / 2;
        acumulador->acumulado[0] = acumulador->acumulado[0] / 2;
        acumulador->acumulado[1] = acumulador->acumulado[1] / 2;
        acumulador->acumulado[2] = acumulador->acumulado[2] / 2;
    }
}


/***************************************************************************************
**  Nombre:         void acumularLecturas7(acumulador7_t *acumulador, float *valor, uint8_t maxLecturas)
**  Descripcion:    Acumula las medidas
**  Parametros:     Acumulador, valor a acumular, numero maximo de acumulaciones
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void acumularLecturas7(acumulador7_t *acumulador, float *valor, uint8_t maxLecturas)
{
    acumulador->acumulado[0] = acumulador->acumulado[0] + valor[0];
    acumulador->acumulado[1] = acumulador->acumulado[1] + valor[1];
    acumulador->acumulado[2] = acumulador->acumulado[2] + valor[2];
    acumulador->acumulado[3] = acumulador->acumulado[3] + valor[3];
    acumulador->acumulado[4] = acumulador->acumulado[4] + valor[4];
    acumulador->acumulado[5] = acumulador->acumulado[5] + valor[5];
    acumulador->acumulado[6] = acumulador->acumulado[6] + valor[6];
    acumulador->contador++;

    if (acumulador->contador == maxLecturas) {
        acumulador->contador = maxLecturas / 2;
        acumulador->acumulado[0] = acumulador->acumulado[0] / 2;
        acumulador->acumulado[1] = acumulador->acumulado[1] / 2;
        acumulador->acumulado[2] = acumulador->acumulado[2] / 2;
        acumulador->acumulado[3] = acumulador->acumulado[3] / 2;
        acumulador->acumulado[4] = acumulador->acumulado[4] / 2;
        acumulador->acumulado[5] = acumulador->acumulado[5] / 2;
        acumulador->acumulado[6] = acumulador->acumulado[6] / 2;
    }
}
