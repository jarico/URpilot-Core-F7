/***************************************************************************************
**  sd_estandar.c - Datos estandar de la tarjeta SD
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/09/2019
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
#include "sd_estandar.h"
#include "Comun/matematicas.h"


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
**  Nombre:         uint32_t leerCampoBitCSDsd(uint8_t *buffer, uint32_t indiceBit, uint32_t lonBit)
**  Descripcion:    Lee un campo de bits de un array de bits (el bit en el indice 0 es el bit mas
**                  significativo del primer byte en el buffer).
**  Parametros:     Buffer del bit a leer, indice del bit, longitud del bit
**  Retorno:        valor del campo
****************************************************************************************/
uint32_t leerCampoBitCSDsd(uint8_t *buffer, uint32_t indiceBit, uint32_t lonBit)
{
    uint32_t campo = 0;
    uint32_t offsetBitEnByte = indiceBit % 8;
    uint8_t byteBuffer;

    buffer += indiceBit / 8;

    // Alinea el campo de bits para leerlo en la parte superior del buffer
    byteBuffer = *buffer << offsetBitEnByte;

    while (lonBit > 0) {
        uint32_t bitsCiclo = MIN(8 - offsetBitEnByte, lonBit);

        campo = (campo << bitsCiclo) | (byteBuffer >> (8 - bitsCiclo));

        buffer++;
        byteBuffer = *buffer;

        lonBit -= bitsCiclo;
        offsetBitEnByte = 0;
    }

    return campo;
}
