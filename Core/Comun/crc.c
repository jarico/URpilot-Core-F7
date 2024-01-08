/***************************************************************************************
**  crc.c - Funciones de calculo de crc
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "crc.h"


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
uint16_t actualizarCRC16(uint16_t crc, uint8_t dato);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         uint16_t calcularCRC4(uint16_t *dato)
**  Descripcion:    Actualiza el CRC
**  Parametros:     Puntero de los datos para calcular el CRC
**  Retorno:        CRC
****************************************************************************************/
uint16_t calcularCRC4(uint16_t *dato)
{
    uint16_t crc = 0;

    for (uint8_t i = 0; i < 16; i++) {
        // Bytes impares
        if (i & 1)
            crc ^= (uint8_t)((dato[i >> 1]) & 0x00FF);
        else
            crc ^= (uint8_t)(dato[i >> 1] >> 8);


        for (uint8_t j = 8; j > 0; j--) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x3000;
            else
                crc = (crc << 1);
        }
    }

    return (crc >> 12) & 0xF;
}


/***************************************************************************************
**  Nombre:         uint16_t actualizarCRC16(uint16_t crc, uint8_t dato)
**  Descripcion:    Calcula el crc dado un dato
**  Parametros:     CRC, dato
**  Retorno:        CRC
****************************************************************************************/
uint16_t actualizarCRC16(uint16_t crc, uint8_t dato)
{
    crc ^= (uint16_t)dato << 8;

    for (int i = 0; i < 8; ++i) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc = crc << 1;
    }

    return crc;
}


/***************************************************************************************
**  Nombre:         uint16_t calcularCRC16(uint16_t crc, const void *dato, uint32_t longitud)
**  Descripcion:    Actualiza el CRC
**  Parametros:     CRC, dato a actualizar, longitud de los datos
**  Retorno:        CRC
****************************************************************************************/
uint16_t calcularCRC16(uint16_t crc, const void *dato, uint32_t longitud)
{
    const uint8_t *p = (const uint8_t *)dato;
    const uint8_t *pFin = p + longitud;

    while (p != pFin) {
        crc = actualizarCRC16(crc, *p);
        p++;
    }

    return crc;
}

