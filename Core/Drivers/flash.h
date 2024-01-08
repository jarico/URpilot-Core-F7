/***************************************************************************************
**  flash.h - Funciones para la gestion de la flash y en concreto las funciones para
**            grabar la configuracion en la flash
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 08/05/2019
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

#ifndef __FLASH_H
#define __FLASH_H

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
    uintptr_t dir;
    int32_t tam;
    union {
        uint8_t b[4];
        uint32_t w;
    } buffer;
    int32_t at;
    int32_t err;
    bool desbloqueado;
} grabadorFlash_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void resetearGrabadorFlash(grabadorFlash_t *grabador);
int32_t estadoGrabadorFlash(grabadorFlash_t *grabador);
void desbloquearGrabadorFlash(grabadorFlash_t *grabador, uintptr_t base, int32_t tam);
int32_t bloquearGrabadorFlash(grabadorFlash_t *grabador);
int32_t escribirGrabadorFlash(grabadorFlash_t *grabador, const uint8_t *p, uint32_t tam);
int32_t flushGrabadorFlash(grabadorFlash_t *grabador);

#endif // __FLASH_H
