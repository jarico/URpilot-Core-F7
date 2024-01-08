/***************************************************************************************
**  gp_sdmmc.h - Funciones y grupo de parametros del SDMMC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/05/2020
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

#ifndef __GP_SDMMC_H
#define __GP_SDMMC_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "gp.h"
#include "Drivers/sdmmc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	numSDMMC_e numSDMMC;
    bool ancho4bits;
    uint8_t pinCK;
    uint8_t pinCMD;
    uint8_t pinD0;
    uint8_t pinD1;
    uint8_t pinD2;
    uint8_t pinD3;
#ifdef USAR_DMA_SDMMC
    DMA_Stream_TypeDef *streamDMAtx;
    DMA_Stream_TypeDef *streamDMArx;
#endif
} configSDMMC_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_GP(configSDMMC_t, configSDMMC);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_SDMMC_H
