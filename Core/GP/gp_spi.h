/***************************************************************************************
**  gp_spi.h - Funciones y grupo de parametros del SPI
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 25/07/2020
**  Fecha de modificacion: 25/07/2020
**
**  El proyecto URpilot NO es libre. No se puede distribuir y/o modificar este fichero
**  bajo ningun concepto.
**
**  En caso de modificacion y/o solicitud de informacion pongase en contacto con
**  el grupo de investigacion ICON a traves de: www.unirioja.es/urpilot
**
****************************************************************************************/

#ifndef __GP_SPI_H
#define __GP_SPI_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Drivers/spi.h"
#include "gp.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t pinSCK;
    uint8_t pinMISO;
    uint8_t pinMOSI;
    bool leadingEdge;
} configSPI_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_ARRAY_GP(configSPI_t, NUM_MAX_SPI, configSPI);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_SPI_H
