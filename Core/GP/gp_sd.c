/***************************************************************************************
**  gp_sd.c - Funciones de configuracion de la tarjeta SD
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/09/2019
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
#include "gp_sd.h"

#ifdef USAR_SD
#include "Blackbox/sd.h"
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef USAR_CACHE_SD
  #define USAR_CACHE_SD            false
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configSD_t, configSD, GP_CONFIGURACION_SD, 1);

TEMPLATE_RESET_GP(configSD_t, configSD,
    .modo = MODO_SD,
    .usarCache = USAR_CACHE_SD,
    .pinTarjetaDetectada = DEFIO_TAG(PIN_DETECCION_SD),
    .deteccionInvertida = PIN_INVERTIDO_SD,
#ifdef USAR_SD_SPI
    .numSPI = DRIVER_SPI_SD,
    .pinCS = DEFIO_TAG(CS_SPI_SD),
#endif
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

#endif
