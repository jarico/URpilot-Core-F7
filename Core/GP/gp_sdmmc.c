/***************************************************************************************
**  gp_sdmmc.c - Funciones y grupo de parametros del SDMMC
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "gp_sdmmc.h"

#ifdef USAR_SDMMC
#include "Drivers/sdmmc.h"
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef BUS_4BITS_SDMMC
  #define BUS_4BITS_SDMMC        false
#endif

#ifndef PIN_CK_SDMMC
  #define PIN_CK_SDMMC           NINGUNO
#endif

#ifndef PIN_CMD_SDMMC
  #define PIN_CMD_SDMMC          NINGUNO
#endif

#ifndef PIN_D0_SDMMC
  #define PIN_D0_SDMMC           NINGUNO
#endif

#ifndef PIN_D1_SDMMC
  #define PIN_D1_SDMMC           NINGUNO
#endif

#ifndef PIN_D2_SDMMC
  #define PIN_D2_SDMMC           NINGUNO
#endif

#ifndef PIN_D3_SDMMC
  #define PIN_D3_SDMMC           NINGUNO
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configSDMMC_t, configSDMMC, GP_CONFIGURACION_SDMMC, 1);

TEMPLATE_RESET_GP(configSDMMC_t, configSDMMC,
    .numSDMMC = DRIVER_SDMMC,
    .ancho4bits = BUS_4BITS_SDMMC,
    .pinCK = DEFIO_TAG(PIN_CK_SDMMC),
    .pinCMD = DEFIO_TAG(PIN_CMD_SDMMC),
    .pinD0 = DEFIO_TAG(PIN_D0_SDMMC),
    .pinD1 = DEFIO_TAG(PIN_D1_SDMMC),
    .pinD2 = DEFIO_TAG(PIN_D2_SDMMC),
    .pinD3 = DEFIO_TAG(PIN_D3_SDMMC),
#ifdef USAR_DMA_SDMMC
    .streamDMAtx = DMA_TX_SDMMC,
    .streamDMArx = DMA_RX_SDMMC,
#endif
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

#endif
