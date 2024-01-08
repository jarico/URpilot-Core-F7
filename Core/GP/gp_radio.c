/***************************************************************************************
**  gp_radio.c - Funciones y grupo de parametros de la radio
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 27/06/2020
**  Fecha de modificacion: 13/09/2020
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
#include "gp_radio.h"

#ifdef USAR_RADIO
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define VALOR_MIN_FAILSAFE     980
#define VALOR_MAX_FAILSAFE     2020

#ifndef PROTOCOLO_RADIO
  #define PROTOCOLO_RADIO      RX_IBUS
#endif

#ifndef UART_RADIO
  #define UART_RADIO           UART_NINGUNO
#endif

#ifndef PIN_PPM
  #define PIN_PPM              NINGUNO
#endif

#ifndef TIMER_PPM
  #define TIMER_PPM            TIMER_NINGUNO
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configRadio_t, configRadio, GP_CONFIGURACION_RADIO, 1);

TEMPLATE_RESET_GP(configRadio_t, configRadio,
    .protocolo = PROTOCOLO_RADIO,
    .dispUART = UART_RADIO,
    .pinPPM.pin = DEFIO_TAG(PIN_PPM),
    .pinPPM.numTimer = TIMER_PPM,
    .frecLeer = FREC_LEER_RADIO_HZ,
    .minFailsafe = VALOR_MIN_FAILSAFE,
    .maxFailsafe = VALOR_MAX_FAILSAFE,
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


#endif
