/***************************************************************************************
**  gp_rtc.c - Funciones y grupo de parametros del RTC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 22/07/2020
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
#include "gp_rtc.h"

#ifdef USAR_RTC


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define OFFSET_MINUTOS_RTC      60
#define OFFSET_GPS_UTC_RTC      18


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_GP_CON_TEMPLATE_RESET(configRTC_t, configRTC, GP_CONFIGURACION_RTC, 1);

TEMPLATE_RESET_GP(configRTC_t, configRTC,
    .offsetMinutos = OFFSET_MINUTOS_RTC,
    .offsetGPSutc = OFFSET_GPS_UTC_RTC
);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


#endif
