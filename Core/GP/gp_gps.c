/***************************************************************************************
**  gp_gps.c - Funciones y grupo de parametros del GPS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 13/06/2020
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
#include "gp_gps.h"

#ifdef USAR_GPS
#include "Drivers/io.h"
#include "Drivers/uart.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define ELEVACION_MIN_GPS           -100
#define PERIODO_MUESTREO_GPS_MS     200

#define SBAS_GPS                    SBAS_SIN_CAMBIOS
#define ENGINE_GPS                  GPS_ENGINE_AIRBORNE_4G
#define GNSS_GPS                    GPS

#ifndef TIPO_GPS_1
  #define TIPO_GPS_1                GPS_NINGUNO
#endif

#ifndef AUXILIAR_GPS_1
  #define AUX_GPS_1                 false
#else
  #define AUX_GPS_1                 true
#endif

#ifndef UART_GPS_1
  #define UART_GPS_1                UART_NINGUNO
#endif

#ifndef TIPO_GPS_2
  #define TIPO_GPS_2                GPS_NINGUNO
#endif

#ifndef AUXILIAR_GPS_2
  #define AUX_GPS_2                 false
#else
  #define AUX_GPS_2                 true
#endif

#ifndef UART_GPS_2
  #define UART_GPS_2                UART_NINGUNO
#endif

#ifndef TIPO_GPS_3
  #define TIPO_GPS_3                GPS_NINGUNO
#endif

#ifndef AUXILIAR_GPS_3
  #define AUX_GPS_3                 false
#else
  #define AUX_GPS_3                 true
#endif

#ifndef UART_GPS_3
  #define UART_GPS_3                UART_NINGUNO
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configGPS_t, NUM_MAX_GPS, configGPS, GP_CONFIGURACION_GPS, 2);

static const configGPS_t configGPSdefecto[] = {
    { TIPO_GPS_1, AUX_GPS_1, UART_GPS_1, SBAS_GPS, ENGINE_GPS, ELEVACION_MIN_GPS, GNSS_GPS, PERIODO_MUESTREO_GPS_MS, FREC_LEER_GPS_HZ},
    { TIPO_GPS_2, AUX_GPS_2, UART_GPS_2, SBAS_GPS, ENGINE_GPS, ELEVACION_MIN_GPS, GNSS_GPS, PERIODO_MUESTREO_GPS_MS, FREC_LEER_GPS_HZ},
    { TIPO_GPS_3, AUX_GPS_3, UART_GPS_3, SBAS_GPS, ENGINE_GPS, ELEVACION_MIN_GPS, GNSS_GPS, PERIODO_MUESTREO_GPS_MS, FREC_LEER_GPS_HZ},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


/***************************************************************************************
**  Nombre:         void fnResetGP_configGPS(configGPS_t *configGPS)
**  Descripcion:    Funcion de reset de la configuracion del GPS
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configGPS(configGPS_t *configGPS)
{
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
    	configGPS[i].tipoGPS = configGPSdefecto[i].tipoGPS;
    	configGPS[i].auxiliar = configGPSdefecto[i].auxiliar;
    	configGPS[i].dispUART = configGPSdefecto[i].dispUART;
    	configGPS[i].modoConf = configGPSdefecto[i].modoConf;
    	configGPS[i].modoSBAS = configGPSdefecto[i].modoSBAS;
    	configGPS[i].elevacionMin = configGPSdefecto[i].elevacionMin;
    	configGPS[i].gnss = configGPSdefecto[i].gnss;
    	configGPS[i].periodoMuestreo = configGPSdefecto[i].periodoMuestreo;
    	configGPS[i].frecLeer = configGPSdefecto[i].frecLeer;
    }
}

#endif
