/***************************************************************************************
**  gp_ids.h - Definiciones de los id de los grupos de parametros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/05/2019
**  Fecha de modificacion: 22/09/2020
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

#ifndef __GP_IDS_H
#define __GP_IDS_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
// Configuraciones de sistema
#define GP_CONFIGURACION_SISTEMA         1
#define GP_CONFIGURACION_RTC             2
#define GP_CONFIGURACION_TIMER           3
#define GP_CONFIGURACION_ADC             4
#define GP_CONFIGURACION_SPI             5
#define GP_CONFIGURACION_I2C             6
#define GP_CONFIGURACION_UART            7
#define GP_CONFIGURACION_SDMMC           8
#define GP_CONFIGURACION_USB             9

// Configuraciones de perifericos externos
#define GP_CONFIGURACION_POWER_MODULE    100
#define GP_CONFIGURACION_BAROMETRO       101
#define GP_CONFIGURACION_MAGNETOMETRO    102
#define GP_CONFIGURACION_IMU             103
#define GP_CONFIGURACION_GPS             104
#define GP_CONFIGURACION_RADIO           105
#define GP_CONFIGURACION_MOTORES         106
#define GP_CONFIGURACION_SD              107
#define GP_CONFIGURACION_BLACKBOX        108
#define GP_CONFIGURACION_RC              109
#define GP_CONFIGURACION_REF_RC          110
#define GP_CONFIGURACION_MODO_SYS        111
#define GP_CONFIGURACION_SECUENCIA_RC    112
#define GP_CONFIGURACION_MIXER           113
#define GP_CONFIGURACION_AHRS            114
#define GP_CONFIGURACION_FC              115
#define GP_CONFIGURACION_PID             116
#define GP_CONFIGURACION_CAL_IMU         117
#define GP_CONFIGURACION_CAL_MAG         118

#endif // __GP_IDS_H
