/***************************************************************************************
**  rtc.h - Funciones del sistema de tiempo RTC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 29/11/2019
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

#ifndef __RTC_H
#define __RTC_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "GP/gp.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define ANO_REFERENCIA_RTC               2000
#define TAMANIO_FECHA_HORA_FORMATEADA    30


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint16_t ano;
    uint8_t mes;
    uint8_t dia;
    uint8_t horas;
    uint8_t minutos;
    uint8_t segundos;
    uint16_t milisegundos;
} fechaHora_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarRTC(void);
bool rtcIniciado(void);
bool tieneHoraRTC(void);
void ajustarUnixRTC(int64_t horaRTC);
void ajustarFechaHoraRTC(fechaHora_t fechaHora);
void ajustarHoraInicioUnix(fechaHora_t fechaHora);
bool fechaHoraRTC(fechaHora_t *fechaHora);
bool formatearfechaHoraUTC(char *buffer, fechaHora_t fechaHora);
bool formatearFechaHoraLocal(char *buffer, fechaHora_t fechaHora);
bool formatearFechaHoraLocalCorta(char *buffer, fechaHora_t fechaHora);

bool iniciarDispositivoRTChw(void);
bool fechaRTChw(void);
bool horaRTChw(void);
void ajustarFechaHoraRTChw(fechaHora_t fechaHora);

#endif // __RTC_H
