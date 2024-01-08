/***************************************************************************************
**  rtc.c - Funciones del sistema de tiempo RTC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/05/2019
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
#include <stdio.h>
#include <stdlib.h>

#include "rtc.h"

#ifdef USAR_RTC
#include "GP/gp_rtc.h"
#include "tiempo.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define OFFSET_ANO_REFERENCIA_RTC    946684800      // Offset en segundos desde el 01-01-1970 al 01-01-2000

#define MILIS_POR_SEGUNDO_RTC        1000

#define OFFSET_MINUTOS_UTC_MIN_RTC  -780            // -13 horas
#define OFFSET_MINUTOS_UTC_MAX_RTC   780            // +13 horas


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const uint16_t dias[4][12] =
{
    {   0,  31,     60,     91,     121,    152,    182,    213,    244,    274,    305,    335},
    { 366,  397,    425,    456,    486,    517,    547,    578,    609,    639,    670,    700},
    { 731,  762,    790,    821,    851,    882,    912,    943,    974,    1004,   1035,   1065},
    {1096,  1127,   1155,   1186,   1216,   1247,   1277,   1308,   1339,   1369,   1400,   1430},
};

static int64_t horaInicioUnix = 0;
static bool rtcIni = false;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void fechaHoraPorDefectoRTC(fechaHora_t *fechaHora);
bool fechaHoraValidaRTC(fechaHora_t fechaHora);
int64_t generarHoraUnixRTC(int32_t segundos, uint16_t milisegundos);
bool horaUnixRTC(int64_t *horaUnix);
int32_t segundosHoraUnixRTC(int64_t horaUnix);
uint16_t milisegundosHoraUnixRTC(int64_t horaUnix);
int64_t fechaHoraAtiempoUnixRTC(fechaHora_t fechaHora);
void horaUnixAfechaHoraRTC(int64_t horaUnix, fechaHora_t *fechaHora);
void fechaHoraConOffsetRTC(fechaHora_t fechaHoraInicial, int16_t offsetMinutos, fechaHora_t *FechaHoraConOffset);
void fechaHoraUTCaLocal(fechaHora_t fechaHoraUTC, fechaHora_t *fechaHoraLocal);
bool formatearFechaHora(char *buffer, fechaHora_t fechaHora, int16_t offsetMinutos, bool versionCorta);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarRTC(void)
**  Descripcion:    Inicia el driver del RTC y comprueba si hay que resetearlo
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarRTC(void)
{
#ifdef USAR_RTC_HW
    if (!iniciarDispositivoRTChw()) {
#ifdef DEBUG
        printf("Fallo en la inicializacion del RTC hardware\n");
#endif
        return false;
    }
#endif

    if (configRTC()->offsetMinutos > OFFSET_MINUTOS_UTC_MAX_RTC || configRTC()->offsetMinutos < OFFSET_MINUTOS_UTC_MIN_RTC) {
#ifdef DEBUG
        printf("Fallo en la definicion del offset UTC del RTC\n");
#endif
        return false;
    }

    rtcIni = true;
    return true;
}


/***************************************************************************************
**  Nombre:         bool rtcIniciado(void)
**  Descripcion:    Devuelve si el RTC esta iniciado
**  Parametros:     Ninguno
**  Retorno:        Iniciado
****************************************************************************************/
bool rtcIniciado(void)
{
    return rtcIni;
}


/***************************************************************************************
**  Nombre:         void fechaHoraPorDefectoRTC(fechaHoraRTC_t *fechaHora)
**  Descripcion:    Ajusta la fecha y la hora a unos valores por defecto
**  Parametros:     Fecha y hora
**  Retorno:        Ninguno
****************************************************************************************/
void fechaHoraPorDefectoRTC(fechaHora_t *fechaHora)
{
    fechaHora->ano = 0;
    fechaHora->mes = 1;
    fechaHora->dia = 1;
    fechaHora->horas = 0;
    fechaHora->minutos = 0;
    fechaHora->segundos = 0;
    fechaHora->milisegundos = 0;
}


/***************************************************************************************
**  Nombre:         bool tieneHoraRTC(void)
**  Descripcion:    Comprueba si el RTC tiene una hora ajustada
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool tieneHoraRTC(void)
{
    return horaInicioUnix != 0;
}


/***************************************************************************************
**  Nombre:         bool fechaHoraValidaRTC(fechaHora_t fechaHora)
**  Descripcion:    Comprueba si la fecha y la hora son validas
**  Parametros:     Fecha y hora
**  Retorno:        True si ok
****************************************************************************************/
bool fechaHoraValidaRTC(fechaHora_t fechaHora)
{
    return (fechaHora.ano >= ANO_REFERENCIA_RTC) &&
           (fechaHora.mes >= 1 && fechaHora.mes <= 12) &&
           (fechaHora.dia >= 1 && fechaHora.dia <= 31) &&
           (fechaHora.horas <= 23) &&
           (fechaHora.minutos <= 59) &&
           (fechaHora.segundos <= 59) &&
           (fechaHora.milisegundos <= 999);
}


/***************************************************************************************
**  Nombre:         void ajustarUnixRTC(int64_t horaRTC)
**  Descripcion:    ajusta la fecha y la hora en formato Unix
**  Parametros:     Fecha y Hora
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarUnixRTC(int64_t horaRTC)
{
    fechaHora_t fechaHora;

    horaInicioUnix = horaRTC - millis();
    horaUnixAfechaHoraRTC(horaRTC, &fechaHora);

#ifdef USAR_RTC_HW
    ajustarFechaHoraRTChw(fechaHora);
#endif
}


/***************************************************************************************
**  Nombre:         void ajustarFechaHoraRTC(fechaHora_t fechaHora)
**  Descripcion:    Ajusta la fecha y la hora
**  Parametros:     Fecha y Hora
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFechaHoraRTC(fechaHora_t fechaHora)
{
#ifdef USAR_RTC_HW
    ajustarFechaHoraRTChw(fechaHora);
#endif

    ajustarHoraInicioUnix(fechaHora);
}


/***************************************************************************************
**  Nombre:         void ajustarHoraInicioUnix(fechaHora_t fechaHora)
**  Descripcion:    Obtiene la fecha y la hora
**  Parametros:     Fecha y Hora
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarHoraInicioUnix(fechaHora_t fechaHora)
{
    int64_t horaRTC = fechaHoraAtiempoUnixRTC(fechaHora);
    horaInicioUnix = horaRTC - millis();
}


/***************************************************************************************
**  Nombre:         int64_t generarHoraUnixRTC(int32_t segundos, uint16_t milisegundos)
**  Descripcion:    Genera la hora en formato Unix
**  Parametros:     Segundos, milisegundos
**  Retorno:        Fecha y hora en formato Unix
****************************************************************************************/
int64_t generarHoraUnixRTC(int32_t segundos, uint16_t milisegundos)
{
    return ((int64_t)segundos) * MILIS_POR_SEGUNDO_RTC + milisegundos;
}


/***************************************************************************************
**  Nombre:         bool horaUnixRTC(int64_t *horaUnix)
**  Descripcion:    Obtiene la hora Unix
**  Parametros:     Hora Unix
**  Retorno:        True si ok
****************************************************************************************/
bool horaUnixRTC(int64_t *horaUnix)
{
    if (!tieneHoraRTC())
        return false;

    *horaUnix = horaInicioUnix + millis();
    return true;
}


/***************************************************************************************
**  Nombre:         bool fechaHoraRTC(fechaHora_t *fechaHora)
**  Descripcion:    Obtiene la hora Unix
**  Parametros:     Fecha y Hora
**  Retorno:        True si ok
****************************************************************************************/
bool fechaHoraRTC(fechaHora_t *fechaHora)
{
    int64_t horaRTC;

    if (horaUnixRTC(&horaRTC)) {
        horaUnixAfechaHoraRTC(horaRTC, fechaHora);
        return true;
    }

    // Fecha por defecto 0000-01-01T00:00:00.000
    fechaHoraPorDefectoRTC(fechaHora);
    return false;
}


/***************************************************************************************
**  Nombre:         int32_t segundosHoraUnixRTC(int64_t horaUnix)
**  Descripcion:    Obtiene los segundos de la hora Unix
**  Parametros:     Hora Unix
**  Retorno:        Segundos
****************************************************************************************/
int32_t segundosHoraUnixRTC(int64_t horaUnix)
{
    return horaUnix / MILIS_POR_SEGUNDO_RTC;
}


/***************************************************************************************
**  Nombre:         uint16_t milisegundosHoraUnixRTC(int64_t horaUnix)
**  Descripcion:    Obtiene los milisegundos de la hora Unix
**  Parametros:     Hora Unix
**  Retorno:        Milisegundos
****************************************************************************************/
uint16_t milisegundosHoraUnixRTC(int64_t horaUnix)
{
    return horaUnix % MILIS_POR_SEGUNDO_RTC;
}


/***************************************************************************************
**  Nombre:         int64_t fechaHoraAtiempoUnixRTC(fechaHoraRTC_t fechaHora)
**  Descripcion:    Convierte la fecha y la hora a formato RTC
**  Parametros:     Fecha y hora
**  Retorno:        Fecha y hora convertida
****************************************************************************************/
int64_t fechaHoraAtiempoUnixRTC(fechaHora_t fechaHora)
{
    uint8_t segundos = fechaHora.segundos;             // 0-59
    uint8_t minutos = fechaHora.minutos;               // 0-59
    uint8_t hora = fechaHora.horas;                    // 0-23
    uint8_t dia = fechaHora.dia - 1;                   // 0-30
    uint8_t mes = fechaHora.mes - 1;                   // 0-11
    uint8_t ano = fechaHora.ano - ANO_REFERENCIA_RTC;  // 0-99

    int32_t fechaUnix = (((ano / 4 * (365 * 4 + 1) + dias[ano % 4][mes] + dia) * 24 + hora) * 60 + minutos) * 60 + segundos + OFFSET_ANO_REFERENCIA_RTC;
    return generarHoraUnixRTC(fechaUnix, fechaHora.milisegundos);
}


/***************************************************************************************
**  Nombre:         void horaUnixAfechaHoraRTC(int64_t horaUnix, fechaHora_t *fechaHora)
**  Descripcion:    Convierte la fecha RTC a formato fecha y hora
**  Parametros:     Fecha y hora, fecha RTC
**  Retorno:        Ninguno
****************************************************************************************/
void horaUnixAfechaHoraRTC(int64_t horaUnix, fechaHora_t *fechaHora)
{
    int32_t fechaUnix = horaUnix / MILIS_POR_SEGUNDO_RTC - OFFSET_ANO_REFERENCIA_RTC;

    fechaHora->segundos = fechaUnix % 60;
    fechaUnix /= 60;
    fechaHora->minutos = fechaUnix % 60;
    fechaUnix /= 60;
    fechaHora->horas = fechaUnix % 24;
    fechaUnix /= 24;

    uint32_t anos = fechaUnix / (365 * 4 + 1) * 4;
    fechaUnix %= 365 * 4 + 1;

    uint32_t ano;
    for (ano = 3; ano > 0; ano--) {
        if (fechaUnix >= dias[ano][0])
            break;
    }

    uint32_t mes;
    for (mes = 11; mes > 0; mes--) {
        if (fechaUnix >= dias[ano][mes])
            break;
    }

    fechaHora->ano = anos + ano + ANO_REFERENCIA_RTC;
    fechaHora->mes = mes + 1;
    fechaHora->dia = fechaUnix - dias[ano][mes] + 1;
    fechaHora->milisegundos = horaUnix % MILIS_POR_SEGUNDO_RTC;
}


/***************************************************************************************
**  Nombre:         void fechaHoraConOffsetRTC(fechaHora_t fechaHoraInicial, int16_t offsetMinutos, fechaHora_t *FechaHoraConOffset)
**  Descripcion:    Anade un offset a la fecha y la hora
**  Parametros:     Fecha y hora con offset, fecha y hora de partida, minutos de offset
**  Retorno:        Ninguno
****************************************************************************************/
void fechaHoraConOffsetRTC(fechaHora_t fechaHoraInicial, int16_t offsetMinutos, fechaHora_t *FechaHoraConOffset)
{
    int64_t horaInicial = fechaHoraAtiempoUnixRTC(fechaHoraInicial);
    int64_t horaUnixConOffset = generarHoraUnixRTC(segundosHoraUnixRTC(horaInicial) + offsetMinutos * 60, milisegundosHoraUnixRTC(horaInicial));
    horaUnixAfechaHoraRTC(horaUnixConOffset, FechaHoraConOffset);
}


/***************************************************************************************
**  Nombre:         void fechaHoraUTCaLocal(fechaHora_t fechaHoraUTC, fechaHora_t fechaHoraLocal)
**  Descripcion:    Convierte la fecha y hora de formato UTC a local
**  Parametros:     Fecha y hora UTC, fecha y hora local
**  Retorno:        Ninguno
****************************************************************************************/
void fechaHoraUTCaLocal(fechaHora_t fechaHoraUTC, fechaHora_t *fechaHoraLocal)
{
    fechaHoraConOffsetRTC(fechaHoraUTC, configRTC()->offsetMinutos, fechaHoraLocal);
}


/***************************************************************************************
**  Nombre:         bool formatearFechaHora(char *buffer, fechaHora_t fechaHora, int16_t offsetMinutos, bool versionCorta)
**  Descripcion:    Formatea la fecha y hora en una cadena de caracteres
**  Parametros:     Buffer de caracteres, fecha y hora, offset de minutos, version corta o no
**  Retorno:        True si ok
****************************************************************************************/
bool formatearFechaHora(char *buffer, fechaHora_t fechaHora, int16_t offsetMinutos, bool versionCorta)
{
    fechaHora_t local;

    int16_t offHoras = 0;
    int16_t offMinutos = 0;
    bool valor = true;

    if (offsetMinutos != 0) {
        offHoras = offsetMinutos / 60;
        offMinutos = ABS(offsetMinutos % 60);
        fechaHoraConOffsetRTC(fechaHora, offsetMinutos, &local);
        fechaHora = local;
    }

    if (!fechaHoraValidaRTC(fechaHora)) {
        fechaHoraPorDefectoRTC(&local);
        fechaHora = local;
        valor = false;
    }

    if (versionCorta)
        sprintf(buffer, "%04u-%02u-%02u %02u:%02u:%02u", fechaHora.ano, fechaHora.mes, fechaHora.dia, fechaHora.horas, fechaHora.minutos, fechaHora.segundos);
    else       // Formato ISO_8601 https://en.wikipedia.org/wiki/ISO_8601
        sprintf(buffer, "%04u-%02u-%02uT%02u:%02u:%02u.%03u%c%02d:%02d", fechaHora.ano, fechaHora.mes, fechaHora.dia, fechaHora.horas, fechaHora.minutos, fechaHora.segundos, fechaHora.milisegundos, offHoras >= 0 ? '+' : '-', ABS(offHoras), offMinutos);

    return valor;
}


/***************************************************************************************
**  Nombre:         bool formatearfechaHoraUTC(char *buffer, fechaHora_t fechaHora)
**  Descripcion:    Convierte la fecha y hora en formato UTC
**  Parametros:     Buffer de caracteres, fecha y hora
**  Retorno:        True si ok
****************************************************************************************/
bool formatearfechaHoraUTC(char *buffer, fechaHora_t fechaHora)
{
    return formatearFechaHora(buffer, fechaHora, 0, false);
}


/***************************************************************************************
**  Nombre:         bool formatearFechaHoraLocal(char *buffer, fechaHora_t fechaHora)
**  Descripcion:    Convierte la fecha y hora en formato local
**  Parametros:     Buffer de caracteres, fecha y hora
**  Retorno:        True si ok
****************************************************************************************/
bool formatearFechaHoraLocal(char *buffer, fechaHora_t fechaHora)
{
    const int16_t offsetHoraZona = fechaHoraValidaRTC(fechaHora) ? configRTC()->offsetMinutos : 0;
    return formatearFechaHora(buffer, fechaHora, offsetHoraZona, false);
}


/***************************************************************************************
**  Nombre:         bool formatearFechaHoraLocalCorta(char *buffer, fechaHora_t fechaHora)
**  Descripcion:    Convierte la fecha y hora en formato corto
**  Parametros:     Buffer de caracteres, fecha y hora
**  Retorno:        True si ok
****************************************************************************************/
bool formatearFechaHoraLocalCorta(char *buffer, fechaHora_t fechaHora)
{
    return formatearFechaHora(buffer, fechaHora, configRTC()->offsetMinutos, true);
}


#endif

