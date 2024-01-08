/***************************************************************************************
**  rtc_hal.c - Funciones HAL del sistema de tiempo RTC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 24/04/2020
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
#include "rtc.h"

#ifdef USAR_RTC_HW
#include "tiempo.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define BKP_FECHA_ESTANDAR_RTC     1


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    RTC_TimeTypeDef hora;
    RTC_DateTypeDef fecha;
    RTC_HandleTypeDef hrtc;
} rtcHW_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static rtcHW_t rtcHW;
static uint32_t valorFechaEstandarBKP = 0x32F2;          // Con este valor comprobamos que hay dato en el registro

// Variables para ajustar el RTC por Defecto: 01:01:2000 a las 00:00:00
static RTC_DateTypeDef fechaPorDefectoRTC = {
    .Date = 1,
    .Month = RTC_MONTH_JANUARY,
    .Year = 0
};

static RTC_TimeTypeDef horaPorDefectoRTC = {
    .Hours = 0,
    .Minutes = 0,
    .Seconds = 0,
    .DayLightSaving = 0,
    .StoreOperation = RTC_STOREOPERATION_RESET
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool esPrimerArranqueRTChw(void);
void escribirBackupRTChw(uint32_t numBKP, uint32_t valor);
uint32_t leerBackupRTChw(uint32_t numBKP);
bool ajustarFechaRTChw(RTC_DateTypeDef *fecha);
bool ajustarHoraRTChw(RTC_TimeTypeDef *hora);
void ajustarFechaHoraPorDefectoRTChw(void);
void convertirFechaUnixAhw(fechaHora_t fechaHora, RTC_DateTypeDef *fecha, RTC_TimeTypeDef *hora);
void convertirFechaHWAunix(RTC_DateTypeDef fecha, RTC_TimeTypeDef hora, fechaHora_t *fechaHora);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarDispositivoRTChw(void)
**  Descripcion:    Inicia el RTC hardware
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDispositivoRTChw(void)
{
    rtcHW_t *disp = &rtcHW;

    // Se habilita el reloj del RTC HW
    __HAL_RCC_GPIOC_CLK_ENABLE();

    disp->hrtc.Instance = RTC;

    disp->hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    disp->hrtc.Init.AsynchPrediv = 127;
    disp->hrtc.Init.SynchPrediv = 255;
    disp->hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    disp->hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    disp->hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    HAL_PWR_EnableBkUpAccess();
    if (HAL_RTC_Init(&disp->hrtc) != HAL_OK)
        return false;

    if (esPrimerArranqueRTChw())
        ajustarFechaHoraPorDefectoRTChw();
    else
        {
        // Leemos el RTC hardware
        fechaRTChw();
        horaRTChw();

        // Asignamos la fecha
        fechaHora_t fechaHora;
        convertirFechaHWAunix(rtcHW.fecha, rtcHW.hora, &fechaHora);
        ajustarHoraInicioUnix(fechaHora);
    }

    return true;

}


/***************************************************************************************
**  Nombre:         bool esPrimerArranqueRTChw(void)
**  Descripcion:    Comprueba si es necesario guardar el dato en el backup
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool esPrimerArranqueRTChw(void)
{
    if (leerBackupRTChw(BKP_FECHA_ESTANDAR_RTC) != valorFechaEstandarBKP)
        return true;

    return false;
}


/***************************************************************************************
**  Nombre:         void escribirBackupRTChw(uint32_t numBKP, uint32_t valor)
**  Descripcion:    Escribe un valor en el registro de backup
**  Parametros:     Numero del registro a escribir, valor a escribir
**  Retorno:        Ninguno
****************************************************************************************/
void escribirBackupRTChw(uint32_t numBKP, uint32_t valor)
{
    rtcHW_t *disp = &rtcHW;

    HAL_RTCEx_BKUPWrite(&disp->hrtc, numBKP, valor);
}


/***************************************************************************************
**  Nombre:         uint32_t leerBackupRTChw(void)
**  Descripcion:    Obtiene el valor del registro de backup
**  Parametros:     Numero del registro que se quiere leer
**  Retorno:        Valor del registro de backup
****************************************************************************************/
uint32_t leerBackupRTChw(uint32_t numBKP)
{
    rtcHW_t *disp = &rtcHW;

    return HAL_RTCEx_BKUPRead(&disp->hrtc, numBKP);
}


/***************************************************************************************
**  Nombre:         bool fechaRTChw(void)
**  Descripcion:    Obtiene la fecha del RTC
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool fechaRTChw(void)
{
    rtcHW_t *disp = &rtcHW;

    if (HAL_RTC_GetDate(&disp->hrtc, &disp->fecha, RTC_FORMAT_BIN) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool horaRTChw(void)
**  Descripcion:    Obtiene la hora del RTC
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool horaRTChw(void)
{
    rtcHW_t *disp = &rtcHW;

    if (HAL_RTC_GetTime(&disp->hrtc, &disp->hora, RTC_FORMAT_BIN) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool ajustarFechaRTChw(RTC_DateTypeDef *fecha)
**  Descripcion:    Ajusta la fecha del RTC
**  Parametros:     Fecha RTC
**  Retorno:        True si ok
****************************************************************************************/
bool ajustarFechaRTChw(RTC_DateTypeDef *fecha)
{
    rtcHW_t *disp = &rtcHW;

    if (HAL_RTC_SetDate(&disp->hrtc, fecha, RTC_FORMAT_BIN) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool ajustarHoraRTChw(RTC_TimeTypeDef *hora)
**  Descripcion:    Ajusta la hora del RTC
**  Parametros:     Hora RTC
**  Retorno:        True si ok
****************************************************************************************/
bool ajustarHoraRTChw(RTC_TimeTypeDef *hora)
{
    rtcHW_t *disp = &rtcHW;

    if (HAL_RTC_SetTime(&disp->hrtc, hora, RTC_FORMAT_BIN) != HAL_OK)
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void ajustarFechaHoraPorDefectoRTChw(void)
**  Descripcion:    Ajusta la fecha y la hora a unos valores por defecto
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFechaHoraPorDefectoRTChw(void)
{
    ajustarFechaRTChw(&fechaPorDefectoRTC);
    ajustarHoraRTChw(&horaPorDefectoRTC);
    escribirBackupRTChw(BKP_FECHA_ESTANDAR_RTC, valorFechaEstandarBKP);

    fechaHora_t fechaHora;
    convertirFechaHWAunix(fechaPorDefectoRTC, horaPorDefectoRTC, &fechaHora);
    ajustarHoraInicioUnix(fechaHora);
}


/***************************************************************************************
**  Nombre:         void ajustarFechaHoraRTChw(fechaHora_t *fechaHora)
**  Descripcion:    Ajusta la fecha y la hora del RTC hardware
**  Parametros:     Fecha y hora
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFechaHoraRTChw(fechaHora_t fechaHora)
{

    RTC_TimeTypeDef hora;
    RTC_DateTypeDef fecha;

    convertirFechaUnixAhw(fechaHora, &fecha, &hora);
    ajustarFechaRTChw(&fecha);
    ajustarHoraRTChw(&hora);

}


/***************************************************************************************
**  Nombre:         void convertirFechaUnixAhw(fechaHora_t fechaHora, RTC_DateTypeDef *fecha, RTC_TimeTypeDef *hora)
**  Descripcion:    Convierte la fecha y hora del formato unix al hardware
**  Parametros:     Fecha y hora hw, fecha en unix, hora en unix
**  Retorno:        Ninguno
****************************************************************************************/
void convertirFechaUnixAhw(fechaHora_t fechaHora, RTC_DateTypeDef *fecha, RTC_TimeTypeDef *hora)
{
    fecha->Date = fechaHora.dia;
    fecha->Month = fechaHora.mes;
    fecha->Year = (uint8_t)(fechaHora.ano - ANO_REFERENCIA_RTC);

    hora->Hours = fechaHora.horas;
    hora->Minutes = fechaHora.minutos;
    hora->Seconds = fechaHora.segundos;
}


/***************************************************************************************
**  Nombre:         void convertirFechaHWAunix(RTC_DateTypeDef fecha, RTC_TimeTypeDef hora, fechaHora_t *fechaHora)
**  Descripcion:    Convierte la fecha y hora del formato hardware al unix
**  Parametros:     Fecha en unix, hora en unix, fecha y hora hw
**  Retorno:        Ninguno
****************************************************************************************/
void convertirFechaHWAunix(RTC_DateTypeDef fecha, RTC_TimeTypeDef hora, fechaHora_t *fechaHora)
{
    fechaHora->ano = fecha.Year + ANO_REFERENCIA_RTC;
    fechaHora->mes = fecha.Month;
    fechaHora->dia = fecha.Date;
    fechaHora->horas = hora.Hours;
    fechaHora->minutos = hora.Minutes;
    fechaHora->segundos = hora.Seconds;
    fechaHora->milisegundos = millis();
}


#endif
