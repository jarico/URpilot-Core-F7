/***************************************************************************************
**  version.c - Fichero que determina la version del firmware. Cambiar la version con los
**              cambios que se desarrollen en el codigo
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2019
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

#include "version.h"
#include "Drivers/adc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_RANGOS_VERSION     10


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
char* nombrePlaca = NOMBRE_PLACA;
char* diaCompilacion = __DATE__;
char* horaCompilacion = __TIME__;

static uint8_t version, revision;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
uint8_t calcularVersionPlaca(void);
uint8_t calcularRevisionPlaca(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarVersion(void)
**  Descripcion:    Calcula la version de la placa
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool iniciarVersion(void)
{
#ifdef USAR_VERSION_HW_ADC
    if (!iniciadoADC()) {
        if (!iniciarADC()) {
#ifdef DEBUG
            printf("Fallo al calcular la version HW\n");
#endif
	        return false;
        }
    }

    version = calcularVersionPlaca();
    revision = calcularRevisionPlaca();

#else
    version = 0;
    revision = 0;
#endif
    return true;
}


/***************************************************************************************
**  Nombre:         char* horaCompilacionFirm(void)
**  Descripcion:    Retorna la hora de compilacion del firmware
**  Parametros:     Ninguno
**  Retorno:        Hora de compilacion
****************************************************************************************/
char* horaCompilacionFirm(void)
{
    return horaCompilacion;
}


/***************************************************************************************
**  Nombre:         char* diaCompilacionFirm(void)
**  Descripcion:    Retorna el dia de compilacion del firmware
**  Parametros:     Ninguno
**  Retorno:        Dia de compilacion
****************************************************************************************/
char* diaCompilacionFirm(void)
{
    return diaCompilacion;
}


/***************************************************************************************
**  Nombre:         uint8_t versionHW(void)
**  Descripcion:    Retorna la version de la placa
**  Parametros:     Ninguno
**  Retorno:        Version
****************************************************************************************/
uint8_t versionHW(void)
{
    return version;
}


/***************************************************************************************
**  Nombre:         uint8_t revisionHW(void)
**  Descripcion:    Retorna la revision de la placa
**  Parametros:     Ninguno
**  Retorno:        Revision
****************************************************************************************/
uint8_t revisionHW(void)
{
    return revision;
}


/***************************************************************************************
**  Nombre:         uint8_t calcularVersionPlaca(void)
**  Descripcion:    Calcula la version de la placa
**  Parametros:     Ninguno
**  Retorno:        Version
****************************************************************************************/
uint8_t calcularVersionPlaca(void)
{
#ifdef USAR_VERSION_HW_ADC
	uint16_t rango = VALOR_LIMITE_ADC / NUM_RANGOS_VERSION;
    return (uint8_t)leerCanalADC(HW_VER_ADC) / rango + 1;
#else
    return 0;
#endif
}


/***************************************************************************************
**  Nombre:         uint8_t calcularRevisionPlaca(void)
**  Descripcion:    Calcula la revision de la placa
**  Parametros:     Ninguno
**  Retorno:        Revision
****************************************************************************************/
uint8_t calcularRevisionPlaca(void)
{
#ifdef USAR_VERSION_HW_ADC
    uint16_t rango = VALOR_LIMITE_ADC / NUM_RANGOS_VERSION;
    return (uint8_t)leerCanalADC(HW_REV_ADC) / rango + 1;
#else
    return 0;
#endif
}

