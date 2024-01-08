/***************************************************************************************
**  sd.c - Funciones de gestion de la tarjeta SD
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
#include <string.h>

#include "sd.h"

#ifdef USAR_SD
#include "GP/gp_sd.h"
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static sd_t sd;
static tablaFnSD_t *tablaFnSD;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarPinDeteccionSD(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         sd_t *punteroSD(void)
**  Descripcion:    Devuelve el puntero al SD
**  Parametros:     Ninguno
**  Retorno:        Puntero al SD
****************************************************************************************/
sd_t *punteroSD(void)
{
    return &sd;
}


/***************************************************************************************
**  Nombre:         void iniciarPinDeteccionSD(void)
**  Descripcion:    Configura el GPIO del pin de deteccion de tarjeta insertada
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarPinDeteccionSD(void)
{
    if (configSD()->pinTarjetaDetectada)
        configurarIO(configSD()->pinTarjetaDetectada, CONFIG_IO(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP), 0);
}


/***************************************************************************************
**  Nombre:         void iniciarSD(void)
**  Descripcion:    Inicia la tarjeta SD
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarSD(void)
{
#ifdef USAR_DETECCION_SD
    // Configuramos el GPIO del pin de deteccion de tarjeta insertada
	iniciarPinDeteccionSD();
#endif

    switch (configSD()->modo) {
        case SD_MODO_SPI:
#ifdef USAR_SD_SPI
        	tablaFnSD = &tablaFnSDspi;
#endif
            break;

        case SD_MODO_SDIO:
#ifdef USAR_SD_SDIO
        	tablaFnSD = &tablaFnSDsdio;
#endif
            break;

        default:
            break;
    }

    if (tablaFnSD)
    	sd.iniciada = tablaFnSD->iniciarSD();
}


/***************************************************************************************
**  Nombre:         bool tarjetaSDinsertada(void)
**  Descripcion:    Detecta si hay una tarjeta SD insertada en el slot
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool tarjetaSDinsertada(void)
{
#ifdef USAR_DETECCION_SD
    bool estado = true;
    if (configSD()->pinTarjetaDetectada) {
        estado = leerIO(configSD()->pinTarjetaDetectada);
        if (configSD()->deteccionInvertida)
            estado = !estado;
    }
    return estado;
#else
    return true;
#endif
}


/***************************************************************************************
**  Nombre:         bool tarjetaSDiniciada(void)
**  Descripcion:    Comprueba si la SD esta inicializada
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool tarjetaSDiniciada(void)
{
    return sd.estado >= SD_ESTADO_READY;
}


/***************************************************************************************
**  Nombre:         bool tarjetaSDfuncional(void)
**  Descripcion:    Comprueba si la SD es funcional
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool tarjetaSDfuncional(void)
{
    return sd.estado != SD_ESTADO_NO_PRESENTE;
}


/***************************************************************************************
**  Nombre:         const metadatosSD_t* leerMetadatosSD(void)
**  Descripcion:    Lee los metadatos de la SD
**  Parametros:     Ninguno
**  Retorno:        Metadatos
****************************************************************************************/
const metadatosSD_t* leerMetadatosSD(void)
{
    return &sd.metadatos;
}


/***************************************************************************************
**  Nombre:         bool sondearSD(void)
**  Descripcion:    Hace un poll a la SD
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool sondearSD(void)
{
    if (sd.iniciada && tablaFnSD)
        return tablaFnSD->sondearSD();
    else
        return false;
}


/***************************************************************************************
**  Nombre:         bool leerBloqueSD(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
**  Descripcion:    Lee un bloque de datos de la SD
**  Parametros:     Indice del bloque de datos, buffer de recepcion de los datos, funcion de callback, dato de callback
**  Retorno:        True si ok
****************************************************************************************/
bool leerBloqueSD(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
{
    return tablaFnSD->leerBloqueSD(indice, buffer, callback, datoCallback);
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e iniciarEscrituraBloquesSD(uint32_t indice, uint32_t numBloques)
**  Descripcion:    Inicia la escritura de bloques en la SD
**  Parametros:     Indice del bloque de datos, numero de bloques
**  Retorno:        Estado de la operacion
****************************************************************************************/
estadoOperacionSD_e iniciarEscrituraBloquesSD(uint32_t indice, uint32_t numBloques)
{
    return tablaFnSD->iniciarEscrituraBloquesSD(indice, numBloques);
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e escribirBloqueSD(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
**  Descripcion:    Escribe un bloque en la SD
**  Parametros:     Indice del bloque de datos, buffer de escritura de los datos, funcion de callback, dato de callback
**  Retorno:        Estado de la operacion
****************************************************************************************/
estadoOperacionSD_e escribirBloqueSD(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
{
    return tablaFnSD->escribirBloqueSD(indice, buffer, callback, datoCallback);
}


#endif
