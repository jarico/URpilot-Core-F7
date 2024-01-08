/***************************************************************************************
**  usb.c - Funciones de gestion del USB
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 01/05/2021
**  Fecha de modificacion: 01/05/2021
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "usb.h"

#ifdef USAR_USB
#include "tiempo.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static usb_t usb;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         usb_t *punteroUSB(void)
**  Descripcion:    Devuelve el puntero al USB
**  Parametros:     Ninguno
**  Retorno:        Puntero
****************************************************************************************/
usb_t *punteroUSB(void)
{
    return &usb;
}


/***************************************************************************************
**  Nombre:         bool iniciarUSB(void)
**  Descripcion:    Inicia el USB
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarUSB(void)
{
    usb_t *driver = &usb;

	memset(driver, 0, sizeof(*driver));
    driver->iniciado = false;

    if (iniciarDriverUSB()) {
    	driver->iniciado = true;
        return true;
    }
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion del USB\n");
#endif
        return false;
    }
}


/***************************************************************************************
**  Nombre:         bool usbAbierto(void)
**  Descripcion:    Devuelve si el puerto USB esta abierto
**  Parametros:     Ninguno
**  Retorno:        True si abierto
****************************************************************************************/
bool usbAbierto(void)
{
    return usb.puertoAbierto;
}


/***************************************************************************************
**  Nombre:         bool usbConfigurado(void)
**  Descripcion:    Devuelve si el USB esta configurado
**  Parametros:     Ninguno
**  Retorno:        True si configurado
****************************************************************************************/
bool usbConfigurado(void)
{
    return usb.hal.hUSB.dev_state == USBD_STATE_CONFIGURED;
}


/***************************************************************************************
**  Nombre:         bool usbConectado(void)
**  Descripcion:    Devuelve si el USB esta conectado
**  Parametros:     Ninguno
**  Retorno:        True si conectado
****************************************************************************************/
bool usbConectado(void)
{
    return usb.hal.hUSB.dev_state != USBD_STATE_DEFAULT;
}

#endif

