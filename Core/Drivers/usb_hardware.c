/***************************************************************************************
**  usb_hardware.c - Hardware del USB
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 08/05/2021
**  Fecha de modificacion: 08/05/2021
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
#include "usb.h"

#ifdef USAR_USB
#include "io.h"
#include "nvic.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	PCD_TypeDef* reg;
    pin_t pinDP;
    pin_t pinDM;
    uint8_t IRQ;
    uint8_t prioridadIRQ;
} hardwareUSB_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const hardwareUSB_t hardwareUSB = {
	.reg = USB_OTG_FS,
    .pinDP = { DEFIO_TAG(PA12), GPIO_AF10_OTG_FS },
    .pinDM = { DEFIO_TAG(PA11), GPIO_AF10_OTG_FS },
    .IRQ = OTG_FS_IRQn,
    .prioridadIRQ = NVIC_PRIO_USB
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool asignarHALusb(void)
**  Descripcion:    Asigna el HAL a la estructura del driver
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool asignarHALusb(void)
{
    usb_t *driver = punteroUSB();

    // Asignamos los pines
    driver->hal.pinDP = hardwareUSB.pinDP;
    driver->hal.pinDM = hardwareUSB.pinDM;

    // Asignamos las interrupciones
    driver->hal.IRQ = hardwareUSB.IRQ;
    driver->hal.prioridadIRQ = hardwareUSB.prioridadIRQ;

    // Asignamos la instancia
    driver->hal.hPCD.Instance = hardwareUSB.reg;

    return true;
}

#endif // USAR_USB
