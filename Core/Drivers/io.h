/***************************************************************************************
**  io.h - Funciones para las IOs
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

#ifndef __IO_H
#define __IO_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "io_def.h"
#include "Comun/util.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define DEFIO_TAG(pinid)             CONCAT(DEFIO_TAG__, pinid)
#define DEFIO_PIN(tag)               (1 << (tag & 0x0F))
#define DEFIO_GPIO(tag)              (GPIO_TypeDef *)(GPIOA_BASE + (((((tag & 0xF0) >> 4) - 1) * 4) << 8))
#define DEFIO_TAG__NINGUNO           0
#define TAG_VACIO(tag)               (tag == DEFIO_TAG__NINGUNO)

// Empaquetado de las opciones del GPIO
#define CONFIG_IO(modo, vel, pupd) ((modo) | ((vel) << 8) | ((pupd) << 11))


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t pin;
    uint8_t af;
} pin_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void configurarIO(uint8_t tag, uint16_t cfg, uint8_t af);
void escribirIO(uint8_t tag, bool estado);
bool leerIO(uint8_t tag);
void invertirIO(uint8_t tag);

#endif // __IO_H
