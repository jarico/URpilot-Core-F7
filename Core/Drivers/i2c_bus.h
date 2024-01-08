/***************************************************************************************
**  i2c_bus.h - Funciones de alto nivel para leer y escribir por el I2C
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/05/2019
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

#ifndef __I2C_BUS_H
#define __I2C_BUS_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool ocupadoBusI2C(const bus_t *bus);
void drenarRecepcionBusI2C(const bus_t *bus);

bool escribirBusI2C(const bus_t *bus, uint8_t datoTx);
bool escribirRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t datoTx);
bool escribirBufferRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud);
bool leerBusI2C(const bus_t *bus, uint8_t *byteRx);
bool leerRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *byteRx);
bool leerBufferRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint16_t longitud);

#endif // __I2C_BUS_H
