/***************************************************************************************
**  spi_bus.h - Funciones de alto nivel para leer y escribir por el SPI
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

#ifndef __SPI_BUS_H
#define __SPI_BUS_H

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
bool ocupadoBusSPI(const bus_t *bus);
void drenarRecepcionBusSPI(const bus_t *bus);

bool escribirRawBusSPI(const bus_t *bus, uint8_t byteTx);
bool escribirBusSPI(const bus_t *bus, uint8_t byteTx);
bool escribirRawRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t byteTx);
bool escribirRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t byteTx);
bool escribirRawBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud);
bool escribirBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud);

bool transferirRawBusSPI(const bus_t *bus, uint8_t byteTx, uint8_t *byteRx);
bool transferirBusSPI(const bus_t *bus, uint8_t byteTx, uint8_t *byteRx);
bool transferirRawBufferBusSPI(const bus_t *bus, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud);
bool transferirBufferBusSPI(const bus_t *bus, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud);

bool leerRawRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *byteRx);
bool leerRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *byteRx);
bool leerRawBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint16_t longitud);
bool leerBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint16_t longitud);

#endif // __SPI_BUS_H
