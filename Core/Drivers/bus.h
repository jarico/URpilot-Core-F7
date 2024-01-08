/***************************************************************************************
**  bus.h - Funciones de bus generales que incluyen el SPI y el I2C
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

#ifndef __BUS_H
#define __BUS_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "spi.h"
#include "i2c.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    BUS_NINGUNO = -1,
    BUS_I2C     =  0,
    BUS_SPI,
} tipoBus_e;

typedef struct {
	tipoBus_e tipo;
    union {
        struct {
            numSPI_e numSPI;
            uint8_t pinCS;
        } spi;
        struct {
            numI2C_e numI2C;
            uint8_t dir;
        } i2c;
    } bus_u;
} bus_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool busOcupado(const bus_t *bus);
void drenarRecepcionBus(const bus_t *bus);
bool escribirRegistroBus(const bus_t *bus, uint8_t reg, uint8_t byteTx);
bool escribirBufferRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint8_t longitud);
bool leerRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *byteRx);
bool leerBufferRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint8_t longitud);

#endif // __BUS_H
