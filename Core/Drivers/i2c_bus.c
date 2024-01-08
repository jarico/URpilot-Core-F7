/***************************************************************************************
**  i2c_bus.c - Funciones de alto nivel para leer y escribir por el I2C
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "i2c.h"

#if defined(USAR_I2C)
#include "bus.h"


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


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool ocupadoBusI2C(const bus_t *bus)
**  Descripcion:    Comprueba si el bus I2C esta ocupado
**  Parametros:     Bus
**  Retorno:        True si ocupado
****************************************************************************************/
bool ocupadoBusI2C(const bus_t *bus)
{
    return ocupadoI2C(bus->bus_u.i2c.numI2C);
}


/***************************************************************************************
**  Nombre:         void drenarRecepcionBusI2C(const bus_t *bus)
**  Descripcion:    Limpia el buffer de recepcion del I2C
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void drenarRecepcionBusI2C(const bus_t *bus)
{
    drenarBufferRecepcionI2C(bus->bus_u.i2c.numI2C);
}


/***************************************************************************************
**  Nombre:         bool escribirBusI2C(const bus_t *bus, uint8_t datoTx)
**  Descripcion:    Escribe un dato en el bus I2C
**  Parametros:     Bus, dato a escribir
**  Retorno:        True si ok
****************************************************************************************/
bool escribirBusI2C(const bus_t *bus, uint8_t datoTx)
{
    return escribirI2C(bus->bus_u.i2c.numI2C, bus->bus_u.i2c.dir, datoTx);
}


/***************************************************************************************
**  Nombre:         bool escribirRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t datoTx)
**  Descripcion:    Escribe en un registro por el bus I2C
**  Parametros:     Bus, registro, dato a escribir
**  Retorno:        True si ok
****************************************************************************************/
bool escribirRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t datoTx)
{
    return escribirMemI2C(bus->bus_u.i2c.numI2C, bus->bus_u.i2c.dir, reg, datoTx);
}


/***************************************************************************************
**  Nombre:         bool escribirBufferRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
**  Descripcion:    Escribe un buffer en un registro por el bus I2C
**  Parametros:     Bus, registro, buffer a escribir, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
bool escribirBufferRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
{
    return escribirBufferMemI2C(bus->bus_u.i2c.numI2C, bus->bus_u.i2c.dir, reg, datoTx, longitud);
}


/***************************************************************************************
**  Nombre:         bool leerBusI2C(const bus_t *bus, uint8_t *byteRx)
**  Descripcion:    Lee un dato por el I2C
**  Parametros:     Bus, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leerBusI2C(const bus_t *bus, uint8_t *byteRx)
{
    return leerI2C(bus->bus_u.i2c.numI2C, bus->bus_u.i2c.dir, byteRx);
}


/***************************************************************************************
**  Nombre:         bool leerRegistroBusI2C(const bus_t *bus, uint8_t reg)
**  Descripcion:    Lee un registro por el I2C
**  Parametros:     Bus, registro a leer, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leerRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *byteRx)
{
    return leerMemI2C(bus->bus_u.i2c.numI2C, bus->bus_u.i2c.dir, reg, byteRx);
}


/***************************************************************************************
**  Nombre:         bool leerBufferRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint8_t longitud)
**  Descripcion:    Lee un buffer de un registro por el I2C
**  Parametros:     Bus, registro a leer, buffer de recepcion, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
bool leerBufferRegistroBusI2C(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint8_t longitud)
{
    return leerBufferMemI2C(bus->bus_u.i2c.numI2C, bus->bus_u.i2c.dir, reg, datoRx, longitud);
}

#endif
