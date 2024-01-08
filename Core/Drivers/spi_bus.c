/***************************************************************************************
**  spi_bus.c - Funciones de alto nivel para leer y escribir por el SPI
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
#include "spi.h"

#if defined(USAR_SPI)
#include "bus.h"
#include "io.h"


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
**  Nombre:         bool ocupadoBusSPI(const bus_t *bus)
**  Descripcion:    Comprueba si el bus SPI esta ocupado
**  Parametros:     Bus
**  Retorno:        True si ocupado
****************************************************************************************/
bool ocupadoBusSPI(const bus_t *bus)
{
    return ocupadoSPI(bus->bus_u.spi.numSPI);
}


/***************************************************************************************
**  Nombre:         void drenarRecepcionBusI2C(const bus_t *bus)
**  Descripcion:    Limpia el buffer de recepcion del SPI
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
void drenarRecepcionBusSPI(const bus_t *bus)
{
    drenarBufferRecepcionSPI(bus->bus_u.spi.numSPI);
}


/***************************************************************************************
**  Nombre:         bool escribirRawBusSPI(const bus_t *bus, uint8_t byteTx)
**  Descripcion:    Escribe un dato por el SPI sin gestionar el pin CS
**  Parametros:     Bus, dato a escribir
**  Retorno:        True si ok
****************************************************************************************/
bool escribirRawBusSPI(const bus_t *bus, uint8_t byteTx)
{
    return escribirSPI(bus->bus_u.spi.numSPI, byteTx);
}


/***************************************************************************************
**  Nombre:         bool escribirBusSPI(const bus_t *bus, uint8_t byteTx)
**  Descripcion:    Escribe un dato por el SPI
**  Parametros:     Bus, dato a escribir
**  Retorno:        True si ok
****************************************************************************************/
bool escribirBusSPI(const bus_t *bus, uint8_t byteTx)
{
    bool estado;

    escribirIO(bus->bus_u.spi.pinCS, false);
    estado = escribirRawBusSPI(bus, byteTx);
    escribirIO(bus->bus_u.spi.pinCS, true);
    return estado;
}


/***************************************************************************************
**  Nombre:         bool escribirRawRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t byteTx)
**  Descripcion:    Escribe un dato en un registro por el SPI sin gestionar el pin CS
**  Parametros:     Bus, registro, dato a escribir
**  Retorno:        True si ok
****************************************************************************************/
bool escribirRawRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t byteTx)
{
    bool estado1, estado2;

    estado1 = escribirSPI(bus->bus_u.spi.numSPI, reg);
    estado2 = escribirSPI(bus->bus_u.spi.numSPI, byteTx);

    if (estado1 == false || estado2 == false)
        return false;
    else
        return true;
}


/***************************************************************************************
**  Nombre:         bool escribirRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t byteTx)
**  Descripcion:    Escribe un dato en un registro por el SPI
**  Parametros:     Bus, registro, dato a escribir
**  Retorno:        True si ok
****************************************************************************************/
bool escribirRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t byteTx)
{
    bool estado;

    escribirIO(bus->bus_u.spi.pinCS, false);
    estado = escribirRawRegistroBusSPI(bus, reg, byteTx);
    escribirIO(bus->bus_u.spi.pinCS, true);
    return estado;
}


/***************************************************************************************
**  Nombre:         bool escribirRawBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
**  Descripcion:    Escribe un buffer en un registro por el SPI sin gestionar el pin CS
**  Parametros:     Bus, registro, buffer a escribir, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
bool escribirRawBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
{
    bool estado1, estado2;

    estado1 = escribirSPI(bus->bus_u.spi.numSPI, reg);
    estado2 = escribirBufferSPI(bus->bus_u.spi.numSPI, datoTx, longitud);

    if (estado1 == false || estado2 == false)
        return false;
    else
        return true;
}


/***************************************************************************************
**  Nombre:         bool escribirBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
**  Descripcion:    Escribe un buffer en un registro por el SPI
**  Parametros:     Bus, registro, buffer a escribir, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
bool escribirBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint16_t longitud)
{
    bool estado;

    escribirIO(bus->bus_u.spi.pinCS, false);
    estado = escribirRawBufferRegistroBusSPI(bus, reg, datoTx, longitud);
    escribirIO(bus->bus_u.spi.pinCS, true);
    return estado;
}


/***************************************************************************************
**  Nombre:         bool transferirRawBusSPI(const bus_t *bus, uint8_t byteTx, uint8_t *byteRx)
**  Descripcion:    Lee y escribe un dato por el SPI sin gestionar el pin CS
**  Parametros:     Bus, dato a escribir, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool transferirRawBusSPI(const bus_t *bus, uint8_t byteTx, uint8_t *byteRx)
{
    return transferirSPI(bus->bus_u.spi.numSPI, byteTx, byteRx);
}


/***************************************************************************************
**  Nombre:         bool transferirBusSPI(const bus_t *bus, uint8_t byteTx, uint8_t *byteRx)
**  Descripcion:    Lee y escribe un dato por el SPI
**  Parametros:     Bus, dato a escribir, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool transferirBusSPI(const bus_t *bus, uint8_t byteTx, uint8_t *byteRx)
{
    bool estado;

    escribirIO(bus->bus_u.spi.pinCS, false);
    estado = transferirRawBusSPI(bus, byteTx, byteRx);
    escribirIO(bus->bus_u.spi.pinCS, true);
    return estado;
}


/***************************************************************************************
**  Nombre:         bool transferirRawBufferBusSPI(const bus_t *bus, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee y escribe un buffer por el SPI sin gestionar el pin CS
**  Parametros:     Bus, buffer a escribir, buffer de lectura, longitud de los buffer
**  Retorno:        True si ok
****************************************************************************************/
bool transferirRawBufferBusSPI(const bus_t *bus, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud)
{
    return transferirBufferSPI(bus->bus_u.spi.numSPI, datoTx, datoRx, longitud);
}


/***************************************************************************************
**  Nombre:         bool transferirBufferBusSPI(const bus_t *bus, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee y escribe un buffer por el SPI
**  Parametros:     Bus, buffer a escribir, buffer de lectura, longitud de los buffer
**  Retorno:        True si ok
****************************************************************************************/
bool transferirBufferBusSPI(const bus_t *bus, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud)
{
    bool estado;

    escribirIO(bus->bus_u.spi.pinCS, false);
    estado = transferirRawBufferBusSPI(bus, datoTx, datoRx, longitud);
    escribirIO(bus->bus_u.spi.pinCS, true);
    return estado;
}


/***************************************************************************************
**  Nombre:         bool leerRawRegistroBusSPI(const busDriver_t *bus, uint8_t reg, uint8_t *byteRx)
**  Descripcion:    Lee un registro por el SPI sin gestionar el pin CS
**  Parametros:     Bus, registro, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leerRawRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *byteRx)
{
    return transferirRawBusSPI(bus, reg, byteRx);
}


/***************************************************************************************
**  Nombre:         bool leerRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *byteRx)
**  Descripcion:    Lee un registro por el SPI
**  Parametros:     Bus, registro, dato leido
**  Retorno:        True si ok
****************************************************************************************/
bool leerRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *byteRx)
{
    bool estado;

    escribirIO(bus->bus_u.spi.pinCS, false);
    estado = leerRawRegistroBusSPI(bus, reg, byteRx);
    escribirIO(bus->bus_u.spi.pinCS, true);
    return estado;
}


/***************************************************************************************
**  Nombre:         bool leerRawBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee un buffer de un registro por el SPI sin gestionar el pin CS
**  Parametros:     Bus, registro, bufer de lectura, longitud del buffer
**  Retorno:        Dato leido
****************************************************************************************/
bool leerRawBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint16_t longitud)
{
    bool estado1, estado2;

    estado1 = escribirSPI(bus->bus_u.spi.numSPI, reg);
    estado2 = leerBufferSPI(bus->bus_u.spi.numSPI, datoRx, longitud);

    if (estado1 == false || estado2 == false)
        return false;
    else
        return true;
}


/***************************************************************************************
**  Nombre:         bool leerBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee un buffer de un registro por el SPI
**  Parametros:     Bus, registro, bufer de lectura, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
bool leerBufferRegistroBusSPI(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint16_t longitud)
{
    bool estado;

    escribirIO(bus->bus_u.spi.pinCS, false);
    estado = leerRawBufferRegistroBusSPI(bus, reg, datoRx, longitud);
    escribirIO(bus->bus_u.spi.pinCS, true);
    return estado;
}

#endif
