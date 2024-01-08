/***************************************************************************************
**  bus.c - Funciones de bus generales que incluyen el SPI y el I2C
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
#include "bus.h"

#if defined(USAR_SPI) || defined(USAR_I2C)
#include "spi_bus.h"
#include "i2c_bus.h"


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
**  Nombre:         bool busOcupado(const bus_t *bus)
**  Descripcion:    Comprueba si el bus esta ocupado
**  Parametros:     Bus
**  Retorno:        True si ocupado
****************************************************************************************/
CODIGO_RAPIDO bool busOcupado(const bus_t *bus)
{
#if !defined(USAR_SPI) && !defined(USAR_I2C)
    UNUSED(bus);
    return false;
#else
    switch (bus->tipo) {
#ifdef USAR_SPI
    case BUS_SPI:
        return ocupadoBusSPI(bus);
#endif
#ifdef USAR_I2C
    case BUS_I2C:
        return ocupadoBusI2C(bus);
#endif
    default:
        return false;
    }
#endif
}


/***************************************************************************************
**  Nombre:         void drenarRecepcionBus(const bus_t *bus)
**  Descripcion:    Drena el buffer de recepcion del bus
**  Parametros:     Bus
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void drenarRecepcionBus(const bus_t *bus)
{
#if !defined(USAR_SPI) && !defined(USAR_I2C)
    UNUSED(bus);
    return;
#else
    switch (bus->tipo) {
#ifdef USAR_SPI
    case BUS_SPI:
        drenarRecepcionBusSPI(bus);
#endif
#ifdef USAR_I2C
    case BUS_I2C:
    	drenarRecepcionBusI2C(bus);
#endif
    default:
        return;
    }
#endif
}


/***************************************************************************************
**  Nombre:         bool escribirRegistroBus(const bus_t *bus, uint8_t reg, uint8_t byteTx)
**  Descripcion:    Escribe un dato en un registro
**  Parametros:     Bus, registro, dato a escribir
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirRegistroBus(const bus_t *bus, uint8_t reg, uint8_t byteTx)
{
#if !defined(USAR_SPI) && !defined(USAR_I2C)
    UNUSED(bus);
    UNUSED(reg);
    UNUSED(byteTx);
#else
    return escribirBufferRegistroBus(bus, reg, &byteTx, 1);
#endif
}


/***************************************************************************************
**  Nombre:         bool escribirBufferRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint8_t longitud)
**  Descripcion:    Escribe un buffer en un registro
**  Parametros:     Bus, registro, buffer a escribir, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool escribirBufferRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *datoTx, uint8_t longitud)
{
#if !defined(USAR_SPI) && !defined(USAR_I2C)
    UNUSED(bus);
    UNUSED(reg);
    UNUSED(datoTx);
    UNUSED(longitud);
#endif
    if (busOcupado(bus))
        return false;

    switch (bus->tipo) {
#ifdef USAR_SPI
        case BUS_SPI:
            return escribirBufferRegistroBusSPI(bus, reg, datoTx, longitud);
            break;
#endif
#ifdef USAR_I2C
        case BUS_I2C:
            return escribirBufferRegistroBusI2C(bus, reg, datoTx, longitud);
            break;
#endif
        default:
            return false;
    }
}


/***************************************************************************************
**  Nombre:         bool leerRegistroBus(const bus_t *bus, uint8_t reg)
**  Descripcion:    Lee un dato de un registro
**  Parametros:     Bus, registro, dato leido
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *byteRx)
{
#if !defined(USAR_SPI) && !defined(USAR_I2C)
    UNUSED(bus);
    UNUSED(reg);
#else
    return leerBufferRegistroBus(bus, reg, byteRx, 1);
#endif
}


/***************************************************************************************
**  Nombre:         bool leerBufferRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint8_t longitud)
**  Descripcion:    Lee un buffer de un registro
**  Parametros:     Bus, registro, buffer de recepcion, longitud del buffer
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool leerBufferRegistroBus(const bus_t *bus, uint8_t reg, uint8_t *datoRx, uint8_t longitud)
{
#if !defined(USAR_SPI) && !defined(USAR_I2C)
    UNUSED(bus);
    UNUSED(reg);
    UNUSED(datoRx);
    UNUSED(longitud);
#endif

    if (busOcupado(bus))
        return false;

    switch (bus->tipo) {
#ifdef USAR_SPI
        case BUS_SPI:
            return leerBufferRegistroBusSPI(bus, reg, datoRx, longitud);
            break;
#endif
#ifdef USAR_I2C
        case BUS_I2C:
            return leerBufferRegistroBusI2C(bus, reg, datoRx, longitud);
            break;
#endif
        default:
            return false;
    }
}

#endif
