/***************************************************************************************
**  spi.c - Funciones generales del SPI
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
#include <stdio.h>
#include <string.h>

#include "spi.h"

#ifdef USAR_SPI
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
static spi_t spi[NUM_MAX_SPI];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         spi_t *punteroSPI(numSPI_e numSPI)
**  Descripcion:    Devuelve el puntero a un SPI seleccionado
**  Parametros:     Dispositivo a devolver
**  Retorno:        Puntero
****************************************************************************************/
spi_t *punteroSPI(numSPI_e numSPI)
{
    return &spi[numSPI];
}


/***************************************************************************************
**  Nombre:         bool iniciarSPI(numSPI_e numSPI)
**  Descripcion:    Inicia el SPI
**  Parametros:     Dispositivo a iniciar
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarSPI(numSPI_e numSPI)
{
    if (numSPI == SPI_NINGUNO) {
#ifdef DEBUG
        printf("Fallo en la definicion del SPI\n");
#endif
        return false;
    }

    spi_t *driver = &spi[numSPI];

    memset(driver, 0, sizeof(*driver));
    resetearContadorErrorSPI(numSPI);
    driver->iniciado = false;

    if (iniciarDriverSPI(numSPI)) {
        driver->iniciado = true;
        return true;
    }
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion del bus SPI %u\n", numSPI + 1);
#endif
        return false;
    }
}


/***************************************************************************************
**  Nombre:         bool spiIniciado(numSPI_e numSPI)
**  Descripcion:    Comprueba si el SPI esta iniciado
**  Parametros:     Numero de SPI
**  Retorno:        True si iniciado
****************************************************************************************/
bool spiIniciado(numSPI_e numSPI)
{
    return spi[numSPI].iniciado;
}


/***************************************************************************************
**  Nombre:         void errorCallbackSPI(numSPI_e numSPI)
**  Descripcion:    Incrementa el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void errorCallbackSPI(numSPI_e numSPI)
{
    spi[numSPI].numErrores++;
}


/***************************************************************************************
**  Nombre:         uint16_t contadorErrorSPI(numSPI_e numSPI)
**  Descripcion:    Devuelve el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Retorna el numero de errores
****************************************************************************************/
uint16_t contadorErrorSPI(numSPI_e numSPI)
{
    return spi[numSPI].numErrores;
}


/***************************************************************************************
**  Nombre:         void resetearContadorErrorSPI(numSPI_e numSPI)
**  Descripcion:    Resetea el contador de errores
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
void resetearContadorErrorSPI(numSPI_e numSPI)
{
    spi[numSPI].numErrores = 0;
}

#endif
