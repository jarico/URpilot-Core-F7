/***************************************************************************************
**  spi.h - Funciones generales del SPI
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

#ifndef __SPI_H
#define __SPI_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "io.h"
#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_SPI     6


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    SPI_NINGUNO = -1,
    SPI_1       =  0,
    SPI_2,
    SPI_3,
    SPI_4,
    SPI_5,
    SPI_6,
} numSPI_e;

typedef enum {
    SPI_RELOJ_INICIALIZACION = 256,
	SPI_RELOJ_LENTO          = 256,    // 00.42188 MHz
	SPI_RELOJ_ESTANDAR       = 16,     // 06.57500 MHz
	SPI_RELOJ_RAPIDO         = 8,      // 13.50000 MHz
	SPI_RELOJ_ULTRARAPIDO    = 2,      // 54.00000 MHz
} divisorRelojSPI_e;

typedef struct {
    bool asignado;
	SPI_HandleTypeDef hspi;
    pin_t pinSCK;
    pin_t pinMISO;
    pin_t pinMOSI;
} halSPI_t;

typedef struct {
	bool iniciado;
	halSPI_t hal;
    volatile uint16_t numErrores;
} spi_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool asignarHALspi(numSPI_e numSPI);
spi_t *punteroSPI(numSPI_e numSPI);
bool iniciarSPI(numSPI_e numSPI);
bool iniciarDriverSPI(numSPI_e numSPI);
bool spiIniciado(numSPI_e numSPI);
void ajustarRelojSPI(numSPI_e numSPI, divisorRelojSPI_e divisor);
void drenarBufferRecepcionSPI(numSPI_e numSPI);
bool ocupadoSPI(numSPI_e numSPI);

void errorCallbackSPI(numSPI_e numSPI);
uint16_t contadorErrorSPI(numSPI_e numSPI);
void resetearContadorErrorSPI(numSPI_e numSPI);

bool escribirSPI(numSPI_e numSPI, uint8_t byteTx);
bool escribirBufferSPI(numSPI_e numSPI, uint8_t *datoTx, uint16_t longitud);
bool leerSPI(numSPI_e numSPI, uint8_t *byteRx);
bool leerBufferSPI(numSPI_e numSPI, uint8_t *datoRx, uint16_t longitud);
bool transferirSPI(numSPI_e numSPI, uint8_t byteTx, uint8_t *byteRx);
bool transferirBufferSPI(numSPI_e numSPI, uint8_t *datoTx, uint8_t *datoRx, uint16_t longitud);

#endif // __SPI_H
