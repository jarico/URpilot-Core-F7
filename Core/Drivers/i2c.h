/***************************************************************************************
**  i2c.h - Funciones generales del I2C
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

#ifndef __I2C_H
#define __I2C_H

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
#define NUM_MAX_I2C      4


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    I2C_NINGUNO = -1,
    I2C_1       =  0,
    I2C_2,
    I2C_3,
    I2C_4,
} numI2C_e;

typedef enum {
    I2C_RELOJ_ESTANDAR    = 0x20404768,       // 100 KHz
    I2C_RELOJ_RAPIDO      = 0x6000030D,       // 400 KHz
    I2C_RELOJ_RAPIDO_PLUS = 0x00200922,       // 1000 KHz
    I2C_RELOJ_OVERCLOCK   = 0x00500D1D,       // 8000 KHz
} divisorRelojI2C_e;

typedef struct {
    bool asignado;
    I2C_HandleTypeDef hi2c;
    pin_t pinSCL;
    pin_t pinSDA;
} halI2C_t;

typedef struct {
    bool iniciado;
    volatile uint16_t numErrores;
    halI2C_t hal;
} i2c_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
i2c_t *punteroI2C(numI2C_e numI2C);
bool asignarHALi2c(numI2C_e numI2C);
bool iniciarI2C(numI2C_e numI2C);
bool iniciarDriverI2C(numI2C_e numI2C);
bool i2cIniciado(numI2C_e numI2C);
void ajustarRelojI2C(numI2C_e numI2C, divisorRelojI2C_e divisor);
void drenarBufferRecepcionI2C(numI2C_e numI2C);
bool ocupadoI2C(numI2C_e numI2C);

void errorCallbackI2C(numI2C_e numI2C);
uint16_t contadorErrorI2C(numI2C_e numI2C);
void resetearContadorErrorI2C(numI2C_e numI2C);

bool escribirMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t byteTx);
bool escribirBufferMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *datoTx, uint16_t longitud);
bool escribirI2C(numI2C_e numI2C, uint8_t dir, uint8_t byteTx);
bool escribirBufferI2C(numI2C_e numI2C, uint8_t dir, uint8_t *datoTx, uint16_t longitud);
bool leerMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *byteRx);
bool leerBufferMemI2C(numI2C_e numI2C, uint8_t dir, uint8_t reg, uint8_t *datoRx, uint16_t longitud);
bool leerI2C(numI2C_e numI2C, uint8_t dir, uint8_t *byteRx);
bool leerBufferI2C(numI2C_e numI2C, uint8_t dir, uint8_t *datoRx, uint16_t longitud);

#endif // __I2C_H
