/***************************************************************************************
**  barometro.h - Funciones y variables comunes a todos los barometros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 15/05/2019
**  Fecha de modificacion: 21/09/2020
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

#ifndef __BAROMETRO_H_
#define __BAROMETRO_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Drivers/bus.h"
#include "Sensores/sensor.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if defined(STM32F767xx)
  #define NUM_MAX_BARO                5
#elif defined(STM32F722xx)
  #define NUM_MAX_BARO                2
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    BARO_1 = 0,
    BARO_2,
    BARO_3,
    BARO_4,
} numBaro_e;

typedef enum {
    BARO_NINGUNO = -1,
    BARO_MS5611  =  0,
    BARO_BMP180,
} tipoBaro_e;

typedef struct {
    uint32_t ultimaActualizacion;        // Tiempo en us
    uint32_t ultimaMedida;               // Tiempo en us
    uint32_t ultimoCambio;               // Tiempo en us
} timingBaro_t;

typedef struct {
	numBaro_e numBaro;
    bus_t bus;
    void *driver;
    uint8_t drdy;
    float presion;                       // Presion en mBar
    float temperatura;                   // Temperatura en ºC
    float presionSuelo;
    float temperaturaSuelo;
    float correccionPresion;
    float presionMedia;
    uint16_t cntErrorFiltro;
    bool iniciado;
    bool operativo;
    bool calibrado;
    bool nuevaMedida;
    timingBaro_t timing;
} baro_t;

typedef struct {
    bool (*iniciarBaro)(baro_t *dBaro);
    void (*leerBaro)(baro_t *dBaro);
    void (*actualizarBaro)(baro_t *dBaro);
} tablaFnBaro_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern tablaFnBaro_t tablaFnBaroBosch;
extern tablaFnBaro_t tablaFnBaroTEConectivity;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarBaro(void);
void leerBaro(uint32_t tiempoActual);
void actualizarBaro(uint32_t tiempoActual);
bool actualizarCalibracionBaro(void);
bool barosOperativos(void);
bool presionBaroOk(baro_t *dBaro, float presion);
uint8_t numBarosConectados(void);
bool baroGenOperativo(void);

float presionBaro(void);
float temperaturaBaro(void);
float presionSueloBaro(void);
float temperaturaSueloBaro(void);
float presionNumBaro(numBaro_e numBaro);
float temperaturaNumBaro(numBaro_e numBaro);
float presionSueloNumBaro(numBaro_e numBaro);
float temperaturaSueloNumBaro(numBaro_e numBaro);

#endif // __BAROMETRO_H_
