/***************************************************************************************
**  adc.h - Funciones generales del ADC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/05/2019
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

#ifndef __ADC_H
#define __ADC_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_ADC                 3
#if defined(STM32F767xx)
#define NUM_CANALES_ADC             16      // 16 Canales por cada ADC
#define NUM_MAX_PUERTOS_ADC         16      // Numero maximo de puertos fisicos
#else
#define NUM_CANALES_ADC             14      // 14 Canales por cada ADC
#define NUM_MAX_PUERTOS_ADC         14      // Numero maximo de puertos fisicos
#endif
#define NUM_MAX_CANALES_ADC         NUM_MAX_ADC * NUM_CANALES_ADC
#define VALOR_LIMITE_ADC            4096


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    ADC_1 = 0,
    ADC_2,
    ADC_3,
} numADC_e;

typedef struct {
	bool pinAsignado;
	numADC_e numADC;
    uint8_t canalADC;
} puertoADC_t;

typedef struct {
    uint8_t pin;
    uint8_t canal;
} pinADC_t;

typedef struct {
	pinADC_t pin[NUM_CANALES_ADC];
    ADC_HandleTypeDef hadc;
    DMA_HandleTypeDef hdma;
    bool canalesHabilitados[NUM_CANALES_ADC];
} halADC_t;

typedef struct {
    bool iniciado;
    bool usado;
    halADC_t hal;
    volatile uint16_t valores[NUM_CANALES_ADC];
} adc_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
adc_t *punteroADC(numADC_e numADC);
bool asignarHALadc(numADC_e numADC);
bool iniciarADC(void);
bool iniciarDriverADC(numADC_e numADC);
bool iniciadoADC(void);
puertoADC_t *puertoADC(uint8_t puerto);
void asignarPuertoADC(uint8_t numPuerto, puertoADC_t puerto);
uint16_t leerCanalADC(uint8_t canal);

#ifdef USAR_ADC_INTERNO
void actualizarADCinterno(uint32_t tiempoActual);
void leerADCinterno(uint16_t *vRef, int16_t *tCore);
float temperaturaCore(void);
#endif
float tensionReferencia(void);

#endif // __ADC_H
