/***************************************************************************************
**  timer.h - Funciones generales de los Timer
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 17/08/2019
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

#ifndef __TIMER_H
#define __TIMER_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "io.h"
#include "GP/gp.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_TIMERS                   14
#define NUM_CANALES_POR_TIMER        4
#define NUM_CANALES_TIMER            (NUM_TIMERS * NUM_CANALES_POR_TIMER)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    TIMER_NINGUNO = -1,
    TIMER_1       =  0,
    TIMER_2,
    TIMER_3,
    TIMER_4,
    TIMER_5,
    TIMER_6,
    TIMER_7,
    TIMER_8,
    TIMER_9,
    TIMER_10,
    TIMER_11,
    TIMER_12,
    TIMER_13,
    TIMER_14,
} numTimer_e;

typedef enum {
    T_CANAL_1 = 0,
    T_CANAL_2,
    T_CANAL_3,
    T_CANAL_4,
} numCanalTimer_e;

typedef enum {
    TIMER_SALIDA_NORMAL = 0,
    TIMER_SALIDA_INVERTIDA,
} tipoSalida_e;

typedef enum {
	TIMER_CANAL_ESTANDAR = 0,
	TIMER_CANAL_N,
} tipoCanal_e;

typedef enum {
    TIM_USO_NINGUNO = 0x00,
    TIM_USO_PPM     = 0X01,
    TIM_USO_PWM     = 0X02,
    TIM_USO_MOTOR   = 0X04,
    TIM_USO_SERVO   = 0X08,
    TIM_USO_LED     = 0x10,
    TIM_USO_BEEPER  = 0x20,
    TIM_USO_USB     = 0x40,
} usoTimer_e;

struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* callback, uint16_t captura);
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* callback, uint16_t captura);
typedef void timerPeriodoCallback(void);

typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerCCHandlerRec_t;

typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* siguiente;
} timerOvrHandlerRec_t;

typedef struct {
    TIM_TypeDef *tim;
    volatile uint32_t *ccr;
} canal_t;

typedef struct {
    uint8_t pin;
    numTimer_e numTimer;
} pinConfigTimer_t;

typedef struct {
    numTimer_e numTimer;
    bool asignado;
    uint32_t canal;
    tipoCanal_e tipoCanal;
    pin_t pin;
} tim_t;

typedef struct {
    bool asignado;
    TIM_HandleTypeDef htim;
    uint8_t IRQ;
    uint8_t prioridadIRQ;
} halTim_t;

typedef struct {
    DMA_Stream_TypeDef *DMAy_Streamx;
    uint32_t canalDMA;
    uint8_t dmaTimIrqHandler;
} halTimDMA_t;

typedef struct {
    usoTimer_e uso;
    bool baseConfigurada;
    halTim_t hal;
    halTimDMA_t halTimDMA[NUM_CANALES_POR_TIMER];
    halTimDMA_t halTimUPdma;
    timerCCHandlerRec_t *edgeCallback[NUM_CANALES_POR_TIMER];
    timerOvrHandlerRec_t *overflowCallback[NUM_CANALES_POR_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActivo;
    timerPeriodoCallback *periodoCallback;
    uint32_t valorForzadoOverflowTimer;
} timerHAL_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
tim_t *punteroTim(uint8_t numTim);
timerHAL_t *punteroTimer(numTimer_e numTimer);
bool timerUsado(tim_t *dTim);
tim_t *timerPorTag(uint8_t tag, bool *encontrado);
bool asignarPinTimer(uint8_t numDriver, numTimer_e numTimer, uint8_t pin);
bool asignarHALtimer(numTimer_e numTimer);
bool iniciarTimer(numTimer_e numTimer, uint8_t pin, usoTimer_e uso);
bool configurarBaseTiempoTimer(numTimer_e numTimer, bool interrupcion, uint16_t periodo, uint32_t frec);
bool configurarOCtimer(tim_t *dTim, uint16_t pulsoReposo, bool inversion);
bool configurarSalidaPWMtimer(tim_t *dTim, canal_t *canal, uint32_t frec, uint16_t periodo, uint16_t pulsoReposo, bool inversion);
bool configurarICtimer(tim_t *dTim, bool polaridad);
bool configurarCapturaEntradaTimer(tim_t *tim, uint16_t periodo, uint32_t frec, bool polaridad);
uint16_t fuenteDMAtimer(uint8_t canal);
uint16_t indiceDMAtimer(uint8_t canal);
void asignarCallbackPeriodoTimer(numTimer_e numTimer, timerPeriodoCallback *fn);
void asignarCallbackCCtimer(timerCCHandlerRec_t *callback, timerCCHandlerCallback *fn);
void asignarCallbackOvrTimer(timerOvrHandlerRec_t *callback, timerOvrHandlerCallback *fn);
void configurarCallbacksTimer(tim_t *dTtim, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void forzarOverflowTimer(tim_t *dTim);

#endif // __TIMER_H
