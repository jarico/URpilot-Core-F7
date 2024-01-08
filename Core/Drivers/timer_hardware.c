/***************************************************************************************
**  timer_hardware.c - Hardware de los Timer
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <string.h>

#include "timer.h"
#include "io.h"
#include "dma.h"
#include "nvic.h"

#ifdef USAR_TIMERS


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_PIN_SEL_TIMER        17
#define NUM_CANALES_DMA_TIMER        23
#define NUM_TIMER_UP                 8


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t pin;
    tipoCanal_e tipoCanal;
    uint8_t af;
    uint8_t canal;
} pinTimer_t;

typedef struct {
    numTimer_e numTimer;
    TIM_TypeDef *reg;
    pinTimer_t pin[NUM_MAX_PIN_SEL_TIMER];
    uint8_t IRQ;
    uint8_t prioridadIRQ;
} hardwareTimer_t;

typedef struct {
    TIM_TypeDef *tim;
    uint32_t canal;
    DMA_Stream_TypeDef *DMAy_Streamx;
    uint32_t canalDMA;
    uint8_t dmaTimIrqHandler;
} hardwareDMAcanalTimer_t;

typedef struct {
    TIM_TypeDef *tim;
    DMA_Stream_TypeDef *DMAy_Streamx;
    uint32_t canalDMA;
    uint8_t dmaTimUPirqHandler;
} hardwareDMAtimerUP_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const hardwareTimer_t hardwareTimer[] = {
    {
        .numTimer = TIMER_1,
        .reg = TIM1,
        .pin = {
            // CH1
            { DEFIO_TAG(PA8),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PE9),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_1 },
            // CH1N
            { DEFIO_TAG(PA7),  TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PB13), TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PE8),  TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_1 },
            // CH2
            { DEFIO_TAG(PA9),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PE11), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_2 },
            // CH2N
            { DEFIO_TAG(PB0),  TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PB14), TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PE10), TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_2 },
            // CH3
            { DEFIO_TAG(PA10), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_3 },
            { DEFIO_TAG(PE13), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_3 },
            // CH3N
            { DEFIO_TAG(PB1),  TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_3 },
            { DEFIO_TAG(PB15), TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_3 },
            { DEFIO_TAG(PE12), TIMER_CANAL_N,         GPIO_AF1_TIM1,  TIM_CHANNEL_3 },
            // CH4
            { DEFIO_TAG(PA11), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_4 },
            { DEFIO_TAG(PE14), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM1,  TIM_CHANNEL_4 },
        },
        .IRQ = TIM1_CC_IRQn,
		.prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_2,
        .reg = TIM2,
        .pin = {
            // CH1
            { DEFIO_TAG(PA0),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PA5),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_1 },
#if defined(STM32F722xx)
			{ DEFIO_TAG(PA15), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_1 },
#endif
            // CH2
            { DEFIO_TAG(PA1),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PB3),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_2 },
            // CH3
            { DEFIO_TAG(PA2),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_3 },
            { DEFIO_TAG(PB10), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_3 },
            // CH4
            { DEFIO_TAG(PA3),  TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_4 },
            { DEFIO_TAG(PB11), TIMER_CANAL_ESTANDAR,  GPIO_AF1_TIM2,  TIM_CHANNEL_4 },
        },
        .IRQ = TIM2_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_3,
        .reg = TIM3,
        .pin = {
            // CH1
            { DEFIO_TAG(PA6),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PB4),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PC6),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_1 },
            // CH2
            { DEFIO_TAG(PA7),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PB5),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PC7),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_2 },
            // CH3
            { DEFIO_TAG(PB0),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_3 },
            { DEFIO_TAG(PC8),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_3 },
            // CH4
            { DEFIO_TAG(PB1),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_4 },
            { DEFIO_TAG(PC9),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM3,  TIM_CHANNEL_4 },
        },
        .IRQ = TIM3_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_4,
        .reg = TIM4,
        .pin = {
            // CH1
            { DEFIO_TAG(PB6),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PD12), TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_1 },
            // CH2
            { DEFIO_TAG(PB7),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PD13), TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_2 },
            // CH3
            { DEFIO_TAG(PB8),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_3 },
            { DEFIO_TAG(PD14), TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_3 },
            // CH4
            { DEFIO_TAG(PB9),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_4 },
            { DEFIO_TAG(PD15), TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM4,  TIM_CHANNEL_4 },
        },
        .IRQ = TIM4_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_5,
        .reg = TIM5,
        .pin = {
            // CH1
            { DEFIO_TAG(PA0),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM5,  TIM_CHANNEL_1 },
            // CH2
            { DEFIO_TAG(PA1),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM5,  TIM_CHANNEL_2 },
            // CH3
            { DEFIO_TAG(PA2),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM5,  TIM_CHANNEL_3 },
            // CH4
            { DEFIO_TAG(PA3),  TIMER_CANAL_ESTANDAR,  GPIO_AF2_TIM5,  TIM_CHANNEL_4 },
        },
        .IRQ = TIM5_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_6,
        .reg = TIM6,
        .pin = {
        },
        .IRQ = TIM6_DAC_IRQn,
        .prioridadIRQ = NVIC_PRIO_USB,
    },
    {
        .numTimer = TIMER_7,
        .reg = TIM7,
        .pin = {
        },
        .IRQ = TIM7_IRQn,
        .prioridadIRQ = NVIC_PRIO_USB,
    },
    {
        .numTimer = TIMER_8,
        .reg = TIM8,
        .pin = {
            // CH1
            { DEFIO_TAG(PC6),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM8,  TIM_CHANNEL_1 },
            // CH1N
            { DEFIO_TAG(PA5),  TIMER_CANAL_N,         GPIO_AF3_TIM8,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PA7),  TIMER_CANAL_N,         GPIO_AF3_TIM8,  TIM_CHANNEL_1 },
            // CH2
            { DEFIO_TAG(PC7),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM8,  TIM_CHANNEL_2 },
            // CH2N
            { DEFIO_TAG(PB0),  TIMER_CANAL_N,         GPIO_AF3_TIM8,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PB14), TIMER_CANAL_N,         GPIO_AF3_TIM8,  TIM_CHANNEL_2 },
            // CH3
            { DEFIO_TAG(PC8),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM8,  TIM_CHANNEL_3 },
            // CH3N
            { DEFIO_TAG(PB1),  TIMER_CANAL_N,         GPIO_AF3_TIM8,  TIM_CHANNEL_3 },
            { DEFIO_TAG(PB15), TIMER_CANAL_N,         GPIO_AF3_TIM8,  TIM_CHANNEL_3 },
            // CH4
            { DEFIO_TAG(PC9),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM8,  TIM_CHANNEL_4 },
        },
        .IRQ = TIM8_CC_IRQn,
		.prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_9,
        .reg = TIM9,
        .pin = {
            // CH1
            { DEFIO_TAG(PA2),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM9,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PE5),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM9,  TIM_CHANNEL_1 },
            // CH2
            { DEFIO_TAG(PA3),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM9,  TIM_CHANNEL_2 },
            { DEFIO_TAG(PE6),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM9,  TIM_CHANNEL_2 },
        },
        .IRQ = TIM1_BRK_TIM9_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_10,
        .reg = TIM10,
        .pin = {
            // CH1
            { DEFIO_TAG(PB8),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM10,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PF6),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM10,  TIM_CHANNEL_1 },
        },
        .IRQ = TIM1_UP_TIM10_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_11,
        .reg = TIM11,
        .pin = {
            // CH1
            { DEFIO_TAG(PB9),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM11,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PF7),  TIMER_CANAL_ESTANDAR,  GPIO_AF3_TIM11,  TIM_CHANNEL_1 },
        },
        .IRQ = TIM1_TRG_COM_TIM11_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_12,
        .reg = TIM12,
        .pin = {
            // CH1
            { DEFIO_TAG(PB14),  TIMER_CANAL_ESTANDAR,  GPIO_AF9_TIM12,  TIM_CHANNEL_1 },
            // CH2
            { DEFIO_TAG(PB15),  TIMER_CANAL_ESTANDAR,  GPIO_AF9_TIM12,  TIM_CHANNEL_2 },
        },
        .IRQ = TIM8_BRK_TIM12_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_13,
        .reg = TIM13,
        .pin = {
            // CH1
            { DEFIO_TAG(PA6),  TIMER_CANAL_ESTANDAR,  GPIO_AF9_TIM13,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PF8),  TIMER_CANAL_ESTANDAR,  GPIO_AF9_TIM13,  TIM_CHANNEL_1 },
        },
        .IRQ = TIM8_UP_TIM13_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
    {
        .numTimer = TIMER_14,
        .reg = TIM14,
        .pin = {
            // CH1
            { DEFIO_TAG(PA7),  TIMER_CANAL_ESTANDAR,  GPIO_AF9_TIM14,  TIM_CHANNEL_1 },
            { DEFIO_TAG(PF9),  TIMER_CANAL_ESTANDAR,  GPIO_AF9_TIM14,  TIM_CHANNEL_1 },
        },
        .IRQ = TIM8_TRG_COM_TIM14_IRQn,
        .prioridadIRQ = NVIC_PRIO_TIMER,
    },
};

static const hardwareDMAcanalTimer_t hardwareDMAcanalTimer[] = {
    { .tim = TIM1,  .canal = TIM_CHANNEL_1,  .DMAy_Streamx = DMA2_Stream6,  .canalDMA = DMA_CHANNEL_0,  .dmaTimIrqHandler = DMA2_ST6_HANDLER },
    { .tim = TIM1,  .canal = TIM_CHANNEL_2,  .DMAy_Streamx = DMA2_Stream2,  .canalDMA = DMA_CHANNEL_6,  .dmaTimIrqHandler = DMA2_ST2_HANDLER },
    { .tim = TIM1,  .canal = TIM_CHANNEL_3,  .DMAy_Streamx = DMA2_Stream6,  .canalDMA = DMA_CHANNEL_6,  .dmaTimIrqHandler = DMA2_ST6_HANDLER },
    { .tim = TIM1,  .canal = TIM_CHANNEL_4,  .DMAy_Streamx = DMA2_Stream4,  .canalDMA = DMA_CHANNEL_6,  .dmaTimIrqHandler = DMA2_ST4_HANDLER },
    { .tim = TIM2,  .canal = TIM_CHANNEL_1,  .DMAy_Streamx = DMA1_Stream5,  .canalDMA = DMA_CHANNEL_3,  .dmaTimIrqHandler = DMA1_ST5_HANDLER },
    { .tim = TIM2,  .canal = TIM_CHANNEL_2,  .DMAy_Streamx = DMA1_Stream6,  .canalDMA = DMA_CHANNEL_3,  .dmaTimIrqHandler = DMA1_ST6_HANDLER },
    { .tim = TIM2,  .canal = TIM_CHANNEL_3,  .DMAy_Streamx = DMA1_Stream1,  .canalDMA = DMA_CHANNEL_3,  .dmaTimIrqHandler = DMA1_ST1_HANDLER },
    { .tim = TIM2,  .canal = TIM_CHANNEL_4,  .DMAy_Streamx = DMA1_Stream6,  .canalDMA = DMA_CHANNEL_3,  .dmaTimIrqHandler = DMA1_ST6_HANDLER },
    { .tim = TIM3,  .canal = TIM_CHANNEL_1,  .DMAy_Streamx = DMA1_Stream4,  .canalDMA = DMA_CHANNEL_5,  .dmaTimIrqHandler = DMA1_ST4_HANDLER },
    { .tim = TIM3,  .canal = TIM_CHANNEL_2,  .DMAy_Streamx = DMA1_Stream5,  .canalDMA = DMA_CHANNEL_5,  .dmaTimIrqHandler = DMA1_ST5_HANDLER },
    { .tim = TIM3,  .canal = TIM_CHANNEL_3,  .DMAy_Streamx = DMA1_Stream7,  .canalDMA = DMA_CHANNEL_5,  .dmaTimIrqHandler = DMA1_ST7_HANDLER },
    { .tim = TIM3,  .canal = TIM_CHANNEL_4,  .DMAy_Streamx = DMA1_Stream2,  .canalDMA = DMA_CHANNEL_5,  .dmaTimIrqHandler = DMA1_ST2_HANDLER },
    { .tim = TIM4,  .canal = TIM_CHANNEL_1,  .DMAy_Streamx = DMA1_Stream0,  .canalDMA = DMA_CHANNEL_2,  .dmaTimIrqHandler = DMA1_ST0_HANDLER },
    { .tim = TIM4,  .canal = TIM_CHANNEL_2,  .DMAy_Streamx = DMA1_Stream3,  .canalDMA = DMA_CHANNEL_2,  .dmaTimIrqHandler = DMA1_ST3_HANDLER },
    { .tim = TIM4,  .canal = TIM_CHANNEL_3,  .DMAy_Streamx = DMA1_Stream7,  .canalDMA = DMA_CHANNEL_2,  .dmaTimIrqHandler = DMA1_ST7_HANDLER },
    { .tim = TIM5,  .canal = TIM_CHANNEL_1,  .DMAy_Streamx = DMA1_Stream2,  .canalDMA = DMA_CHANNEL_6,  .dmaTimIrqHandler = DMA1_ST2_HANDLER },
    { .tim = TIM5,  .canal = TIM_CHANNEL_2,  .DMAy_Streamx = DMA1_Stream4,  .canalDMA = DMA_CHANNEL_6,  .dmaTimIrqHandler = DMA1_ST4_HANDLER },
    { .tim = TIM5,  .canal = TIM_CHANNEL_3,  .DMAy_Streamx = DMA1_Stream0,  .canalDMA = DMA_CHANNEL_6,  .dmaTimIrqHandler = DMA1_ST0_HANDLER },
    { .tim = TIM5,  .canal = TIM_CHANNEL_4,  .DMAy_Streamx = DMA1_Stream1,  .canalDMA = DMA_CHANNEL_6,  .dmaTimIrqHandler = DMA2_ST1_HANDLER },
    { .tim = TIM8,  .canal = TIM_CHANNEL_1,  .DMAy_Streamx = DMA2_Stream2,  .canalDMA = DMA_CHANNEL_7,  .dmaTimIrqHandler = DMA2_ST2_HANDLER },
    { .tim = TIM8,  .canal = TIM_CHANNEL_2,  .DMAy_Streamx = DMA2_Stream3,  .canalDMA = DMA_CHANNEL_7,  .dmaTimIrqHandler = DMA2_ST3_HANDLER },
    { .tim = TIM8,  .canal = TIM_CHANNEL_3,  .DMAy_Streamx = DMA2_Stream4,  .canalDMA = DMA_CHANNEL_7,  .dmaTimIrqHandler = DMA2_ST4_HANDLER },
    { .tim = TIM8,  .canal = TIM_CHANNEL_4,  .DMAy_Streamx = DMA2_Stream7,  .canalDMA = DMA_CHANNEL_7,  .dmaTimIrqHandler = DMA2_ST7_HANDLER },
};

static const hardwareDMAtimerUP_t hardwareDMAtimerUP[] = {
    { .tim = TIM1,  .DMAy_Streamx = DMA2_Stream5,  .canalDMA = DMA_CHANNEL_6,  .dmaTimUPirqHandler = DMA2_ST5_HANDLER },
    { .tim = TIM2,  .DMAy_Streamx = DMA1_Stream7,  .canalDMA = DMA_CHANNEL_3,  .dmaTimUPirqHandler = DMA1_ST7_HANDLER },
    { .tim = TIM3,  .DMAy_Streamx = DMA1_Stream2,  .canalDMA = DMA_CHANNEL_5,  .dmaTimUPirqHandler = DMA1_ST2_HANDLER },
    { .tim = TIM4,  .DMAy_Streamx = DMA1_Stream6,  .canalDMA = DMA_CHANNEL_2,  .dmaTimUPirqHandler = DMA1_ST6_HANDLER },
    { .tim = TIM5,  .DMAy_Streamx = DMA1_Stream6,  .canalDMA = DMA_CHANNEL_6,  .dmaTimUPirqHandler = DMA1_ST6_HANDLER },
    { .tim = TIM6,  .DMAy_Streamx = DMA1_Stream1,  .canalDMA = DMA_CHANNEL_7,  .dmaTimUPirqHandler = DMA1_ST1_HANDLER },
    { .tim = TIM7,  .DMAy_Streamx = DMA1_Stream2,  .canalDMA = DMA_CHANNEL_1,  .dmaTimUPirqHandler = DMA1_ST2_HANDLER },
    { .tim = TIM8,  .DMAy_Streamx = DMA2_Stream1,  .canalDMA = DMA_CHANNEL_7,  .dmaTimUPirqHandler = DMA2_ST1_HANDLER },
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool pinCanalTimer(numTimer_e numTimer, uint8_t pinBusqueda, pin_t *pinTimer, uint32_t *canal, tipoCanal_e *tipoCanal);
void dmaTimer(timerHAL_t *dTimer);
void dmaTimerUP(timerHAL_t *dTimer);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool asignarPinTimer(uint8_t numDriver, numTimer_e numTimer, uint8_t pin)
**  Descripcion:    Asigna el hardware del pin
**  Parametros:     Numero del driver, numero de timer, pin del driver
**  Retorno:        True si ok
****************************************************************************************/
bool asignarPinTimer(uint8_t numDriver, numTimer_e numTimer, uint8_t pin)
{
    if (numTimer == TIMER_6 || numTimer == TIMER_7)
        return true;

    tim_t *driver = punteroTim(numDriver);

    if (!pinCanalTimer(numTimer, pin, &driver->pin, &driver->canal, &driver->tipoCanal))
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool asignarHALtimer(numTimer_e numTimer)
**  Descripcion:    Asigna el HAL a la estructura del driver
**  Parametros:     Numero de timer
**  Retorno:        True si ok
****************************************************************************************/
bool asignarHALtimer(numTimer_e numTimer)
{
    timerHAL_t *driver = punteroTimer(numTimer);

    // Asignamos la instancia
    driver->hal.htim.Instance = hardwareTimer[numTimer].reg;

    // Asignamos las interrupciones
    driver->hal.IRQ = hardwareTimer[numTimer].IRQ;
    driver->hal.prioridadIRQ = hardwareTimer[numTimer].prioridadIRQ;

    if (numTimer != TIMER_6 && numTimer != TIMER_7) {
        // Obtenemos el DMA
        dmaTimer(driver);
        dmaTimerUP(driver);
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool pinCanalTimer(numTimer_e numTimer, uint8_t pinBusqueda, pin_t *pinTimer, uint32_t *canal, tipoCanal_e *tipoCanal)
**  Descripcion:    Encuentra el pin de la tabla de hardware
**  Parametros:     Numero de Timer, pin a buscar, pin del timer, canal, tipo de canal
**  Retorno:        True si OK
****************************************************************************************/
bool pinCanalTimer(numTimer_e numTimer, uint8_t pinBusqueda, pin_t *pinTimer, uint32_t *canal, tipoCanal_e *tipoCanal)
{
    for (uint8_t i = 0; i < NUM_MAX_PIN_SEL_TIMER; i++) {
        if (pinBusqueda == hardwareTimer[numTimer].pin[i].pin) {
        	pinTimer->pin = hardwareTimer[numTimer].pin[i].pin;
        	pinTimer->af = hardwareTimer[numTimer].pin[i].af;
        	*canal = hardwareTimer[numTimer].pin[i].canal;
        	*tipoCanal = hardwareTimer[numTimer].pin[i].tipoCanal;
            return true;
        }
    }

    return false;
}


/***************************************************************************************
**  Nombre:         (timerHAL_t *dTimer)
**  Descripcion:    Obtiene el DMA del Timer
**  Parametros:     Timer
**  Retorno:        Ninguno
****************************************************************************************/
void dmaTimer(timerHAL_t *dTimer)
{
    for (uint8_t i = 0; i < NUM_CANALES_POR_TIMER; i++) {
        uint32_t channel = i << 2;

        for (uint8_t j = 0; j < NUM_CANALES_DMA_TIMER; j++) {
            if (hardwareDMAcanalTimer[j].tim == dTimer->hal.htim.Instance && hardwareDMAcanalTimer[j].canal == channel) {
        	    dTimer->halTimDMA[i].DMAy_Streamx = hardwareDMAcanalTimer[j].DMAy_Streamx;
        	    dTimer->halTimDMA[i].canalDMA = hardwareDMAcanalTimer[j].canalDMA;
        	    dTimer->halTimDMA[i].dmaTimIrqHandler = hardwareDMAcanalTimer[j].dmaTimIrqHandler;
            }
        }
    }
}


/***************************************************************************************
**  Nombre:         void dmaTimerUP(timerHAL_t *dTimer)
**  Descripcion:    Obtiene el DMA del Timer UP
**  Parametros:     Timer
**  Retorno:        Ninguno
****************************************************************************************/
void dmaTimerUP(timerHAL_t *dTimer)
{
    for (uint8_t i = 0; i < NUM_TIMER_UP; i++) {
        if (hardwareDMAtimerUP[i].tim == dTimer->hal.htim.Instance) {
        	dTimer->halTimUPdma.DMAy_Streamx = hardwareDMAtimerUP[i].DMAy_Streamx;
        	dTimer->halTimUPdma.canalDMA = hardwareDMAtimerUP[i].canalDMA;
        	dTimer->halTimUPdma.dmaTimIrqHandler = hardwareDMAtimerUP[i].dmaTimUPirqHandler;
        }
    }
}

#endif // USAR_TIMERS

