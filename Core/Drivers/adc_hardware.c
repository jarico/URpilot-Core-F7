/***************************************************************************************
**  adc_hardware.c - Hardware del ADC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 25/07/2020
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
#include "adc.h"

#ifdef USAR_ADC
#include "GP/gp_adc.h"
#include "io.h"
#include "dma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_STREAMS_DMA_ADC         2
#define PUERTO_ADC_NO_ENCONTRADO    255

#ifndef PUERTO_1_ADC
  #define PUERTO_1_ADC              NINGUNO
#endif

#ifndef PUERTO_2_ADC
  #define PUERTO_2_ADC              NINGUNO
#endif

#ifndef PUERTO_3_ADC
  #define PUERTO_3_ADC              NINGUNO
#endif

#ifndef PUERTO_4_ADC
  #define PUERTO_4_ADC              NINGUNO
#endif

#ifndef PUERTO_5_ADC
  #define PUERTO_5_ADC              NINGUNO
#endif

#ifndef PUERTO_6_ADC
  #define PUERTO_6_ADC              NINGUNO
#endif

#ifndef PUERTO_7_ADC
  #define PUERTO_7_ADC              NINGUNO
#endif

#ifndef PUERTO_8_ADC
  #define PUERTO_8_ADC              NINGUNO
#endif

#ifndef PUERTO_9_ADC
  #define PUERTO_9_ADC              NINGUNO
#endif

#ifndef PUERTO_10_ADC
  #define PUERTO_10_ADC             NINGUNO
#endif

#ifndef PUERTO_11_ADC
  #define PUERTO_11_ADC             NINGUNO
#endif

#ifndef PUERTO_12_ADC
  #define PUERTO_12_ADC             NINGUNO
#endif

#ifndef PUERTO_13_ADC
  #define PUERTO_13_ADC             NINGUNO
#endif

#ifndef PUERTO_14_ADC
  #define PUERTO_14_ADC             NINGUNO
#endif

#ifndef PUERTO_15_ADC
  #define PUERTO_15_ADC             NINGUNO
#endif

#ifndef PUERTO_16_ADC
  #define PUERTO_16_ADC             NINGUNO
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	numADC_e numADC;
    ADC_TypeDef *reg;
    pinADC_t pin[NUM_CANALES_ADC];
    canalStreamDMA_t dma[NUM_STREAMS_DMA_ADC];
} hardwareADC_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const hardwareADC_t hardwareADC[] = {
    {
        .numADC = ADC_1,
        .reg = ADC1,
		.pin = {
            { DEFIO_TAG(PA0), ADC_CHANNEL_0  },
            { DEFIO_TAG(PA1), ADC_CHANNEL_1  },
            { DEFIO_TAG(PA2), ADC_CHANNEL_2  },
            { DEFIO_TAG(PA3), ADC_CHANNEL_3  },
            { DEFIO_TAG(PA4), ADC_CHANNEL_4  },
            { DEFIO_TAG(PA5), ADC_CHANNEL_5  },
            { DEFIO_TAG(PA6), ADC_CHANNEL_6  },
            { DEFIO_TAG(PA7), ADC_CHANNEL_7  },
            { DEFIO_TAG(PB0), ADC_CHANNEL_8  },
            { DEFIO_TAG(PB1), ADC_CHANNEL_9  },
            { DEFIO_TAG(PC0), ADC_CHANNEL_10 },
            { DEFIO_TAG(PC1), ADC_CHANNEL_11 },
            { DEFIO_TAG(PC2), ADC_CHANNEL_12 },
            { DEFIO_TAG(PC3), ADC_CHANNEL_13 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PC4), ADC_CHANNEL_14 },
            { DEFIO_TAG(PC5), ADC_CHANNEL_15 },
#endif
		},
        .dma = {
            { DMA2_Stream0, DMA_CHANNEL_0 },
            { DMA2_Stream4, DMA_CHANNEL_0 }
        },
    },
    {
        .numADC = ADC_2,
        .reg = ADC2,
		.pin = {
            { DEFIO_TAG(PA0), ADC_CHANNEL_0  },
            { DEFIO_TAG(PA1), ADC_CHANNEL_1  },
            { DEFIO_TAG(PA2), ADC_CHANNEL_2  },
            { DEFIO_TAG(PA3), ADC_CHANNEL_3  },
            { DEFIO_TAG(PA4), ADC_CHANNEL_4  },
            { DEFIO_TAG(PA5), ADC_CHANNEL_5  },
            { DEFIO_TAG(PA6), ADC_CHANNEL_6  },
            { DEFIO_TAG(PA7), ADC_CHANNEL_7  },
            { DEFIO_TAG(PB0), ADC_CHANNEL_8  },
            { DEFIO_TAG(PB1), ADC_CHANNEL_9  },
            { DEFIO_TAG(PC0), ADC_CHANNEL_10 },
            { DEFIO_TAG(PC1), ADC_CHANNEL_11 },
            { DEFIO_TAG(PC2), ADC_CHANNEL_12 },
            { DEFIO_TAG(PC3), ADC_CHANNEL_13 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PC4), ADC_CHANNEL_14 },
            { DEFIO_TAG(PC5), ADC_CHANNEL_15 },
#endif
		},
        .dma = {
            { DMA2_Stream2, DMA_CHANNEL_1 },
            { DMA2_Stream3, DMA_CHANNEL_1 }
        },
    },
    {
        .numADC = ADC_3,
        .reg = ADC3,
		.pin = {
            { DEFIO_TAG(PA0),  ADC_CHANNEL_0  },
            { DEFIO_TAG(PA1),  ADC_CHANNEL_1  },
            { DEFIO_TAG(PA2),  ADC_CHANNEL_2  },
            { DEFIO_TAG(PA3),  ADC_CHANNEL_3  },
            { DEFIO_TAG(PF6),  ADC_CHANNEL_4  },
            { DEFIO_TAG(PF7),  ADC_CHANNEL_5  },
            { DEFIO_TAG(PF8),  ADC_CHANNEL_6  },
            { DEFIO_TAG(PF9),  ADC_CHANNEL_7  },
            { DEFIO_TAG(PF10), ADC_CHANNEL_8  },
            { DEFIO_TAG(PF3),  ADC_CHANNEL_9  },
            { DEFIO_TAG(PC0),  ADC_CHANNEL_10 },
            { DEFIO_TAG(PC1),  ADC_CHANNEL_11 },
            { DEFIO_TAG(PC2),  ADC_CHANNEL_12 },
            { DEFIO_TAG(PC3),  ADC_CHANNEL_13 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PF4),  ADC_CHANNEL_14 },
            { DEFIO_TAG(PF5),  ADC_CHANNEL_15 },
#endif
		},
        .dma = {
            { DMA2_Stream0, DMA_CHANNEL_2 },
            { DMA2_Stream1, DMA_CHANNEL_2 }
        },
    }
};


static const uint8_t puertosHWadc[] = {
    DEFIO_TAG(PUERTO_1_ADC),
	DEFIO_TAG(PUERTO_2_ADC),
    DEFIO_TAG(PUERTO_3_ADC),
	DEFIO_TAG(PUERTO_4_ADC),
    DEFIO_TAG(PUERTO_5_ADC),
	DEFIO_TAG(PUERTO_6_ADC),
    DEFIO_TAG(PUERTO_7_ADC),
	DEFIO_TAG(PUERTO_8_ADC),
    DEFIO_TAG(PUERTO_9_ADC),
	DEFIO_TAG(PUERTO_10_ADC),
    DEFIO_TAG(PUERTO_11_ADC),
	DEFIO_TAG(PUERTO_12_ADC),
    DEFIO_TAG(PUERTO_13_ADC),
	DEFIO_TAG(PUERTO_14_ADC),
#if defined(STM32F767xx)
    DEFIO_TAG(PUERTO_15_ADC),
	DEFIO_TAG(PUERTO_16_ADC),
#endif
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool verificarPinADC(numADC_e numADC, uint8_t pin);
uint8_t canalADC(numADC_e numADC, uint8_t pin);
bool comprobarStreamDMAadc(numADC_e numADC, DMA_Stream_TypeDef *DMAy_Streamx);
uint32_t canalStreamDMAadc(numADC_e numADC, DMA_Stream_TypeDef *DMAy_Streamx);
uint8_t posicionPuertoADC(uint8_t pin);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool asignarHALadc(numADC_e numADC)
**  Descripcion:    Asigna el HAL del ADC
**  Parametros:     Numero de ADC
**  Retorno:        True si ok
****************************************************************************************/
bool asignarHALadc(numADC_e numADC)
{
    adc_t *driver = punteroADC(numADC);
    bool activo = false;
    uint8_t numCanalesADC = 0;

    for (uint8_t i = 0; i < NUM_CANALES_ADC; i++) {
        if (!verificarPinADC(numADC, configADC(numADC)->pin[i]))
            continue;

        driver->hal.pin[i].pin = configADC(numADC)->pin[i];
        driver->hal.pin[i].canal = canalADC(numADC, configADC(numADC)->pin[i]);
        driver->hal.canalesHabilitados[i] = true;
        activo = true;

        puertoADC_t puerto;
        puerto.pinAsignado = true;
        puerto.numADC = numADC;
        puerto.canalADC = numCanalesADC;

        uint8_t numPuerto = posicionPuertoADC(configADC(numADC)->pin[i]);
        if (numPuerto != PUERTO_ADC_NO_ENCONTRADO)
        	asignarPuertoADC(numPuerto, puerto);

        numCanalesADC++;
    }

    if (activo) {
        driver->hal.hadc.Instance = hardwareADC[numADC].reg;

        if (!comprobarStreamDMAadc(numADC, configADC(numADC)->dma))
            return false;

        driver->hal.hdma.Instance = configADC(numADC)->dma;
        driver->hal.hdma.Init.Channel = canalStreamDMAadc(numADC, configADC(numADC)->dma);
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool verificarPinADC(numADC_e numADC, uint8_t pin)
**  Descripcion:    Verifica que el tag tiene conexion con el driver correcto
**  Parametros:     Numero de driver, pin a verificar
**  Retorno:        True si ok
****************************************************************************************/
bool verificarPinADC(numADC_e numADC, uint8_t pin)
{
    for (uint8_t i = 0; i < NUM_CANALES_ADC; i++) {
        if (hardwareADC[numADC].pin[i].pin == pin)
            return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         uint8_t canalADC(numADC_e numADC, uint8_t pin)
**  Descripcion:    Retorna el canal dado un tag
**  Parametros:     Numero de driver, Tag
**  Retorno:        Canal
****************************************************************************************/
uint8_t canalADC(numADC_e numADC, uint8_t pin)
{
    const hardwareADC_t *hw = &hardwareADC[numADC];

    for (uint8_t i = 0; i < NUM_CANALES_ADC; i++) {
        if (pin == hw->pin[i].pin)
            return hw->pin[i].canal;
    }

    return 0;
}


/***************************************************************************************
**  Nombre:         bool comprobarStreamDMAadc(numADC_e numADC, DMA_Stream_TypeDef *DMAy_Streamx)
**  Descripcion:    Comprueba si el DMA asignado es correcto
**  Parametros:     Numero de driver, DMA configurado
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarStreamDMAadc(numADC_e numADC, DMA_Stream_TypeDef *DMAy_Streamx)
{
    const hardwareADC_t *hw = &hardwareADC[numADC];

    for (uint8_t i = 0; i < NUM_STREAMS_DMA_ADC; i++) {
        if (hw->dma[i].DMAy_Streamx == DMAy_Streamx)
            return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         uint32_t canalStreamDMAadc(numADC_e numADC, DMA_Stream_TypeDef *DMAy_Streamx)
**  Descripcion:    Comprueba si el DMA asignado es correcto
**  Parametros:     Numero del driver, DMA configurado
**  Retorno:        Canal del DMA
****************************************************************************************/
uint32_t canalStreamDMAadc(numADC_e numADC, DMA_Stream_TypeDef *DMAy_Streamx)
{
    const hardwareADC_t *hw = &hardwareADC[numADC];

    for (uint8_t i = 0; i < NUM_STREAMS_DMA_ADC; i++) {
        if (hw->dma[i].DMAy_Streamx == DMAy_Streamx)
            return hw->dma[i].canal;
    }

    return 0;
}


/***************************************************************************************
**  Nombre:         uint8_t posicionPuertoADC(uint8_t pin)
**  Descripcion:    Devuelve la posicion del puerto que tiene que ocupar un pin dado
**  Parametros:     Pin a comprobar
**  Retorno:        Posicion del puerto
****************************************************************************************/
uint8_t posicionPuertoADC(uint8_t pin)
{
    for (uint8_t i = 0; i < NUM_MAX_PUERTOS_ADC; i++) {
        if (puertosHWadc[i] == pin)
            return i;
    }

    return PUERTO_ADC_NO_ENCONTRADO;
}


#endif
