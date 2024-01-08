/***************************************************************************************
**  sdmmc_hardware.c - Datos de configuracion del hardware del SDMMC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2020
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
#include "sdmmc.h"

#ifdef USAR_SDMMC
#include "nvic.h"
#include "dma.h"
#include "GP/gp_sdmmc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_DISPOSITIVOS_SDMMC      2
#define NUM_STREAMS_DMA_SDMMC       2


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    numSDMMC_e numSDMMC;
    SDMMC_TypeDef *reg;
    pin_t pinCK;
    pin_t pinCMD;
    pin_t pinD0;
    pin_t pinD1;
    pin_t pinD2;
    pin_t pinD3;
    canalStreamDMA_t dma[NUM_STREAMS_DMA_SDMMC];
    uint8_t IRQ;
    uint8_t prioridadIRQ;
} hardwareSDMMC_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const hardwareSDMMC_t hardwareSDMMC[NUM_DISPOSITIVOS_SDMMC] = {
    {
        .numSDMMC = SDMMC_1,
        .reg = SDMMC1,
        .pinCK = {
            DEFIO_TAG(PC12), GPIO_AF12_SDMMC1,
        },
        .pinCMD = {
            DEFIO_TAG(PD2),  GPIO_AF12_SDMMC1,
        },
        .pinD0 = {
            DEFIO_TAG(PC8),  GPIO_AF12_SDMMC1,
        },
        .pinD1 = {
            DEFIO_TAG(PC9),  GPIO_AF12_SDMMC1,
        },
        .pinD2 = {
            DEFIO_TAG(PC10), GPIO_AF12_SDMMC1,
        },
        .pinD3 = {
            DEFIO_TAG(PC11), GPIO_AF12_SDMMC1,
        },
		.dma = {
	        { DMA2_Stream3, DMA_CHANNEL_4 },
	        { DMA2_Stream6, DMA_CHANNEL_4 },
	    },
        .IRQ = SDMMC1_IRQn,
        .prioridadIRQ = NVIC_PRIO_SDMMC1,
    },
    {
        .numSDMMC = SDMMC_2,
        .reg = SDMMC2,
        .pinCK = {
            DEFIO_TAG(PD6),  GPIO_AF10_SDMMC2,
        },
        .pinCMD = {
            DEFIO_TAG(PD7),  GPIO_AF10_SDMMC2,
        },
        .pinD0 = {
            DEFIO_TAG(PG9),  GPIO_AF10_SDMMC2,
        },
        .pinD1 = {
            DEFIO_TAG(PG10), GPIO_AF10_SDMMC2,
        },
        .pinD2 = {
            DEFIO_TAG(PG11), GPIO_AF10_SDMMC2,
        },
        .pinD3 = {
            DEFIO_TAG(PG12), GPIO_AF10_SDMMC2,
        },
        .dma = {
            { DMA2_Stream0, DMA_CHANNEL_11 },
	        { DMA2_Stream5, DMA_CHANNEL_11 },
        },
        .IRQ = SDMMC2_IRQn,
        .prioridadIRQ = NVIC_PRIO_SDMMC2,
    },
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool comprobarConfigHardwareSDMMC(void);
bool comprobarPinSDMMC(uint8_t pin);
bool comprobarStreamDMAsdmmc(DMA_Stream_TypeDef *DMAy_Streamx);
uint32_t canalStreamDMAsdmmc(DMA_Stream_TypeDef *DMAy_Streamx);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool asignarHALsdmmc(void)
**  Descripcion:    Asigna el HAL a la estructura del driver
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool asignarHALsdmmc(void)
{
    const hardwareSDMMC_t *hw = &hardwareSDMMC[configSDMMC()->numSDMMC];
    sdmmc_t *driver = punteroSDMMC();

    if (!comprobarConfigHardwareSDMMC())
        return false;

    // Asignamos los pines
    driver->hal.pinCK.pin = hw->pinCK.pin;
    driver->hal.pinCK.af = hw->pinCK.af;

    driver->hal.pinCMD.pin = hw->pinCMD.pin;
    driver->hal.pinCMD.af = hw->pinCMD.af;

    driver->hal.pinD0.pin = hw->pinD0.pin;
    driver->hal.pinD0.af = hw->pinD0.af;

    if (configSDMMC()->ancho4bits) {
        driver->hal.pinD1.pin = hw->pinD1.pin;
        driver->hal.pinD1.af = hw->pinD1.af;

        driver->hal.pinD2.pin = hw->pinD2.pin;
        driver->hal.pinD2.af = hw->pinD2.af;

        driver->hal.pinD3.pin = hw->pinD3.pin;
        driver->hal.pinD3.af = hw->pinD3.af;
    }

    // Asignamos la instancia
    driver->hal.hsdmmc.Instance = hw->reg;

#ifdef USAR_DMA_SDMMC
    // Asignamos el DMA
    driver->hal.hdmaTx.Instance = configSDMMC()->streamDMAtx;
    driver->hal.hdmaTx.Init.Channel = canalStreamDMAsdmmc(configSDMMC()->streamDMAtx);

    driver->hal.hdmaRx.Instance = configSDMMC()->streamDMArx;
    driver->hal.hdmaRx.Init.Channel = canalStreamDMAsdmmc(configSDMMC()->streamDMArx);
#endif

    // Asignamos las interrupciones
    driver->hal.IRQ = hw->IRQ;
    driver->hal.prioridadIRQ = hw->prioridadIRQ;

    return true;
}


/***************************************************************************************
**  Nombre:         bool comprobarConfigHardwareSDMMC(void)
**  Descripcion:    Comprueba si el hardware configurado es correcto
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarConfigHardwareSDMMC(void)
{
    if (!comprobarPinSDMMC(configSDMMC()->pinCK))
        return false;

    if (!comprobarPinSDMMC(configSDMMC()->pinCMD))
        return false;

    if (!comprobarPinSDMMC(configSDMMC()->pinD0))
        return false;

    if (configSDMMC()->ancho4bits) {
        if (!comprobarPinSDMMC(configSDMMC()->pinD1))
            return false;

        if (!comprobarPinSDMMC(configSDMMC()->pinD2))
            return false;

        if (!comprobarPinSDMMC(configSDMMC()->pinD3))
            return false;
    }

#ifdef USAR_DMA_SDMMC
    if (!comprobarStreamDMAsdmmc(configSDMMC()->streamDMAtx))
        return false;

    if (!comprobarStreamDMAsdmmc(configSDMMC()->streamDMArx))
        return false;
#endif

    return true;
}


/***************************************************************************************
**  Nombre:         bool comprobarPinSDMMC(uint8_t pin)
**  Descripcion:    Comprueba si los pines son correctos
**  Parametros:     Pin a configurar
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarPinSDMMC(uint8_t pin)
{
    const hardwareSDMMC_t *hw = &hardwareSDMMC[configSDMMC()->numSDMMC];

    if (pin == hw->pinCK.pin || pin == hw->pinCMD.pin || pin == hw->pinD0.pin ||
        pin == hw->pinD1.pin || pin == hw->pinD2.pin  || pin == hw->pinD3.pin )
        return true;

    return false;
}


/***************************************************************************************
**  Nombre:         bool comprobarStreamDMAsdmmc(DMA_Stream_TypeDef *DMAy_Streamx)
**  Descripcion:    Comprueba si el DMA asignado es correcto
**  Parametros:     DMA configurado
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarStreamDMAsdmmc(DMA_Stream_TypeDef *DMAy_Streamx)
{
    const hardwareSDMMC_t *hw = &hardwareSDMMC[configSDMMC()->numSDMMC];

    for (uint8_t i = 0; i < NUM_STREAMS_DMA_SDMMC; i++) {
        if (hw->dma[i].DMAy_Streamx == DMAy_Streamx)
            return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         uint32_t canalStreamDMAsdmmc(DMA_Stream_TypeDef *DMAy_Streamx)
**  Descripcion:    Comprueba si el DMA asignado es correcto
**  Parametros:     DMA configurado
**  Retorno:        Canal del DMA
****************************************************************************************/
uint32_t canalStreamDMAsdmmc(DMA_Stream_TypeDef *DMAy_Streamx)
{
    const hardwareSDMMC_t *hw = &hardwareSDMMC[configSDMMC()->numSDMMC];

    for (uint8_t i = 0; i < NUM_STREAMS_DMA_SDMMC; i++) {
        if (hw->dma[i].DMAy_Streamx == DMAy_Streamx)
            return hw->dma[i].canal;
    }

    return 0;
}

#endif // USAR_SDMMC

