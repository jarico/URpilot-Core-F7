/***************************************************************************************
**  uart_hardware.c - Hardware de las UART
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 30/06/2020
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

#include "uart.h"

#ifdef USAR_UART
#include "GP/gp_uart.h"
#include "io.h"
#include "nvic.h"
#include "dma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_STREAMS_DMA_UART       2
#if defined(STM32F767xx)
#define NUM_MAX_PIN_SEL_UART       4
#else
#define NUM_MAX_PIN_SEL_UART       3
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    numUART_e numUART;
    USART_TypeDef* reg;
    uint32_t canalDMA;
    DMA_Stream_TypeDef *streamTxDMA;
    DMA_Stream_TypeDef *streamRxDMA;
    pin_t pinRx[NUM_MAX_PIN_SEL_UART];
    pin_t pinTx[NUM_MAX_PIN_SEL_UART];
    canalStreamDMA_t dmaTx[NUM_STREAMS_DMA_UART];
    canalStreamDMA_t dmaRx[NUM_STREAMS_DMA_UART];
    uint8_t IRQ;
    uint8_t prioridadIRQ;
} hardwareUART_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const hardwareUART_t hardwareUART[NUM_MAX_UART] = {
    {
        .numUART = UART_1,
        .reg = USART1,
        .pinRx = {
            { DEFIO_TAG(PA10), GPIO_AF7_USART1 },
            { DEFIO_TAG(PB7),  GPIO_AF7_USART1 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PB15), GPIO_AF4_USART1 }
#endif
        },
        .pinTx = {
            { DEFIO_TAG(PA9),  GPIO_AF7_USART1 },
            { DEFIO_TAG(PB6),  GPIO_AF7_USART1 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PB14), GPIO_AF4_USART1 }
#endif
        },
        .dmaTx = {
            { DMA2_Stream7, DMA_CHANNEL_4 },
        },
        .dmaRx = {
            { DMA2_Stream2, DMA_CHANNEL_4 },
            { DMA2_Stream5, DMA_CHANNEL_4 },
        },
        .IRQ = USART1_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART1
    },
    {
        .numUART = UART_2,
        .reg = USART2,
        .pinRx = {
            { DEFIO_TAG(PA3), GPIO_AF7_USART2 },
            { DEFIO_TAG(PD6), GPIO_AF7_USART2 }
        },
        .pinTx = {
            { DEFIO_TAG(PA2), GPIO_AF7_USART2 },
            { DEFIO_TAG(PD5), GPIO_AF7_USART2 }
        },
        .dmaTx = {
            { DMA1_Stream6, DMA_CHANNEL_4 },
        },
        .dmaRx = {
            { DMA1_Stream5, DMA_CHANNEL_4 },
        },
        .IRQ = USART2_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART2
    },
    {
        .numUART = UART_3,
        .reg = USART3,
        .pinRx = {
            { DEFIO_TAG(PB11), GPIO_AF7_USART3 },
            { DEFIO_TAG(PC11), GPIO_AF7_USART3 },
            { DEFIO_TAG(PD9),  GPIO_AF7_USART3 }
        },
        .pinTx = {
            { DEFIO_TAG(PB10), GPIO_AF7_USART3 },
            { DEFIO_TAG(PC10), GPIO_AF7_USART3 },
            { DEFIO_TAG(PD8),  GPIO_AF7_USART3 }
        },
        .dmaTx = {
            { DMA1_Stream3, DMA_CHANNEL_4 },
            { DMA1_Stream4, DMA_CHANNEL_7 },
        },
        .dmaRx = {
            { DMA1_Stream1, DMA_CHANNEL_4 },
        },
        .IRQ = USART3_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART3
    },
    {
        .numUART = UART_4,
        .reg = UART4,
        .pinRx = {
            { DEFIO_TAG(PA1),  GPIO_AF8_UART4 },
            { DEFIO_TAG(PC11), GPIO_AF8_UART4 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PA11), GPIO_AF6_UART4 },
#endif
            { DEFIO_TAG(PD0),  GPIO_AF8_UART4 },
        },
        .pinTx = {
            { DEFIO_TAG(PA0),  GPIO_AF8_UART4 },
            { DEFIO_TAG(PC10), GPIO_AF8_UART4 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PA12), GPIO_AF6_UART4 },
#endif
            { DEFIO_TAG(PD1),  GPIO_AF8_UART4 }
        },
        .dmaTx = {
            { DMA1_Stream4, DMA_CHANNEL_4 },
        },
        .dmaRx = {
            { DMA1_Stream2, DMA_CHANNEL_4 },
        },
        .IRQ = UART4_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART4
    },
    {
        .numUART = UART_5,
        .reg = UART5,
        .pinRx = {
            { DEFIO_TAG(PD2),  GPIO_AF8_UART5 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PB5),  GPIO_AF1_UART5 },
#endif
            { DEFIO_TAG(PB8),  GPIO_AF7_UART5 },
            { DEFIO_TAG(PB12), GPIO_AF8_UART5 }
        },
        .pinTx = {
            { DEFIO_TAG(PC12), GPIO_AF8_UART5 },
#if defined(STM32F767xx)
            { DEFIO_TAG(PB6),  GPIO_AF1_UART5 },
#endif
            { DEFIO_TAG(PB9),  GPIO_AF7_UART5 },
            { DEFIO_TAG(PB13), GPIO_AF8_UART5 }
        },
        .dmaTx = {
            { DMA1_Stream7, DMA_CHANNEL_4 },
        },
        .dmaRx = {
            { DMA1_Stream0, DMA_CHANNEL_4 },
        },
        .IRQ = UART5_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART5
    },
    {
        .numUART = UART_6,
        .reg = USART6,
        .pinRx = {
            { DEFIO_TAG(PC7),  GPIO_AF8_USART6 },
            { DEFIO_TAG(PG9),  GPIO_AF8_USART6 }
        },
        .pinTx = {
            { DEFIO_TAG(PC6),  GPIO_AF8_USART6 },
            { DEFIO_TAG(PG14), GPIO_AF8_USART6 }
        },
        .dmaTx = {
            { DMA2_Stream6, DMA_CHANNEL_5 },
            { DMA2_Stream7, DMA_CHANNEL_5 },
        },
        .dmaRx = {
            { DMA2_Stream1, DMA_CHANNEL_5 },
            { DMA2_Stream2, DMA_CHANNEL_5 },
        },
        .IRQ = USART6_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART6
    },
#if defined(STM32F767xx)
    {
        .numUART = UART_7,
        .reg = UART7,
        .pinRx = {
            { DEFIO_TAG(PE7),  GPIO_AF8_UART7  },
            { DEFIO_TAG(PF6),  GPIO_AF8_UART7  },
            { DEFIO_TAG(PA8),  GPIO_AF12_UART7 },
            { DEFIO_TAG(PB3),  GPIO_AF12_UART7 }
        },
        .pinTx = {
            { DEFIO_TAG(PE8),  GPIO_AF8_UART7  },
            { DEFIO_TAG(PF7),  GPIO_AF8_UART7  },
            { DEFIO_TAG(PA15), GPIO_AF12_UART7 },
            { DEFIO_TAG(PB4),  GPIO_AF12_UART7 }
        },
        .dmaTx = {
            { DMA1_Stream1, DMA_CHANNEL_5 },
        },
        .dmaRx = {
            { DMA1_Stream3, DMA_CHANNEL_5 },
        },
        .IRQ = UART7_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART7
    },
    {
        .numUART = UART_8,
        .reg = UART8,
        .pinRx = {
            { DEFIO_TAG(PE0), GPIO_AF8_UART8 }
        },
        .pinTx = {
            { DEFIO_TAG(PE1), GPIO_AF8_UART8 }
        },
        .dmaTx = {
            { DMA1_Stream0, DMA_CHANNEL_5 },
        },
        .dmaRx = {
            { DMA1_Stream6, DMA_CHANNEL_5 },
        },
        .IRQ = UART8_IRQn,
        .prioridadIRQ = NVIC_PRIO_SERIALUART8
    },
#endif
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool comprobarConfigHardwareUART(numUART_e numUART);
bool comprobarPinUART(numUART_e numUART, uint8_t pin);
bool comprobarStreamDMAuart(numUART_e numUART, DMA_Stream_TypeDef *DMAy_Streamx);
bool pinUART(numUART_e numUART, uint8_t pinBusqueda, pin_t *pinDriver);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool asignarHALuart(numUART_e numUART)
**  Descripcion:    Asigna el HAL a la estructura del driver
**  Parametros:     Numero del driver
**  Retorno:        True si ok
****************************************************************************************/
bool asignarHALuart(numUART_e numUART)
{
    uart_t *driver = punteroUART(numUART);

    if (!comprobarConfigHardwareUART(numUART))
        return false;

    // Asignamos los pines
    if (!pinUART(numUART, configUART(numUART)->pinTx, &driver->hal.pinTx))
        return false;

    if (!pinUART(numUART, configUART(numUART)->pinRx, &driver->hal.pinRx))
        return false;

    // Asignamos la instancia
    driver->hal.huart.Instance = hardwareUART[numUART].reg;

    // Asignamos las interrupciones
    driver->hal.IRQ = hardwareUART[numUART].IRQ;
    driver->hal.prioridadIRQ = hardwareUART[numUART].prioridadIRQ;

    return true;
}


/***************************************************************************************
**  Nombre:         bool comprobarConfigHardwareUART(numUART_e numUART)
**  Descripcion:    Comprueba si el hardware configurado es correcto
**  Parametros:     Numero del driver
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarConfigHardwareUART(numUART_e numUART)
{
    if (!comprobarPinUART(numUART, configUART(numUART)->pinTx))
        return false;

    if (!comprobarPinUART(numUART, configUART(numUART)->pinRx))
        return false;

#ifdef USAR_DMA_UART
    if (configUART(numUART)->usarDMA) {
        if (!comprobarStreamDMAuart(numUART, configUART(numUART)->dmaTx))
            return false;

        if (!comprobarStreamDMAuart(numUART, configUART(numUART)->dmaRx))
            return false;
    }
#endif

    return true;
}


/***************************************************************************************
**  Nombre:         bool comprobarPinUART(numUART_e numUART, uint8_t pin)
**  Descripcion:    Comprueba si los pines son correctos
**  Parametros:     Numero del driver, Pin a configurar
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarPinUART(numUART_e numUART, uint8_t pin)
{
    for (uint8_t i = 0; i < NUM_MAX_PIN_SEL_UART; i++) {
        if (pin == hardwareUART[numUART].pinTx[i].pin || pin == hardwareUART[numUART].pinRx[i].pin)
            return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         bool comprobarStreamDMAuart(numUART_e numUART, DMA_Stream_TypeDef *DMAy_Streamx)
**  Descripcion:    Comprueba si el DMA asignado es correcto
**  Parametros:     Numero del driver, DMA configurado
**  Retorno:        True si ok
****************************************************************************************/
bool comprobarStreamDMAuart(numUART_e numUART, DMA_Stream_TypeDef *DMAy_Streamx)
{
    for (uint8_t i = 0; i < NUM_STREAMS_DMA_UART; i++) {
        if (DMAy_Streamx == hardwareUART[numUART].dmaTx[i].DMAy_Streamx || DMAy_Streamx == hardwareUART[numUART].dmaRx[i].DMAy_Streamx)
            return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         bool pinUART(numUART_e numUART, uint8_t pinBusqueda, pin_t *pinDriver)
**  Descripcion:    Encuentra el pin de la tabla de hardware
**  Parametros:     Numero de UART, pin a buscar, pin del driver
**  Retorno:        True si OK
****************************************************************************************/
bool pinUART(numUART_e numUART, uint8_t pinBusqueda, pin_t *pinDriver)
{
    for (uint8_t i = 0; i < NUM_MAX_PIN_SEL_UART; i++) {
        if (pinBusqueda == hardwareUART[numUART].pinTx[i].pin) {
        	pinDriver->pin = hardwareUART[numUART].pinTx[i].pin;
        	pinDriver->af = hardwareUART[numUART].pinTx[i].af;
            return true;
        }

        if (pinBusqueda == hardwareUART[numUART].pinRx[i].pin) {
        	pinDriver->pin = hardwareUART[numUART].pinRx[i].pin;
        	pinDriver->af = hardwareUART[numUART].pinRx[i].af;
            return true;
        }
    }

    return false;
}


#endif // USAR_UART
