/***************************************************************************************
**  dma.c - Funciones del periferico dma
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/05/2019
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
#include "dma.h"
#include "nvic.h"

#ifdef USAR_DMA


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static descriptorCanalDMA_t descriptorDMA[DMA_ULTIMO_HANDLER] = {
    DEFINIR_CANAL_DMA(DMA1, 0,  0),
    DEFINIR_CANAL_DMA(DMA1, 1,  6),
    DEFINIR_CANAL_DMA(DMA1, 2, 16),
    DEFINIR_CANAL_DMA(DMA1, 3, 22),
    DEFINIR_CANAL_DMA(DMA1, 4, 32),
    DEFINIR_CANAL_DMA(DMA1, 5, 38),
    DEFINIR_CANAL_DMA(DMA1, 6, 48),
    DEFINIR_CANAL_DMA(DMA1, 7, 54),

    DEFINIR_CANAL_DMA(DMA2, 0,  0),
    DEFINIR_CANAL_DMA(DMA2, 1,  6),
    DEFINIR_CANAL_DMA(DMA2, 2, 16),
    DEFINIR_CANAL_DMA(DMA2, 3, 22),
    DEFINIR_CANAL_DMA(DMA2, 4, 32),
    DEFINIR_CANAL_DMA(DMA2, 5, 38),
    DEFINIR_CANAL_DMA(DMA2, 6, 48),
    DEFINIR_CANAL_DMA(DMA2, 7, 54),
};

DEFINIR_IRQ_HANDLER_DMA(1, 0, DMA1_ST0_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(1, 1, DMA1_ST1_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(1, 2, DMA1_ST2_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(1, 3, DMA1_ST3_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(1, 4, DMA1_ST4_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(1, 5, DMA1_ST5_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(1, 6, DMA1_ST6_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(1, 7, DMA1_ST7_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 0, DMA2_ST0_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 1, DMA2_ST1_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 2, DMA2_ST2_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 3, DMA2_ST3_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 4, DMA2_ST4_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 5, DMA2_ST5_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 6, DMA2_ST6_HANDLER)
DEFINIR_IRQ_HANDLER_DMA(2, 7, DMA2_ST7_HANDLER)


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void habilitarRelojDMA(uint8_t indice);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarDMA(identificadorDMA_e identificador)
**  Descripcion:    Inicia el DMA
**  Parametros:     Identificador
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarDMA(identificadorDMA_e identificador)
{
    const uint8_t indice = IDENTIFICADOR_A_INDICE_DMA(identificador);
    habilitarRelojDMA(indice);
}


/***************************************************************************************
**  Nombre:         void ajustarHandlerDMA(identificadorDMA_e identificador, callbackHandlerFuncPtrDMA callback, uint32_t prioridad, uint32_t parametros)
**  Descripcion:    Ajusta el handler del DMA
**  Parametros:     Identificador, callback, prioridad, parametros
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarHandlerDMA(identificadorDMA_e identificador, callbackHandlerFuncPtrDMA callback, uint32_t prioridad, uint32_t parametros)
{
    const uint8_t indice = IDENTIFICADOR_A_INDICE_DMA(identificador);

    habilitarRelojDMA(indice);
    descriptorDMA[indice].irqHandlerCallback = callback;
    descriptorDMA[indice].paramUsuario = parametros;

    HAL_NVIC_SetPriority(descriptorDMA[indice].irqN, PRIORIDAD_BASE_NVIC(prioridad), PRIORIDAD_SUB_NVIC(prioridad));
    HAL_NVIC_EnableIRQ(descriptorDMA[indice].irqN);
}


/***************************************************************************************
**  Nombre:         identificadorDMA_e identificadorDMA(const DMA_Stream_TypeDef* stream)
**  Descripcion:    Devuelve el identificador del DMA
**  Parametros:     Stream
**  Retorno:        Identificador
****************************************************************************************/
identificadorDMA_e identificadorDMA(const DMA_Stream_TypeDef* stream)
{
    for (uint8_t i = 0; i < DMA_ULTIMO_HANDLER; i++) {
        if (descriptorDMA[i].ref == stream)
            return i + 1;
    }
    return 0;
}


/***************************************************************************************
**  Nombre:         void habilitarRelojDMA(uint8_t indice)
**  Descripcion:    Habilita el reloj del DMA
**  Parametros:     Indice
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojDMA(uint8_t indice)
{
    if (descriptorDMA[indice].dma == DMA1)
        __HAL_RCC_DMA1_CLK_ENABLE();
    else
        __HAL_RCC_DMA2_CLK_ENABLE();
}

#endif
