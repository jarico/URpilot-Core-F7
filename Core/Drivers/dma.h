/***************************************************************************************
**  dma.h - Funciones del periferico dma
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

#ifndef __DMA_H
#define __DMA_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define IDENTIFICADOR_A_INDICE_DMA(x) ((x) - 1)

#define DEFINIR_CANAL_DMA(d, s, f) {    \
    .dma = d,                           \
    .ref = d ## _Stream ## s,           \
    .stream = s,                        \
    .irqHandlerCallback = NULL,         \
    .flagsShift = f,                    \
    .irqN = d ## _Stream ## s ## _IRQn, \
    .paramUsuario = 0,                  \
    }

#define DEFINIR_IRQ_HANDLER_DMA(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) { \
    const uint8_t indice = IDENTIFICADOR_A_INDICE_DMA(i);                                        \
    if (descriptorDMA[indice].irqHandlerCallback)                                             \
        descriptorDMA[indice].irqHandlerCallback(&descriptorDMA[indice]);                     \
}

#define LIMPIAR_FLAG_DMA(d, flag) if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
#define OBTENER_FLAG_STATUS_DMA(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))

#define DMA_IT_TCIF         ((uint32_t)0x00000020)
#define DMA_IT_HTIF         ((uint32_t)0x00000010)
#define DMA_IT_TEIF         ((uint32_t)0x00000008)
#define DMA_IT_DMEIF        ((uint32_t)0x00000004)
#define DMA_IT_FEIF         ((uint32_t)0x00000001)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
struct descriptorCanalDMA_s;
typedef void (*callbackHandlerFuncPtrDMA)(struct descriptorCanalDMA_s *descriptorCanal);

typedef struct descriptorCanalDMA_s{
    DMA_TypeDef* dma;
    DMA_Stream_TypeDef* ref;
    uint8_t stream;
    callbackHandlerFuncPtrDMA irqHandlerCallback;
    uint8_t flagsShift;
    IRQn_Type irqN;
    uint32_t paramUsuario;
    uint32_t completeFlag;
} descriptorCanalDMA_t;

typedef enum {
    DMA_NINGUNO = 0,
    DMA1_ST0_HANDLER,
    DMA1_ST1_HANDLER,
    DMA1_ST2_HANDLER,
    DMA1_ST3_HANDLER,
    DMA1_ST4_HANDLER,
    DMA1_ST5_HANDLER,
    DMA1_ST6_HANDLER,
    DMA1_ST7_HANDLER,
    DMA2_ST0_HANDLER,
    DMA2_ST1_HANDLER,
    DMA2_ST2_HANDLER,
    DMA2_ST3_HANDLER,
    DMA2_ST4_HANDLER,
    DMA2_ST5_HANDLER,
    DMA2_ST6_HANDLER,
    DMA2_ST7_HANDLER,
    DMA_ULTIMO_HANDLER = DMA2_ST7_HANDLER,
} identificadorDMA_e;

typedef struct {
    DMA_Stream_TypeDef *DMAy_Streamx;
    uint32_t canal;
} canalStreamDMA_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarDMA(identificadorDMA_e identificador);
void ajustarHandlerDMA(identificadorDMA_e identifier, callbackHandlerFuncPtrDMA callback, uint32_t prioridad, uint32_t parametros);
identificadorDMA_e identificadorDMA(const DMA_Stream_TypeDef* stream);

#endif // __DMA_H
