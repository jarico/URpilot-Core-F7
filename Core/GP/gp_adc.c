/***************************************************************************************
**  gp_adc.c - Funciones y grupo de parametros del ADC
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
#include "gp_adc.h"

#ifdef USAR_ADC
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef DMA_ADC_1
  #define DMA_ADC_1            DMA2_Stream0
#endif

#ifndef DMA_ADC_2
  #define DMA_ADC_2            DMA2_Stream2
#endif

#ifndef DMA_ADC_3
  #define DMA_ADC_3            DMA2_Stream1
#endif

#ifndef PIN_1_ADC_1
  #define PIN_1_ADC_1          NINGUNO
#endif

#ifndef PIN_2_ADC_1
  #define PIN_2_ADC_1          NINGUNO
#endif

#ifndef PIN_3_ADC_1
  #define PIN_3_ADC_1          NINGUNO
#endif

#ifndef PIN_4_ADC_1
  #define PIN_4_ADC_1          NINGUNO
#endif

#ifndef PIN_5_ADC_1
  #define PIN_5_ADC_1          NINGUNO
#endif

#ifndef PIN_6_ADC_1
  #define PIN_6_ADC_1          NINGUNO
#endif

#ifndef PIN_7_ADC_1
  #define PIN_7_ADC_1          NINGUNO
#endif

#ifndef PIN_8_ADC_1
  #define PIN_8_ADC_1          NINGUNO
#endif

#ifndef PIN_9_ADC_1
  #define PIN_9_ADC_1          NINGUNO
#endif

#ifndef PIN_10_ADC_1
  #define PIN_10_ADC_1         NINGUNO
#endif

#ifndef PIN_11_ADC_1
  #define PIN_11_ADC_1         NINGUNO
#endif

#ifndef PIN_12_ADC_1
  #define PIN_12_ADC_1         NINGUNO
#endif

#ifndef PIN_13_ADC_1
  #define PIN_13_ADC_1         NINGUNO
#endif

#ifndef PIN_14_ADC_1
  #define PIN_14_ADC_1         NINGUNO
#endif

#ifndef PIN_15_ADC_1
  #define PIN_15_ADC_1         NINGUNO
#endif

#ifndef PIN_16_ADC_1
  #define PIN_16_ADC_1         NINGUNO
#endif

#ifndef PIN_1_ADC_2
  #define PIN_1_ADC_2          NINGUNO
#endif

#ifndef PIN_2_ADC_2
  #define PIN_2_ADC_2          NINGUNO
#endif

#ifndef PIN_3_ADC_2
  #define PIN_3_ADC_2          NINGUNO
#endif

#ifndef PIN_4_ADC_2
  #define PIN_4_ADC_2          NINGUNO
#endif

#ifndef PIN_5_ADC_2
  #define PIN_5_ADC_2          NINGUNO
#endif

#ifndef PIN_6_ADC_2
  #define PIN_6_ADC_2          NINGUNO
#endif

#ifndef PIN_7_ADC_2
  #define PIN_7_ADC_2          NINGUNO
#endif

#ifndef PIN_8_ADC_2
  #define PIN_8_ADC_2          NINGUNO
#endif

#ifndef PIN_9_ADC_2
  #define PIN_9_ADC_2          NINGUNO
#endif

#ifndef PIN_10_ADC_2
  #define PIN_10_ADC_2         NINGUNO
#endif

#ifndef PIN_11_ADC_2
  #define PIN_11_ADC_2         NINGUNO
#endif

#ifndef PIN_12_ADC_2
  #define PIN_12_ADC_2         NINGUNO
#endif

#ifndef PIN_13_ADC_2
  #define PIN_13_ADC_2         NINGUNO
#endif

#ifndef PIN_14_ADC_2
  #define PIN_14_ADC_2         NINGUNO
#endif

#ifndef PIN_15_ADC_2
  #define PIN_15_ADC_2         NINGUNO
#endif

#ifndef PIN_16_ADC_2
  #define PIN_16_ADC_2         NINGUNO
#endif

#ifndef PIN_1_ADC_3
  #define PIN_1_ADC_3          NINGUNO
#endif

#ifndef PIN_2_ADC_3
  #define PIN_2_ADC_3          NINGUNO
#endif

#ifndef PIN_3_ADC_3
  #define PIN_3_ADC_3          NINGUNO
#endif

#ifndef PIN_4_ADC_3
  #define PIN_4_ADC_3          NINGUNO
#endif

#ifndef PIN_5_ADC_3
  #define PIN_5_ADC_3          NINGUNO
#endif

#ifndef PIN_6_ADC_3
  #define PIN_6_ADC_3          NINGUNO
#endif

#ifndef PIN_7_ADC_3
  #define PIN_7_ADC_3          NINGUNO
#endif

#ifndef PIN_8_ADC_3
  #define PIN_8_ADC_3          NINGUNO
#endif

#ifndef PIN_9_ADC_3
  #define PIN_9_ADC_3          NINGUNO
#endif

#ifndef PIN_10_ADC_3
  #define PIN_10_ADC_3         NINGUNO
#endif

#ifndef PIN_11_ADC_3
  #define PIN_11_ADC_3         NINGUNO
#endif

#ifndef PIN_12_ADC_3
  #define PIN_12_ADC_3         NINGUNO
#endif

#ifndef PIN_13_ADC_3
  #define PIN_13_ADC_3         NINGUNO
#endif

#ifndef PIN_14_ADC_3
  #define PIN_14_ADC_3         NINGUNO
#endif

#ifndef PIN_15_ADC_3
  #define PIN_15_ADC_3         NINGUNO
#endif

#ifndef PIN_16_ADC_3
  #define PIN_16_ADC_3         NINGUNO
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configADC_t, NUM_MAX_ADC, configADC, GP_CONFIGURACION_ADC, 1);

static const configADC_t configADCdefecto[] = {
    { {DEFIO_TAG(PIN_1_ADC_1),  DEFIO_TAG(PIN_2_ADC_1),  DEFIO_TAG(PIN_3_ADC_1),  DEFIO_TAG(PIN_4_ADC_1),
       DEFIO_TAG(PIN_5_ADC_1),  DEFIO_TAG(PIN_6_ADC_1),  DEFIO_TAG(PIN_7_ADC_1),  DEFIO_TAG(PIN_8_ADC_1),
       DEFIO_TAG(PIN_9_ADC_1),  DEFIO_TAG(PIN_10_ADC_1), DEFIO_TAG(PIN_11_ADC_1), DEFIO_TAG(PIN_12_ADC_1),
       DEFIO_TAG(PIN_13_ADC_1), DEFIO_TAG(PIN_14_ADC_1),
#if defined(STM32F767xx)
	   DEFIO_TAG(PIN_15_ADC_1), DEFIO_TAG(PIN_16_ADC_1),
#endif
	   }, DMA_ADC_1},
    { {DEFIO_TAG(PIN_1_ADC_2),  DEFIO_TAG(PIN_2_ADC_2),  DEFIO_TAG(PIN_3_ADC_2),  DEFIO_TAG(PIN_4_ADC_2),
       DEFIO_TAG(PIN_5_ADC_2),  DEFIO_TAG(PIN_6_ADC_2),  DEFIO_TAG(PIN_7_ADC_2),  DEFIO_TAG(PIN_8_ADC_2),
       DEFIO_TAG(PIN_9_ADC_2),  DEFIO_TAG(PIN_10_ADC_2), DEFIO_TAG(PIN_11_ADC_2), DEFIO_TAG(PIN_12_ADC_2),
       DEFIO_TAG(PIN_13_ADC_2), DEFIO_TAG(PIN_14_ADC_2),
#if defined(STM32F767xx)
	   DEFIO_TAG(PIN_15_ADC_2), DEFIO_TAG(PIN_16_ADC_2),
#endif
       }, DMA_ADC_2},
    { {DEFIO_TAG(PIN_1_ADC_3),  DEFIO_TAG(PIN_2_ADC_3),  DEFIO_TAG(PIN_3_ADC_3),  DEFIO_TAG(PIN_4_ADC_3),
       DEFIO_TAG(PIN_5_ADC_3),  DEFIO_TAG(PIN_6_ADC_3),  DEFIO_TAG(PIN_7_ADC_3),  DEFIO_TAG(PIN_8_ADC_3),
       DEFIO_TAG(PIN_9_ADC_3),  DEFIO_TAG(PIN_10_ADC_3), DEFIO_TAG(PIN_11_ADC_3), DEFIO_TAG(PIN_12_ADC_3),
       DEFIO_TAG(PIN_13_ADC_3), DEFIO_TAG(PIN_14_ADC_3),
#if defined(STM32F767xx)
	   DEFIO_TAG(PIN_15_ADC_3), DEFIO_TAG(PIN_16_ADC_3),
#endif
	   }, DMA_ADC_3},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void fnResetGP_configADC(configADC_t *configADC)
**  Descripcion:    Funcion de reset de la configuracion del ADC
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configADC(configADC_t *configADC)
{
    for (uint8_t i = 0; i < NUM_MAX_ADC; i++) {
        for (uint8_t j = 0; j < NUM_CANALES_ADC; j++)
    	    configADC[i].pin[j] = configADCdefecto[i].pin[j];

    	configADC[i].dma = configADCdefecto[i].dma;
    }
}

#endif
