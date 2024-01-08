/***************************************************************************************
**  plataforma.h - Este fichero contiene los defines generales relacionados con la familia
**                 del microcontrolador escogido
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/04/2019
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

#ifndef __PLATAFORMA_H
#define __PLATAFORMA_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "hardware.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F730xx) || defined(STM32F732xx) || defined(STM32F733xx) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F750xx) \
 || defined(STM32F756xx) || defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F768xx) || defined(STM32F769xx) || defined(STM32F777xx) || defined(STM32F778xx) || defined(STM32F779xx)
  #ifndef STM32F7
    #define STM32F7
    #include "stm32f7xx.h"
    #include "stm32f7xx_hal.h"
  #endif
#endif

#ifdef STM32F7
  #define USAR_ESTADISTICAS_TAREAS
  #define USAR_ITCM_RAM
  #define USAR_SRAM2
  #define USAR_DTCM_RAM
#endif

#ifndef DEBUG
  #ifdef USAR_ITCM_RAM
  #define CODIGO_RAPIDO       __attribute__((section(".codigoRapido")))
  #else
  #define CODIGO_RAPIDO
  #endif
#else
  #define CODIGO_RAPIDO
#endif

#ifdef USAR_SRAM2
  #define SRAM2               __attribute__ ((section(".sram2_data"), aligned(4)))
  #define SRAM2_INI           __attribute__ ((section(".sram2_bss"), aligned(4)))
#else
  #define SRAM2
  #define SRAM2_INI
#endif

#ifdef USAR_DTCM_RAM
  #define RAM_RAPIDA          __attribute__ ((section(".ramRapida_data"), aligned(4)))
  #define RAM_RAPIDA_INI      __attribute__ ((section(".ramRapida_bss"), aligned(4)))
#else
  #define RAM_RAPIDA
  #define RAM_RAPIDA_INI
#endif

#if defined(STM32F7)
  #define LIMITE_FREC_US_SCHEDULER    10     // Limite de frecuencia en us (10us = 100kHz)
#else
  #define LIMITE_FREC_US_SCHEDULER    100    // Limite de frecuencia en us (100us = 10kHz)
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __PLATAFORMA_H

