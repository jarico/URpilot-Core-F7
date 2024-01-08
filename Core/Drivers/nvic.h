/***************************************************************************************
**  nvic.h - Funciones y definiciones relacionadas con las interrupciones del sistema
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/05/2019
**  Fecha de modificacion: 03/12/2020
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

#ifndef __NVIC_H
#define __NVIC_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NVIC_PRIO_MAX                      CONSTRUIR_PRIORIDAD_NVIC(0, 1)    // No se puede usar 0 porque el valor 0 no toma efecto
#define NVIC_PRIO_TIMER                    CONSTRUIR_PRIORIDAD_NVIC(1, 1)
#define NVIC_PRIO_USB                      CONSTRUIR_PRIORIDAD_NVIC(6, 0)
#define NVIC_PRIO_LED                      CONSTRUIR_PRIORIDAD_NVIC(6, 1)
#define NVIC_PRIO_SERIALUART1              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SERIALUART2              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SERIALUART3              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SERIALUART4              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SERIALUART5              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SERIALUART6              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SERIALUART7              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SERIALUART8              CONSTRUIR_PRIORIDAD_NVIC(1, 2)
#define NVIC_PRIO_SDMMC1                   CONSTRUIR_PRIORIDAD_NVIC(1, 0)
#define NVIC_PRIO_SDMMC2                   CONSTRUIR_PRIORIDAD_NVIC(1, 0)

// Macros para generar o partir la prioridad
#define CONSTRUIR_PRIORIDAD_NVIC(base,sub)      (((((base) << (__NVIC_PRIO_BITS - (7 - (NVIC_PRIORITYGROUP_2)))) | ((sub) & (0x0F >> (7 - (NVIC_PRIORITYGROUP_2))))) << __NVIC_PRIO_BITS) & 0xf0)
#define PRIORIDAD_BASE_NVIC(prio)               (((prio) >> (__NVIC_PRIO_BITS - (7 - (NVIC_PRIORITYGROUP_2)))) >> __NVIC_PRIO_BITS)
#define PRIORIDAD_SUB_NVIC(prio)                (((prio) & (0x0F >> (7 - (NVIC_PRIORITYGROUP_2)))) >> __NVIC_PRIO_BITS)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
extern void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SysTick_Handler(void);

#endif // __NVIC_H
