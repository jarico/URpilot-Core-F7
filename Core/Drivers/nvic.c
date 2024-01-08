/***************************************************************************************
**  nvic.c - Funciones y definiciones relacionadas con las interrupciones del sistema
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/05/2019
**  Fecha de modificacion: 20/09/2020
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
#include <stdio.h>

#include "nvic.h"
#include "tiempo.h"
#include "atomico.h"
#include "Core/led_estado.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void NMI_Handler(void)
**  Descripcion:    Interrupcion NMI
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void NMI_Handler(void)
{
#ifdef DEBUG
	printf("Interrupcion NMI\n");
#endif

    while (1) {
        escribirLedEstado(ROJO);
        delay(50);
        escribirLedEstado(NEGRO);
        delay(50);
    }
}


/***************************************************************************************
**  Nombre:         void HardFault_Handler(void)
**  Descripcion:    Interrupcion por fallo de hardware
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void HardFault_Handler(void)
{
#ifdef DEBUG
	printf("Interrupcion HardFault\n");
#endif

    while (1) {
        escribirLedEstado(ROJO);
        delay(50);
        escribirLedEstado(NEGRO);
        delay(50);
    }
}


/***************************************************************************************
**  Nombre:         void MemManage_Handler(void)
**  Descripcion:    Interrupcion por fallo de memoria
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void MemManage_Handler(void)
{
#ifdef DEBUG
	printf("Interrupcion MemManage\n");
#endif

    while (1) {
        escribirLedEstado(ROJO);
        delay(50);
        escribirLedEstado(NEGRO);
        delay(50);
    }
}


/***************************************************************************************
**  Nombre:         void BusFault_Handler(void)
**  Descripcion:    Interrupcion por fallo de Pre-fetch, fallo de acceso a memoria
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void BusFault_Handler(void)
{
#ifdef DEBUG
	printf("Interrupcion BusFault\n");
#endif

    while (1) {
        escribirLedEstado(ROJO);
        delay(50);
        escribirLedEstado(NEGRO);
        delay(50);
    }
}


/***************************************************************************************
**  Nombre:         void UsageFault_Handler(void)
**  Descripcion:    Interrupcion por fallo de instruccion no definida o estado ilegal
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void UsageFault_Handler(void)
{
#ifdef DEBUG
    printf("Interrupcion UsageFault\n");
#endif

    while (1) {
        escribirLedEstado(ROJO);
        delay(50);
        escribirLedEstado(NEGRO);
        delay(50);
    }
}


/***************************************************************************************
**  Nombre:         void SysTick_Handler(void)
**  Descripcion:    Interrupcion del System tick timer
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void SysTick_Handler(void)
{
    BLOQUE_ATOMICO(NVIC_PRIO_MAX) {
        tiempoSysTick++;                  // Variable usada en la funcion millis
        ciclosSysTick = SysTick->VAL;
        sysTickPendiente = 0;
        (void)(SysTick->CTRL);
    }

    HAL_IncTick();
}

