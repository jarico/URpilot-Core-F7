/***************************************************************************************
**  tiempo.h - Fichero que contiene las funciones de tiempo
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/12/2020
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

#ifndef __TIEMPO_H
#define __TIEMPO_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern volatile uint32_t tiempoSysTick;           // Variable que se incrementa cada ms en la interrupcion del systick timer
extern volatile uint32_t ciclosSysTick;           // Valor del systick en la interrupcion
extern volatile int32_t sysTickPendiente;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarContadorCiclos(void);                // Inicia la variable que dicta los ciclos por microsegundo. Variable utilizada en las funciones de tiempo
uint32_t microsISR(void);                        // Retorna los microsegundos cuando se esta ejecutando una interrupcion (maximo 70 minutos)
uint32_t micros(void);
uint32_t millis(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

#endif // __TIEMPO_H
