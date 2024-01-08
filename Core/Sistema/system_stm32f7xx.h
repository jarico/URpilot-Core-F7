/***************************************************************************************
**  system_stm32fxx.h - Este fichero contiene las funciones de configuracion del
**                         hardware iniciales.
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

#ifndef __SYSTEM_STM32F7XX_H
#define __SYSTEM_STM32F7XX_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif 


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern uint32_t SystemCoreClock;            // Frecuencia del reloj del sistema
extern const uint8_t  AHBPrescTable[16];    // Valores de los prescalers AHB
extern const uint8_t  APBPrescTable[8];     // Valores de los prescalers APB


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
extern void SystemInit(void);
extern void resetearSiOverclock(uint32_t nivelOverclock);
extern void iniciarSistemaOverclock(void);
extern void configurarRelojSistema(void);


#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_STM32F7XX_H

