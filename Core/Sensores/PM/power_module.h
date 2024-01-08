/***************************************************************************************
**  power_module.h - Funciones y datos del power module
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/12/2020
**  Fecha de modificacion: 11/12/2020
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

#ifndef __POWER_MODULE_H
#define __POWER_MODULE_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Drivers/bus.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if defined(STM32F767xx)
  #define NUM_MAX_POWER_MODULE    2
#elif defined(STM32F722xx)
  #define NUM_MAX_POWER_MODULE    1
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    POWER_MODULE_1 = 0,
    POWER_MODULE_2
} numPowerModule_e;

typedef enum {
    POWER_MODULE_NINGUNO   = -1,
    POWER_MODULE_ANALOGICO =  0,
    POWER_MODULE_I2C,
} tipoPowerModule_e;

typedef struct {
    uint32_t ultimaActualizacion;        // Tiempo en us
    uint32_t ultimaMedida;               // Tiempo en us
    uint32_t ultimoCambio;               // Tiempo en us
} timingPM_t;

typedef struct {
    numPowerModule_e numPM;
    tipoPowerModule_e tipo;
    bus_t bus;
    bool iniciado;
    bool conectado;
    bool operativo;
    timingPM_t timing;
    float tension;
    float corriente;
} powerModule_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarPowerModule(void);
void leerPowerModule(uint32_t tiempoActual);
void actualizarPowerModule(uint32_t tiempoActual);

float tensionPowerModule(void);
float corrientePowerModule(void);
float potenciaPowerModule(void);
float energiaPowerModule(void);

#endif // __POWER_MODULE_H
