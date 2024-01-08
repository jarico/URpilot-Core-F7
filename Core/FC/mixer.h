/***************************************************************************************
**  mixer.h - Funciones relativas al PID Mix
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 30/08/2019
**  Fecha de modificacion: 21/09/2020
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

#ifndef __MIXER_H_
#define __MIXER_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    DRON_QUADCOPTER_X = 0,
    DRON_QUADCOPTER_P,
	DRON_HEXACOPTER_X,
	DRON_HEXACOPTER_H,
    DRON_OCTOCOPTER_X,
    DRON_OCTOCOPTER_H,
	// Doble motor
	DRON_QUADCOPTER_2X,
    DRON_QUADCOPTER_2P,
	DRON_HEXACOPTER_2X,
	DRON_HEXACOPTER_2H,
} tipoDrone_e;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern bool ordenPararMotores;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarMixer(void);
void actualizarMixer(void);
uint8_t numMotores(void);
void encenderMotoresMixer(void);
void apagarMotoresMixer(void);
bool motoresEncendidosMixer(void);

#endif // __MIXER_H_
