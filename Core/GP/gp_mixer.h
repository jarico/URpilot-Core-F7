/***************************************************************************************
**  gp_mixer.h - Funciones y grupo de parametros del mixer
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/09/2020
**  Fecha de modificacion: 07/09/2020
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

#ifndef __GP_MIXER_H
#define __GP_MIXER_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "gp.h"
#include "FC/mixer.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	tipoDrone_e tipoDrone;
	float curvaPWM;         // Valor entre 0 y 1: thrust = (1 - curvaPWM) * PWM + curvaPWM * PWM^2
	float valorArmado;      // Valor entre 0 y 1: valor en el cual los motores empiezan a girar
	float valorMaximo;      // Valor entre 0 y 1: valor en el cual los motores saturan
	float valorMinimo;      // Valor entre 0 y 1: valor en el cual los motores empiezan a ejercer fuerza
} configMixer_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
DECLARAR_GP(configMixer_t, configMixer);


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


#endif // __GP_MIXER_H
