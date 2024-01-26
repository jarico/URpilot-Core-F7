/***************************************************************************************
**  control_ModelMatching.h - Funciones generales de los controladores cuando se emplea
**  una arquitectura de control Model Matching en cascada
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Javier Rico
**  Fecha de creacion: 20/11/2023
**  Fecha de modificacion: 20/11/2022
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
**  v1.0  Javier Rico. Se ha liberado la primera version estable
**
****************************************************************************************/

#ifndef __CONTROL_MODEL_MATCHING_H_
#define __CONTROL_MODEL_MATCHING_H_

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


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarControladoresMM(void);
void actualizarControlVelAngularMM(void);
void actualizarControlActitudMM(void);
void get_uFF_MM(float *out);
void get_uActPID_MM(float *out);
void get_uPID_MM(float *out);
void get_uTotal_MM(float *out);
void get_rActitud_MM(float *out);
void get_rModVelAng_MM(float *out);
void get_rVelAng_MM(float *out);

//void actualizarModelos(void);
//void actualizarControlFF(void);


#endif // __CONTROL_MODEL_MATCHING_H_
