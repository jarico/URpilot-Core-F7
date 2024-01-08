/***************************************************************************************
**  calibrador_imu.h - Funciones y variables comunes a la calibracion de la IMU
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 24/03/2021
**  Fecha de modificacion: 24/03/2021
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

#ifndef __CALIBRADOR_IMU_H_
#define __CALIBRADOR_IMU_H_

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
typedef struct {
    bool calibrado;
    float offset[3];
} calParamGiroscopio_t;

typedef struct {
    bool calibrado;
    float offset[3];
    float ganancia[3][3];
} calParamAcelerometro_t;

typedef struct {
	calParamGiroscopio_t calGiroscopio;
	calParamAcelerometro_t calAcelerometro;
} calIMU_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarCalAcel(void);
void iniciarCalGir(void);
void terminarCalAcel(void);
void terminarCalGir(void);
void actualizarCalAcel(uint32_t tiempoActual);
void actualizarCalGir(uint32_t tiempoActual);


#endif // __CALIBRADOR_IMU_H_
